# Gazebo simulation: the whole stack in a virtual room

**Goal:** run the exact code that goes on the Pi (Phases 2 to 13, nothing changed) against a
simulated drone in Gazebo: ArduCopter 4.6.3 SITL flies a Gazebo quadcopter, a Gazebo camera on
the drone feeds YOLO, and 3D people stand or walk in a 10 x 10 m room.

**Where:** a **PC** with Ubuntu 24.04 and ROS 2 Jazzy, not the Pi (Gazebo's camera rendering is
too heavy for the Pi 5). Nothing here talks to your Pixhawk.

---

## What is in `sim/gazebo/`

| File | What it does |
|---|---|
| `worlds/indoor_search.sdf` | the room: floor, walls, two boxes, `person_a` (standing), `person_b` (walking), the drone (ArduPilot's iris + a forward camera, 640x480, 60 deg, 15 Hz) |
| `params/gazebo_indoor.parm` | SITL parameters: no GPS, optical flow + rangefinder simulated by SITL, EKF3 on flow (same setup as your drone). **Not for the Pixhawk** |
| `scripts/setup_gazebo.sh` | one-time install: Gazebo Harmonic + ros_gz, ArduPilot SITL Copter-4.6.3, the ArduPilot Gazebo plugin |
| `scripts/run_sim.sh` | starts Gazebo, SITL (MAVLink on tcp 5760) and the camera bridge; `--gui` shows the window; `--stop` stops it |
| `scripts/camera_relay.py` | Gazebo camera → `/camera/image_raw`, re-stamped with the normal clock like the real camera |
| `scripts/move_person.py` | `place`, `hide`, or `walk` a person |
| `scripts/gz_scenarios.py` | 6 automatic mission tests, writes `~/drone_ws/gazebo_report.txt` + snapshots |
| `models/person/` | the two person meshes (CC BY 3.0, see `ATTRIBUTION.md`) |

**How it fits together**

```
Gazebo (physics, room, people, camera) <-- JSON/UDP 9002 --> ArduCopter SITL 4.6.3
   | camera                                                       | MAVLink tcp 5760
   v                                                              v
ros_gz_bridge -> camera_relay -> /camera/image_raw        MAVROS (fcu_url:=tcp://127.0.0.1:5760)
                                   |                              |
                                   +---- your unchanged stack: fcu, safety, yolo, tracker,
                                         mission, intent_bridge ----+
```

The only differences from the Pi: `fcu_url` points at SITL, `use_camera:=false` (the Gazebo camera
replaces v4l2_camera), and a SITL copy of drone.yaml (step 3).

**Tested here** (Gazebo Harmonic 8.10, ArduCopter SITL Copter-4.6.3, the Phase 13 workspace,
no GPU, about 0.9x real time, camera about 10 frames/s):

| Test | Result |
|---|---|
| G1 person standing 120 deg left | found after about 90 deg of turning, tracked 15 s (error median 0.03), turned back, landed and disarmed: **COMPLETED** |
| G2 person walking round the drone at 8 deg/s | followed for 20 s, kept in view (steady error 0.36, the price of the gentle gain): COMPLETED |
| G3 person hidden for 3 s while tracked | TARGET_LOST, REACQUIRING, TRACKING again: COMPLETED |
| G4 nobody in the room | 361 deg search, turned back, landed: TARGET_NOT_FOUND |
| G5 typed "look for a person and track them for 60 seconds", then "stop" | started by text, cancelled by text, landed and disarmed: CANCELLED |
| G6 cancel while tracking | landed and disarmed: CANCELLED |

The first run of G2 failed with "no climb within 5 s": in Gazebo the iris needs 3.5 to 4.7 s
from the takeoff command to lift-off, too close to the 5 s limit. The simulator copy of
drone.yaml now allows 8 s (step 3). On the real drone the limit stays 5 s and is marked
`[TUNE]`: if your first real takeoff (Phase 8, props on) says "no climb" although the motors
spun up, raise `takeoff_start_timeout_s` to 7.

---

## Step 1: one-time setup on the PC

**WHAT** installs Gazebo, builds ArduPilot SITL and the plugin. 20 to 40 minutes.

```bash
mkdir -p ~/drone_ws && tar -xzf drone_ws_complete.tar.gz -C ~/drone_ws --strip-components=1
bash ~/drone_ws/sim/gazebo/scripts/setup_gazebo.sh
```

**EXPECTED** ends with `SETUP DONE`.

Then build the workspace on the PC, as on the Pi (Phase 2/3 steps):

```bash
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras geographiclib-tools
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
cd ~/drone_ws && rosdep install --from-paths src --ignore-src -r -y
bash ~/drone_ws/tools/setup_venv.sh
bash ~/drone_ws/tools/verify_workspace.sh
```

**EXPECTED** `Summary: 260 tests, 0 errors, 0 failures, 0 skipped`, `WORKSPACE CHECK: ALL PASSED`.
(The YOLO model is inside the tarball, in `models/`. drone.yaml points at `/home/virtua/...`;
step 3 fixes the path for the PC.)

**IF NOT**
- `setup_gazebo.sh` fails in step 2 (ArduPilot): run `~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y`,
  log out and in, and run `setup_gazebo.sh` again.
- `libgz-sim8-dev` / cmake cannot find gz-sim: `sudo apt install libgz-sim8-dev`, re-run.

## Step 2: start the simulation (terminal A)

```bash
source ~/drone_ws/env.sh
bash ~/drone_ws/sim/gazebo/scripts/run_sim.sh --gui
```

**EXPECTED** a Gazebo window with the room, the drone in the middle and a person to its left.
`MAVLink on tcp://127.0.0.1:5760` and `camera bridge started`.

**IF NOT** no window / black window: run without `--gui` (headless works the same). Logs are in
`~/.drone_sim/`.

## Step 3: start the drone stack against the simulator (terminal B)

A SITL copy of your drone.yaml: EKF origin set, SITL's flow quality (about 51) allowed, 8 s
instead of 5 s for the lift-off (the Gazebo iris needs 3.5 to 4.7 s, too close to 5 s), text
commands allowed to execute, and the model path of this PC. Your real drone.yaml is not changed.

```bash
source ~/drone_ws/env.sh
python3 - <<'EOF'
import yaml, os
src = os.path.expanduser('~/drone_ws/src/drone_bringup/config/drone.yaml')
c = yaml.safe_load(open(src))
c['fcu_command_node']['ros__parameters'].update(ekf_origin_lat_deg=-35.3633, ekf_origin_lon_deg=149.1652)
c['fcu_state_node']['ros__parameters']['flow_min_quality'] = 40
c['fcu_command_node']['ros__parameters']['takeoff_start_timeout_s'] = 8.0
c['yolo_detector']['ros__parameters']['model_path'] = os.path.expanduser('~/drone_ws/models/yolo11n_320.onnx')
c['intent_bridge']['ros__parameters']['allow_execute'] = True
yaml.safe_dump(c, open(os.path.expanduser('~/sim_drone.yaml'), 'w'), sort_keys=False)
print('wrote ~/sim_drone.yaml')
EOF
ros2 launch drone_bringup bringup.launch.py fcu_url:=tcp://127.0.0.1:5760 \
  config:=$HOME/sim_drone.yaml use_camera:=false enable_fcu_output:=true allow_arming:=true
```

**EXPECTED** within 30 s: `FCU link up`, `yolo_detector ... up`, `target_tracker ... target NONE`
then (the person is not in view yet) staying NONE. A `PreArm: Gyros inconsistent` at start is
normal in SITL and clears.

## Step 4: one mission by hand (terminal C)

```bash
source ~/drone_ws/env.sh
python3 ~/drone_ws/tools/flight_op.py preflight
python3 ~/drone_ws/tools/mission.py start --track 15
```

**EXPECTED** in about 55 s:

```
PREFLIGHT, TAKING_OFF, SEARCHING (turning left), PERSON_DETECTED at about 90 deg,
TRACKING, RETURNING, LANDING
RESULT: COMPLETED: tracked for 15 s; back at the start heading; landed and disarmed
```

In the Gazebo window the drone takes off, turns left until the person is in view, holds them
centred, turns back and lands.

**Before the next mission** set the mode back to STABILIZE (your RC mode switch in real life):

```bash
python3 -c "from pymavlink import mavutil as u; m=u.mavlink_connection('tcp:127.0.0.1:5762', source_system=255); m.wait_heartbeat(); m.set_mode('STABILIZE')"
```

Try text commands too: `python3 ~/drone_ws/tools/command.py -i`, then
`look for a person and track them for 20 seconds`, `y`, and later `stop`.

Move people: `python3 ~/drone_ws/sim/gazebo/scripts/move_person.py place person_a 45 3`
(45 deg left, 3 m), `hide person_a`, `walk person_b 30 3 8`.

## Step 5: the automatic scenarios (terminal C)

```bash
python3 ~/drone_ws/sim/gazebo/scripts/gz_scenarios.py
```

**EXPECTED** 6 `PASS` lines and `GAZEBO SCENARIOS: ALL PASSED` (about 10 minutes). Report:
`~/drone_ws/gazebo_report.txt`, pictures `~/drone_ws/gazebo_G1.jpg` ...

**IF NOT** send me `gazebo_report.txt` and terminal B's output.

Stop everything: Ctrl-C in terminal B, then `bash ~/drone_ws/sim/gazebo/scripts/run_sim.sh --stop`.

---

## Honest limits

- The flow sensor and rangefinder are ArduPilot's simulated ones, fed by Gazebo's physics. They
  are cleaner than your MTF-01 over tiles and boxes. The Phase 5 flow test on the real drone is
  still required.
- The people are grey 3D models in good light. YOLO found them with 0.9 confidence. A real room,
  real clothes and real light will differ; Phases 10 and 11 on the Pi check that.
- Gazebo's quadcopter is ArduPilot's standard iris, not your frame. Tuning and physical yaw
  direction are checked at the first real tethered hover (Phase 8).
- A simulator pass is necessary, not sufficient: it never replaces the props-off and tethered
  steps in the phase docs.
