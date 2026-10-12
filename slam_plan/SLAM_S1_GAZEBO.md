# SLAM S1/S2: build a map of the sim rooms with one camera

**Goal:** the simulated drone flies slowly round a textured two-room copy of the sim room,
stella_vslam builds a map from the forward camera alone, `slam_scale` turns it into metres, and
`make_map` turns it into a floor map with named rooms. This is the whole webcam mapping chain,
tested where nothing can crash. Plan: [SLAM_NAV_PLAN.md](SLAM_NAV_PLAN.md).

**Where:** the PC / WSL that already runs the Gazebo sim ([GAZEBO_SIM.md](GAZEBO_SIM.md) steps 1
to 3 work). Nothing here talks to the Pixhawk.

**Safety:** `sim_survey.py` flies the SIMULATED drone directly through SITL, bypassing the
safety supervisor. It is only for moving the sim camera around. The real drone will be carried by
hand for mapping.

---

## What is new (all in `src/drone_slam/`)

| File | What it does |
|---|---|
| `sim/worlds/indoor_search_textured.sdf` | copy of the sim room with picture-like textures on the floor, walls and boxes, and a partition wall at x = 1.5 m with a doorway: **west room** (drone starts here) and **east room**. Same world name, drone, camera and people as the original |
| `sim/make_textured_world.py` | made that world and `sim/textures/` (only re-run to change the room) |
| `sim/setup_stella.sh` | one-time install of stella_vslam + stella_vslam_ros + the ORB vocabulary |
| `sim/sim_survey.py` | sim only: GUIDED take-off to 1 m, slow loop of both rooms (0.25 m/s, turns 15 deg/s, about 4 min), land |
| `launch/slam.launch.py` | starts stella_vslam (`run_slam`) on `/camera/image_raw` + `slam_scale`; `mode:=map` saves the map on Ctrl-C, `mode:=localize` only finds itself in a saved map |
| `config/stella_sim.yaml` | stella settings for the sim camera (640x480, 60 deg) |
| `config/rooms_sim.yaml` | `start`, `west_room`, `east_room` |

**Checked here, not yet run on a PC:** the drone_slam unit tests (46 pass); the textured world
file is valid and keeps every name the sim scripts use; stella_vslam's topics and map format were
read from its source code; and the stella_vslam library **compiles on Ubuntu 24.04** (built here
with g2o from source; your install uses ROS's `ros-jazzy-libg2o` instead). The ROS wrapper,
Gazebo, SITL and an actual SLAM run have not run here (no ROS packages reachable from here).

---

## Step 1: install the SLAM engine (once)

**COMMAND**

```bash
source ~/drone_ws/env.sh
bash ~/drone_ws/src/drone_slam/sim/setup_stella.sh
```

Then add the line it prints to the **end** of `~/drone_ws/env.sh`, open a new terminal, and build
the new package:

```bash
source ~/drone_ws/env.sh
cd ~/drone_ws && colcon build --symlink-install --packages-select drone_slam
source ~/drone_ws/env.sh
ros2 pkg executables stella_vslam_ros | grep run_slam
colcon test --packages-select drone_slam && colcon test-result --verbose | tail -1
```

**EXPECTED** `STELLA SETUP DONE`, then `stella_vslam_ros run_slam`, then `0 errors, 0 failures`.

**IF NOT**
- the build is killed / `c++: fatal error: Killed`: out of memory. `JOBS=2 bash .../setup_stella.sh`.
- `ros-jazzy-libg2o` not found: `sudo apt update` and try again; send me the output if it stays.
- any other compile error: send me the last 40 lines.

## Step 2: the textured sim (terminal A)

```bash
source ~/drone_ws/env.sh
bash ~/drone_ws/sim/gazebo/scripts/run_sim.sh --gui \
  --world ~/drone_ws/src/drone_slam/sim/worlds/indoor_search_textured.sdf
```

**EXPECTED** the room with colourful walls, a wooden floor, and a wall with a doorway in front of
the drone. `MAVLink on tcp://127.0.0.1:5760`, `camera bridge started`.

**IF NOT** walls grey/white with no pictures: the textures were not found. Send me the lines of
`~/.drone_sim/gz.log` that mention `png` or `jpg`.

## Step 3: MAVROS only (terminal B)

Only the link, for the drone's height and position (the `slam_scale` input). The rest of the
stack stays off, so nothing else commands the drone.

```bash
source ~/drone_ws/env.sh
ros2 launch drone_bringup fcu.launch.py fcu_url:=tcp://127.0.0.1:5760 config:=$HOME/sim_drone.yaml
```

**EXPECTED** `FCU link up`. (`~/sim_drone.yaml` is from GAZEBO_SIM.md step 3.)

## Step 4: start SLAM (terminal C)

```bash
source ~/drone_ws/env.sh
ros2 launch drone_slam slam.launch.py config:=sim map:=~/drone_ws/maps/sim.msg
```

**EXPECTED** stella prints its settings and waits. It can't start yet: the drone isn't moving.

## Step 5: fly the survey (terminal D)

```bash
source ~/drone_ws/env.sh
python3 ~/drone_ws/src/drone_slam/sim/sim_survey.py
```

**EXPECTED**
- `connected`, `airborne; flying the survey`; in Gazebo the drone shuffles left and right, then
  loops the west room, goes through the doorway, loops the east room, comes back and lands:
  `SURVEY DONE: landed and disarmed` after about 5 minutes.
- terminal C, during the shuffle: stella reports that **initialization succeeded**.
- in another terminal, `ros2 topic echo /diagnostics | grep -A8 slam_scale`: the message goes from
  `n/8 moves seen` to **`locked`**, with `scale_m_per_unit` staying about the same.
- `ros2 topic echo /drone/slam/pose --field pose.position` while flying: x and y should follow
  the drone (x forward at the start, y left), within a few tens of centimetres of
  `ros2 topic echo /mavros/local_position/pose --field pose.position` (MAVROS is east/north).

**IF NOT**
- stella never initializes: send me terminal C. (Images not arriving: `ros2 topic hz /camera/image_raw`
  should say about 10.)
- stella says tracking is **lost** after a turn: note where in the path (west room, doorway, east
  room). It should relocalize when it sees a known place again. Tell me if it doesn't.
- `slam_scale` never locks: send the `/diagnostics` slam_scale lines.
- the survey stops with `FAILED: arm`: the `FCU:` lines above it say why (send them).

## Step 6: save the map and draw the floor plan

Ctrl-C in **terminal C** (stella saves the map now: `save the MessagePack file of database to
.../sim.msg`). Then:

```bash
ros2 run drone_slam make_map ~/drone_ws/maps/sim.msg --session ~/drone_ws/slam_session.yaml \
  --rooms $(ros2 pkg prefix drone_slam)/share/drone_slam/config/rooms_sim.yaml \
  --out ~/drone_ws/maps/sim
```

**EXPECTED** `ROOMS: OK` and `~/drone_ws/maps/sim_preview.png`: two rooms, black walls, the
doorway open, the rooms marked in red. The room is 10 x 10 m with the partition 1.5 m in front of
the start point, so the walls should be about 5 m left/right/behind and the partition about
1.5 m ahead (the start mark is the green triangle).

**IF NOT** `ROOMS: FIX ... unknown`: the map is too thin there. Send the preview anyway.

## Step 7: send me

1. `~/drone_ws/maps/sim_preview.png` and the lines `make_map` printed;
2. `~/drone_ws/slam_session.yaml`;
3. whether stella lost tracking anywhere, and where.

Stop: Ctrl-C in terminals B and C, then `bash ~/drone_ws/sim/gazebo/scripts/run_sim.sh --stop`.
