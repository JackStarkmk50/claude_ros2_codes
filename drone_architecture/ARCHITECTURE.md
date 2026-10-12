# Indoor Autonomous Drone — ROS 2 Jazzy Architecture (Phase 1, v2)

Status: **v2 confirmed 2026-09-27. Phases 2–7 delivered in `drone_ws/` (Phase 6, safety_supervisor + setpoint path in bench mode; Phase 7, first real setpoints props off: 2026-09-29, passed 2026-09-30; Phase 8, Pi-commanded TAKEOFF/LAND, SITL-verified: 2026-09-30). Phase 5's hand-held flow test is deferred until the drone runs on battery; it must pass before the first real flight.**

Phase 6 changes (both stricter): the command gate checks source, sequence, age, numbers and mode *before* the safety level, so an invalid command is never counted as a live stream; override detection runs in `safety_supervisor` (which will also own every flight operation's expected mode), and `fcu_command_node` goes silent on OVERRIDE and stays silent if safety then dies. ABORT → LAND is executed from Phase 8; before that an ABORT holds and the pilot lands.

Phase 8 additions: P4 resolved, the EKF origin IS needed without GPS (ArduPilot refuses GUIDED arming with "waiting for home"), so `fcu_command_node` sets a configured origin after link-up. Measured in ArduCopter 4.6.3 SITL: if `fcu_command_node` dies while hovering, GUIDED keeps holding position indefinitely, so `safety_supervisor` raises ABORT `COMMAND_NODE_LOST` (pilot must land); nothing else can command the FCU. After a LAND the mode stays LAND and PF9 blocks the next takeoff until the pilot moves the mode switch (deliberate interlock).

Phase 7 addition: `safety_supervisor` parameter `bench_ground_test` (default false, props-off only) lets yaw commands through the gate while armed on the ground, forces translation to zero, and makes the pre-flight check fail while on.

Phase 3 findings on the real Pixhawk (2026-09-29): the rangefinder arrives as `DISTANCE_SENSOR` (not `RANGEFINDER`), so `fcu_state_node` decodes it raw, together with `EKF_STATUS_REPORT` and `OPTICAL_FLOW`, from the FCU's own sysid/compid only. The MTF-01's own messages are forwarded to TELEM1 at 100 Hz (49 % of 115200); see `drone_ws/docs/PHASE4_DRONE_STATE.md` Part A. `SR1_*` are 0, so `fcu_state_node` requests every stream itself on each (re)connect. `ARMING_CHECK = 0` on the FCU makes PF4 meaningless until you set it to 1.

Confirmed by you: RC mode switch will have LOITER and GUIDED. **There is no motor kill switch yet** (see §11.6: strongly recommended before any powered test).

Changes from v1 (2026-09-27):
- **Fully autonomous flight is now the design target**: pre-flight validation → GUIDED → arm → takeoff → verified hover → mission → return/position → LAND → disarm confirmed from the FCU. The RC is the manual override and emergency mechanism, not the normal arm/takeoff path.
- New `FlightOperation` action (TAKEOFF, LAND) that goes **Mission → Safety → FCU**, same as velocity commands. Full command / acknowledgement / success / failure table in §11.5.
- RC / FCU override detection: any mode change the Pi did not request = manual authority; the Pi goes silent (§11.6).
- Your hardware is filled in: MicoAir **MTF-01** (optical flow + ToF rangefinder), **USB camera (Zebronics)**, **TELEM1**, **ArduCopter V4.6.3**, active cooler, baud 115200 now with a planned move to 921600.
- ArduPilot SITL on a PC is now a **required** step before any autonomous arm/takeoff on the real aircraft.

Target: Raspberry Pi 5 · Ubuntu 24.04 · ROS 2 Jazzy · Pixhawk1 · ArduCopter 4.6.3 · MAVLink 2 on SERIAL1 (TELEM1) @ 115200 (921600 later).

Everything defaults to **PROPS OFF / BENCH TEST**. Nothing commands the real aircraft until a phase says so and you confirm.

Legend:
- **[TUNE]** must be tuned experimentally. Numbers shown are conservative starting points, not validated values.
- **[VERIFY]** I am not certain for your exact hardware/firmware. Each has a concrete check in a phase.
- **[AIRCRAFT]** can affect the real aircraft.

Contents
1. Final architecture
2. Final package list
3. Node list
4. Topic list (with QoS)
5. Service list
6. Action list
7. Custom interface list
8. Dependency list
9. Data flow and mission state machine
10. TF / frame strategy
11. Safety architecture (incl. pre-flight, takeoff, landing, RC override)
12. Current vs future scope
13. Implementation phases
14. Exact installation requirements
15. Potential architectural problems
Appendix A: Flask decision · B: config sketch · C: reference repo notes · D: ArduPilot parameters to read (not change)

---

## 1. Final architecture

### 1.1 Layers

```text
                      RC transmitter (pilot)
                              │  manual override / emergency, always wins
                              ▼  (acts directly on the Pixhawk; the Pi only DETECTS it)
 ┌──────────────────────────────────────────────────────────────────────┐
 │  INTENT LAYER (slow, off the control loop)                           │
 │    drone_llm/intent_bridge  "look for a person and track him"        │
 │      → structured intent → validated → SearchAndTrack goal / cancel  │
 └───────────────────────────────┬──────────────────────────────────────┘
                                 │ action
 ┌───────────────────────────────▼──────────────────────────────────────┐
 │  MISSION LAYER (deterministic)  drone_mission/mission_manager        │
 │   preflight → takeoff → search → track → return → land → done        │
 │   outputs: FlightOperation goals (TAKEOFF / LAND)                    │
 │            ControlCommand stream (hold / yaw rate)                   │
 └──────────────┬───────────────────────────────▲───────────────────────┘
                │                               │ TrackedTarget, VehicleState, SafetyStatus
 ┌──────────────▼──────────────┐   ┌────────────┴────────────────────────┐
 │  SAFETY LAYER (veto)        │   │  PERCEPTION                         │
 │  drone_safety/              │   │  v4l2 camera → yolo_detector        │
 │   safety_supervisor         │   │               → target_tracker      │
 │   - health + pre-flight     │   └─────────────────────────────────────┘
 │   - command gate            │
 │   - flight-operation gate   │
 │   - override detection      │
 └──────────────┬──────────────┘
                │ validated command stream + approved flight operations
 ┌──────────────▼──────────────────────────────────────────────────────┐
 │  FCU LAYER — the ONLY code that talks to MAVROS                     │
 │   drone_fcu/fcu_command_node  setpoints · set_mode · arm · takeoff  │
 │                               · land · verification of each step    │
 │   drone_fcu/fcu_state_node    reads /mavros/* + raw MAVLink         │
 └──────────────┬──────────────────────────────────────────────────────┘
                │ MAVROS (mavros_node) — TELEM1 / SERIAL1, MAVLink 2
 ┌──────────────▼──────────────────────────────────────────────────────┐
 │  PIXHAWK / ArduCopter 4.6.3: stabilization, motors, EKF3 (fuses     │
 │  MTF-01 flow + MTF-01 rangefinder + IMU + baro), arming checks,     │
 │  modes, LAND controller, low-level failsafes                        │
 └─────────────────────────────────────────────────────────────────────┘
```

### 1.2 Core rules

1. **One door to the flight controller.** Only `fcu_command_node` publishes setpoints or calls MAVROS command services (set_mode, arming, takeoff). A test greps the source tree to enforce it.
2. **Safety holds a veto over everything that moves the aircraft**: velocity/yaw commands *and* flight operations (arm + takeoff, land). The LLM can only reach the mission action.
3. **Stale means invalid.** Every consumer checks message age on the node's steady clock. Missing data is a failure, never "keep the last value".
4. **Pixhawk owns flight.** The Pi sends high-level requests (mode, arm, takeoff altitude, body velocity + yaw rate, LAND). ArduPilot's EKF3 fuses optical flow. **No EKF on the Pi.** The Pi monitors flow and estimator health.
5. **Autonomous by design, pilot always has authority.** Normal missions arm, take off, fly and land autonomously. The pilot can take over at any moment by switching mode on the RC; the Pi detects that, stops sending anything, and **never** takes control back by itself (§11.6).
6. **Use ArduPilot's own mechanisms.** Takeoff uses `MAV_CMD_NAV_TAKEOFF` in GUIDED. Landing uses ArduPilot's **LAND mode**, not a Pi-controlled descent. Disarm after landing is done by ArduPilot's own landing detector; the Pi only confirms it. A LAND in progress completes even if the Pi crashes, because it is an FCU mode.
7. **Every step is verified from telemetry, not assumed from an acknowledgement.** "Command accepted" is necessary but never sufficient; each step has a state that must be observed within a timeout (§11.5).
8. **Bench mode is the default.** `fcu_command_node` starts with `output_enabled: false` and `allow_arming: false`. It logs what it would send and sends nothing. Each is enabled by an explicit launch argument.
9. **Logic is pure Python, ROS is a thin wrapper.** State machines, pre-flight checks, the command gate, the flight-operation sequencer, flow health and frame conversion live in modules without `rclpy`, so plain `pytest` tests them.

### 1.3 Decisions where there were real alternatives

**MAVLink bridge** — chosen: **MAVROS 2 (ros-jazzy-mavros)**.

| Option | For | Against |
|---|---|---|
| **MAVROS 2** (chosen) | apt package; `set_mode`, `cmd/arming`, `cmd/takeoff` services with MAV_RESULT; heartbeat; ENU/NED conversion; ArduPilot mode names; STATUSTEXT (PreArm reasons) as a topic; raw MAVLink tap | ArduPilot-specific messages (EKF_STATUS_REPORT, OPTICAL_FLOW) may not have a dedicated topic **[VERIFY]** |
| Custom pymavlink node | Full control, lighter | Re-implements command-ack handling, mode mapping, frame conversion: more safety-critical code of ours that can be wrong |
| ArduPilot native DDS | Native ROS 2 from the FCU | Changes SERIAL1 protocol, needs an agent; not now |

For messages MAVROS lacks, `fcu_state_node` decodes them read-only from MAVROS's raw stream `/uas1/mavlink_source` with `pymavlink`. No second serial connection.

**Landing mechanism** — chosen: **switch to LAND mode** (`/mavros/set_mode` "LAND"), not `MAV_CMD_NAV_LAND` inside GUIDED and not a Pi-driven descent. LAND mode holds horizontal position using flow, descends at `LAND_SPEED`, detects touchdown and disarms by itself, and keeps going if the Pi dies.

**Takeoff mechanism** — chosen: **GUIDED + `MAV_CMD_NAV_TAKEOFF`** via `/mavros/cmd/takeoff`. ArduPilot climbs to the requested relative altitude with its own controller. The Pi does not ramp throttle or send climb velocities.

**Language** — Python (rclpy) for all our nodes. 10–20 Hz loops are fine in Python; YOLO tooling is Python; one build system. Core modules can move to C++ later without interface changes.

**Mission command** — ROS 2 **action** `SearchAndTrack` (long-running, feedback, cancel, result).

**Flight operations** — ROS 2 **action** `FlightOperation` (takeoff and landing take seconds, have stages to report, and can fail midway).

**Detections** — standard `vision_msgs/Detection2DArray`; image size from `CameraInfo`.

---

## 2. Final package list

Seven packages. Optical flow is not its own package: the Pixhawk fuses it; the Pi monitors it inside `drone_fcu` and enforces it in `drone_safety`.

| Package | Build type | Responsibility | Depends on |
|---|---|---|---|
| `drone_interfaces` | ament_cmake (rosidl) | Custom msg/srv/action only | rosidl_default_generators, std_msgs, geometry_msgs, vision_msgs |
| `drone_fcu` | ament_python | FCU state aggregation, flow/EKF/pre-arm monitoring, the single MAVROS output, flight-operation executor (mode/arm/takeoff/land + verification), frame conversion | rclpy, drone_interfaces, mavros_msgs, sensor_msgs, geometry_msgs, diagnostic_updater, pymavlink (venv) |
| `drone_safety` | ament_python | Health supervision, pre-flight validation, command gate, flight-operation gate, override handling, latching faults, reset | rclpy, drone_interfaces, std_srvs, diagnostic_updater |
| `drone_perception` | ament_python | YOLO inference, detections, tracking / selection / loss | rclpy, drone_interfaces, vision_msgs, sensor_msgs, cv_bridge, numpy, onnxruntime (venv) |
| `drone_mission` | ament_python | SearchAndTrack action server, mission state machine (takeoff → search → track → return → land), search pattern, tracking controller | rclpy, drone_interfaces |
| `drone_llm` | ament_python | Text → validated intent → action goal; keyword fallback | rclpy, drone_interfaces, anthropic (venv, optional) |
| `drone_bringup` | ament_python | Launch files, YAML, MAVROS config, static TF, rosbag profile, SITL launch variant | launch, launch_ros, mavros, v4l2_camera, all above |

```text
drone_ws/src/
├── drone_interfaces/
│   ├── CMakeLists.txt  package.xml
│   ├── msg/    VehicleState FlowStatus TrackedTarget ControlCommand SafetyStatus MissionStatus
│   ├── srv/    SubmitCommand
│   └── action/ SearchAndTrack FlightOperation
├── drone_fcu/
│   ├── package.xml setup.py setup.cfg resource/drone_fcu
│   ├── drone_fcu/__init__.py
│   ├── drone_fcu/fcu_state_node.py fcu_command_node.py
│   ├── drone_fcu/core/ flow_health.py ekf_flags.py frames.py flight_ops.py mode_tracker.py
│   └── test/ test_flow_health.py test_frames.py test_ekf_flags.py test_flight_ops.py test_mode_tracker.py
├── drone_safety/      safety_supervisor.py + core/ gate.py health.py preflight.py op_gate.py
├── drone_perception/  yolo_detector.py target_tracker.py + core/ yolo_postprocess.py tracker.py selection.py
├── drone_mission/     mission_manager.py + core/ state_machine.py search.py track_control.py
├── drone_llm/         intent_bridge.py + core/ intent_schema.py keyword_parser.py
└── drone_bringup/
    ├── launch/ bringup.launch.py fcu.launch.py perception.launch.py autonomy.launch.py
    ├── config/ drone.yaml mavros_apm.yaml mavros_pluginlists.yaml
    └── README.md
```

Every Python package's `setup.cfg`:

```ini
[develop]
script_dir=$base/lib/<pkg>
[install]
install_scripts=$base/lib/<pkg>
```

pip-only packages reach the nodes through `drone_ws/env.sh`, which adds the venv's site-packages to `PYTHONPATH` (see P8).

---

## 3. Node list

| Node (executable) | Package | Rate | Purpose |
|---|---|---|---|
| `mavros_node` | mavros (apt) | — | MAVLink ↔ ROS 2, launched with our config (`fcu_url`, `tf.send: false`, plugin allow-list). |
| `v4l2_camera_node` | v4l2_camera (apt) | camera fps | USB camera → `/camera/image_raw` + `camera_info`. |
| `static_transform_publisher` ×2 | tf2_ros | latched | `base_link → camera_link → camera_optical_frame` |
| `fcu_state_node` | drone_fcu | 10 Hz | Aggregates MAVROS topics + decoded EKF_STATUS_REPORT / OPTICAL_FLOW / SYS_STATUS pre-arm bit into `VehicleState` and `FlowStatus`. Keeps the last PreArm STATUSTEXT. Requests message rates once on connect. |
| `fcu_command_node` | drone_fcu | 10 Hz | The single output. (a) Streams validated setpoints in GUIDED; HOLD if input stale. (b) Executes approved `FlightOperation`s (internal action) with step-by-step verification. (c) Tracks the mode *it* requested, to detect overrides. `output_enabled:=false`, `allow_arming:=false` by default. |
| `safety_supervisor` | drone_safety | 20 Hz | Health checks, pre-flight validation, `SafetyStatus`; command gate; public `FlightOperation` action server that validates then forwards to `fcu_command_node`; requests LAND on ABORT; declares OVERRIDE. |
| `yolo_detector` | drone_perception | best effort, newest frame only | ONNX YOLO on CPU → `Detection2DArray` stamped with image capture time. |
| `target_tracker` | drone_perception | per detection + 10 Hz timer | Association, selection, loss/reacquire → `TrackedTarget`. |
| `mission_manager` | drone_mission | 10 Hz | `SearchAndTrack` action server and mission state machine. Requests flight operations, streams `ControlCommand`. |
| `intent_bridge` | drone_llm | on request | `SubmitCommand` service → LLM tool call or keyword parser → validated goal / cancel. |

Why the flight-operation gate lives in `safety_supervisor` and execution in `fcu_command_node`: safety decides *whether* (it has all health data and the veto); the FCU node decides *how* (it alone talks to MAVROS). If `safety_supervisor` dies mid-takeoff, `fcu_command_node` sees `SafetyStatus` go stale and turns the takeoff into LAND (§11.5). If `fcu_command_node` dies, the FCU keeps its current mode (GUIDED holds position, measured in SITL; LAND completes), and safety raises `COMMAND_NODE_LOST` so the pilot lands.

`safety_supervisor` runs a `MultiThreadedExecutor` with a reentrant callback group, because it is both an action server (for mission) and an action client (to `fcu_command_node`).

---

## 4. Topic list

Namespaces: MAVROS default `/mavros`; ours under `/drone`; camera under `/camera`.

### 4.1 From MAVROS (all subscribed **BEST_EFFORT**)

A BEST_EFFORT subscriber matches both BEST_EFFORT and RELIABLE publishers, so it is always compatible. (A RELIABLE subscriber to a BEST_EFFORT publisher silently receives nothing.)

| Topic | Type | Used for |
|---|---|---|
| `/mavros/state` | mavros_msgs/State | connected, armed, guided, mode, system_status |
| `/mavros/extended_state` | mavros_msgs/ExtendedState | **landed_state** (ON_GROUND / TAKEOFF / IN_AIR / LANDING) **[VERIFY]** ArduPilot 4.6 sends EXTENDED_SYS_STATE at the requested rate |
| `/mavros/statustext/recv` | mavros_msgs/StatusText | PreArm failure reasons, arming/takeoff refusals, failsafe announcements |
| `/mavros/battery` | sensor_msgs/BatteryState | voltage, current, remaining |
| `/mavros/imu/data` | sensor_msgs/Imu | attitude, tilt check |
| `/mavros/local_position/velocity_body` | geometry_msgs/TwistStamped | body velocity (FLU) from EKF |
| `/mavros/local_position/pose` | geometry_msgs/PoseStamped | EKF relative position (drifts; informational + start heading) |
| `/mavros/rangefinder/rangefinder` or `/mavros/distance_sensor/*` | sensor_msgs/Range | MTF-01 height above floor **[VERIFY]** exact topic in Phase 3 |
| `/mavros/rc/in` | mavros_msgs/RCIn | RC link present, mode-switch channel (logging / override evidence) |
| `/mavros/setpoint_raw/target_local` | mavros_msgs/PositionTarget | FCU echo of the setpoint it follows (bench sign test) **[VERIFY]** |
| `/uas1/mavlink_source` | mavros_msgs/Mavlink | raw frames → decode EKF_STATUS_REPORT (193), OPTICAL_FLOW (100), SYS_STATUS (1) pre-arm bit |

### 4.2 Our topics

| Topic | Type | Publisher → Subscribers | Rate | QoS (reliability / durability / history / depth) | Why |
|---|---|---|---|---|---|
| `/camera/image_raw` | sensor_msgs/Image | camera → yolo_detector | cam fps | RELIABLE / VOLATILE / KEEP_LAST / 1 | Large; only newest frame matters. RELIABLE since Phase 9: BEST_EFFORT lost most large frames on FastDDS (measured 7 of 30 fps). |
| `/camera/camera_info` | sensor_msgs/CameraInfo | camera → detector, tracker | cam fps | RELIABLE / VOLATILE / KEEP_LAST / 1 | Same stream. |
| `/drone/perception/detections` | vision_msgs/Detection2DArray | detector → tracker | det fps | RELIABLE / VOLATILE / KEEP_LAST / 5 | Small; want each; age by stamp. |
| `/drone/perception/target` | drone_interfaces/TrackedTarget | tracker → mission, safety | 10 Hz | RELIABLE / VOLATILE / KEEP_LAST / 1 | Timer-published, so silence = tracker dead. |
| `/drone/perception/debug_image` | sensor_msgs/Image | detector (off by default) | ≤ 2 Hz | RELIABLE / VOLATILE / KEEP_LAST / 1 | Bench only. |
| `/drone/vehicle_state` | drone_interfaces/VehicleState | fcu_state_node → safety, mission, fcu_command | 10 Hz | RELIABLE / VOLATILE / KEEP_LAST / 1 | Newest only; age by stamp. |
| `/drone/flow_status` | drone_interfaces/FlowStatus | fcu_state_node → safety | 10 Hz | RELIABLE / VOLATILE / KEEP_LAST / 1 | Same. |
| `/drone/safety_status` | drone_interfaces/SafetyStatus | safety → mission, fcu_command, intent_bridge | 20 Hz + on change | RELIABLE / **TRANSIENT_LOCAL** / KEEP_LAST / 1 | Late joiners see the current verdict immediately. |
| `/drone/control/command` | drone_interfaces/ControlCommand | mission → safety | 10 Hz | RELIABLE / VOLATILE / KEEP_LAST / 1 | **Never** TRANSIENT_LOCAL: an old command must never be replayed. |
| `/drone/control/validated_command` | drone_interfaces/ControlCommand | safety → fcu_command | 10 Hz | RELIABLE / VOLATILE / KEEP_LAST / 1 | Same. |
| `/drone/mission/status` | drone_interfaces/MissionStatus | mission → anyone | 5 Hz + on change | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST / 1 | Late joiners see mission state. |
| `/mavros/setpoint_raw/local` | mavros_msgs/PositionTarget | **fcu_command_node only** | 10 Hz, only in GUIDED, airborne, not overridden | RELIABLE / VOLATILE / KEEP_LAST / 1 | Reliable publisher matches any subscriber. |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | every node | 1 Hz | default | Standard tooling. |
| `/tf_static` | tf2_msgs/TFMessage | static publishers | latched | static | Camera mount. |

Phase 2 integration test T-QOS checks every pair with `ros2 topic info -v`.

---

## 5. Service list

| Service | Type | Server | Callers | Purpose |
|---|---|---|---|---|
| `/drone/safety/preflight_check` | std_srvs/Trigger | safety_supervisor | operator, mission | Runs pre-flight validation now; `message` lists every failing check. Read-only. |
| `/drone/safety/reset` | std_srvs/Trigger | safety_supervisor | operator only | Clears latched faults and OVERRIDE, only if all checks currently pass **and the vehicle is disarmed on the ground**. |
| `/drone/llm/submit_command` | drone_interfaces/SubmitCommand | intent_bridge | operator | Text → parsed intent; executes only with `execute: true` and validation passing. |
| `/mavros/set_mode` | mavros_msgs/SetMode | mavros | **fcu_command_node only** | GUIDED before arming; LAND for landing. **[AIRCRAFT]** |
| `/mavros/cmd/arming` | mavros_msgs/CommandBool | mavros | **fcu_command_node only** | Arm before takeoff; disarm fallback only when FCU says landed. Never forced. **[AIRCRAFT]** |
| `/mavros/cmd/takeoff` | mavros_msgs/CommandTOL | mavros | **fcu_command_node only** | MAV_CMD_NAV_TAKEOFF in GUIDED. **[AIRCRAFT]** |
| `/mavros/set_message_interval` | mavros_msgs/MessageInterval | mavros | fcu_state_node | Request stream rates. |
| EKF origin setter (`/mavros/global_position/set_gp_origin`) | geographic_msgs/GeoPointStamped (topic) | mavros | fcu_command_node (needed: P4 confirmed in Phase 8) | Set EKF origin for GPS-denied GUIDED. |

Not implemented now: **position setpoints** (flow-only EKF position drifts; search/track need only yaw). **Forced disarm** (magic value 21196) is never sent by the Pi: disarming in the air is a crash. The pilot's kill switch covers emergencies.

---

## 6. Action list

| Action | Type | Server | Client(s) | Notes |
|---|---|---|---|---|
| `/drone/mission/search_and_track` | SearchAndTrack | mission_manager | intent_bridge, CLI | Whole mission, optionally including autonomous takeoff and landing. |
| `/drone/flight/operation` | FlightOperation | **safety_supervisor** (public gate) | mission_manager, operator CLI | TAKEOFF (pre-flight → GUIDED → arm → takeoff → verify) or LAND (LAND → verify → disarm confirmed). |
| `/drone/fcu/execute_operation` | FlightOperation | **fcu_command_node** (internal) | **safety_supervisor only** (source-grep test) | Executes an approved operation against MAVROS with verification. Re-checks `SafetyStatus` freshness itself. |

`SearchAndTrack` goal is **rejected** unless: no mission active; `target_class` allowed (now only `person`); safety not ABORT/OVERRIDE and not latched; and either
- `autonomous_takeoff: true` and the vehicle is **disarmed and on the ground** with pre-flight passing, or
- `autonomous_takeoff: false` and the vehicle is **armed, in GUIDED, airborne**, height in band (for re-tasking in the air, and for early tests).

Cancelling a running mission: if airborne → mission requests LAND (default `on_cancel: land`; `hover` is a testing option) → result CANCELLED once landed.

Cancelling a `FlightOperation`: a TAKEOFF in progress turns into LAND. Cancelling a LAND is **refused** from ROS; only the pilot can abort a landing (RC mode switch).

---

## 7. Custom interface list

Only what no standard message covers. Enumerations are `uint8` constants.

**msg/VehicleState.msg**
```text
std_msgs/Header header             # stamp = when fcu_state_node built it (ROS clock)
bool    connected
float32 heartbeat_age_s
bool    armed
bool    guided
string  mode                       # ArduPilot mode name from MAVROS
uint8   system_status              # MAV_STATE
uint8   landed_state               # MAV_LANDED_STATE: 0 undefined,1 on ground,2 in air,3 takeoff,4 landing
bool    fcu_prearm_ok              # SYS_STATUS MAV_SYS_STATUS_PREARM_CHECK health bit [VERIFY]
string  last_prearm_text           # latest "PreArm: ..." STATUSTEXT, empty if none recent
bool    rc_present                 # RC input seen recently
float32 battery_voltage_v
float32 battery_current_a          # NaN if not reported
float32 battery_remaining          # 0..1, NaN if not reported
float32 roll_rad                   # REP-103 conventions
float32 pitch_rad
float32 yaw_rad                    # ENU: 0 = east, CCW positive
geometry_msgs/Vector3 velocity_body_flu
bool    velocity_valid
bool    local_position_valid
geometry_msgs/Point local_position_enu    # relative, drifts; informational
float32 height_above_ground_m      # MTF-01 rangefinder; NaN if invalid
bool    height_valid
uint16  ekf_flags                  # raw EKF_STATUS_REPORT flags
bool    ekf_attitude_ok
bool    ekf_velocity_horiz_ok
bool    ekf_pos_horiz_rel_ok       # or pred_pos_horiz_rel while on ground
bool    ekf_const_pos_mode         # EKF has no horizontal aiding
float32 ekf_velocity_variance
float32 ekf_pos_horiz_variance
float32 ekf_age_s
```

**msg/FlowStatus.msg**
```text
std_msgs/Header header
bool    source_seen
float32 age_s
bool    fresh
uint8   quality                    # 0..255 as reported by FCU
bool    quality_ok                 # quality >= min for N consecutive samples
bool    quality_applies            # false below flow_min_height (e.g. on the ground)
float32 flow_rate_x_rad_s          # sensor frame, as reported
float32 flow_rate_y_rad_s
float32 ground_distance_m          # NaN if not provided
bool    in_valid_height_band
bool    healthy_for_flight         # fresh && in band && quality_ok && ekf velocity ok
bool    healthy_for_takeoff        # fresh && rangefinder valid && ekf ok (ground rules)
string  reason
```

**msg/TrackedTarget.msg**
```text
std_msgs/Header header             # capture stamp of the image that last saw it
uint8 STATE_NONE=0
uint8 STATE_CANDIDATE=1
uint8 STATE_TRACKING=2
uint8 STATE_LOST=3
uint8 state
uint32 track_id
string class_id
float32 confidence
vision_msgs/BoundingBox2D bbox     # pixels in ORIGINAL image resolution
uint32 image_width
uint32 image_height
float32 error_x_norm               # (cx - W/2)/(W/2): -1 left .. +1 right
float32 error_y_norm               # (cy - H/2)/(H/2): -1 top .. +1 bottom
float32 bbox_height_norm           # size cue only, NOT a distance
float32 time_since_seen_s
uint32 frames_seen
int8   last_seen_side              # -1 left, 0 centre, +1 right
```

**msg/ControlCommand.msg**
```text
std_msgs/Header header
uint8 MODE_HOLD=0                  # zero velocity, zero yaw rate
uint8 MODE_BODY_VELOCITY=1         # vx, vy, vz, yaw_rate in body FLU
uint8 mode
string source                      # gate allow-list
uint32 seq                         # must increase
float32 vx_mps                     # +forward
float32 vy_mps                     # +left
float32 vz_mps                     # +up
float32 yaw_rate_rad_s             # +CCW from above (turn LEFT)
float32 max_age_s
```

**msg/SafetyStatus.msg**
```text
std_msgs/Header header
uint8 LEVEL_OK=0
uint8 LEVEL_DEGRADED=1             # warnings; motion allowed within limits
uint8 LEVEL_HOLD=2                 # motion rejected; zero-velocity hold
uint8 LEVEL_ABORT=3                # hold + mission aborted + LAND requested if airborne
uint8 LEVEL_OVERRIDE=4             # pilot/FCU took authority; Pi sends nothing
uint8 level
bool  latched
bool  motion_allowed
bool  preflight_ok                 # all takeoff conditions currently satisfied
string[] preflight_failures        # e.g. ["BATTERY_LOW", "FCU_PREARM: Compass not calibrated"]
string[] active_faults
string[] warnings
uint32 commands_rejected
uint32 commands_clamped
```

**msg/MissionStatus.msg**
```text
std_msgs/Header header
uint8 IDLE=0
uint8 PREFLIGHT=1
uint8 TAKING_OFF=2
uint8 SEARCHING=3
uint8 PERSON_DETECTED=4
uint8 TARGET_SELECTED=5
uint8 TRACKING=6
uint8 TARGET_LOST=7
uint8 REACQUIRING=8
uint8 RETURNING=9
uint8 LANDING=10
uint8 MISSION_COMPLETE=11
uint8 MISSION_CANCELLED=12
uint8 SAFETY_STOP=13
uint8 MANUAL_OVERRIDE=14
uint8 ERROR=15
uint8 state
string state_name
string target_class
uint32 target_track_id
float32 elapsed_s
float32 search_yaw_accumulated_deg
uint8  reacquire_attempts
string detail
```

**srv/SubmitCommand.srv**
```text
string text
bool   execute                     # false = parse and show only
bool   use_llm                     # false = keyword parser only
---
bool   understood
string intent_json
bool   executed
string message
```

**action/SearchAndTrack.action**
```text
# Goal
string  target_class               # now only "person"
bool    autonomous_takeoff         # true: start disarmed on ground; false: start airborne in GUIDED
float32 takeoff_altitude_m         # 0 = config default; clamped to safety max
float32 search_timeout_s           # 0 = default; clamped
float32 track_duration_s           # 0 = until cancelled; clamped to max
bool    land_on_finish             # true: land + disarm at end
---
# Result
uint8 OUTCOME_COMPLETED=0
uint8 OUTCOME_CANCELLED=1
uint8 OUTCOME_TARGET_NOT_FOUND=2
uint8 OUTCOME_TARGET_LOST=3
uint8 OUTCOME_SAFETY_STOP=4
uint8 OUTCOME_MANUAL_OVERRIDE=5
uint8 OUTCOME_PREFLIGHT_FAILED=6
uint8 OUTCOME_TAKEOFF_FAILED=7
uint8 OUTCOME_ERROR=8
uint8 outcome
bool   landed_and_disarmed         # final FCU-confirmed state
string message
float32 duration_s
---
# Feedback
drone_interfaces/MissionStatus status
```

**action/FlightOperation.action**
```text
# Goal
uint8 OP_TAKEOFF=0                 # preflight -> GUIDED -> arm -> takeoff -> verify stable hover
uint8 OP_LAND=1                    # LAND mode -> descend -> touchdown -> FCU disarm confirmed
uint8 op
float32 altitude_m                 # TAKEOFF only; clamped by safety
string source                      # allow-list: "mission_manager", "operator"
string reason
---
# Result
uint8 STAGE_NONE=0
uint8 STAGE_PREFLIGHT=1
uint8 STAGE_SET_GUIDED=2
uint8 STAGE_ARM=3
uint8 STAGE_TAKEOFF_CMD=4
uint8 STAGE_CLIMB=5
uint8 STAGE_STABILIZE=6
uint8 STAGE_SET_LAND=7
uint8 STAGE_DESCEND=8
uint8 STAGE_TOUCHDOWN=9
uint8 STAGE_DISARM=10
bool    success
uint8   failed_stage               # STAGE_NONE on success
string  message                    # includes FCU STATUSTEXT reason when available
bool    armed_at_end
uint8   landed_state_at_end
float32 height_at_end_m
---
# Feedback
uint8   stage
float32 height_m
float32 elapsed_in_stage_s
string  detail
```

Standard messages reused: `vision_msgs/Detection2DArray`, `sensor_msgs/Image`/`CameraInfo`/`BatteryState`/`Range`, `geometry_msgs/*`, `diagnostic_msgs/DiagnosticArray`, `std_srvs/Trigger`.

`drone_interfaces` build config (produced and verified in Phase 2):
- `package.xml`: `ament_cmake` + `rosidl_default_generators` buildtool deps; `<depend>` std_msgs, geometry_msgs, vision_msgs; `<exec_depend>rosidl_default_runtime</exec_depend>`; `<member_of_group>rosidl_interface_packages</member_of_group>`.
- `CMakeLists.txt`: `find_package` for each; one `rosidl_generate_interfaces(${PROJECT_NAME} <9 files> DEPENDENCIES std_msgs geometry_msgs vision_msgs)`; `ament_export_dependencies(rosidl_default_runtime)`; `ament_package()`.
- Verify: `ros2 interface list | grep drone_interfaces` shows all 9; `ros2 interface show drone_interfaces/action/FlightOperation`; `python3 -c "from drone_interfaces.action import FlightOperation"`.

---

## 8. Dependency list

### 8.1 apt

| Package | Why |
|---|---|
| `ros-jazzy-ros-base` (installed) | core |
| `ros-jazzy-mavros`, `ros-jazzy-mavros-extras`, `ros-jazzy-mavros-msgs` | MAVLink bridge |
| GeographicLib datasets (MAVROS script) | MAVROS aborts without them |
| `ros-jazzy-v4l2-camera` | USB camera driver |
| `v4l-utils` | inspect camera formats (`v4l2-ctl`) |
| `ros-jazzy-vision-msgs` | detections |
| `ros-jazzy-cv-bridge`, `python3-opencv`, `python3-numpy` | image → numpy |
| `ros-jazzy-image-transport`, `ros-jazzy-image-transport-plugins` | compressed debug images |
| `ros-jazzy-diagnostic-updater`, `ros-jazzy-diagnostic-aggregator` | diagnostics |
| `ros-jazzy-tf2-ros` | static camera transform |
| `ros-jazzy-launch-testing`, `ros-jazzy-launch-testing-ros` | integration tests |
| `python3-colcon-common-extensions`, `python3-rosdep`, `python3-pytest`, `python3-venv` | build + test |

### 8.2 pip in one venv (`~/drone_venv`, `--system-site-packages`)

| Package | Why | Note |
|---|---|---|
| `numpy==1.26.4` | pin to Ubuntu's version | numpy 2.x breaks apt-built cv_bridge |
| `onnxruntime` | YOLO on Pi 5 CPU | no PyTorch on the Pi |
| `pymavlink` | decode raw EKF / flow / SYS_STATUS | read-only |
| `anthropic` | LLM (optional) | key via env `ANTHROPIC_API_KEY`, never in YAML/git |

### 8.3 On a development PC (not the Pi)

- `ultralytics` to export the model once: `yolo export model=yolo11n.pt format=onnx imgsz=320`. Copy the `.onnx` to the Pi. Ultralytics weights are AGPL-3.0.
- **ArduPilot SITL (Copter 4.6)** + MAVROS + our stack, for the autonomous arm/takeoff/land sequence before real hardware. Ubuntu 24.04 PC or WSL2. SITL can simulate a rangefinder and optical flow.

### 8.4 rosdep

All apt deps are declared in `package.xml` so `rosdep install --from-paths src --ignore-src -r -y` installs them. pip deps live in the venv, not rosdep.

---

## 9. Data flow and mission state machine

### 9.1 Full autonomous mission sequence

```text
"Look for a person and track him"
   │ /drone/llm/submit_command
   ▼
intent_bridge → {"action":"search_and_track","target":"person"} → validated
   │ SearchAndTrack goal (autonomous_takeoff: true, land_on_finish: true)
   ▼
mission_manager
   │ PREFLIGHT  ── /drone/safety/preflight_check must pass
   │ TAKING_OFF ── FlightOperation{TAKEOFF, alt} ──► safety_supervisor (re-validates)
   │                                                 └─► fcu_command_node: GUIDED → arm → takeoff → verify
   │ SEARCHING / TRACKING ── ControlCommand ──► safety gate ──► fcu_command_node ──► /mavros/setpoint_raw/local
   │ RETURNING  ── yaw back to start heading (no position return, see §9.4)
   │ LANDING    ── FlightOperation{LAND} ──► safety_supervisor ──► fcu_command_node: LAND mode → verify → disarm confirmed
   ▼
MISSION_COMPLETE (result: landed_and_disarmed = true)
```

### 9.2 State path (bottom to top)

```text
MTF-01 flow + ToF range ──► Pixhawk EKF3 (fusion HERE)     IMU, baro ──► EKF3
      │ MAVLink: HEARTBEAT, SYS_STATUS, EXTENDED_SYS_STATE, STATUSTEXT, ATTITUDE,
      │          LOCAL_POSITION_NED, BATTERY_STATUS, EKF_STATUS_REPORT, OPTICAL_FLOW,
      │          DISTANCE_SENSOR, RC_CHANNELS
      ▼
mavros_node ──► /mavros/* + /uas1/mavlink_source
      ▼
fcu_state_node ──► /drone/vehicle_state, /drone/flow_status
      ▼
safety_supervisor ──► /drone/safety_status ──► mission_manager, fcu_command_node, intent_bridge
```

### 9.3 Perception path

```text
USB camera ──► v4l2_camera ──► /camera/image_raw (stamp = capture time)
   ▼
yolo_detector: newest frame only → letterbox → ONNX → NMS → rescale to ORIGINAL pixels → class/score filter
   ▼ /drone/perception/detections (same stamp)
target_tracker: associate → tracks → select → loss timer (10 Hz)
   ▼ /drone/perception/target
mission_manager: error_x_norm → yaw-rate command
```

### 9.4 Mission state machine

```text
 IDLE ──goal──► PREFLIGHT ──ok──► TAKING_OFF ──stable hover──► SEARCHING ◄───────────────┐
                   │fail              │fail                       │ person seen N frames   │ reacquire timeout,
                   ▼                  ▼                           ▼                        │ attempts left
            result PREFLIGHT    LANDING if airborne,         PERSON_DETECTED               │
            _FAILED, stays      else confirm disarmed;            │ select (policy)        │
            disarmed            result TAKEOFF_FAILED        TARGET_SELECTED               │
                                                                  ▼                        │
                                                  ┌──────────► TRACKING ── unseen > lost_timeout
                                                  │ seen again    │                  ▼
                                                  └────────── REACQUIRING ◄── TARGET_LOST (hold)
                                                                  │
         search timeout / max yaw / track duration / reacquire exhausted
                                                                  ▼
                                                             RETURNING (yaw to start heading)
                                                                  ▼
                                                              LANDING ──FCU landed + disarmed──► MISSION_COMPLETE

 From ANY airborne state:
   cancel                        → LANDING → MISSION_CANCELLED
   safety ABORT                  → safety itself requests LAND → SAFETY_STOP
   safety HOLD                   → HOLD streamed; mission pauses; HOLD longer than hold_to_land_s → LANDING
   mode changed by pilot/FCU     → MANUAL_OVERRIDE (terminal; the Pi sends nothing more)
   exception in mission logic    → ERROR (safety sees mission commands stop → HOLD → LAND)
 Every terminal state reached in the air leaves the vehicle either landing in FCU LAND mode or under pilot control.
```

State commands (current scope: all translational velocities 0; takeoff and landing are FCU-controlled):

| State | What the Pi sends | Exit |
|---|---|---|
| PREFLIGHT | nothing | pre-flight OK / fail |
| TAKING_OFF | FlightOperation TAKEOFF (FCU climbs) | op success / fail |
| SEARCHING | yaw at `search_yaw_rate`, bounded by `search_max_total_yaw_deg`, `search_timeout_s` | candidate confirmed |
| PERSON_DETECTED / TARGET_SELECTED | HOLD (one tick each; observable and testable) | → TRACKING |
| TRACKING | yaw_rate = −Kp·error_x_norm, deadband, clamp, yaw-accel limit | lost / duration / abort |
| TARGET_LOST | HOLD for `lost_hold_s` | → REACQUIRING |
| REACQUIRING | slow yaw toward `last_seen_side`, ≤ `reacquire_max_yaw_deg`, ≤ `reacquire_timeout_s` | seen / timeout |
| RETURNING | yaw to heading recorded at takeoff (±5°), with timeout | → LANDING |
| LANDING | FlightOperation LAND (FCU descends) | FCU disarmed |

**"Return/positioning as appropriate": the honest limit.** With a flow-only EKF, horizontal position is dead-reckoned and drifts. Flying "back to the launch point" by position would look precise but isn't, indoors near walls. So the current design **lands in place** after restoring the start heading: `return_mode: land_in_place | yaw_to_start_then_land` (default the latter). True return-to-launch becomes possible with VIO/SLAM (future). ArduPilot RTL mode is **not** used indoors.

**Tracking honesty.** A 2D box gives bearing, not distance. Tracking = yaw to keep the person centred while ArduPilot holds position on flow. Forward/back approach stays disabled on the aircraft (`enable_approach: false`, `allow_approach: false`, gate horizontal limit 0). Added 2026-10-08, simulator only: an optional forward-only approach that keeps an *estimated* distance (box height with an assumed person height, or head position when the feet are cut off), max 0.3 m/s, only while centred, 4 m travel cap; see `drone_ws/docs/APPROACH.md`.

Target selection: `largest_bbox` by default, tie → closest to centre; sticky to the selected `track_id` through short occlusions.

---

## 10. TF / frame strategy

### 10.1 Conventions

| Thing | ROS (REP-103) | ArduPilot / MAVLink |
|---|---|---|
| World | ENU: x East, y North, z Up | NED: x North, y East, z Down |
| Body | FLU (`base_link`) | FRD |
| Yaw angle | 0 = East, CCW positive | 0 = North, CW positive |
| Yaw rate | + = turn LEFT | + = turn RIGHT |
| Camera optical | z forward, x right, y down (`camera_optical_frame`) | n/a |
| Image | u right, v down, origin top-left, px | n/a |
| Takeoff altitude | — | metres **above home (arming point)**, positive up, `MAV_CMD_NAV_TAKEOFF` param7 |
| Rangefinder | `sensor_msgs/Range.range`, metres along sensor axis | DISTANCE_SENSOR in cm; MAVROS converts |

### 10.2 Where conversion happens

- MAVROS converts ENU↔NED and FLU↔FRD on its standard topics (`local_position/*`, `imu/data`, `setpoint_raw/local` with `FRAME_BODY_NED`). Our nodes see and send only ROS conventions.
- Our code never does ENU↔NED math, except for raw messages we decode (EKF flags: no frame; OPTICAL_FLOW rates: sensor frame). Those are used for **health only**.
- All frame logic sits in `drone_fcu/core/frames.py` with unit tests, including:

```text
person RIGHT of centre   → error_x_norm > 0
must turn RIGHT          → ROS yaw_rate < 0 (FLU)
MAVROS → ArduPilot       → NED yaw_rate > 0 (clockwise)
```

This is tested by unit test, props-off via `target_local`, in SITL (watch heading), then hand-held/tethered before free flight.

- **Takeoff altitude vs rangefinder height.** ArduPilot's takeoff target is relative to home altitude from its altitude source (`EK3_SRC1_POSZ`: baro or rangefinder). Our verification uses the MTF-01 rangefinder height, which is height above whatever floor is under the drone. On a flat indoor floor they agree within tolerance; over furniture they don't. The tolerance and the height band account for this. **[VERIFY]** your EKF altitude source (Appendix D).
- **[VERIFY] body-frame velocity semantics** of `FRAME_BODY_NED` on ArduCopter 4.6 (velocity relative to current heading). Only matters once approach is enabled. SITL 4.6.3 + Gazebo (2026-10-08): +vx flew the drone towards the person it faced, so forward = current heading there; still to confirm on the aircraft.

### 10.3 TF tree

```text
base_link ──(static)──► camera_link ──(static, optical rotation)──► camera_optical_frame
```

No `map`/`odom`. MAVROS `local_position.tf.send: false`. MAVROS still writes `frame_id: map` in some headers; we never look those up in TF. Camera offsets and tilt come from `drone.yaml` **[TUNE: measure]**.

### 10.4 Timestamps

- Timeouts use the steady clock (Pi 5 has no RTC battery by default; NTP can jump wall time).
- Detections/targets keep the image capture stamp; `now − stamp` = perception latency, published on `/diagnostics`.
- Each flight-operation stage timeout starts when that stage starts, on the steady clock.

---

## 11. Safety architecture

### 11.1 Layers, outermost first

| # | Layer | Owner | Triggers | Action |
|---|---|---|---|---|
| L0 | **Pilot RC** | you | anything | Switch mode (LOITER / ALT_HOLD / LAND) → Pi detects and goes silent. Motor kill / disarm switch for emergencies. Tested before every session. |
| L1 | ArduPilot failsafes | Pixhawk | battery, RC loss, EKF failure, crash check | FCU's own action (`BATT_FS_*`, `FS_THR_*`, `FS_EKF_ACTION`). A failsafe mode change is treated like an override (§11.6). |
| L2 | ArduPilot GUIDED timeout | Pixhawk | no setpoint for `GUID_TIMEOUT` (~3 s) **[VERIFY 4.6.3 value]** | vehicle stops and holds. Covers "Pi crashed while in GUIDED". |
| L3 | fcu_command_node | Pi | validated command older than `cmd_timeout_s` (0.3 s), or SafetyStatus stale | stream HOLD; SafetyStatus stale > `safety_stale_land_s` while airborne → LAND |
| L4 | safety_supervisor | Pi | checks below | reject / clamp / HOLD / ABORT (→ LAND) / OVERRIDE |
| L5 | mission_manager | Pi | target lost, timeouts, max yaw | HOLD / terminal state / LAND |

Any one of L0–L3 alone stops motion. The LLM sits above L5.

### 11.2 Continuous health checks (in flight)

| Check | Source | Default | Fault → level |
|---|---|---|---|
| FCU connected | vehicle_state.connected | true | HOLD, latched (Pi can't command anyway; L2 takes over) |
| Telemetry fresh | vehicle_state age | < 1.0 s **[TUNE]** | HOLD, latched |
| Heartbeat fresh | heartbeat_age_s | < 2.0 s | HOLD, latched |
| Armed | armed | true while airborne | unexpected disarm → OVERRIDE |
| Mode | mode == mode the Pi requested | GUIDED in mission | mismatch → OVERRIDE (§11.6) |
| Battery | voltage / remaining | warn / land thresholds **[TUNE per pack]** | DEGRADED → ABORT (land) |
| EKF | velocity_horiz_ok, pos_horiz_rel_ok, attitude_ok, not const_pos_mode, age | all ok, age < 2 s | ABORT, latched |
| Optical flow | flow_status.healthy_for_flight | quality ≥ min for 5 samples **[TUNE on your floor]** | short dips DEGRADED; bad > `flow_bad_timeout_s` (1.0 s) → ABORT |
| Height band | rangefinder | `[min_height, max_height]` **[TUNE]** | HOLD; above `hard_ceiling_m` → ABORT |
| Tilt | roll/pitch | < 25° **[TUNE]** | ABORT, latched |
| Perception alive | target topic age | < 0.5 s while mission needs it | HOLD (mission treats it as target lost) |
| Mission alive | command age | < 0.3 s while mission active | HOLD → LAND after `hold_to_land_s` |
| Pi thermal | thermal_zone0 | warn 75 °C, fault 82 °C **[TUNE]** | DEGRADED → ABORT |

ABORT = HOLD + mission aborted + **LAND requested by safety itself** if airborne and armed (`allow_auto_land: true`). v1 left this off because the pilot took off; with autonomous flight, landing on ABORT is the design.

### 11.3 Pre-flight validation (before any arm)

Run by `safety_supervisor` continuously (→ `preflight_ok`) and again, fresh, when a TAKEOFF operation arrives. **All** must pass or the Pi does not arm.

| # | Check | Pass condition |
|---|---|---|
| PF1 | MAVLink link | connected, heartbeat age < 2 s, telemetry age < 1 s |
| PF2 | Vehicle state valid | all VehicleState fields populated within the last 1 s; system_status STANDBY |
| PF3 | Disarmed and on ground | `armed == false`, `landed_state == ON_GROUND` (or undefined + height < `ground_height_max_m`) |
| PF4 | FCU's own pre-arm checks | `fcu_prearm_ok == true` and no "PreArm:" STATUSTEXT in the last 5 s |
| PF5 | Battery | voltage ≥ `takeoff_min_voltage` **[TUNE per pack]**, remaining ≥ `takeoff_min_remaining` if reported |
| PF6 | Estimator | EKF attitude ok, velocity ok, `pos_horiz_rel` or `pred_pos_horiz_rel` ok, not const-pos-only (**[VERIFY]** which flags ArduPilot sets on the ground with flow), EKF age < 2 s |
| PF7 | Flow / localization | `healthy_for_takeoff`: flow messages fresh, rangefinder valid and reading ground distance < `ground_height_max_m`. Flow *quality* is not judged on the ground (MTF-01 is below its minimum working height); it is judged after the climb, in STABILIZE. |
| PF8 | Attitude | level: roll, pitch < 5° |
| PF9 | Mode | current mode is an allowed pre-takeoff mode (`STABILIZE`, `ALT_HOLD`, `LOITER`, `GUIDED`), i.e. not LAND/RTL/a failsafe mode. The Pi switches to GUIDED as step 1. |
| PF10 | No active safety fault | level OK (or DEGRADED by warnings only), not latched, not OVERRIDE |
| PF11 | Request freshness and source | op goal stamp < 1 s old; `source` in allow-list |
| PF12 | Altitude limits | `min_takeoff_alt ≤ altitude ≤ max_takeoff_alt`, `altitude ≤ hard_ceiling_m − ceiling_margin_m`, and within the MTF-01 valid range **[VERIFY datasheet]** |
| PF13 | Output permission | `output_enabled && allow_arming` on `fcu_command_node` (bench defaults are false → op rejected with a clear message) |
| PF14 | RC present | RC link seen within 1 s, so pilot override is available. Configurable, default **required**. |
| PF15 | Perception ready (mission only) | detector and tracker publishing within timeout, so we don't take off blind |

### 11.4 Command gate (velocity / yaw stream)

In order: (1) level OK/DEGRADED, not latched, not OVERRIDE → else HOLD. (2) source in allow-list. (3) seq increasing. (4) age ≤ min(msg.max_age_s, 0.3 s) → else HOLD. (5) all numbers finite. (6) known mode. (7) mode GUIDED, landed_state IN_AIR, no flight operation in progress → else HOLD. (8) clamp |yaw_rate| ≤ max, |vx|,|vy| ≤ 0.0 now, |vz| ≤ 0.0 now. (9) yaw-acceleration limit. (10) publish, stamped now. Limits cannot be changed at runtime while armed.

### 11.5 Flight operations: what is sent, what confirms it, what happens on failure

All steps run inside `fcu_command_node` after `safety_supervisor` approved the operation. Timeouts are starting values **[TUNE]**. "ACK" = the MAVROS service response: `CommandBool` / `CommandTOL` return `success` and `result` (MAV_RESULT, 0 = ACCEPTED); `SetMode` returns `mode_sent` (sent, not confirmed), which is why every step also waits for an observed state.

**TAKEOFF** (goal altitude `h`, already clamped by PF12)

| Step | Pre-condition (re-checked right before sending) | Command sent | Expected ACK | Success = observed state | Timeout | On failure |
|---|---|---|---|---|---|---|
| T1 Pre-flight | — | none | — | PF1–PF15 all pass | — | reject op; nothing sent; result lists failures |
| T2 GUIDED | PF still ok; disarmed | `/mavros/set_mode` `{custom_mode: "GUIDED"}` | `mode_sent: true` | `/mavros/state.mode == "GUIDED"` | 3 s | abort op; vehicle stays disarmed; nothing else sent |
| T3 Arm | mode GUIDED; PF still ok | `/mavros/cmd/arming` `{value: true}` | `success: true`, `result: 0` | `/mavros/state.armed == true` | 3 s | record the STATUSTEXT reason; **no retry loop**; if armed shows up late → disarm request (allowed: on ground); result ARM failed |
| T4 Takeoff command | armed; GUIDED; on ground; **sent within 2 s of confirmed arm** (ArduPilot auto-disarms after `DISARM_DELAY`, default 10 s) | `/mavros/cmd/takeoff` `{altitude: h, latitude: 0, longitude: 0, min_pitch: 0, yaw: 0}` → MAV_CMD_NAV_TAKEOFF, param7 = h metres above home | `success: true`, `result: 0` | `landed_state` → TAKEOFF / IN_AIR and height increasing | 3 s to start climbing | rejected → disarm request (still on ground), result TAKEOFF_CMD failed. Accepted but no climb → LAND procedure |
| T5 Climb | GUIDED; armed; health ok | none (FCU climbs) | — | rangefinder height ≥ h − `alt_tol_m` (0.15) | `h / min_climb_rate + 5 s` | LAND procedure; result CLIMB failed |
| T6 Stabilize | — | Pi starts streaming HOLD setpoints (zero velocity) | — | for `stable_window_s` (2 s): \|height − h\| < tol, \|vz\| < 0.15 m/s, horizontal speed < 0.2 m/s, `healthy_for_flight` (flow quality judged now), EKF ok | `stable_timeout_s` (8 s) | LAND procedure; result STABILIZE failed |
| Done | — | continue HOLD stream | — | op success; mission proceeds | — | — |

Abort paths during TAKEOFF: safety ABORT → LAND procedure. Mode changed by pilot/FCU → OVERRIDE, op ends, Pi silent. SafetyStatus stale > 0.5 s → LAND procedure. Goal cancelled → LAND procedure.

**LAND**

Safety-gate rule: LAND is **never blocked by health faults**, because landing is the safe direction. It is refused only if: disarmed (nothing to do), OVERRIDE active (the pilot has authority, don't fight them), or a LAND is already running.

| Step | Pre-condition | Command sent | Expected ACK | Success = observed state | Timeout | On failure |
|---|---|---|---|---|---|---|
| L1 LAND mode | armed; not OVERRIDE | stop setpoint stream; `/mavros/set_mode` `{custom_mode: "LAND"}` | `mode_sent: true` | `/mavros/state.mode == "LAND"` | 2 s | retry once; if still not LAND → ABORT stays latched, alert "LAND NOT ACCEPTED: take manual control", keep streaming HOLD in GUIDED so it at least holds position |
| L2 Descend | mode LAND | none (FCU descends at `LAND_SPEED`) | — | height decreasing; `landed_state` LANDING | `h / min_descent_rate + 10 s` | keep LAND (never disarm in the air); alert; keep waiting |
| L3 Touchdown | — | none | — | `landed_state == ON_GROUND` and height < `ground_height_max_m` | within the L2 timeout | as L2 |
| L4 Disarm | on ground | none: ArduPilot's LAND mode disarms by itself after touchdown | — | `armed == false` | `disarm_wait_s` (5 s) after touchdown | if still armed **and** `landed_state == ON_GROUND` **and** height < `ground_height_max_m` → `/mavros/cmd/arming {value: false}` (normal, not forced; ArduPilot refuses if it thinks it is flying). If refused → alert and stay in LAND; the operator disarms. |
| Done | — | — | — | `armed == false` confirmed | — | result `landed_and_disarmed = true` |

If telemetry is lost during LAND, the Pi can no longer verify anything, but LAND is an FCU mode and continues on the Pixhawk. The op reports "LAND in progress, telemetry lost" and never claims success.

### 11.6 RC / FCU override detection

```text
fcu_command_node keeps requested_mode = the last mode IT asked for (GUIDED or LAND).

On every /mavros/state:
  if armed and mode != requested_mode and no set_mode of ours is pending:
        → OVERRIDE
  if armed → disarmed while landed_state != ON_GROUND and not during our step L4:
        → OVERRIDE (pilot kill / FCU disarm)

On OVERRIDE:
  fcu_command_node : stop the setpoint stream immediately, abort any flight op, send NOTHING
                     (no set_mode, no arm, no takeoff, no land)
  safety_supervisor: level = OVERRIDE, latched; publishes the reason (new mode, STATUSTEXT)
  mission_manager  : → MANUAL_OVERRIDE (terminal), action result MANUAL_OVERRIDE
  The Pi never switches back to GUIDED by itself. New autonomy requires: disarmed on the
  ground, /drone/safety/reset, then a new mission goal.
```

This covers a pilot mode switch, a pilot disarm, and ArduPilot failsafes (battery → LAND, EKF failsafe, RC failsafe). `/mavros/rc/in` is logged as extra evidence, but the **mode** is the source of truth, because it is what actually decides who flies. In GUIDED, ArduPilot does not hand control to the sticks without a mode change, so a mode switch on the RC is the override gesture. **You need a switch position that exits GUIDED (e.g. LOITER) and a kill switch; check both on the bench before every session.**

You confirmed LOITER + GUIDED on the mode switch and that **no kill switch exists yet**. Recommendation before Phase 7 (first powered test): assign a spare RC switch to ArduPilot's motor emergency stop (`RCx_OPTION = 31`, Motor Emergency Stop) or arm/emergency-stop (`RCx_OPTION = 165`). It is an FCU parameter change, so it is your decision and your change; Phase 7's pre-test checklist will ask whether it exists. A mode switch cannot stop motors in a crash or a runaway; a kill switch can.

### 11.7 Can a stale or unauthorized command move the drone?

- Velocity: VOLATILE topics + gate age check + fcu_command age check + ArduPilot GUIDED timeout.
- Arm/takeoff: only via an approved FlightOperation, pre-flight re-checked at T2/T3/T4, `allow_arming` param, request freshness (PF11).
- After an override: nothing is sent until reset on the ground.
- ROS 2 has no caller authentication by default, so any node on the same DDS domain could call the public actions. Mitigation now: a unique `ROS_DOMAIN_ID`, network isolation of the Pi, allow-listed `source`. SROS2 (DDS security) is a later hardening step (P18).

### 11.8 Bench mode

- `output_enabled: false` → setpoints and flight ops are simulated: each step logs "WOULD SEND ..." and the op fails at PF13 with a clear message. The full mission can still run ROS-only.
- `enable_fcu_output:=true` allows setpoints; `allow_arming:=true` allows arm/takeoff. Both **[AIRCRAFT]**, both print a banner.
- A `sitl:=true` launch variant points MAVROS at SITL over UDP; every real-flight feature is exercised there first.

---

## 12. Current vs future scope

### Current (implemented now)

ROS 2 Jazzy workspace · MAVROS on TELEM1/SERIAL1 · telemetry and vehicle state · EKF health · **MTF-01 optical-flow and rangefinder monitoring (fusion stays in ArduPilot EKF3)** · battery · pre-flight validation · **autonomous GUIDED → arm → takeoff → verified hover** · **autonomous LAND → FCU disarm confirmed** · RC/FCU override detection · control abstraction (HOLD, body velocity + yaw rate; behaviours use yaw only) · safety supervisor, command gate, flight-op gate · USB camera · **YOLO person detection on the Pi 5** · tracking, selection, loss, reacquisition · deterministic search · full SearchAndTrack mission · restore start heading then land in place · LLM intent bridge with keyword fallback · diagnostics, logging, rosbag · unit, integration and SITL tests · staged launch · YAML config.

### Current but off by default

| Feature | Why off | Enabled in |
|---|---|---|
| `output_enabled` (setpoints) | bench safety | Phase 7, props off |
| `allow_arming` (arm + takeoff) | bench safety | Phase 8, after SITL |
| Forward/back approach | no real range | later, separate confirmation |

### Future (NOT implemented, no placeholders)

LiDAR · VIO / VINS-Mono · SLAM · 2D mapping · occupancy grids · map-based navigation · true return-to-launch · advanced localization · obstacle avoidance.

Extension points: no `map`/`odom` today, so localization can own them later. VIO would feed ArduPilot through MAVROS `vision_pose`/`odometry`, keeping the EKF on the Pixhawk. Obstacle avoidance becomes a new safety check plus possibly a new command mode. `ControlCommand` can gain `MODE_POSITION`; `return_mode` can gain `return_to_launch` once position is trustworthy.

---

## 13. Implementation phases

Hardware phases follow the ladder: **1 code/build → 2 ROS-only → 3 SITL (flight features) → 4 props-off hardware → 5 controlled (tethered / hand-held, pilot on RC) → 6 flight**. I stop at every ◆.

| Phase | What | Key tests | Aircraft? |
|---|---|---|---|
| **1** Architecture | this doc | your review | no |
| ◆ | **you confirm or request changes** | | |
| **2** Workspace + interfaces | 7 packages, build files, 9 interfaces, node skeletons, launch skeleton, venv, clean-build script | clean `colcon build`; `ros2 interface list/show`; `ros2 pkg executables`; `colcon test`; source-grep tests "only fcu_command_node calls MAVROS write APIs" and "only safety_supervisor calls execute_operation" | no |
| ◆ | | | |
| **3** MAVROS link | `fcu.launch.py` (serial device found on your Pi, 115200), `tf.send: false`, plugin allow-list, stream rates, link-load measurement, read Appendix D params | `/mavros/state connected: true`; `ros2 topic hz` per stream; list which flow/EKF/extended_state topics exist with 4.6.3; STATUSTEXT visible | read-only; props off |
| **4** Drone state | `fcu_state_node`, raw decode, pre-arm bit, landed_state | unit tests; unplug telemetry → disconnected within timeout; cause a PreArm failure (e.g. unplug MTF-01) → shows in `last_prearm_text` | read-only |
| **5** Optical flow | FlowStatus with ground and flight rules | cover the sensor; lift by hand → range and band; move by hand → flow sign; EKF flags with flow on your floor | read-only |
| ◆ | | | |
| **6** Safety + control (merged) | `safety_supervisor` (health, pre-flight, gate, override), `fcu_command_node` setpoint path | gate / pre-flight / override unit tests; stale-command and killed-node tests in bench mode | no (bench mode) |
| **7** Setpoints live, props off | `enable_fcu_output:=true`; for this test only you arm on the RC | GUIDED props-off: yaw sign via `target_local`; kill safety → HOLD; kill command node → GUIDED timeout; RC mode switch → OVERRIDE detected | **[AIRCRAFT]** props off |
| ◆ | | | |
| **8** Autonomous takeoff / land | FlightOperation gate + executor | unit tests for every step and failure path. **SITL**: full arm → takeoff → hover → LAND → disarm, plus injected failures (battery low, flow off, pilot mode switch mid-climb, Pi nodes killed mid-climb, takeoff rejected). **Props off**: pre-flight rejections; arm → takeoff → climb timeout → LAND → FCU disarm (the expected props-off result, since it can't climb). **Controlled**: tethered low takeoff/land with you ready on the RC | **[AIRCRAFT]** |
| ◆ | | | |
| **9** Camera | v4l2_camera, format check, camera_info, static TF | `v4l2-ctl --list-formats-ext`; `ros2 topic hz`; encoding; stamp sanity | no |
| **10** YOLO | detector + ONNX model | postprocess unit tests; **measured** fps, latency, CPU, temperature on your Pi 5 | no |
| **11** Tracking | tracker | association, selection, loss, reacquire, error-sign tests; walk-through test | no |
| ◆ | | | |
| **12** Mission | mission_manager, full sequence | state-machine tests (every transition, cancel, timeouts, override, abort); ROS-only run in bench mode; SITL full mission (tracker fed from recorded detections, since SITL has no person); props off; controlled; first free flight low and slow | **[AIRCRAFT]** |
| ◆ | | | |
| **13** LLM bridge | intent_bridge | parser tests incl. hostile inputs ("disable safety", "fly to the ceiling", "arm now"); dry-run; bench; then full | only through the mission |

---

## 14. Exact installation requirements

On the Pi 5. Nothing here changes the Pixhawk.

```bash
# 1. Confirm ROS 2
source /opt/ros/jazzy/setup.bash && echo $ROS_DISTRO          # EXPECTED: jazzy

# 2. apt
sudo apt update
sudo apt install -y \
  ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
  ros-jazzy-v4l2-camera v4l-utils \
  ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins ros-jazzy-diagnostic-updater \
  ros-jazzy-diagnostic-aggregator ros-jazzy-tf2-ros \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ros \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-venv python3-numpy python3-opencv

# 3. GeographicLib datasets (MAVROS will not start without these)
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# 4. Permissions (log out and back in afterwards)
sudo usermod -aG dialout,video $USER

# 5. rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

# 6. venv (outside the workspace)
python3 -m venv --system-site-packages ~/drone_venv
source ~/drone_venv/bin/activate
pip install "numpy==1.26.4" onnxruntime pymavlink anthropic
python3 -c "import cv_bridge, numpy, onnxruntime; print(numpy.__version__)"   # EXPECTED: 1.26.4

# 7. Facts to collect for Phase 3 (read-only)
ls -l /dev/serial* /dev/ttyAMA* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null   # which one is TELEM1?
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext                               # Zebronics formats and resolutions
```

Every terminal, in this order:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_venv/bin/activate
source ~/drone_ws/install/setup.bash
```

Development PC (for SITL, Phase 8): Ubuntu 24.04 (or WSL2) with ROS 2 Jazzy + MAVROS + ArduPilot source built for Copter 4.6. Exact commands come in Phase 8.

Hardware now known:

| Item | Value | Consequence |
|---|---|---|
| FCU | Pixhawk1, ArduCopter V4.6.3 | GUIDED takeoff, LAND mode, EXTENDED_SYS_STATE and SET_MESSAGE_INTERVAL are all 4.x features; exact behaviours still bench-verified |
| Link | TELEM1 = SERIAL1, 115200, MAVLink 2 | the baud is a config parameter, so 921600 later is a one-line change here plus `SERIAL1_BAUD=921` on the FCU (your call, later) |
| Flow + range | MicoAir MTF-01 (flow + ToF) | one module provides both flow and height; if it drops out, **both** go at once → treated as one critical fault. Working range and minimum height from its datasheet **[VERIFY]** |
| Camera | Zebronics USB | `v4l2_camera`; format (YUYV vs MJPEG) checked in Phase 9 |
| Cooling | active cooler | good for sustained YOLO |
| RC | exists | needs a switch position outside GUIDED and a kill switch (please confirm) |

Still unknown, and found in Phase 3 (no need to answer now): the Pi's serial device name for TELEM1.

---

## 15. Potential architectural problems

| # | Problem | Why it matters | What the design does |
|---|---|---|---|
| P1 | **115200 baud is narrow** (~11 KB/s) | streams + command acks + GCS can saturate it; late setpoints and acks | minimal stream set at modest rates; Phase 3 measures load; you plan 921600 later (config param only) |
| P2 | **MTF-01 is flow and range in one** | one failure removes both scale and flow; also has a minimum working height | single critical fault; takeoff judges quality only after the climb; height band from datasheet |
| P3 | **Reflective / textureless floors** | flow quality collapses → drift | quality hysteresis, ABORT → LAND, test on your floor; a textured mat helps |
| P4 | **GPS-denied GUIDED arming/takeoff may need an EKF origin** | ArduPilot may refuse GUIDED arming or takeoff without an origin/position **[VERIFY on 4.6.3]** | SITL + props-off test it; if needed, fcu_command_node sets a fixed dummy origin once at startup (standard for indoor setups), decision recorded then |
| P5 | **MAVROS may lack EKF_STATUS_REPORT / OPTICAL_FLOW topics** | we need flags and quality | raw decode via pymavlink |
| P6 | **Pre-arm bit in SYS_STATUS** | PF4 relies on it **[VERIFY that 4.6.3 sets MAV_SYS_STATUS_PREARM_CHECK]** | fallback: no recent PreArm STATUSTEXT; and ArduPilot itself refuses to arm with a reason, which T3 handles |
| P7 | **YOLO on Pi 5 CPU**: speed, heat, latency unknown | slow perception → oscillating yaw | ONNX at 320 px, newest frame only, latency published, low gains; an accelerator is a later option |
| P8 | **Python venv vs ROS** (PEP 668, numpy 2, new setuptools) | entry points run `/usr/bin/python3` and miss the venv; cv_bridge breaks with numpy 2; colcon fails with setuptools ≥ 80 (`--editable not recognized`) | venv with `--system-site-packages` added via `PYTHONPATH` in `env.sh`; numpy pinned; never pip-upgrade setuptools/pytest (verified while building Phase 2) |
| P9 | **No depth → no distance** | "track him" ≠ "follow at 2 m" | yaw-only tracking; approach disabled on the aircraft. Simulator-only approach uses an estimated distance, capped; optional blind reverse (0.2 m/s, 1.5 m, own gate limit `max_reverse_speed_mps`, 0 on the aircraft) (APPROACH.md) |
| P10 | **Several people / ID switches** | wrong target | sticky target, confirmation frames, conservative reacquire |
| P11 | **Yaw sign errors** | spin away from the target | single conversion module; unit + SITL + props-off + hand-held tests; search max-yaw and time caps |
| P12 | **Your phase order had Control before Safety** | output without the gate | merged: nothing goes live without safety |
| P13 | **FCU can't detect a dead Pi except via GUIDED timeout** | GCS failsafe keys on `SYSID_MYGCS` | L2 covers GUIDED; LAND continues on its own; making the Pi count as GCS is a later decision |
| P14 | **LLM network / hallucination** | can't depend on the cloud; invented actions | keyword fallback, strict schema, dry-run, mission re-validates; the LLM can't request arm/takeoff directly, only a mission |
| P15 | **Wi-Fi + DDS** | image traffic, latency | compressed debug only, off by default; own domain id |
| P16 | **Node respawn** | restarting safety/command mid-flight is unpredictable | no respawn for fcu/safety/mission (they fail to HOLD → LAND); respawn only camera and detector |
| P17 | **Power** | Pi brownout mid-flight | proper 5 V / 5 A supply; LAND and the GUIDED timeout keep the aircraft safe if the Pi resets; watch `dmesg` |
| P18 | **No authentication on ROS 2 actions** | any node on the domain can request TAKEOFF | domain isolation + source allow-list + pre-flight now; SROS2 later |
| P19 | **Arm → takeoff window** | ArduPilot disarms if no takeoff within `DISARM_DELAY` (10 s) | T4 sent within 2 s of confirmed arm; if missed, abort and confirm disarm |
| P20 | **Ground effect, low altitude, small rooms** | flow is poor near the floor | min takeoff altitude above flow min height; conservative `max_takeoff_alt` vs ceiling; ArduPilot's LAND controller handles touchdown |
| P21 | **Takeoff altitude reference vs rangefinder** | home-relative altitude ≠ height over furniture | flat-floor assumption stated; tolerance + height band; check `EK3_SRC1_POSZ` |
| P22 | **Override vs our own mode changes** | race between our set_mode and the state update could look like an override | `requested_mode` set before sending, plus a "pending" window until confirmed or timed out |
| P23 | **SITL has no real person** | mission test in SITL lacks vision | feed the tracker from recorded detections in SITL; vision verified separately with the real camera |

### Contradiction check

- Pixhawk owns EKF, stabilization, arming checks, the LAND controller and disarm-on-land ✔. The Pi *requests* and *verifies*; it never replaces them.
- Autonomous arm/takeoff/land ✔ (§11.5); RC is the override, not the normal path ✔ (§11.6).
- Landing goes through safety ✔ (FlightOperation → safety_supervisor → fcu_command_node). Safety never blocks LAND on health faults, on purpose, and says so.
- Disarm follows the FCU's landed/armed state ✔ (L4).
- Optical flow current ✔; YOLO and tracking current ✔; no map/odom ✔; the LLM only reaches the mission action ✔.
- One door to MAVROS ✔; one caller of the internal op action ✔. Both are enforced by tests.
- "Return" is honest: restore heading + land in place; true return is future ✔.
- Position commands still not implemented, with the reason ✔.

---

## Appendix A — Flask server

Drop it. Mission Planner gets telemetry via MAVROS `gcs_url` (UDP), but watch P1 on the 115200 link. ROS services and actions cover commands. A second path to the aircraft would break the one-door rule. A future phone/web UI would be a thin layer (or `rosbridge_suite`) allowed only to call `/drone/llm/submit_command`, cancel the mission, and read status topics.

## Appendix B — Config sketch (`drone_bringup/config/drone.yaml`)

Starting points only.

```yaml
mavros:                                   # rendered into the MAVROS config by the launch file
  fcu_url: "serial:///dev/<discovered>:115200"   # 921600 later
  gcs_url: ""                             # e.g. udp://@<laptop-ip>:14550 (adds link load)
  tf_send: false

fcu_command_node:
  ros__parameters:
    output_enabled: false                 # [AIRCRAFT] launch arg enable_fcu_output
    allow_arming: false                   # [AIRCRAFT] launch arg allow_arming
    rate_hz: 10.0
    cmd_timeout_s: 0.3                    # [TUNE]
    safety_stale_land_s: 1.0
    set_mode_timeout_s: 3.0
    arm_timeout_s: 3.0
    takeoff_start_timeout_s: 3.0
    max_arm_to_takeoff_s: 2.0
    min_climb_rate_mps: 0.2               # for the climb timeout
    alt_tol_m: 0.15                       # [TUNE]
    stable_window_s: 2.0
    stable_timeout_s: 8.0
    stable_max_vz_mps: 0.15               # [TUNE]
    stable_max_vxy_mps: 0.2               # [TUNE]
    min_descent_rate_mps: 0.1
    disarm_wait_s: 5.0
    ground_height_max_m: 0.15             # [TUNE] MTF-01 reading when sitting on the floor

fcu_state_node:
  ros__parameters:
    telemetry_timeout_s: 1.0
    flow_timeout_s: 0.5
    flow_min_quality: 100                 # [TUNE] on your floor
    flow_quality_samples: 5
    flow_min_height_m: 0.3                # [TUNE] MTF-01 datasheet
    flow_max_height_m: 3.0                # [TUNE] MTF-01 datasheet / your ceiling
    message_rates_hz: {attitude: 10.0, local_position: 10.0, optical_flow: 10.0,
                       distance_sensor: 10.0, ekf_status: 2.0, sys_status: 2.0,
                       extended_sys_state: 2.0, rc_channels: 2.0}

safety_supervisor:
  ros__parameters:
    max_yaw_rate_deg_s: 30.0              # [TUNE]
    max_yaw_accel_deg_s2: 60.0            # [TUNE]
    max_horizontal_speed_mps: 0.0         # locked in current scope
    max_vertical_speed_mps: 0.0           # locked (takeoff/land are FCU-controlled)
    max_cmd_age_s: 0.3
    min_takeoff_alt_m: 0.8                # [TUNE] above flow min height + ground effect
    max_takeoff_alt_m: 1.5                # [TUNE] indoor
    hard_ceiling_m: 2.2                   # [TUNE] your room
    ceiling_margin_m: 0.5
    min_height_m: 0.5                     # in-flight band [TUNE]
    max_height_m: 2.0                     # [TUNE]
    max_tilt_deg: 25.0                    # [TUNE]
    preflight_max_tilt_deg: 5.0
    takeoff_min_voltage: 0.0              # [TUNE] per pack; 0 = pre-flight FAILS until you set it
    battery_warn_v: 0.0                   # [TUNE]
    battery_land_v: 0.0                   # [TUNE]
    flow_bad_timeout_s: 1.0               # [TUNE]
    hold_to_land_s: 5.0                   # [TUNE]
    require_rc_present: true
    allow_auto_land: true
    cpu_temp_warn_c: 75.0
    cpu_temp_fault_c: 82.0
    allowed_sources: ["mission_manager", "operator"]
    allowed_pretakeoff_modes: ["STABILIZE", "ALT_HOLD", "LOITER", "GUIDED"]

yolo_detector:
  ros__parameters:
    model_path: ""                        # absolute path to .onnx, required
    input_size: 320
    confidence_threshold: 0.5             # [TUNE]
    nms_iou_threshold: 0.45
    allowed_classes: ["person"]
    publish_debug_image: false

target_tracker:
  ros__parameters:
    confirm_frames: 3
    lost_timeout_s: 1.0                   # [TUNE]
    selection_policy: largest_bbox
    iou_match_threshold: 0.3

mission_manager:
  ros__parameters:
    default_takeoff_altitude_m: 1.2       # [TUNE]
    search_yaw_rate_deg_s: 15.0           # [TUNE]
    search_direction: 1                   # +1 = CCW (left) in ROS convention
    search_max_total_yaw_deg: 360.0
    search_timeout_s: 60.0
    max_search_timeout_s: 120.0
    track_kp: 0.5                         # [TUNE]
    track_deadband: 0.05
    max_track_yaw_rate_deg_s: 20.0        # [TUNE]
    lost_hold_s: 1.0
    reacquire_timeout_s: 5.0
    reacquire_max_yaw_deg: 45.0
    max_reacquire_attempts: 2
    max_track_duration_s: 300.0
    return_mode: yaw_to_start_then_land   # or land_in_place
    on_cancel: land                       # or hover (testing only)
    enable_approach: false

intent_bridge:
  ros__parameters:
    llm_enabled: false                    # [SET] Phase 13: optional, keyword parser otherwise
    allow_execute: false                  # [SET] Phase 13: dry run only until set true
    llm_model: "claude-sonnet-5-5"
    llm_timeout_s: 10.0
```

`takeoff_min_voltage: 0.0` deliberately makes pre-flight **fail** until you set a real value for your battery. An unset safety number must never mean "no limit".

## Appendix C — Reference repo (EspiroFares/Autonomous-UAV)

Ideas only, nothing copied. Same principles confirmed there: a single FC bridge, safety veto, stale = invalid, BEST_EFFORT subscriptions for MAVROS, explicit message-rate requests, flow + rangefinder fused on the FC (their params use EK3_SRC1_VELXY = optical flow). Differences here: YOLO instead of MediaPipe, action-based mission and flight-op APIs, all-Python, autonomous arm/takeoff/land with per-step verification, override detection, bench-mode output switches, a SITL gate before real autonomous takeoff, no metric distance claim from the camera.

## Appendix D — ArduPilot parameters to READ in Phase 3 (not change without your OK)

| Param | Why we care |
|---|---|
| `SERIAL1_PROTOCOL` (2), `SERIAL1_BAUD` (115) | link |
| `FLOW_TYPE`, `FLOW_ORIENT_YAW`, `RNGFND1_TYPE`, `RNGFND1_MIN` / `RNGFND1_MAX` | how the MTF-01 is connected (MAVLink vs MSP) and its range |
| `EK3_SRC1_VELXY` (5 = optical flow), `EK3_SRC1_POSZ` (1 baro / 2 rangefinder), `EK3_SRC1_POSXY`, `EK3_SRC1_YAW` | what the EKF fuses; takeoff altitude reference |
| `GUID_TIMEOUT`, `GUID_OPTIONS` | GUIDED setpoint timeout (L2); arming behaviour in GUIDED |
| `DISARM_DELAY` | arm → takeoff window (P19) |
| `LAND_SPEED`, `LAND_SPEED_HIGH`, `PILOT_SPEED_UP`, `WPNAV_SPEED_UP` | climb/descent rates → timeouts |
| `ARMING_CHECK` | pre-arm behaviour |
| `FS_THR_ENABLE`, `FS_EKF_ACTION`, `FS_EKF_THRESH`, `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT`, `BATT_LOW_VOLT`, `BATT_CRT_VOLT`, `FS_GCS_ENABLE` | L1 failsafes |
| `FENCE_ENABLE`, `FENCE_TYPE`, `FENCE_ALT_MAX` | an FCU-side altitude ceiling is a good second layer indoors (recommended, your decision) |
| `FLTMODE1..6`, `FLTMODE_CH`, `RCx_OPTION` (motor emergency stop / kill) | confirms the RC override and the kill switch exist |
| `SYSID_MYGCS` | GCS failsafe identity (P13) |
