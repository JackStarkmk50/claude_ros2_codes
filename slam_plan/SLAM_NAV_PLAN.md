# Visual SLAM and navigation: plan

**Decision (2026-10-08): webcam only.** We use the Zebronics USB webcam (one lens) for SLAM, no new
camera. This file is the plan for that path. The depth-camera option is kept at the end in case we
come back to it.

## 1. What we have

| Item | Fact | Source |
|---|---|---|
| Camera | Zebronics USB webcam, **mono**, 640x480 YUYV, **not calibrated yet**, FOV unknown | `PHASE9_CAMERA.md`, `drone.yaml` |
| Position | ArduPilot EKF3 fuses MTF-01 flow + ToF range; good over seconds, **drifts** over minutes | `ARCHITECTURE.md` §9, §12 |
| Pi 5 | runs YOLO + the stack; has had **undervoltage** | project history |
| Sim | Gazebo room 10 x 10 m, camera 640x480, 60 deg, 15 Hz; walls and floor are **flat colours** | `sim/gazebo/worlds/indoor_search.sdf` |

## 2. The three problems with one lens, and what we do about each

| Problem | What we do |
|---|---|
| **No scale**: the map comes out in unknown units | `slam_scale` learns metres per SLAM unit by comparing short moves with the EKF (flow + range). It only reports a pose once the moves agree (`drone_slam/core/scale.py`) |
| **Blur and rolling shutter on fast turns** | map slowly: the first map is made by **carrying** the drone (props off), turning gently. Fixed exposure if the webcam allows it. Calibrate once |
| **No obstacle distance** | the floor map comes from the SLAM points (walls with texture show up, plain white walls may not). Unknown = not free. Live obstacle avoidance from the camera is **not** solved by this; flights stay slow, in mapped space, with the pilot ready |

Plain walls give SLAM nothing to track. Expect to need some texture (posters, pictures) in bare rooms.

## 3. Software

```
webcam -> /camera/image_raw -> stella_vslam (mono)  -> camera pose (no units)  -+
                                   |                                             v
                                   +-> saved map (.msg)      EKF position -> slam_scale -> /drone/slam/pose (metres, "map")
                                           |
                     make_map (laptop) ----+-> map.pgm/map.yaml (Nav2) + rooms.yaml (named places)
```

- **SLAM engine: stella_vslam** (open-source ORB-SLAM successor, BSD licence). Mono support, saves
  and reloads maps, has a localization-only mode (lighter on the Pi), and its map file is easy to
  read, which `make_map` needs. Built from source with `drone_slam/sim/setup_stella.sh`. Its topic
  (`/run_slam/camera_pose`, ROS axes) and map keys were checked against its source code.
  Fallback: ORB-SLAM3 (GPL, map file much harder to read).
- **`drone_slam` package:** `calibrate_camera`, `slam_scale`, `make_map`, rooms files, `slam.launch.py`,
  and for the simulator a textured two-room world and `sim_survey.py`.
- **Later:** Nav2 on `map.yaml`, a `cmd_vel -> ControlCommand` adapter so the safety supervisor still
  checks every command, and SLAM pose into ArduPilot (`vision_pose`) so the EKF stops drifting.
  Those touch packages other threads own and come after S1 to S3 work.
- **LLM:** gets `rooms.names()` only and picks one, as agreed in the LLM thread.

## 4. Keeping the Pi light

1. Mapping runs on the laptop from a recorded rosbag. The Pi only records.
2. The Pi later runs stella_vslam in **localization mode** on the saved map, at reduced rate.
3. Measure CPU and power in S5 before relying on it; YOLO and SLAM share four cores.

## 5. Phases

| Phase | What | Needs |
|---|---|---|
| S0 | Calibrate the webcam with a printed checkerboard: `ros2 run drone_slam calibrate_camera --device /dev/video0` | printed board |
| S1 | Gazebo: textured two-room world; build stella_vslam on WSL; fly the sim survey; `slam_scale` locks; save the map. **Ready to run: [SLAM_S1_GAZEBO.md](SLAM_S1_GAZEBO.md)** | WSL |
| S2 | `make_map` on the sim map -> floor map + rooms; compare with the true room (same doc, step 6) | WSL |
| S3 | Gazebo: localize in the saved map, Nav2 plans "go to room X" (adapter into ControlCommand) | WSL |
| S4 | Real house: record a rosbag while **carrying** the drone room to room; build the map on the laptop; mark rooms | webcam, laptop |
| S5 | Pi: localization only, measure CPU / power / pose jumps, props off | drone |
| S6 | SLAM pose into EKF3, position hold, then one short supervised room-to-room flight | drone, pilot |

Sim changes (textured world) go through the thread that owns the Gazebo files.

## 6. Honest limits of the webcam path

- Scale needs motion the flow sensor can see; hovering or turning on the spot teaches it nothing.
- Fast yaw (our search turns) can make SLAM lose track; it must re-find itself in the map, and the
  mission must treat "SLAM lost" as "stop and hold", not keep flying on old positions.
- Glass, mirrors, dark rooms and plain walls are weak spots.
- If S1 to S4 show it isn't reliable enough, the depth-camera option below is the upgrade path,
  and everything in `drone_slam` except `slam_scale` still applies.

## Appendix: depth-camera option (not chosen)

Luxonis OAK-D Lite (about USD 150 from memory, ~61 g): stereo depth in metres, global shutter,
IMU, can run YOLO on the camera. RTAB-Map would then build the grid directly from depth and give
obstacle distances.
