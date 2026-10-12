# drone_slam

Monocular visual SLAM support for the Zebronics webcam: camera calibration, metric scale, a 2D floor
map for Nav2, and named rooms. The SLAM engine itself (stella_vslam) is built separately; this
package is what goes around it. Plan: `docs/SLAM_NAV_PLAN.md`.

| Executable | What it does | Where it runs |
|---|---|---|
| `calibrate_camera` | checkerboard calibration of the webcam (`--device` live or `--images`), or the Gazebo camera from its FOV (`--sim`). Writes `~/drone_ws/camera_calib.yaml`, prints PASS/REDO | Pi (live) or laptop (pictures) |
| `slam_scale` | node: SLAM camera pose (no units) + EKF position -> `/drone/slam/pose` in metres on the `map` frame, plus scale status on `/diagnostics`. Commands nothing | Pi or laptop |
| `make_map` | saved SLAM map (`.msg` from stella_vslam, or `.csv`) -> `map.pgm` + `map.yaml` (nav2_map_server) + `map_preview.png` with the rooms; checks every room point is on free floor | laptop |

Launch: `slam.launch.py` starts stella_vslam (`run_slam`, built separately by `sim/setup_stella.sh`)
and `slam_scale`; `mode:=map` saves the map on Ctrl-C, `mode:=localize` uses a saved one.

Config: `stella_sim.yaml` (Gazebo camera), `stella_webcam.yaml` (template; `calibrate_camera` fills
it in), `rooms.example.yaml` and `rooms_sim.yaml` (the room list the LLM will choose from).

Simulator (`sim/`): `worlds/indoor_search_textured.sdf` (textured two-room copy of the sim room,
made by `make_textured_world.py`), `sim_survey.py` (sim-only mapping flight). Steps:
`docs/SLAM_S1_GAZEBO.md`.

Pure logic in `core/` (no ROS): `calibration`, `scale`, `frames`, `grid`, `rooms`, `stella_map`, `survey`.

Test:

```bash
colcon test --packages-select drone_slam && colcon test-result --verbose
```

Extra apt packages: `python3-msgpack` (reads stella_vslam maps). Everything else is already
installed for the other packages.
