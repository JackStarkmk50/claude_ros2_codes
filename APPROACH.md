# Approach: fly closer to the person and keep a distance (simulator first)

**Goal:** besides turning to keep the person in view, the mission can fly **forward** to get to
an estimated distance from them and keep it, for example "go near them and stay for 10 seconds,
then land", or "follow me for 60 seconds and keep 3 m away". With reverse switched on it also
**backs away** when the person walks closer.

**Where it is on:** only in the simulator copy of drone.yaml (`make_sim_config.py`). In your real
`drone.yaml` it stays **off** in three places, and each one alone blocks it:
`mission_manager enable_approach: false`, `intent_bridge allow_approach: false`, and
`safety_supervisor max_horizontal_speed_mps: 0.0` (the gate clamps any forward speed to 0).
Backing away has two more switches of its own, both off in `drone.yaml`:
`mission_manager approach_allow_reverse: false` and `safety_supervisor max_reverse_speed_mps: 0.0`.
To try it on the real drone, follow [Real drone](#real-drone) below, step by step.

---

## How it works

| Part | What it does |
|---|---|
| distance estimate | from the person's box height: `d = H / (2 tan(vfov/2) h)`, with H = `approach_person_height_m` (1.7 m) and vfov = `camera_vfov_deg` (46.8 deg). When the feet leave the bottom of the picture (closer than about 3 m at 1.2 m height) it uses where the head is instead, with the rangefinder height. When the head leaves the top: "too close", the drone does not move |
| forward speed | 0.3 m/s per metre of distance error, at most `approach_max_speed_mps` (0.3), and **only while the person is centred** (`|error_x| < 0.15`). Inside +-0.3 m of the distance: no movement |
| backwards (off by default) | `approach_allow_reverse: false`: the drone only holds when the person walks closer. With `true` it backs away at most `approach_max_reverse_speed_mps` (0.2) and at most `approach_max_reverse_travel_m` (1.5 m) per mission, then only holds. Also only while the person is centred. When the head leaves the top of the picture (closer than about 0.8 m) it backs away at full reverse speed. **Backing up is blind**: there is no camera behind the drone |
| reverse travel count | from the FCU's velocity. While hovering, the optical flow reads small speeds both ways: below `approach_travel_noise_mps` (0.05) they are not counted unless the drone was asked to back up |
| travel limit | at most `approach_max_travel_m` (4 m) forward per mission, counted from the FCU's velocity. After that it only turns |
| timer | the track time (`--track 10`) starts once the drone is at the distance. If it is not there within `approach_timeout_s` (30 s), the timer starts anyway and the result says so |
| return | it turns back to the start heading and lands **where it is**. It does not fly back to the start point (with flow only, the position drifts) |
| safety | every forward or backward command still goes through the safety gate: HOLD/ABORT, stale commands, mode, and the speed limits (`max_horizontal_speed_mps`, and `max_reverse_speed_mps` for backwards, never above the horizontal one). A safety HOLD stops the motion at once |

Distances you can ask for: 2.0 to 5.0 m (`approach_min_distance_m` / `approach_max_distance_m`).
Default: 3.0 m.

**Honest limits**
- The distance is an **estimate**, not a measurement. It assumes an adult standing upright. A
  child, someone sitting or bending, or a person partly hidden makes the drone stop too far or
  go too close. A 10 % error in the assumed height is a 10 % error in the distance.
  In Gazebo the person model is 1.9 m tall, so the simulator copy uses 1.9.
- Forward flight indoors relies on the optical flow holding position and the camera seeing the
  person. Walls and obstacles are not seen at all. Keep the room clear between drone and person.
- The travel count reads high (in Gazebo 2.4 m counted for 1.6 m flown forward, 1.3 m counted
  for 0.96 m backwards), so the travel limits stop the drone early rather than late.
- Backing away at 0.2 m/s is slower than a person walks (about 1 m/s). If someone walks
  straight at the drone it gets closer before the drone has moved far. It is for keeping the
  distance to someone who steps closer, not for escaping.

---

## Commands

By hand:

```bash
python3 ~/drone_ws/tools/mission.py start --approach --track 10      # go near (~3 m), stay 10 s, land
python3 ~/drone_ws/tools/mission.py start --approach 2.5 --track 60  # keep ~2.5 m for 60 s
```

By text (`command.py -i`, or `command.py --go "..."`):

| You type | Result |
|---|---|
| `go near the person and stay for 10 seconds` | approach to 3 m (default), 10 s, land |
| `take off, detect a person, go near them and stay for 10 secs and land` | the same |
| `follow me for 60 seconds and keep 3 m away` | approach to 3 m, 60 s, land |
| `look for a person at 1.2 m and stay 2.5 m from them` | 1.2 m height, 2.5 m distance |
| `look for a person and keep 1 m away` | refused (2.0 to 5.0 m) |
| `follow me at 3 m` | refused: "at 3 m" is a **height**, and 3 m is too high. Say "3 m away" |
| `chase him`, `fly towards her`, `touch them`, `go near the door` | refused, always |

Backing away needs no special words: with `approach_allow_reverse: true`, every approach mission
also backs away when the person comes closer.

With `allow_approach: false` (your real drone.yaml) every approach command is refused with
"approaching a person is switched off". With the LLM enabled, the LLM only gets the approach
fields when `allow_approach` is true.

## Tested

- 284 tests in the workspace, all passing. For approach: distance estimates, forward speed,
  travel limit, centring before moving, safety HOLD, the parser's approach phrases and refusals,
  the gate's speed limits. For reverse: off by default, backing away to the distance, the
  reverse travel limit, centring, too close, and hover drift not counted.
- Gazebo, ArduCopter SITL Copter-4.6.3, the whole stack with YOLO:
  - G7: person standing 4.6 m away, `--approach --track 10`: flew forward to 2.97 m (true
    distance from Gazebo), held 10 s, turned back, landed and disarmed: COMPLETED. Run twice:
    2.97 m both times.
  - G8: typed `look for a person, follow them for 20 seconds and keep 2.5 m away`: held 2.49 m
    and 2.45 m in two runs,
    switched to the head estimate when the feet left the picture: COMPLETED.
  - G9 (reverse): person standing 3 m away walks 1.2 m towards the drone at 0.3 m/s while it
    keeps 3 m: it backed 0.96 m (true), held 2.76 m (the mission estimated 2.9 m), closest
    1.84 m while the person was walking, landed and disarmed: COMPLETED.
  - G1 to G6 still pass (one full G1 to G9 run: ALL PASSED).

---

## Real drone

Do these in order. Each step needs the one before it to pass. The pilot keeps the RC in hand
with the mode switch on STABILIZE/LOITER, ready to take over, exactly as in the earlier phases.

**1. Distance check on the ground (props off, nothing switched on).** Start perception
(`ros2 launch drone_bringup perception.launch.py`). Hold the drone, or put it on a stand, with
the camera at about 1.2 m and level. Stand upright with your whole body in the picture at
tape-measured distances, 2, 3 and 4 m:

```bash
python3 ~/drone_ws/tools/distance_check.py --at 3.0 --height 1.75   # your own height
```

It prints the estimate, the error, and the `camera_vfov_deg` that would make it exact. Put your
height in `approach_person_height_m` and that value in `camera_vfov_deg` (mission_manager in
drone.yaml). Check again at 2 m and 4 m. **Go on only when all three are within ±0.3 m.**

**2. Forward only.** In `drone.yaml`:

```yaml
safety_supervisor:   max_horizontal_speed_mps: 0.3
mission_manager:     enable_approach: true        # approach_allow_reverse stays false
intent_bridge:       allow_approach: true         # only if you want it by text
```

Clear room, at least 5 m in front. Stand 4.5 m away:
`python3 ~/drone_ws/tools/mission.py start --approach --track 10`. Expected: it turns to you,
creeps forward to about 3 m, holds 10 s, turns back and lands. Measure where it stopped.

**3. Backwards.** Only after step 2 has worked twice. In `drone.yaml`:

```yaml
safety_supervisor:   max_reverse_speed_mps: 0.2
mission_manager:     approach_allow_reverse: true
```

At least **2 m clear behind the drone**: it cannot see there. Start with you 3 m away:
`python3 ~/drone_ws/tools/mission.py start --approach --track 30`. Once the status says
"keeping distance", take one slow step (about 0.5 m) towards it. Expected: "backing away", it
moves back about 0.5 m and holds. Walk away again and it follows forward. The result line says
how far it went back ("back 0.6 m").

To switch everything off again, put back `max_horizontal_speed_mps: 0.0` and
`max_reverse_speed_mps: 0.0`: with those two at 0 the gate blocks all forward and backward
motion, whatever the mission asks.

| Problem | Likely cause |
|---|---|
| stops too far or comes too close | `approach_person_height_m` / `camera_vfov_deg` wrong: redo step 1 |
| does not move at all | gate limit still 0 (the safety_supervisor startup log prints the limits), or you are not centred in the picture |
| moves backwards when it should go forward | check `[VERIFY] FRAME_BODY_NED` in ARCHITECTURE.md: Gazebo confirms +vx = forward; on the real FCU, check step 2 before step 3 |
| "reverse limit 1.5 m reached" | it has used up its backward travel for this mission: it only holds and turns from then on |
