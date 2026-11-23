# lasr_person_following

Person tracking and Nav2-based person following for TIAGo. Used by the
`FollowPerson` skill (`skills/src/lasr_skills/follow_person.py`), e.g. "follow the
host" in HRI and `follow_person` in GPSR.

## How it works

```
 RGB + depth ──► person_tracker  (always running; detects only while enabled)
                  ├─ /yolo/detect3d, called asynchronously on the newest frame
                  ├─ Kalman filter per person (odom frame), gated Hungarian association
                  │  (position + torso colour histogram), tentative/confirmed/coasting tracks
                  ├─ host: locked on request, re-identified by appearance after a loss
                  └─ head: PointHead goals keep the host in view (fire and forget)
                         │ ~/host_state (JSON), ~/host_pose, ~/host_point, ~/markers
                         ▼
                 follow_manager  (Nav2 only, never cmd_vel)
                  ├─ on the host's trail  → FollowPath with the trail itself
                  │                          (takes the host's route, e.g. mid-doorway)
                  ├─ off the trail         → NavigateToPose to a point on the trail
                  ├─ host slow and close   → turn on the spot to face them
                  ├─ host lost             → drive to where they were last seen
                  ├─ stuck watchdog        → clear costmaps + Nav2 BackUp
                  └─ raises Nav2 speed limits while following, restores them after
                         ▼
                 FollowPerson (yasmin state machine in `skills`)
                  lock host → follow → host stood still → "Have we arrived?"
                     yes → succeeded        no → follow again
                  lost → wait / head sweep / turn round (Nav2 Spin) → ask to come back
```

### Why this design
The previous follower ran detection, association, head pointing and navigation in one
blocking yasmin loop, so it reacted at ~1 Hz, had no motion model or identity, and gave
up after a single missed frame. Its five `Detect3D` states each kept their own full-rate
camera subscriptions, which starved the executor so badly that in simulation no frame was
ever processed. Here perception runs continuously in its own node, people are tracked with
a motion model and appearance, and the state machine only makes decisions.

## Nodes

### person_tracker
| | |
|---|---|
| `~/enable` (SetBool) | start/stop detecting (YOLO is only called while enabled; off forgets all tracks) |
| `~/lock_host` (Trigger) | lock onto the confirmed person closest in front (within 3 m, ±35°) |
| `~/release_host`, `~/new_leg` (Trigger) | forget the host / reset "distance walked" |
| `~/head_tracking` (SetBool) | point the head at the host (on by default) |
| `~/host_state` (String, JSON) | status `no_host/tracking/coasting/lost`, position + velocity (odom), `still_for`, `travelled`, `last_seen_age` |
| `~/host_pose`, `~/host_point`, `~/markers` | host estimate, all tracks for RViz |

Main parameters: `model` (`yolo11n-seg.pt`), `confidence` (0.45), `max_range` (7 m),
`stationary_time` (4 s), `head_target_z` (1.1 m, lowered automatically when close),
`surface_offset` (0.12 m: mask points lie on the body surface).

Tracker details (`tracking.py`, unit tested in `test/test_tracking.py`):
* constant-velocity Kalman filter, 99% Mahalanobis gate, plus an appearance-backed gate
  (fast camera motion can throw a detection ~1 m sideways)
* appearance veto: an established track never takes a detection that clearly looks like
  someone else
* a track coasts 3 s without detections before it is dropped
* re-identification after a loss needs: confirmed track, appearance similarity ≥ 0.75 and
  clearly better than any other candidate, within 3 m of where the host was last seen or
  was heading, and **not** a person ever detected in the same frame as the host and >1 m
  from them. If nobody qualifies, a relaxed rule (`reid_relaxed`, on by default) takes the
  only *moving* person within 6 m who looks similar (≥ 0.7) and walks the host's way
* "stationary" (arrival) is restarted only by movement, not by short detection gaps

### follow_manager
| | |
|---|---|
| `~/start`, `~/stop` (Trigger) | start/stop following (stop cancels the Nav2 goal) |
| `~/status` (String, JSON) | `mode` (`following/facing/turning/to_last_seen/recovering`), `settled`, distance |

Main parameters: `follow_distance` (1.0 m), `follow_max_speed` (0.7 m/s),
`controller_node` / `controller_speed_params` / `smoother_node` (which Nav2 parameters
to raise), `use_trail_path` (true), `global_frame` (`odom`: following is relative to the
host, so it keeps working when AMCL jumps; `map` is the alternative).

## Running

```bash
ros2 launch lasr_person_following person_following.launch.py   # tracker + manager
# options: global_frame:=map|odom (default odom), reid_relaxed:=true|false (default true)
```
Needs the YOLO service (`lasr_vision_yolo service_launch.xml`) and Nav2. `HRI.launch.py`
and `skills follow_person.launch.py` include it.

## On the robot: Nav2 speed
`follow_manager` raises the controller's `max_vel_x`/`max_speed_xy` and the velocity
smoother's `max_velocity` to `follow_max_speed` while following, and restores them after
(also on Ctrl-C). **Humble's DWB only accepts runtime values up to the speed it was
configured with**, so this only works if the robot's Nav2 configuration has a high enough
maximum. Check on the robot:
```bash
ros2 param get /controller_server controller_plugins
ros2 param get /controller_server FollowPath.max_vel_x
```
If that maximum is below walking pace (~0.5 m/s), configure it higher and keep normal
navigation slower at runtime (or with a Nav2 speed limit), or set `controller_*` params
to match a different controller plugin.

## Simulation tests

```bash
cd src/Base/common/navigation/lasr_person_following/scripts
./start_sim_stack.sh              # headless Gazebo + Nav2 + YOLO + stubs (needs no GPU)
./run_sim_tests.sh mytest         # all scenarios; or: ./run_sim_tests.sh mytest s1_basic
FOLLOW_ARGS="global_frame:=map" ./run_sim_tests.sh maptest   # launch options under test
```
Results (summary, CSV, plot) land in `$OUT` (default `/tmp/follow_eval`).

* `scripted_person`: a person model (laser-visible) walks a scenario in
  `config/scenarios/` (S1 basic, S2 turns, S3 stop-and-go, S4 crossing distractor,
  S5 passing distractor, S6 sidestep + come back, S7 fast walker)
* `follow_evaluator`: scores against Gazebo ground truth (distance band, left behind,
  tracking error, distractor confusion, AMCL error)
* `sim_stubs`: `point_head_action` and `transcribe_speech` (answers "have we arrived?"
  like the scripted host would) which the public TIAGo sim lacks
* `reset_episode`: puts the robot back at the start with Nav2 (Gazebo's `reset_world`
  breaks this sim's physics)

Sim quirks: the camera only renders if gzserver can reach the X display (hence the
`/run/user` bind); `LIBGL_ALWAYS_SOFTWARE=1` without an NVIDIA GPU; the sim runs at
~0.4–0.8× real time, so all timings are in sim time; AMCL in this map drifts up to
~0.6 m, which occasionally makes the controller clip the doorway (the stuck watchdog
recovers).

