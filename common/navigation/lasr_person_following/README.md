# lasr_person_following

Person tracking and Nav2-based person following for TIAGo. Used by the
`FollowPerson` skill (`skills/src/lasr_skills/follow_person.py`), e.g. "follow the
host" in HRI and `follow_person` in GPSR.

## How it works

Simple version: it assumes nobody walks between the robot and the host and nothing blocks
the robot. Other people may be around; the robot must never switch to them.

```
 RGB + depth ──► person_tracker  (always running; detects only while enabled)
                  ├─ /yolo/detect3d, called asynchronously on the newest frame
                  ├─ Kalman filter per person (odom frame), gated Hungarian association
                  │  (position + torso colour histogram)
                  ├─ host: locked on request; after a loss taken back only if clearly
                  │  them (otherwise stays lost)
                  └─ head: PointHead goals keep the host in view (fire and forget)
                         │ ~/host_state (JSON), ~/host_pose, ~/host_point, ~/markers
                         ▼
                 follow_manager  (Nav2 NavigateToPose only, never cmd_vel)
                  ├─ host far or walking → goal on the line robot → host, facing them:
                  │    1 m short of them + 2 s of their speed away from the robot,
                  │    never closer than 0.8 m (never beside/past them, see below)
                  ├─ host close, standing → turn on the spot to face them
                  ├─ host lost            → drive to where they were last seen
                  ├─ Nav2 recovery (spin/wait/back up) starts → cancel, send a fresh goal
                  └─ raises Nav2 speed limits while following, restores them after
                         ▼
                 FollowPerson (yasmin state machine in `skills`)
                  torso down/head centred → lock host → follow → host stood still → "Have we arrived?"
                     yes → succeeded        no → follow again
                  lost → wait 8 s to see them again → "please come back" → lock again
```

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
`surface_offset` (0.12 m: mask points lie on the body surface), `camera_latency` (0 s:
time between capture and image stamp; a real driver stamping on arrival makes the host
appear shifted sideways whenever the robot or head turns. Negative = add latency, for
sim tests). It warns if `/head_controller/point_head_action` is missing (then the head
does not follow the host and a turn of ~30° loses them).

The camera must not move between locking the host and following, other than by head
tracking: `FollowPerson` lowers the torso and centres the head **before** locking. (It
used to do so after the lock: the host's detections jumped and changed appearance just
as the robot started driving, and the tracker dropped them, seen in sim S5 and on the
real robot as "starts following, turns, host lost".)

Tracker details (`tracking.py`, unit tested in `test/test_tracking.py`):
* constant-velocity Kalman filter, 99% Mahalanobis gate, plus an appearance-backed gate
  (fast camera motion can throw a detection ~1 m sideways)
* appearance veto: an established track never takes a detection that clearly looks like
  someone else
* a track coasts 3 s without detections before it is dropped
* re-identification after a loss needs: confirmed track, appearance similarity ≥ 0.75 and
  clearly better than any other candidate, within 3 m of where the host was last seen or
  was heading, and **not** a person ever detected in the same frame as the host and >1 m
  from them (or seen meanwhile further away than the host could have walked). If nobody
  qualifies the host stays lost and is asked to come back: never guess
* "stationary" (arrival) is restarted only by movement, not by short detection gaps

### follow_manager
| | |
|---|---|
| `~/start`, `~/stop` (Trigger) | start/stop following (stop cancels the Nav2 goal) |
| `~/status` (String, JSON) | `mode` (`idle/following/facing/to_last_seen`), `settled`, distance, `nav` |

Main parameters: `follow_distance` (1.0 m), `lookahead` (2.0 s: Nav2's controller slows
down near its goal, so the goal is pushed towards a host walking away), `min_distance`
(0.8 m), `recovery_actions` (`/spin`, `/backup`, `/wait`), `follow_max_speed`
(0.7 m/s), `controller_node` / `controller_speed_params` / `smoother_node` (which Nav2
parameters to raise). Following is computed in `odom` (does not jump when AMCL corrects);
goals go to Nav2 in `map`.

Why the goal stays on the robot → host line: the host is an obstacle in Nav2's costmap.
A goal beside or past them (what aiming at "where they will be in 2 s" gives for a host
walking off at 1 m/s from 1 m away) makes the planner route around them, so the robot
turns sideways, the camera loses the host, and the next plan may go round the other
side. On the line the robot keeps facing the host, and the line is clear (the camera sees
them along it). Nav2's default recovery behaviours (spin 90°, wait 5 s, back up) never
help while following, so they are cancelled.

## Running

```bash
ros2 launch lasr_person_following person_following.launch.py   # tracker + manager
# options: follow_distance:=1.0
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
./start_sim_stack.sh              # headless Gazebo + Nav2 + YOLO + stubs (GPU via --nv if present)
./run_sim_tests.sh mytest         # all scenarios; or: ./run_sim_tests.sh mytest s1_basic
FOLLOW_ARGS="follow_distance:=1.2" ./run_sim_tests.sh d12   # launch options under test
# a robot that can keep up with a walking host (configured max 1.0 m/s):
export SIM_MAX_SPEED=1.0; ./start_sim_stack.sh; FOLLOW_ARGS="follow_max_speed:=1.0" ./run_sim_tests.sh fast
```
Results (summary, CSV, plot) land in `$OUT` (default `/tmp/follow_eval`).

* `scripted_person`: a person model (laser-visible) walks a scenario in
  `config/scenarios/` (S1 basic, S2 turns, S3 stop-and-go, S4 crossing distractor,
  S5 passing distractor, S6 sidestep + come back, S7 fast walker, S8 host walks off at
  1 m/s from 1.3 m in front, like a real test)
* `follow_evaluator`: scores against Gazebo ground truth (distance band, left behind,
  tracking error, distractor confusion, AMCL error)
* `sim_stubs`: `point_head_action` and `transcribe_speech` (answers "have we arrived?"
  like the scripted host would) which the public TIAGo sim lacks
* `reset_episode`: puts the robot back at the start with Nav2 (Gazebo's `reset_world`
  breaks this sim's physics)

The tests need `simulation` (house world, `nav_params` launch argument) and
`lasr_vision_yolo` (metre depth from the sim's 32FC1 camera; an older build divides it by
1000 and puts every person at the camera) built from this branch. Sim quirks: the camera only renders if gzserver can reach the X display
(hence the `/run/user` bind); `LIBGL_ALWAYS_SOFTWARE=1` without an NVIDIA GPU; the sim runs at
~0.4–0.8× real time, so all timings are in sim time; AMCL in this map drifts up to
~0.6 m.

