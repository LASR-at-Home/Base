# skills

Skills that can be used in tasks, or to build bigger skills.

A skill is a [yasmin](https://github.com/uleroboticsgroup/yasmin) state or a small state machine.
Every skill is importable from `lasr_skills`, has a fixed set of **outcomes** that you map to the
next state with `transitions`, and exchanges data with other skills through the **blackboard**.

## Using a skill

```python
import yasmin
from lasr_skills import Say, PlayMotion, SafeGoToLocation

sm = yasmin.StateMachine(outcomes=["succeeded", "failed"])
sm.add_state(
    "SAY_HELLO",
    Say(text="Hello"),
    transitions={"succeeded": "LOOK", "aborted": "failed", "canceled": "failed"},
)
sm.add_state(
    "LOOK",
    PlayMotion("look_left"),
    transitions={"succeeded": "GO", "aborted": "failed", "canceled": "failed"},
)
sm.add_state(
    "GO",
    SafeGoToLocation(location_param="start_pose"),
    transitions={"succeeded": "succeeded", "failed": "failed"},
)
outcome = sm()
```

- **Arguments vs blackboard:** most skills take an input either as a constructor argument (fixed) *or*
  from a blackboard key (decided at runtime) if the argument is left out. Use
  `remappings={"skill_key": "your_key"}` in `add_state` to read a different key.
- **Outcomes:** ROS action based skills return `succeeded` / `aborted` / `canceled`; most others
  return `succeeded` / `failed`. Always check the table.
- **Examples:** real usage is in [`tasks/`](../tasks), e.g. `HRI/HRI/state_machine.py`,
  `GPSR/GPSR/states/dispatch_skill.py`, `restaurant/restaurant/states/survey.py`.
- **Standalone:** skills with a Run entry can be tried on their own with `ros2 run skills <name>`
  (the `main()` is a demo, edit it for different arguments).

---

## Speech

| Skill | What it does | Arguments | Outcomes | Blackboard |
|-------|--------------|-----------|----------|------------|
| `Say` | Speaks a sentence out loud. Only logs the text if `tts_msgs` is not installed. | `text` (fixed) or `format_str` (template). Neither: reads `text`. | `succeeded`, `aborted`, `canceled` | in: `text`, `placeholders` (fills `format_str`) |
| `Listen` | Records one utterance and transcribes it. | none | `succeeded`, `aborted`, `canceled` | out: `sequence` |
| `AskAndListen` | Asks a question, listens, and asks again if nothing was heard. | `tts_phrase` (fixed) or `tts_phrase_format_str` (template) | `succeeded`, `failed` | in: `tts_phrase_placeholders`; out: `transcribed_speech` |

**General information / requirements**

- TTS action server at `/tts_engine/tts` for `Say`.
- Speech recognition: the `transcribe_speech` action (Whisper, `lasr_speech_recognition_whisper`) and a
  running [`microphone`](../common/microphone/README.md) node.
- Prefer `AskAndListen` over `Listen`; it wraps `Say` + `Listen` with retries.

## Navigation

| Skill | What it does | Arguments | Outcomes | Blackboard |
|-------|--------------|-----------|----------|------------|
| `GoToLocation` | Drives to a pose with Nav2 and waits for arrival. | `location` (a `Pose`) or `location_param` (name of a saved pose, e.g. `"start_pose"`). Neither: reads `location`. | `succeeded`, `failed` | in: `location` |
| `SafeGoToLocation` | `GoToLocation` with the arm/head tucked away first (`pre_navigation`) and restored after (`post_navigation`). Retries on failure. Use this for normal moves. | `location_pose` or `location_param` | `succeeded`, `failed` | |
| `ContinuousGoToLocation` | Keeps navigating to a target that is updated while it drives. Used inside `FollowPerson`; rarely needed directly. | none | `succeeded`, `canceled`, `aborted` | in: `location`, `cancel_nav`, `stop_robot_requested`; out: `stop_robot_requested` |
| `Rotate` | Turns the robot on the spot. | `mode="angle"` with `angle`, or `mode="point"` with `target_point` (a `Point`). Missing values are read from the blackboard. | `succeeded`, `failed` | in: `angle`, `target_point` |

**General information / requirements**

- Nav2 must be running, and `Rotate` also needs `/amcl_pose` (localisation).
- Named locations (`location_param`) are ROS parameters in the form `<name>.position.{x,y,z}` and
  `<name>.orientation.{x,y,z,w}`. Provide them with `--ros-args --params-file <file>.yaml` or the
  task's launch file.
- `SafeGoToLocation` plays the `pre_navigation` / `post_navigation` motions, so load the motions
  first (see [Motion](#motion)).

## Motion

| Skill | What it does | Arguments | Outcomes | Blackboard |
|-------|--------------|-----------|----------|------------|
| `PlayMotion` | Plays a named arm/head/torso motion. | `motion_name`. `None`: reads `motion_name`. | `succeeded`, `aborted`, `canceled` | in: `motion_name` |
| `LookToPoint` | Turns the head to look at a point in space. | `pointstamped` (`PointStamped`). `None`: reads `pointstamped`. | `succeeded`, `aborted`, `canceled` | in: `pointstamped` |

**General information / requirements**

- Load the custom motions once the robot/simulation is up:
  ```bash
  ros2 launch skills load_motions.launch.py
  ```
  This loads [`config/motions.yaml`](config/motions.yaml) into `/play_motion2_mgr` and restarts it.
  Re-run it after editing the file.
- List the motions that are available:
  ```bash
  ros2 service call /play_motion2/list_motions play_motion2_msgs/srv/ListMotions
  ```
- Motions in `motions.yaml`: `reach_arm_vertical_gripper`, `reach_arm_horizontal_gripper`,
  `cml_arm_away`, `open_gripper`, `pre_navigation`, `post_navigation`, `look_left`, `look_right`,
  `look_centre`, `look_down_left`, `look_down_right`, `look_down_centre`, `raise_torso`,
  `pointing_to_the_left`, `pointing_to_the_right`, `raising_left_arm`, `raising_right_arm`, `ml`,
  `mm`, `mr`.
- `LookToPoint` needs `/head_controller/point_head_action`.

## Detection

| Skill | What it does | Arguments | Outcomes | Blackboard |
|-------|--------------|-----------|----------|------------|
| `Detect3D` | One camera frame in, every detected object with its 3D position out. | `model`, `filter` (list of class names, e.g. `["person"]`), `confidence`, `target_frame`, camera topics | `succeeded`, `failed` | out: `detections_3d`, `image_raw` |
| `Detect3DInArea` | `Detect3D` keeping only detections inside a polygon (and height range). | `area_polygon`, `filter`, `z_min`, `z_max`, plus the `Detect3D` arguments. `None` polygon: reads `polygon`. | `succeeded`, `failed` | in: `polygon`, `z_sweep_min`, `z_sweep_max`; out: `detections_3d`, `image_raw` | 
| `DetectAllInPolygon` | Sweeps the head over an area and collects every object seen, de-duplicated. For tables/shelves. | `polygon`, `object_filter`, `min_confidence`, `min_coverage`, `model`, `min_new_object_dist`, `use_lang_sam`, `prompt` | `succeeded`, `failed` | out: `detected_objects`, `debug_images` |
| `DetectKeypoints3D` | Finds people and 3D body keypoints (e.g. to see where someone is pointing). | `model`, `confidence`, `target_frame` | `succeeded`, `failed` | out: `keypoint_detections_3d`, `image_raw` |
| `DetectWave` | Checks whether anyone in view is waving. | `model`, `confidence`, `target_frame` (default `base_footprint`) | `waving`, `not_waving`, `failed` | out: `wave_detected`, `wave_position` |
| `WaitForPersonInArea` | Blocks until a person is detected inside a polygon. | `polygon` or `polygon_param`. Neither: reads `polygon`. | `succeeded`, `failed` | in: `polygon`; out: `detections_3d` | 
| `DescribePeople` | Sends the image to a vision-language model and returns attributes (clothing, hair, ...). | none | `succeeded`, `failed` | in: `image_raw` (e.g. from `Detect3D`); out: `attributes` | 

**General information / requirements**

- Start the YOLO service first: `ros2 launch lasr_vision_yolo service_launch.xml`
  (`DetectWave` and `DetectKeypoints3D` use the pose service `/yolo/detect3d_pose`).
- Detections are returned in `target_frame` (default `map`), so they can go straight into navigation.
- `DescribePeople` needs the VLM service, and `image_raw` on the blackboard, so run a detection first.
- Default camera topics are the `/head_front_camera/...` RGB, depth and camera-info topics.
- `DetectAllInPolygon` moves the head and torso while sweeping. If you call it with `.execute()`
  directly, initialise `detected_objects` and `debug_images` to `[]` on the blackboard first.

## Person interaction

| Skill | What it does | Arguments | Outcomes | Blackboard |
|-------|--------------|-----------|----------|------------|
| `FollowPerson` | Full "follow me" behaviour: finds the person, follows with Nav2, rescans if they are lost and asks "have we arrived?" when they stop. | none | `succeeded`, `failed` | |
| `ReceiveObject` | Takes an object from a person: reaches out, asks them to put it in the gripper, closes it. | `object_name`, `vertical` (gripper orientation), `can_hold` (`True` if it can be held, e.g. a bag handle) | `succeeded`, `failed` | in: `object_name` |
| `HandoverObject` | Holds out an object, waits for the person to take it, then opens the gripper. | same as `ReceiveObject` | `succeeded`, `failed` | in: `object_name` |
| `StartEyeTracker` | Makes the robot's eyes follow a person until cancelled. Run it in a `Concurrence` beside other states. | none | action outcomes (`succeeded`, `aborted`, `canceled`) | in: `person_point` | 
| `StopEyeTracker` | Cancels a running `StartEyeTracker`. | none | `succeeded`, `failed` | out: `cancel_eye_tracker` | 

**General information / requirements**

- `FollowPerson` brings its own dependencies up with:
  ```bash
  ros2 launch skills follow_person.launch.py
  ```
  (motions, YOLO service, microphone, Whisper and the skill). Nav2 must also be running.
- `ReceiveObject` / `HandoverObject` use the arm and gripper, so the arm controllers and the loaded
  motions are required, and they speak, so TTS must be running.
- Eye tracker needs the `lasr_vision_eye_tracker` action server (`/lasr_vision_eye_tracker/track_eyes`).

## Others (doors, sound and utilities)

| Skill | What it does | Arguments | Outcomes | Blackboard |
|-------|--------------|-----------|----------|------------|
| `Wait` | Pauses for a number of seconds. | `wait_time` | `succeeded`, `failed` | | 
| `DetectDoorOpening` | Watches the laser scan until the door in front opens (average distance grows by `opened_delta`). | `lasr_scan_topic="/scan"`, `opened_delta=0.5` m, `timeout=15.0` s | `door_opened`, `failed` | |
| `StartDoorSM` | Standard task opening: wait for the door to open, then `SafeGoToLocation` to the start pose. | `location`, `location_param="start_pose"` | `succeeded`, `failed` | |
| `DetectDoorbell` | Listens for a doorbell or similar ringing/beeping sound with [YAMNet](https://www.kaggle.com/models/google/yamnet). | `score_threshold=0.20`, `included_classes` (YAMNet names such as `Doorbell`, `Bell`, `Buzzer`, `Chime`, `Beep, bleep`, `Alarm clock`) | `succeeded` (heard), `failed` (timeout, cancelled or no mic) | |

**General information / requirements**

- `DetectDoorOpening` / `StartDoorSM` need the `/scan` topic (and Nav2 for `StartDoorSM`).
- `DetectDoorbell` needs the [`microphone`](../common/microphone/README.md) node running
  (`/microphone/record` service). To try it alone: `ros2 run microphone mic`, then
  `ros2 run skills detect_doorbell`.
  - The YAMNet model loads on construction (from `~/.cache/kagglehub/models/google/yamnet/tensorFlow2/yamnet/1`,
    downloaded with `kagglehub` the first time), which takes a few seconds, so build the state before
    the robot needs it.
  - It records 0.5 s chunks into a rolling ~0.975 s window; it succeeds when the top class is in
    `included_classes` and scores above `score_threshold`, and fails after 15 s.
  - The default classes are broad on purpose. For false positives (music, speech) raise
    `score_threshold` or narrow `included_classes`.
  - Python dependencies (`tensorflow==2.16.2`, `kagglehub`) are in [requirements.txt](requirements.txt);
    TensorFlow is only imported when the state is constructed.

## Safety node

`safety` is a node, not a skill: `ros2 launch skills safety.launch.py` (parameters in `config/safety.yaml`).

**This is strictly for the RoboCup Safety Test**