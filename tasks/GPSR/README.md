# GPSR Task

General Purpose Service Robot task for RoboCup@Home. The robot listens to a natural language command, plans a sequence of skills via LLM, and executes them.

## Quick start — planner test (simulation, keyboard input)

### 1. Set params

Make sure [config/params.yaml](config/params.yaml) has:

```yaml
gpsr:
  ros__parameters:
    simulation: true        # If using simualtion (uses gtts instead)
    input_mode: "keyboard"  # or "microphone"/"mic"
```

### 2. Start Ollama (first run, if you've already done you don't need to rerun these commands)

```bash
ollama serve
ollama pull gemma3   # only first time
ollama create gemma3-4b-8k -f <path>/Base/tasks/GPSR/external/Modelfile # Once only you can later use `gemma3-4b-8k` like a normal model
```

### 3. Build

```bash
# from workspace root (Base/)
colcon build --packages-select GPSR
source install/setup.bash

ros2 launch GPSR GPSR_tiago.launch.py   # In a terminal
ros2 launch simulation nav.launch.py    # If testing simulation in another terminal.
```

### 4. Run

 - #### 1. Running the Full Task
    > Ensure `simulation` is `false` and `input_mode` is `microphone`.
  
    To launch the state machine 
      ```
        ros2 run GPSR sm --ros-args --params-file tasks/GPSR/config/params.yaml
      ```
    In a new terminal run the following to start the Task
      ```
        ros2 topic pub /gpsr/start std_msgs/msg/Empty --once
      ```

  - #### 2. Using the Service
    To launch the state machine 
      ```
        ros2 run GPSR service --ros-args --params-file tasks/GPSR/config/params.yaml
      ```

    To call the service (in terminal)
      ```
        ros2 service call /gpsr/single_query std_srvs/srv/SetBool '{"data": true}'
      ```
    > The `data` refers to whether to dispatch the skills or not. (default to false) 


## Config files

| File | Purpose |
|------|---------|
| `config/params.yaml` | Runtime parameters (LLM host, input mode, simulation flag) |
| `config/skills.yaml` | Available skill signatures shown to the LLM |
| `config/locations.yaml` | Named 3D poses for navigation |
| `config/objects.yaml` | Object inventory (category, location, size) |
| `config/people.yaml` | Known people identities |
| `config/general_knowledge.yaml` | Arena context for the LLM |

## Skill mapping status

The GPSR dispatcher (`GPSR/states/dispatch_skill.py`) currently executes the following skills in the real ROS runtime:

| Skill | Status | Notes | Tested |
|------|------|------|------|
| `go_to_location` | yes | Uses Nav2 to drive to a mapped pose. Location names match `locations.yaml` ignoring case and `_` vs space. | Both |
| `say` | yes | Uses the TTS action (gtts in simulation). | Both |
| `guide_person` | partial | Speaks a follow-me instruction and navigates to the destination. | No |
| `find_person` | reworked | By gesture (`/gesture/detect`), name (asks to wave), clothes (VLM shirt colour) or any person (Detect3D). Only searches what is in front of the camera, no room scan yet. | No |
| `get_person_info` | reworked | Goes to `location`, then reads attributes from the VLM service (`/vlm/describe_people`). `info` arg is ignored. | No |
| `pick_up` | placeholder | Human-assisted pickup via `ReceiveObject`. | Both |
| `place_object` | placeholder | Human-assisted placement; asks the person to say "done" via `AskAndListen`. | No |
| `give_to_person` | placeholder | Human-assisted handover via `HandoverObject`. | Both |
| `find_object` | reworked | Goes to `location`, searches with head motions (centre, then down), looks at the best match. Returns `failed` if not seen. | No (was SIM only) |
| `follow_person` | yes | Uses the existing `FollowPerson` skill. | No |
| `count_objects` | new | Goes to `location`, counts detections matching the object name/alias/category, says the count. | No |
| `count_people` | new | Counts people by gesture, pose (via gesture service), clothes (VLM shirt colour per person crop) or all people. | No |
| `find_object_by_property` | new | biggest/smallest/thinnest from pixel bbox area, heaviest/lightest from `objects.yaml` `size` (tie-break by pixel area). | No |
| `answer_question` | no-op | Returns `succeeded`. The planner already adds a `say` step with the answer. | Both |

*All skills need to be retested regardless!*

## Dispatcher change log

- `_detect_faces` never ran its `Detect3D` state (always returned False). It now runs it and checks `detections_3d.detected_objects`.
- `_describe_person` bypasses `DescribePeople` and calls `GetVlmAttributes` directly with the latest camera image. `DescribePeople`'s `GetImage` writes `img_msg` while the VLM state reads `image_raw`, so it fails in the `skills` package.
- Replaced `Blackboard.get(key, default)` with `bb[key]` / `key in bb.keys()`, as the rest of the repo does.
- `_find_object` rewritten: respects the `location` arg, an empty object means "any non-person object", and underscores are spoken as spaces. Detection is **unfiltered** and matched by class name, `objects.yaml` alias or category, because the yolo service silently detects all classes when given an unknown class name.
- `Detect3D` instances are cached per filter in the dispatcher (they create camera subscribers in `__init__` that are never destroyed).
- Any step outcome other than `succeeded` (`failed`, `aborted`, `canceled`, exceptions) counts as a failure. The dispatcher says it is moving on and continues with the next step.
- Unknown skills now log an error and return `failed` instead of silently succeeding.
- `main()` calls `yasmin_ros.set_ros_loggers(node)`, needed by skills that use `yasmin_ros.logger_node`.
- New: `count_objects`, `count_people`, `find_object_by_property`.
- `answer_question` is a no-op (no LLM `Agent` dependency in the dispatcher).

Blackboard keys written by the dispatcher: `object_point`, `found_object`, `object_detections`, `count`, `gesture_locations`, `detections_3d`, `image_raw`, `attributes`.

## To test

- **Detection model:** the dispatcher uses `yolo11n-seg.pt` (COCO). Task objects like `red_bull` are never detected until a custom model is set (`self._yolo_model` in `DispatchSkill.__init__`). Class-name/alias/category matching only works for objects the model knows.
- `find_person` by gesture, name, clothes and fallback; confirm `/gesture/detect` is up (`lasr_vision_gesture` + yolo pose model) and that `standing` is a valid gesture (it is not in the gesture README).
- `get_person_info` and `count_people` with clothes (needs `/vlm/describe_people`; only `shirt_color` is matched; crop assumes 8-bit `rgb8`/`bgr8` images).
- `find_object`, `count_objects`, `find_object_by_property`: head motions (`look_centre`, `look_down_centre`) need the `play_motion2` server; in simulation they may block if it is absent.
- `find_object_by_property`: every object in `objects.yaml` is `small`, so heaviest/lightest is effectively decided by pixel area, which assumes similar distance from the camera.
- `guide_person` and `follow_person` end to end.
- Manipulation placeholders (`pick_up`, `place_object`, `give_to_person`) with the real arm.

## Important notes

- **Not run on hardware yet.** These changes were written by reading the `skills` / `common` APIs; nothing has run in the container or on the robot.
- Known issues in the `skills` package, deliberately not changed here: `Detect3D._create_req` returns the string `"failed"` on a 30 s timeout instead of a request (raises rather than failing cleanly), and `GoToLocation` creates a new `BasicNavigator` (a new node) on every call, which leaks over long plans.
- A TF failure inside the yolo service returns an empty response, which looks the same as "nothing seen".
- You can test the dispatcher by launching GPSR directly and running the dispatch node alone:
    - Edit `bb["steps"]` in `main()` of `GPSR/states/dispatch_skill.py` (commented example steps for each skill are there).
    - Then run `ros2 run GPSR dispatch_skill --ros-args --params-file tasks/GPSR/config/params.yaml`.
