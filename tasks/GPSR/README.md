# GPSR Task

General Purpose Service Robot task for RoboCup@Home. The robot takes a natural-language command (microphone or keyboard), plans a sequence of skills with a local LLM, and executes them.

```
command --> transcript check --> [mic: "Is that correct?"] --> LLM planner --> plan check --> dispatcher --> skills
```

There are three ways to run it:

| What | Runs | Use it to test |
|---|---|---|
| **Dispatcher** (`dispatch_skill`) | Fixed skill steps, no LLM | Individual skills |
| **Service** (`service`) | One command → plan → (optionally) execute | Planner + skills, one command at a time |
| **State machine** (`sm`) | The full competition task: 3 commands, then executes them | The whole task |

---

## 1. Setup

### Build

`GPSR` depends on code in other packages (`skills`, `lasr_vision_interfaces`, `lasr_vision_gesture`, ...). After pulling changes, rebuild those too, not just `GPSR`:

```bash
cd ~/LASR/ros2_ws
colcon build 
source install/setup.bash
```

GPSR reads the **installed** copy of `config/*.yaml`. With `--symlink-install` config edits apply straight away; without it, rebuild `GPSR` after every config change.

### Ollama models (once)

```bash
ollama serve
ollama pull gemma3:4b                                                     # VLM
ollama create gemma3-4b-8k -f <path>/Base/tasks/GPSR/external/Modelfile   # planner
```

### Robot

1. Start navigation with the map of the arena you are in (lab: `lasr_map`).
2. Localise the robot in RViz (*2D Pose Estimate*). Every skill navigates by the poses in `locations.yaml`, so bad localisation breaks everything.

### Parameters ([config/params.yaml](config/params.yaml))

| Parameter | Lab value | Meaning |
|---|---|---|
| `simulation` | `false` | `false` = robot TTS, `true` = gtts on the PC speaker |
| `input_mode` | `"keyboard"` | State machine only: `"keyboard"` or `"microphone"` (the service picks per call) |
| `skip_entrance` | `true` | Full task only: `true` skips door detection and `entrance_point` and starts at the instruction point. **Set `false` in the arena.** |
| `object_detection_confidence` | `0.5` | YOLO confidence for object skills. Lower it if objects are missed |
| `instruction_point` / `entrance_point` | | Poses used by the full task. `entrance_point` still has arena values |
| `use_cloud`, `llm_*`, `cloud_*` | | Local Ollama vs cloud LLM |

`input_mode` can be overridden per run without editing the file (see below).

---

## 2. Running

Keep these two running for everything below.

**Terminal A**: LLM:
```bash
ollama serve
```

**Terminal B**: YOLO, gesture service, VLM, Whisper, arm/head motions:
```bash
ros2 launch GPSR GPSR.launch.py
# optional: whisper_model:=medium.en   (more accurate, more GPU memory)
```

### 2.1 Dispatcher: test skills without the LLM

Runs the named cases from [config/dispatch_tests.yaml](config/dispatch_tests.yaml) (one or more per skill, written for the lab object layout).

*This only requires the GPSR to be launched.*

```bash
ros2 run GPSR dispatch_skill                       # list the cases
ros2 run GPSR dispatch_skill say nav               # start here
ros2 run GPSR dispatch_skill find_bottle count_drinks biggest_drink
ros2 run GPSR dispatch_skill pick_and_give pick_and_place
ros2 run GPSR dispatch_skill find_waving describe_person count_sitting guide follow   # needs a helper
ros2 run GPSR dispatch_skill --steps '[{"skill": "say", "args": {"text": "hello"}}]'
```

If a skill fails here, the problem is in the skill or its ROS service, not in the planner.

### 2.2 Service: one command through the planner

**Terminal C**: the service:
```bash
ros2 run GPSR service
```

**Terminal D**: trigger one command per call (`/gpsr/single_query`, type `lasr_helper_interfaces/srv/GPSRQuery`):
```bash
# Typed command (no microphone), plan only (robot does not move)
ros2 service call /gpsr/single_query lasr_helper_interfaces/srv/GPSRQuery \
  '{command: {data: "Go to the dining table"}, dispatch_plan: false}'

# Typed command, plan and execute
ros2 service call /gpsr/single_query lasr_helper_interfaces/srv/GPSRQuery \
  '{command: {data: "Bring me the water bottle"}, dispatch_plan: true}'

# No command: the robot listens on the microphone
ros2 service call /gpsr/single_query lasr_helper_interfaces/srv/GPSRQuery '{dispatch_plan: false}'
```

- **Typed command** (`command.data` not empty): used as if typed. There is no yes/no confirmation and no rephrase retries. If the command is rejected the call returns `success: false`.
- **Microphone** (`command.data` empty, the default): speak after *"I am ready for your command"*, then answer **yes** or **no** when the robot reads it back.

Start with `dispatch_plan: false` and check the planned steps in the log before letting the robot move.

`input_mode` in `params.yaml` is not used by the service, only by the state machine (2.3).

Alternatively, `ros2 launch GPSR service.launch.py` starts the service node together with Ollama, YOLO, Whisper and the motions (no gesture / VLM services, no `robot_ui`), so Terminal A and B are not needed.

### 2.3 State machine: the full task

**Terminal C**:
```bash
ros2 run GPSR sm --ros-args --params-file src/Base/tasks/GPSR/config/params.yaml
```

**Terminal D**: start the task:
```bash
ros2 topic pub /gpsr/start std_msgs/msg/Empty --once
```

The robot goes to the instruction point (after door detection and `entrance_point` when `skip_entrance: false`). It asks for **3 commands**, plans and reads back each one, then executes them all, returning to the instruction point after each.

### Example commands (lab layout)

| Skill | Command |
|---|---|
| `go_to_location` | Go to the dining table |
| `find_object` | Find the water bottle on the dining table |
| `count_objects` | How many drinks are on the dining table |
| `find_object_by_property` | What is the biggest drink on the dining table |
| `pick_up` + `give_to_person` | Bring me the water bottle |
| `place_object` | Take the paper cup from the computer desk and put it on the tv stand |
| `find_person` | Find the waving person in the living room |
| `get_person_info` | Describe the person at the sofa |
| `count_people` | How many sitting people are in the living room |
| `guide_person` | Guide Robin from the sofa to the study table |
| `follow_person` | Follow the person at the dining table |
| `say` | Tell me your team name |

Commands shorter than 3 words are rejected.

### What to look for in the logs

| Log line | Meaning |
|---|---|
| `Command rejected before planning: ...` | No command verb, or nothing known (location / object / person) mentioned |
| `Confirmation answer: '...'` | What Whisper heard for the yes/no |
| `Plan rejected: unknown location ...` | The LLM planned something that is not in the config |
| `=== PLAN [LOCAL] ...` + `[1] skill(args)` | The plan that will be executed |


## 3. Configuration

| File | Purpose |
|---|---|
| [params.yaml](config/params.yaml) | Runtime parameters (see above) |
| [locations.yaml](config/locations.yaml) | Rooms (have a `polygon`) and sub-locations (`room`, `placeable`) with navigation poses |
| [objects.yaml](config/objects.yaml) | Objects: `category`, default `location`, `size`, `aliases`, `detect_as` (YOLO classes the object is detected as) |
| [people.yaml](config/people.yaml) | Known people |
| [general_knowledge.yaml](config/general_knowledge.yaml) | Facts for answering questions |
| [skills.yaml](config/skills.yaml) | Skill signatures shown to the LLM |
| [dispatch_tests.yaml](config/dispatch_tests.yaml) | Dispatcher test cases |

Nothing about the arena is hardcoded: room membership, the planner's object list, the plan checks and the Whisper vocabulary are all built from these files.

### Changing arena

1. Update `locations.yaml` (poses, `room`, `placeable`, room polygons) and `objects.yaml` (`location` must be a key in `locations.yaml`).
2. Update `instruction_point` / `entrance_point` in `params.yaml` and set `skip_entrance`.
3. Regenerate the command-generator data:
   ```bash
   python3 scripts/export_command_generator_data.py
   ```

### Command generator

[command_generator_data/](command_generator_data/) holds the data for the [RoboCup@Home CommandGenerator](https://github.com/RoboCupAtHome/CommandGenerator), in the same layout as the competition repos and exported from the config files. See its [README](command_generator_data/README.md).

```bash
athome-generator -d command_generator_data          # interactive
athome-generator-gpsr-ui -d command_generator_data  # web UI
```

---

## 4. Skill mapping status

Implemented in [dispatch_skill.py](GPSR/states/dispatch_skill.py).

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
| `count_objects` | new | Goes to `location`, counts detections matching the object name / alias / category / `detect_as`, says the count. | No |
| `count_people` | new | Counts people by gesture, pose (via gesture service), clothes (VLM shirt colour per person crop) or all people. | No |
| `find_object_by_property` | new | biggest/smallest/thinnest from pixel bbox area, heaviest/lightest from `objects.yaml` `size` (tie-break by pixel area). | No |
| `answer_question` | no-op | Returns `succeeded`. The planner already adds a `say` step with the answer. | Both |

*All skills need to be retested.*

### Still to check

- **Object detection:** `yolo11n-seg.pt` is a COCO model. Objects that are not COCO classes (cans, tubes, boxes) are only found through `detect_as` and often score below the 0.5 confidence. Several objects can share a class (coke / pringles / paper cup → `cup`).
- `find_person` by gesture, name, clothes and fallback; `/gesture/detect` needs `lasr_vision_gesture` and the yolo pose model.
- `get_person_info` and `count_people` by clothes: only `shirt_color` is matched; the crop assumes `rgb8`/`bgr8` images.
- Object skills use the `look_centre` / `look_down_centre` head motions (needs `play_motion2`).
- `guide_person` and `follow_person` end to end; manipulation placeholders with the real arm.

---

## 5. Skill test log

Tick a skill once it works on the real robot ([config/dispatch_tests.yaml](config/dispatch_tests.yaml) has the test cases). Add notes as indented bullets.

- [x] `go_to_location`
- [x] `say`
- [x] `guide_person`
- [x] `find_person`
  - Gesture, name and location works but requires more. 
  - Add something such as approaching the person and looking at them to ensure it is clear we have found them 
- [ ] `get_person_info`
  - FAILS: 
    ```bash
      [INFO] [1795456871.874275692] [basic_navigator]: Navigating to goal: 2.4020402431488037 0.7907199263572693...
      [ERROR] [1795456872.981423729] [gpsr]: Failed to get camera image: wait_for_message() got an unexpected keyword argument 'timeout_sec'
      [INFO] [1795456872.982736736] [gpsr]: Saying: I cannot see you so can you please step infront of me. I will wait a few seconds

    ```
- [x] `pick_up`
  - Simple open-close gripper (need to add a distance check to ensure the hand out doesnt hit the table for manipulation)
- [x] `place_object`
  - same as `pick_up`
- [x] `give_to_person`
  - same as `pick_up`
  - Need to add a new `approach person` skill - Can also be used for HRI get back
- [x] `find_object`
  - Works but needs to be replaced with VLM (P&P version) for better detection. 
- [ ] `follow_person`
  - Untested but should have same success as HRI version as just wraps the skill call (integrate HRI version)
- [x] `count_objects`
  - Still need to test with multiple objects but can properly count them
- [x] `count_people`
- [ ] `find_object_by_property`
- [ ] `answer_question`

---

## 6. Implementation notes

**Command / plan checks** ([validation.py](GPSR/validation.py)): a command must contain a command verb and mention a known location, object, category, person or generic entity ("me", "person", ...). It is checked before planning. Plans are rejected if they use an unknown skill, location or object. Both lead to "could you rephrase". In microphone mode the robot also asks for a yes/no confirmation before planning.

**Dispatcher:**
- Object detection runs unfiltered (the yolo service detects every class if the filter contains an unknown name). Detections are matched against the target's name, aliases, category and `detect_as`.
- Any step outcome other than `succeeded` counts as a failure. The dispatcher says it is moving on and continues with the next step. Unknown skills fail.
- `Detect3D` instances are cached per filter: they create camera subscribers that are never destroyed.
- `_describe_person` calls `GetVlmAttributes` directly with the latest camera image (`DescribePeople` writes `img_msg` but the VLM state reads `image_raw`).
- Blackboard keys written: `object_point`, `found_object`, `object_detections`, `count`, `gesture_locations`, `detections_3d`, `image_raw`, `attributes`.

**yasmin:** the container builds yasmin from `main` (6.x at the time of writing). yasmin 6 rejects state machines with unreachable states, so optional states (e.g. the entrance sequence) are only added when used.

**Known issues in `skills` (not changed here):** `Detect3D._create_req` returns the string `"failed"` on a 30 s timeout instead of a request. `GoToLocation` creates a new `BasicNavigator` node on every call, which leaks over long plans. A TF failure inside the yolo service returns an empty response, which looks the same as "nothing seen".
