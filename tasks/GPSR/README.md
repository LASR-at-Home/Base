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

The GPSR dispatcher currently executes the following skills in the real ROS runtime:

| Skill | Status | Notes |
|------|------|------|
| `go_to_location` | yes | Uses Nav2 to drive to a mapped pose. |
| `say` | yes | Uses the TTS action. |
| `guide_person` | partial | Speaks a follow-me instruction and navigates to the destination. |
| `find_person` | partial | Goes to the target location, can find a generic person via face detection, can handle waving-person requests by trying wave detection and then prompting for a retry, can handle named-person requests by asking the named person to wave and then falling back to generic face detection, and can check shirt-color cues. |
| `get_person_info` | partial | Uses the vision-backed person description service for visible attributes. |
| `pick_up` | partial | Human-assisted pickup via the existing `ReceiveObject` sequence, including the built-in arm safety warning and handover prompt. |
| `place_object` | partial | Human-assisted placement request at the target location; still depends on `AskAndListen` to confirm completion. |
| `give_to_person` | partial | Uses the existing `HandoverObject` sequence for a nearby person. |
| `find_object` | no | Still not implemented. |
| `follow_person` | no | Not merged yet. |
| `count_objects` | no | Not implemented. |
| `count_people` | no | Not implemented. |
| `find_object_by_property` | no | 
| `answer_question` | no | No general QA beyond `say` yet. |

Notes:
- `partial` means the skill is wired into the real dispatcher but is still an approximation of the full GPSR semantics.
- `find_object` is still not actuated, so full fetch-and-deliver tasks remain incomplete.

## Suggested Testing

Recommended first tests:

1. `guide charlie from kitchen to living room`
2. `find the person waving in the kitchen`
3. `find adel in the kitchen`
4. `describe the person in the office`

Optional follow-up tests if the arm / handover stack is ready:

1. `pick up the cola`
2. `give the cola to a person`

Avoid using these as first validation commands:

1. `bring the cola from the kitchen to the bedroom`
2. any task that requires `find_object`
3. full three-command / interleaved GPSR batches

Those still depend on behavior that is either not actuated yet or not robustly supported in this branch.
