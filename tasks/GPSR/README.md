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

| Skill | Status | Notes | Tested | 
|------|------|------|------|
| `go_to_location` | yes | Uses Nav2 to drive to a mapped pose. | Both |
| `say` | yes | Uses the TTS action. | Both |
| `guide_person` | partial | Speaks a follow-me instruction and navigates to the destination. | No |
| `find_person` | yes/reimplement | Identifies people using, location, gesture or description. | No |
| `get_person_info` | partial | Uses the vision-backed person description service for visible attributes. | No |
| `pick_up` | yes/reimplement | Human-assisted pickup via the existing `ReceiveObject` sequence. | Both |
| `place_object` | reimplement | Human-assisted placement request at the target location; still depends on `AskAndListen` to confirm completion. | No |
| `give_to_person` | yes/reimplement | Uses the existing `HandoverObject` sequence for a nearby person. | Both |
| `find_object` | reimplement/improve | Uses `detect3d` with `LookToPoint` to find Item. However lacks robustness and may need to be reimplemented. | SIM ONLY |
| `follow_person` | yes | Utilises exisiting `FollowPerson` skill. | Both |
| `count_objects` | no | Not implemented. | No |
| `count_people` | no | Not implemented. | No | 
| `find_object_by_property` | no | Not implemented. | No |
| `answer_question` | yes | Utilises `say` skill | Both |

*All skills needs to be retested regardless!*

Notes:
- `partial` means the skill is wired into the real dispatcher but is still an approximation of the full GPSR semantics.
- Multiple Skills contain placeholder code/ needs to be replaced. 
- Current `Manipulation` based skills current contain placeholders of asking the human to do the actual manipulation.

- **You can test the dispatcher by launching GPSR directly and just running the dispatch node directly** 
    - Edit the `bb["steps"]` in the `dispatch_skills.py` main method to contain the skill you want to test.  
    - Then run`ros run GPSR dispatch_skill`
