# GPSR Task

General Purpose Service Robot task for RoboCup@Home. The robot listens to a natural language command, plans a sequence of skills via LLM, and executes them.

## Quick start — planner test (simulation, keyboard input)

### 1. Set params

Make sure [config/params.yaml](config/params.yaml) has:

```yaml
gpsr:
  ros__parameters:
    simulation: true
    input_mode: "keyboard"
```

### 2. Start Ollama (first run, if you've already done you don't need to rerun these commands)

```bash
ollama serve
ollama pull gemma3   # only first time
```

### 3. Build and run

```bash
# from workspace root (Base/)
colcon build --packages-select GPSR
source install/setup.bash

ros2 run GPSR sm --ros-args --params-file tasks/GPSR/config/params.yaml
```

Type your command when prompted, e.g.:

```
> Go to the kitchen and bring me a bottle of water
```

The robot will print and speak the planned steps.

---

## Full simulation (with Nav2)

Launch Nav2 and the GPSR node in two separate terminals.

**Terminal 1 — Nav2:**
```bash
ros2 launch simulation nav.launch.py
```

**Terminal 2 — GPSR:**
```bash
ros2 run GPSR sm --ros-args --params-file tasks/GPSR/config/params.yaml
```

Set `simulation: false` in `params.yaml` to use the robot TTS instead of gtts. (only for real robot)

---

## Config files

| File | Purpose |
|------|---------|
| `config/params.yaml` | Runtime parameters (LLM host, input mode, simulation flag) |
| `config/skills.yaml` | Available skill signatures shown to the LLM |
| `config/locations.yaml` | Named 3D poses for navigation |
| `config/objects.yaml` | Object inventory (category, location, size) |
| `config/people.yaml` | Known people identities |
| `config/general_knowledge.yaml` | Arena context for the LLM |

