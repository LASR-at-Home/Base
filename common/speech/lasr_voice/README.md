# lasr_voice

The lasr_voice package

This package is maintained by:
- [elisabeth](mailto:elisabeth@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- rospy (build)
- rospy (exec)
- actionlib (build)
- actionlib_msgs (build)
- pal_interaction_msgs (build)
- actionlib (exec)
- actionlib_msgs (exec)
- pal_interaction_msgs (exec)
- sound_play (test)

The `sound_play` package must be available to run the `sound_play_tts` node!

## Usage

```python
from lasr_voice.voice import Voice

voice = Voice()

# play something to tts and wait for it to end
voice.sync_tts("hello world")

# play something to tts and let it run in the background
voice.async_tts("hello world")

# wait until we stop talking
while voice.is_running():
    print("still talking")
```

### In Simulation

TIAGo's TTS action server is not available while in simulation, so we need to roll our own.

- Avoid playing back TTS and just log actions:

  ```python
  rosrun lasr_voice log_tts_events
  ```

- Use `sound_play` as the backend for TTS:

  ```python
  rosrun sound_play soundplay_node.py &
  rosrun lasr_voice sound_play_tts
  ```

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.
