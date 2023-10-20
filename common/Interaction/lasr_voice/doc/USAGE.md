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
