# skills

Skills that can be used in tasks, or to build bigger skills

A skill is a state or a small state machine

## Current Skills

### DetectDoorbell

`lasr_skills.DetectDoorbell` – listens for a doorbell (or similar ringing/beeping sound) using
Google's [YAMNet](https://www.kaggle.com/models/google/yamnet) audio classifier.

**Outcomes:** `succeeded` (a matching sound was heard), `failed` (timeout, cancelled, or the
microphone service is unavailable)

**Blackboard:** none read or written

**Requires:** the [`microphone`](../common/microphone/README.md) node to be running
(`/microphone/record` service).

```python
from lasr_skills import DetectDoorbell

sm.add_state(
    "WAIT_FOR_DOORBELL",
    DetectDoorbell(),
    transitions={"succeeded": "ASK_OPEN_DOOR", "failed": "SAY_MISSED_DOORBELL"},
)
```

Run it on its own (with `ros2 run microphone mic` running):

```bash
ros2 run skills detect_doorbell
```

| Argument           | Default         | Description                                                        |
|--------------------|-----------------|--------------------------------------------------------------------|
| `score_threshold`  | `0.20`          | Minimum mean YAMNet score for the top class to count as a detection. |
| `included_classes` | doorbell-like classes (`Doorbell`, `Bell`, `Buzzer`, `Chime`, `Beep, bleep`, `Alarm clock`, ...) | YAMNet class names accepted as a doorbell. |

How it works:

1. On construction the YAMNet model is loaded from `~/.cache/kagglehub/models/google/yamnet/tensorFlow2/yamnet/1`,
   downloading it with `kagglehub` the first time. This takes a few seconds, so construct the state
   before the robot needs it (e.g. when building the state machine).
2. On execute it repeatedly requests 0.5 s of audio from `/microphone/record` (`fixed` mode) and keeps a
   rolling buffer of the latest 15600 samples (~0.975 s at 16 kHz, YAMNet's input window).
3. Each time the buffer is full, YAMNet runs on it. If the highest-scoring class is in
   `included_classes` and its score exceeds `score_threshold`, the state returns `succeeded`.
4. If nothing is detected within 15 s (`WAIT_FOR_DOORBELL_TIMEOUT`), it returns `failed`.

The included classes are deliberately broad because real doorbells are often classified as other
ringing/electronic sounds. If you get false positives (e.g. from music or speech), raise
`score_threshold` or narrow `included_classes`.

Python dependencies: `tensorflow==2.16.2`, `kagglehub` (see [requirements.txt](requirements.txt)).
TensorFlow is imported only when the state is constructed, so importing `lasr_skills` stays fast.
