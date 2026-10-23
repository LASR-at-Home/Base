from .dynamic_gestures import DYNAMIC, WaveClassifier
from .static_gestures import STATIC

ALL_GESTURES = sorted(set(STATIC) | set(DYNAMIC))

__all__ = ["ALL_GESTURES", "DYNAMIC", "STATIC", "WaveClassifier"]
