"""Gestures that need several frames.

Waving uses the same features as the HRI gesture server (the elbow angle and
the wrist-shoulder distance, and how much each changes between frames) but in
normalised pixel units so it works without depth.
"""

from collections import deque
from typing import Callable, Deque, Dict, List, Optional

from .geometry import Person, angle, dist, shoulder_mid, shoulder_width

WINDOW = 12  # frames of history kept per person
MIN_HITS = 3  # frame-to-frame changes that must look like waving
DELTA_ANGLE = 0.10  # radians
DELTA_DIST = 0.15  # in shoulder widths (HRI used 0.25 in metres), tune on real footage
MIN_WRIST_ABOVE_ELBOW = 0.6  # fraction of the window
MAX_MATCH_DIST = 3.0  # shoulder widths a person may move between frames


class _Track:
    def __init__(self, centre, width):
        self.centre = centre
        self.width = width
        # per side: (elbow angle, wrist-shoulder distance / sw, wrist above elbow)
        self.history: Dict[str, Deque] = {
            "left": deque(maxlen=WINDOW),
            "right": deque(maxlen=WINDOW),
        }

    def add(self, kp: Person):
        for side, hist in self.history.items():
            shoulder = kp.get(f"{side}_shoulder")
            elbow = kp.get(f"{side}_elbow")
            wrist = kp.get(f"{side}_wrist")
            if shoulder is None or elbow is None or wrist is None:
                hist.clear()
                continue
            elbow_angle = angle(shoulder, elbow, wrist)
            if elbow_angle is None:
                hist.clear()
                continue
            hist.append(
                (elbow_angle, dist(wrist, shoulder) / self.width, wrist[1] < elbow[1])
            )

    def waving(self) -> bool:
        for hist in self.history.values():
            if len(hist) < MIN_HITS + 1:
                continue
            samples = list(hist)
            hits = sum(
                abs(b[0] - a[0]) >= DELTA_ANGLE and abs(b[1] - a[1]) >= DELTA_DIST
                for a, b in zip(samples, samples[1:])
            )
            above = sum(s[2] for s in samples) / len(samples)
            if hits >= MIN_HITS and above >= MIN_WRIST_ABOVE_ELBOW:
                return True
        return False


class WaveClassifier:
    """Feed one frame (list of people, YOLO order) at a time to ``update``."""

    def __init__(self):
        self._tracks: List[_Track] = []

    def reset(self):
        self._tracks = []

    def update(self, frame: List[Person]) -> List[int]:
        """Add a frame and return indices into it of the people waving now."""
        old = list(self._tracks)
        new_tracks: List[_Track] = []
        waving: List[int] = []
        for i, kp in enumerate(frame):
            centre, width = shoulder_mid(kp), shoulder_width(kp)
            if centre is None or width is None:
                # cannot normalise or match this person, so they cannot be tracked
                continue
            track = self._match(old, centre)
            if track is None:
                track = _Track(centre, width)
            track.centre, track.width = centre, width
            track.add(kp)
            new_tracks.append(track)
            if track.waving():
                waving.append(i)
        self._tracks = new_tracks
        return waving

    @staticmethod
    def _match(tracks: List[_Track], centre) -> Optional[_Track]:
        best, best_d = None, None
        for t in tracks:
            d = dist(t.centre, centre)
            if d <= MAX_MATCH_DIST * t.width and (best_d is None or d < best_d):
                best, best_d = t, d
        if best is not None:
            tracks.remove(best)
        return best


# Dynamic gestures: name -> factory for a fresh classifier
DYNAMIC: Dict[str, Callable[[], WaveClassifier]] = {"waving": WaveClassifier}
