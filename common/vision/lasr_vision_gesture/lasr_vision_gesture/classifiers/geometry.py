"""Small 2D geometry helpers shared by the gesture classifiers.

A person is a dict ``{keypoint_name: (x, y)}`` in image pixels (y points down).
A missing key means the keypoint was not detected.
"""

import math
from typing import Dict, Optional, Tuple

Point = Tuple[float, float]
Person = Dict[str, Point]

TORSO = ("left_shoulder", "right_shoulder", "left_hip", "right_hip")


def has(kp: Person, *names: str) -> bool:
    return all(n in kp for n in names)


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def mean_point(kp: Person, *names: str) -> Optional[Point]:
    """Mean of whichever of the named keypoints are present, or None."""
    pts = [kp[n] for n in names if n in kp]
    if not pts:
        return None
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def shoulder_mid(kp: Person) -> Optional[Point]:
    return mean_point(kp, "left_shoulder", "right_shoulder")


def hip_mid(kp: Person) -> Optional[Point]:
    return mean_point(kp, "left_hip", "right_hip")


def shoulder_width(kp: Person) -> Optional[float]:
    """Pixel distance between the shoulders, used to normalise all distances."""
    if not has(kp, "left_shoulder", "right_shoulder"):
        return None
    width = dist(kp["left_shoulder"], kp["right_shoulder"])
    return width if width > 1.0 else None


def angle(a: Point, b: Point, c: Point) -> Optional[float]:
    """Angle at b between a-b and c-b, in radians (pi = straight line)."""
    v1 = (a[0] - b[0], a[1] - b[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 == 0 or n2 == 0:
        return None
    cos = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
    return math.acos(max(-1.0, min(1.0, cos)))


def person_location(kp: Person) -> Optional[Point]:
    """Where to report a person: shoulder midpoint, else torso, else anything."""
    return shoulder_mid(kp) or mean_point(kp, *TORSO) or mean_point(kp, *kp.keys())
