"""Gestures that can be decided from a single frame.

Each classifier takes one person (``{keypoint_name: (x, y)}`` in pixels) and
returns True if the person is making the gesture. If a keypoint the rule needs
is missing the answer is False. All distances are divided by the shoulder width
so the rules do not depend on how far away the person is.

"Left" and "right" are the person's own left and right, i.e. the ``left_*``
keypoints. For a person facing the camera their left is on the image right.
"""

import math
from typing import Callable, Dict

from .geometry import (
    Person,
    angle,
    hip_mid,
    mean_point,
    shoulder_mid,
    shoulder_width,
)

RAISE_MIN = 0.5  # wrist above shoulder by this many shoulder widths
POINT_MIN_ELBOW_ANGLE = math.radians(150)  # arm is (nearly) straight
POINT_MIN_REACH = 1.2  # wrist this many shoulder widths away sideways
POINT_MAX_HEIGHT_DIFF = 0.5  # wrist within this of shoulder height
SIT_MAX_THIGH_RATIO = 0.4  # vertical thigh length / torso length
STAND_MIN_THIGH_RATIO = 0.6  # between the two it is neither sitting nor standing
LYING_MAX_TORSO_ANGLE = math.radians(35)  # torso angle from horizontal


def _raising(side: str) -> Callable[[Person], bool]:
    def classify(kp: Person) -> bool:
        wrist, shoulder = kp.get(f"{side}_wrist"), kp.get(f"{side}_shoulder")
        sw = shoulder_width(kp)
        if wrist is None or shoulder is None or sw is None:
            return False
        return (shoulder[1] - wrist[1]) / sw > RAISE_MIN

    return classify


def _arm_pointing(kp: Person, side: str, direction: int, sw: float) -> bool:
    """direction: +1 if the wrist should be at larger image x than the shoulder."""
    shoulder = kp.get(f"{side}_shoulder")
    elbow = kp.get(f"{side}_elbow")
    wrist = kp.get(f"{side}_wrist")
    if shoulder is None or elbow is None or wrist is None:
        return False
    elbow_angle = angle(shoulder, elbow, wrist)
    if elbow_angle is None or elbow_angle < POINT_MIN_ELBOW_ANGLE:
        return False
    if abs(wrist[1] - shoulder[1]) / sw > POINT_MAX_HEIGHT_DIFF:
        return False
    return direction * (wrist[0] - shoulder[0]) / sw > POINT_MIN_REACH


def _pointing(direction: int) -> Callable[[Person], bool]:
    def classify(kp: Person) -> bool:
        sw = shoulder_width(kp)
        if sw is None:
            return False
        return any(_arm_pointing(kp, s, direction, sw) for s in ("left", "right"))

    return classify


def _torso_angle_from_horizontal(kp: Person):
    top, bottom = shoulder_mid(kp), hip_mid(kp)
    if top is None or bottom is None:
        return None, None
    dx, dy = bottom[0] - top[0], bottom[1] - top[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return None, None
    return math.atan2(abs(dy), abs(dx)), length


def lying_down(kp: Person) -> bool:
    torso_angle, _ = _torso_angle_from_horizontal(kp)
    return torso_angle is not None and torso_angle < LYING_MAX_TORSO_ANGLE


def _thigh_ratio(kp: Person):
    """Vertical hip-to-knee distance / torso length for an upright torso, else None."""
    torso_angle, torso_len = _torso_angle_from_horizontal(kp)
    knee = mean_point(kp, "left_knee", "right_knee")
    hip = hip_mid(kp)
    if torso_angle is None or knee is None or hip is None:
        return None
    if torso_angle < LYING_MAX_TORSO_ANGLE:
        return None
    return abs(knee[1] - hip[1]) / torso_len


def sitting(kp: Person) -> bool:
    """Knees at roughly hip height. Weak from a head-on 2D view, best effort."""
    ratio = _thigh_ratio(kp)
    return ratio is not None and ratio < SIT_MAX_THIGH_RATIO


def standing(kp: Person) -> bool:
    """Upright torso with the knees well below the hips. Best effort in 2D."""
    ratio = _thigh_ratio(kp)
    return ratio is not None and ratio > STAND_MIN_THIGH_RATIO


STATIC: Dict[str, Callable[[Person], bool]] = {
    "raising_left_arm": _raising("left"),
    "raising_right_arm": _raising("right"),
    "pointing_left": _pointing(+1),
    "pointing_right": _pointing(-1),
    "lying_down": lying_down,
    "sitting": sitting,
    "standing": standing,
}
