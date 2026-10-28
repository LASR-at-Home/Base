import math

from lasr_vision_gesture.classifiers import DYNAMIC, STATIC, WaveClassifier


def person(**overrides):
    """A front-facing standing person, arms down. Shoulder width = 100 px."""
    kp = {
        "left_shoulder": (350, 200),
        "right_shoulder": (250, 200),
        "left_elbow": (360, 300),
        "right_elbow": (240, 300),
        "left_wrist": (360, 400),
        "right_wrist": (240, 400),
        "left_hip": (330, 400),
        "right_hip": (270, 400),
        "left_knee": (330, 550),
        "right_knee": (270, 550),
    }
    kp.update(overrides)
    return kp


def test_neutral_person_is_only_standing():
    kp = person()
    for name, classify in STATIC.items():
        assert classify(kp) == (name == "standing"), name


def test_raising_arms():
    assert STATIC["raising_left_arm"](person(left_wrist=(360, 100)))
    assert not STATIC["raising_right_arm"](person(left_wrist=(360, 100)))
    assert STATIC["raising_right_arm"](person(right_wrist=(240, 100)))


def test_pointing():
    left = person(left_elbow=(500, 200), left_wrist=(650, 205))
    assert STATIC["pointing_left"](left)
    assert not STATIC["pointing_right"](left)
    right = person(right_elbow=(100, 200), right_wrist=(-50, 205))
    assert STATIC["pointing_right"](right)
    assert not STATIC["pointing_left"](right)


def test_pointing_needs_straight_arm():
    bent = person(left_elbow=(450, 300), left_wrist=(450, 200))
    assert not STATIC["pointing_left"](bent)


def test_lying_down_and_sitting():
    lying = person(
        left_hip=(550, 230), right_hip=(550, 270), left_knee=(700, 230),
        right_knee=(700, 270),
    )
    assert STATIC["lying_down"](lying)
    seated = person(left_knee=(330, 420), right_knee=(270, 420))
    assert STATIC["sitting"](seated)
    assert not STATIC["standing"](seated)
    assert not STATIC["sitting"](person())
    assert not STATIC["standing"](lying)


def test_missing_keypoints_cannot_decide():
    assert not STATIC["raising_left_arm"]({"left_wrist": (0, 0)})
    assert not STATIC["sitting"]({})
    assert not STATIC["standing"]({})


def _wave_frame(t):
    # wrist swings side to side above the elbow, elbow angle changes with it
    x = 350 + 90 * math.sin(t * 1.2)
    return [person(left_elbow=(380, 150), left_wrist=(x, 60 + 20 * math.cos(t * 1.2)))]


def test_waving_detected_over_frames():
    wave = DYNAMIC["waving"]()
    assert isinstance(wave, WaveClassifier)
    results = [wave.update(_wave_frame(t)) for t in range(15)]
    assert results[-1] == [0]


def test_static_arm_is_not_waving():
    wave = WaveClassifier()
    for _ in range(15):
        assert wave.update([person(left_wrist=(360, 100))]) == []
