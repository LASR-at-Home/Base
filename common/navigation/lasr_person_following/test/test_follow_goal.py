"""Unit tests for follow_manager's goal geometry: python3 -m pytest test/"""

import math

import pytest

pytest.importorskip("rclpy")
from lasr_person_following.follow_manager_node import drive_goal  # noqa: E402

FD, LA, MIN_D, MIN_MOVE = 1.0, 2.0, 0.8, 0.2


def goal(host, vel, robot=(0.0, 0.0)):
    return drive_goal(host, vel, robot, FD, LA, MIN_D, MIN_MOVE)


def test_standing_host_goal_is_follow_distance_short():
    gx, gy, yaw, _ = goal((3.0, 0.0), (0.0, 0.0))
    assert (gx, gy, yaw) == pytest.approx((2.0, 0.0, 0.0))


def test_fast_host_close_by_goal_never_past_or_beside_them():
    # host 1.2 m ahead walking away at 1.2 m/s: the old lookahead goal was ~1.4 m past them
    gx, gy, yaw, _ = goal((1.2, 0.0), (1.2, 0.3))
    assert gy == pytest.approx(0.0) and yaw == pytest.approx(0.0)  # on the line, facing
    assert 1.2 - gx == pytest.approx(MIN_D)  # stops min_distance short of the host


def test_lookahead_only_for_motion_away_from_robot():
    # walking across the robot's view: no lookahead, plain follow distance
    gx, gy, _, _ = goal((0.0, 3.0), (1.0, 0.0))
    assert (gx, gy) == pytest.approx((0.0, 2.0))
    # walking towards the robot: no lookahead either
    gx, gy, _, _ = goal((0.0, 3.0), (0.0, -1.0))
    assert (gx, gy) == pytest.approx((0.0, 2.0))
    gx, _, _, _ = goal((3.0, 0.0), (0.5, 0.0))
    assert gx == pytest.approx(2.2)  # 3.0 - 1.0 + 2 * 0.5, capped at 3.0 - 0.8


def test_close_enough_gives_no_goal():
    assert goal((1.1, 0.0), (0.0, 0.0)) is None
    assert goal((0.0, 0.0), (0.0, 0.0)) is None


def test_goal_faces_host_from_any_robot_pose():
    gx, gy, yaw, _ = goal((5.0, 5.0), (0.0, 0.0), robot=(2.0, 1.0))
    assert yaw == pytest.approx(math.atan2(4.0, 3.0))
    assert math.hypot(5.0 - gx, 5.0 - gy) == pytest.approx(FD)
