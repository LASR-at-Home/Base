"""Unit tests for the tracker core (no ROS needed): python3 -m pytest test/"""

import math

import numpy as np
import pytest

from lasr_person_following.tracking import (
    Detection,
    HostParams,
    HostStatus,
    HostTracker,
    MultiTracker,
    TrackerParams,
    appearance_similarity,
)

DT = 0.1  # 10 Hz detector


def hist(hot: int, n: int = 16, spread: float = 0.15) -> np.ndarray:
    """A fake appearance histogram peaked at bin `hot`."""
    h = np.full(n, spread / n)
    h[hot] += 1.0
    return h / h.sum()


HOST_LOOK = hist(3)
OTHER_LOOK = hist(11)


def walk(start, velocity, t):
    return start[0] + velocity[0] * t, start[1] + velocity[1] * t


def make(seed=0):
    rng = np.random.default_rng(seed)
    tracker = MultiTracker(TrackerParams())
    host = HostTracker(tracker, HostParams())
    return rng, tracker, host


def det(rng, xy, t, look, noise=0.1):
    return Detection(
        x=xy[0] + rng.normal(0, noise), y=xy[1] + rng.normal(0, noise), stamp=t,
        appearance=look, sigma=0.15,
    )


def lock_on(tracker, host, rng, xy=(1.5, 0.0), look=HOST_LOOK, robot=(0.0, 0.0, 0.0)):
    t = 0.0
    for _ in range(5):
        tracker.step([det(rng, xy, t, look)], t)
        t += DT
    assert host.try_lock(robot)
    return t


def test_appearance_similarity():
    assert appearance_similarity(HOST_LOOK, HOST_LOOK) == pytest.approx(1.0)
    assert appearance_similarity(HOST_LOOK, OTHER_LOOK) < 0.5
    assert appearance_similarity(None, HOST_LOOK) == 0.5


def test_velocity_converges():
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    v = (0.8, 0.3)
    start = (1.5, 0.0)
    t0 = t
    for _ in range(40):
        tracker.step([det(rng, walk(start, v, t - t0), t, HOST_LOOK)], t)
        t += DT
    est = host.update(t)
    assert est.status in (HostStatus.TRACKING, HostStatus.COASTING)
    assert est.velocity[0] == pytest.approx(0.8, abs=0.2)
    assert est.velocity[1] == pytest.approx(0.3, abs=0.2)


def test_lock_picks_person_in_front_not_behind():
    rng, tracker, host = make()
    t = 0.0
    for _ in range(5):
        tracker.step([det(rng, (1.5, 0.2), t, HOST_LOOK), det(rng, (-1.0, 0.0), t, OTHER_LOOK)], t)
        t += DT
    assert host.try_lock((0.0, 0.0, 0.0))
    est = host.update(t)
    assert est.position[0] > 1.0


def test_coasts_through_short_occlusion():
    """2 s without detections while walking: stays the same track, prediction follows."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    v, start, t0 = (0.6, 0.0), (1.5, 0.0), t
    for _ in range(30):
        tracker.step([det(rng, walk(start, v, t - t0), t, HOST_LOOK)], t)
        t += DT
    host_id = host.update(t).track_id
    for _ in range(20):  # occluded
        tracker.step([], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.COASTING
    truth = walk(start, v, t - DT - t0)
    assert math.hypot(est.position[0] - truth[0], est.position[1] - truth[1]) < 0.6
    for _ in range(5):  # reappears
        tracker.step([det(rng, walk(start, v, t - t0), t, HOST_LOOK)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.TRACKING
    assert est.track_id == host_id


def test_crossing_distractor_no_id_switch():
    """Another person walks across the host's path, between them and the robot."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng, xy=(2.0, 0.0))
    t0 = t
    host_v, host_start = (0.5, 0.0), (2.0, 0.0)
    other_v, other_start = (0.0, -0.8), (3.0, 2.4)  # crosses the host's path at t~3s
    on_host = 0
    n = 60
    for _ in range(n):
        hp = walk(host_start, host_v, t - t0)
        op = walk(other_start, other_v, t - t0)
        dets = [det(rng, op, t, OTHER_LOOK)]
        # host is occluded by the distractor for a few frames when they overlap
        if math.hypot(hp[0] - op[0], hp[1] - op[1]) > 0.5:
            dets.append(det(rng, hp, t, HOST_LOOK))
        tracker.step(dets, t)
        est = host.update(t)
        if est.position is not None and math.hypot(est.position[0] - hp[0], est.position[1] - hp[1]) < 0.6:
            on_host += 1
        t += DT
    assert on_host / n > 0.9
    final = walk(host_start, host_v, t - DT - t0)
    assert math.hypot(est.position[0] - final[0], est.position[1] - final[1]) < 0.5


def test_passing_distractor_same_look_far_from_prediction_is_rejected():
    """Host lost for a moment; a different-looking person near the old spot is NOT adopted."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    for _ in range(40):  # learn appearance
        tracker.step([det(rng, (1.5, 0.0), t, HOST_LOOK)], t)
        host.update(t)
        t += DT
    for _ in range(40):  # host gone for 4 s (track deleted), stranger appears nearby
        tracker.step([det(rng, (2.0, 0.5), t, OTHER_LOOK)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.LOST


def test_reidentify_host_after_loss():
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    for _ in range(40):
        tracker.step([det(rng, (1.5, 0.0), t, HOST_LOOK)], t)
        host.update(t)
        t += DT
    for _ in range(40):  # gone for 4 s
        tracker.step([], t)
        host.update(t)
        t += DT
    assert host.update(t).status == HostStatus.LOST
    # host comes back 1.5 m to the side, a stranger appears too
    for _ in range(10):
        tracker.step([det(rng, (2.0, 1.5), t, HOST_LOOK), det(rng, (2.0, -1.0), t, OTHER_LOOK)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.TRACKING
    assert est.position[1] > 1.0


def test_stationary_only_after_walking_and_waiting():
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    # standing still at the start must not count (has not travelled)
    for _ in range(50):
        tracker.step([det(rng, (1.5, 0.0), t, HOST_LOOK)], t)
        est = host.update(t)
        t += DT
    assert not est.stationary
    # walk 3 m, then stop
    t0 = t
    for _ in range(50):
        tracker.step([det(rng, walk((1.5, 0.0), (0.6, 0.0), t - t0), t, HOST_LOOK)], t)
        est = host.update(t)
        t += DT
    assert not est.stationary
    stop = walk((1.5, 0.0), (0.6, 0.0), t - t0)
    first_stationary = None
    for i in range(80):
        tracker.step([det(rng, stop, t, HOST_LOOK, noise=0.05)], t)
        est = host.update(t)
        if est.stationary and first_stationary is None:
            first_stationary = i * DT
        t += DT
    assert first_stationary is not None
    assert first_stationary < 5.0  # stationary_time 3 s + filter settling


def test_short_pause_is_not_stationary():
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    t0 = t
    for _ in range(40):
        tracker.step([det(rng, walk((1.5, 0.0), (0.6, 0.0), t - t0), t, HOST_LOOK)], t)
        est = host.update(t)
        t += DT
    stop = walk((1.5, 0.0), (0.6, 0.0), t - t0)
    for _ in range(15):  # 1.5 s pause
        tracker.step([det(rng, stop, t, HOST_LOOK, noise=0.05)], t)
        est = host.update(t)
        assert not est.stationary
        t += DT


def test_identical_bystander_seen_with_host_is_never_reidentified():
    """A person who looks exactly like the host (same clothes) but was visible at the same
    time as the host must not be taken for the host when the host disappears."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    twin = (3.0, 3.0)
    for _ in range(30):  # host and twin both visible
        tracker.step([det(rng, (1.5, 0.0), t, HOST_LOOK), det(rng, twin, t, HOST_LOOK)], t)
        host.update(t)
        t += DT
    for _ in range(60):  # host gone for 6 s, twin still visible
        tracker.step([det(rng, twin, t, HOST_LOOK)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.LOST


def test_detection_thrown_sideways_keeps_host_track():
    """Fast camera motion can throw a detection ~1 m sideways for a few frames: if it
    still looks like the host it must stay on the host's track."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    for _ in range(30):
        tracker.step([det(rng, (2.0, 0.0), t, HOST_LOOK, noise=0.03)], t)
        host_id = host.update(t).track_id
        t += DT
    for _ in range(3):  # thrown 0.9 m sideways
        tracker.step([det(rng, (2.0, 0.9), t, HOST_LOOK, noise=0.03)], t)
        t += DT
    for _ in range(10):
        tracker.step([det(rng, (2.0, 0.0), t, HOST_LOOK, noise=0.03)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.TRACKING
    assert est.track_id == host_id


def test_reidentify_host_who_walked_on_past_the_robot():
    """Host walks past the robot out of view, then is seen again further along their
    direction of travel, more than the base radius from where they were last seen."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    t0 = t
    for _ in range(30):  # walking at 0.8 m/s along -x (towards and past the robot)
        tracker.step([det(rng, walk((1.5, 0.5), (-0.8, 0.0), t - t0), t, HOST_LOOK, noise=0.03)], t)
        host.update(t)
        t += DT
    for _ in range(40):  # out of view for 4 s
        tracker.step([], t)
        host.update(t)
        t += DT
    for _ in range(10):  # seen again further along
        tracker.step([det(rng, walk((1.5, 0.5), (-0.8, 0.0), t - t0), t, HOST_LOOK, noise=0.03)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.TRACKING


def test_weak_or_far_lookalikes_are_not_taken_after_a_loss():
    """After a loss, neither a static twin nor a weaker-matching walker further away than
    the host could plausibly be is taken: the host stays lost (and is asked to come back)
    rather than risk following someone else."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    for _ in range(40):
        tracker.step([det(rng, (1.5, 0.0), t, HOST_LOOK, noise=0.03)], t)
        host.update(t)
        t += DT
    for _ in range(40):  # gone 4 s
        tracker.step([], t)
        host.update(t)
        t += DT
    seen_from_front = 0.5 * HOST_LOOK + 0.5 * OTHER_LOOK  # weaker match (~0.7)
    static_twin = (4.0, 3.0)
    t0 = t
    for _ in range(15):  # twin standing still; host walking away 4 m off
        tracker.step([det(rng, static_twin, t, HOST_LOOK, noise=0.03),
                      det(rng, walk((5.0, -1.0), (0.6, 0.0), t - t0), t, seen_from_front, noise=0.03)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.LOST


def test_host_reappearing_as_new_track_is_not_marked_as_someone_else():
    """A glitch makes the host's detection start a second track for a moment while the
    host's track is still updated; once the old track dies, the new one is the host."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    for _ in range(30):
        tracker.step([det(rng, (2.0, 0.0), t, HOST_LOOK, noise=0.03)], t)
        host.update(t)
        t += DT
    for _ in range(15):  # host walks past the robot, out of view for 1.5 s
        tracker.step([], t)
        host.update(t)
        t += DT
    t0 = t
    for i in range(60):  # seen again 1.5 m further on, walking away
        pos = walk((3.5, 0.0), (0.6, 0.0), t - t0)
        tracker.step([det(rng, pos, t, HOST_LOOK, noise=0.03)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.TRACKING
    assert est.position[0] > 3.5


def test_stationary_survives_detection_gaps():
    """A host standing still but detected only intermittently is still declared stationary."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    t0 = t
    for _ in range(40):  # walk 2.4 m
        tracker.step([det(rng, walk((1.5, 0.0), (0.6, 0.0), t - t0), t, HOST_LOOK)], t)
        host.update(t)
        t += DT
    stop = walk((1.5, 0.0), (0.6, 0.0), t - t0)
    first = None
    for i in range(100):  # standing; seen in only 1 of every 10 frames (1 s gaps)
        tracker.step([det(rng, stop, t, HOST_LOOK, noise=0.03)] if i % 10 == 0 else [], t)
        est = host.update(t)
        if est.stationary and first is None:
            first = i * DT
        t += DT
    assert first is not None and first < 7.0


def test_lookalike_seen_far_away_while_host_unseen_is_never_reidentified():
    """While the host is briefly unseen, an identical-looking person appears far further
    away than the host could have walked: they must never become the host."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    for _ in range(30):
        tracker.step([det(rng, (2.0, 0.0), t, HOST_LOOK, noise=0.03)], t)
        host.update(t)
        t += DT
    twin = (2.5, 4.5)
    for _ in range(80):  # host gone; the twin stands 4.5 m away, jittering a little
        tracker.step([det(rng, twin, t, HOST_LOOK, noise=0.08)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.LOST


def test_sudden_appearance_change_on_the_spot_keeps_host_track():
    """The host's appearance changes abruptly (e.g. the torso moved the camera and the
    crop now lands elsewhere) while they stand where predicted: same track, still host."""
    rng, tracker, host = make()
    t = lock_on(tracker, host, rng)
    for _ in range(30):
        tracker.step([det(rng, (1.5, 0.0), t, HOST_LOOK, noise=0.03)], t)
        host.update(t)
        t += DT
    host_id = host.host_id
    for _ in range(60):
        tracker.step([det(rng, (1.5, 0.0), t, OTHER_LOOK, noise=0.03)], t)
        est = host.update(t)
        t += DT
    assert est.status == HostStatus.TRACKING
    assert est.track_id == host_id
