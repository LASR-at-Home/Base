"""
Multi-person tracker and host (target) selection. Pure Python + numpy/scipy, no ROS, so it
can be unit tested on its own.

All positions are 2D in a fixed frame (the node uses `odom`, which does not jump when AMCL
corrects the map pose). Times are seconds (any monotonic clock).

    MultiTracker   constant-velocity Kalman filter per person, gated Hungarian association
                   (position + appearance), track lifecycle tentative -> confirmed -> deleted
    HostTracker    which track is the host: locking, appearance model, re-identification
                   after the host's track is lost, and a "stationary" decision
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple
import itertools
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

# 99% gate for a 2-DoF Mahalanobis distance
CHI2_2DOF_99 = 9.21


@dataclass
class Detection:
    """One person seen in one frame."""

    x: float
    y: float
    stamp: float
    confidence: float = 1.0
    # L1-normalised appearance histogram (e.g. torso HSV), or None if unavailable
    appearance: Optional[np.ndarray] = None
    # measurement std (m); grows with range for a depth camera
    sigma: float = 0.15


def appearance_similarity(a: Optional[np.ndarray], b: Optional[np.ndarray]) -> float:
    """Bhattacharyya coefficient in [0, 1] (1 = identical). 0.5 (neutral) if unknown."""
    if a is None or b is None:
        return 0.5
    return float(np.sum(np.sqrt(np.clip(a, 0, None) * np.clip(b, 0, None))))


class TrackState(Enum):
    TENTATIVE = 0
    CONFIRMED = 1


class Track:
    """Constant-velocity Kalman filter, state [x, y, vx, vy]."""

    _ids = itertools.count(1)

    def __init__(self, det: Detection, accel_std: float, init_vel_std: float):
        self.id = next(Track._ids)
        self.x = np.array([det.x, det.y, 0.0, 0.0])
        self.P = np.diag([det.sigma**2, det.sigma**2, init_vel_std**2, init_vel_std**2])
        self.accel_std = accel_std
        self.stamp = det.stamp  # time of the state estimate
        self.last_update = det.stamp
        self.first_seen = det.stamp
        self.hits = 1
        self.state = TrackState.TENTATIVE
        self.appearance = None if det.appearance is None else det.appearance.copy()

    # ---- Kalman ----

    def predict(self, t: float):
        dt = t - self.stamp
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.accel_std**2
        # discrete white-noise acceleration model
        G = np.array([[0.5 * dt**2, 0], [0, 0.5 * dt**2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * q
        self.stamp = t

    def innovation(self, det: Detection) -> Tuple[np.ndarray, np.ndarray]:
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        y = np.array([det.x, det.y]) - H @ self.x
        S = H @ self.P @ H.T + np.eye(2) * det.sigma**2
        return y, S

    def mahalanobis(self, det: Detection) -> float:
        y, S = self.innovation(det)
        return float(y @ np.linalg.solve(S, y))

    def update(self, det: Detection, appearance_rate: float = 0.1):
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        y, S = self.innovation(det)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.last_update = det.stamp
        self.hits += 1
        if det.appearance is not None:
            if self.appearance is None:
                self.appearance = det.appearance.copy()
            else:
                self.appearance = (1 - appearance_rate) * self.appearance + appearance_rate * det.appearance
                self.appearance /= max(self.appearance.sum(), 1e-9)

    # ---- accessors ----

    @property
    def position(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self.x[2]), float(self.x[3])

    @property
    def speed(self) -> float:
        return math.hypot(self.x[2], self.x[3])

    def position_std(self) -> float:
        return math.sqrt(max(self.P[0, 0], self.P[1, 1]))


@dataclass
class TrackerParams:
    accel_std: float = 0.7  # m/s^2, how quickly people change velocity (noise vs lag)
    init_vel_std: float = 1.0
    gate_chi2: float = CHI2_2DOF_99
    max_gate_distance: float = 1.5  # hard cap on association distance (m)
    appearance_weight: float = 4.0  # cost per unit of (1 - similarity)
    # Second gate: a detection outside the Mahalanobis gate but within this distance is
    # still associated if it clearly looks like the track (fast camera motion can throw
    # a detection sideways by more than the noise model allows)
    appearance_gate_distance: float = 1.5
    appearance_gate_similarity: float = 0.7
    # An established track may not take a detection that clearly looks like someone else.
    appearance_veto: float = 0.3  # similarity below this is never associated
    appearance_veto_min_hits: int = 5
    # ...unless the detection is right where the track is predicted: the same person's
    # appearance can change abruptly (camera height or crop changes, turning round), and
    # nobody else can be standing on the same spot
    appearance_veto_free_distance: float = 0.4  # m
    confirm_hits: int = 3
    tentative_timeout: float = 0.6  # s without update before a tentative track is dropped
    confirmed_timeout: float = 3.0  # s a confirmed track coasts before deletion
    max_speed: float = 2.0  # clamp on estimated speed (m/s)
    appearance_rate: float = 0.1


class MultiTracker:
    def __init__(self, params: TrackerParams = TrackerParams()):
        self.p = params
        self.tracks: List[Track] = []

    def predict(self, t: float):
        for tr in self.tracks:
            tr.predict(t)

    def step(self, detections: List[Detection], t: float) -> List[Tuple[Track, Detection]]:
        """Predict to `t`, associate, update, manage lifecycle. Returns matched pairs."""
        self.predict(t)
        matches = []
        unmatched_dets = list(range(len(detections)))
        if self.tracks and detections:
            big = 1e6
            cost = np.full((len(self.tracks), len(detections)), big)
            for i, tr in enumerate(self.tracks):
                for j, det in enumerate(detections):
                    d2 = self._mahalanobis_capped(tr, det)
                    sim = appearance_similarity(tr.appearance, det.appearance)
                    if d2 is None:
                        if not self._appearance_gate(tr, det, sim):
                            continue
                        d2 = self.p.gate_chi2  # outside the gate: worst gated cost
                    if (
                        sim < self.p.appearance_veto
                        and tr.hits >= self.p.appearance_veto_min_hits
                        and math.hypot(det.x - tr.position[0], det.y - tr.position[1])
                        > self.p.appearance_veto_free_distance
                        and tr.appearance is not None
                        and det.appearance is not None
                    ):
                        continue
                    cost[i, j] = d2 + self.p.appearance_weight * (1.0 - sim)
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] >= big:
                    continue
                self.tracks[i].update(detections[j], self.p.appearance_rate)
                matches.append((self.tracks[i], detections[j]))
                unmatched_dets.remove(j)

        for j in unmatched_dets:
            self.tracks.append(Track(detections[j], self.p.accel_std, self.p.init_vel_std))

        for tr in self.tracks:
            if tr.state == TrackState.TENTATIVE and tr.hits >= self.p.confirm_hits:
                tr.state = TrackState.CONFIRMED
            # clamp unrealistic velocities (e.g. after a bad association)
            sp = tr.speed
            if sp > self.p.max_speed:
                tr.x[2:] *= self.p.max_speed / sp

        self.tracks = [tr for tr in self.tracks if not self._expired(tr, t)]
        return matches

    def _appearance_gate(self, tr: Track, det: Detection, sim: float) -> bool:
        if tr.state != TrackState.CONFIRMED or tr.appearance is None or det.appearance is None:
            return False
        px, py = tr.position
        return (sim >= self.p.appearance_gate_similarity
                and math.hypot(det.x - px, det.y - py)
                <= self.p.appearance_gate_distance + 2.0 * tr.position_std())

    def _mahalanobis_capped(self, tr: Track, det: Detection) -> Optional[float]:
        px, py = tr.position
        if math.hypot(det.x - px, det.y - py) > self.p.max_gate_distance + 2 * tr.position_std():
            return None
        d2 = tr.mahalanobis(det)
        return d2 if d2 <= self.p.gate_chi2 else None

    def _expired(self, tr: Track, t: float) -> bool:
        unseen = t - tr.last_update
        if tr.state == TrackState.TENTATIVE:
            return unseen > self.p.tentative_timeout
        return unseen > self.p.confirmed_timeout

    def get(self, track_id: int) -> Optional[Track]:
        for tr in self.tracks:
            if tr.id == track_id:
                return tr
        return None

    def confirmed(self) -> List[Track]:
        return [tr for tr in self.tracks if tr.state == TrackState.CONFIRMED]


class HostStatus(Enum):
    NO_HOST = "no_host"  # nothing locked yet
    TRACKING = "tracking"  # host seen within `visible_timeout`
    COASTING = "coasting"  # host's track alive but not seen recently (predicting)
    LOST = "lost"  # host's track gone; trying to re-identify


@dataclass
class HostParams:
    lock_max_range: float = 3.0  # m, host must be this close to be locked
    lock_max_bearing: float = math.radians(35)  # and roughly in front
    visible_timeout: float = 0.8  # s without a detection before "coasting"
    reid_min_similarity: float = 0.75  # appearance needed to re-identify the host
    reid_margin: float = 0.08  # host candidate must beat the next best by this
    reid_walk_speed: float = 1.5  # m/s, how far the host may have walked while lost
    reid_base_radius: float = 1.5  # m
    reid_max_radius: float = 3.0  # m, never re-identify further than this from last seen
    # (further away -> the host is asked to come back instead of guessing)
    reid_predict_time: float = 2.0  # s of last velocity used to predict where they went
    appearance_rate: float = 0.05
    crowd_radius: float = 1.0  # don't learn appearance when others are this close
    stationary_speed: float = 0.15  # m/s
    stationary_time: float = 3.0  # s
    min_travel: float = 1.0  # m walked before "stationary" may trigger
    stationary_seen_within: float = 1.5  # s: must have been detected this recently


@dataclass
class HostEstimate:
    status: HostStatus
    track_id: Optional[int] = None
    position: Optional[Tuple[float, float]] = None
    velocity: Tuple[float, float] = (0.0, 0.0)
    position_std: float = 0.0
    last_seen: Optional[float] = None
    stationary: bool = False
    still_for: float = 0.0
    travelled: float = 0.0


class HostTracker:
    """Keeps track of which tracked person is the host."""

    def __init__(self, tracker: MultiTracker, params: HostParams = HostParams()):
        self.tracker = tracker
        self.p = params
        self.reset()

    def reset(self):
        self.host_id: Optional[int] = None
        self.appearance: Optional[np.ndarray] = None
        self.last_pos: Optional[Tuple[float, float]] = None
        self.last_vel: Tuple[float, float] = (0.0, 0.0)
        self.last_seen: Optional[float] = None
        self.lost_since: Optional[float] = None
        self.origin: Optional[Tuple[float, float]] = None
        self.travelled = 0.0
        self.moving_since_check: Optional[float] = None
        self.still_since: Optional[float] = None
        # tracks seen at the same time as the host: they are someone else, never re-ID them
        self.others: set = set()

    @property
    def locked(self) -> bool:
        return self.last_pos is not None

    # ---- locking ----

    def try_lock(self, robot_xy_yaw: Tuple[float, float, float]) -> bool:
        """Lock onto the confirmed track closest to the robot, in front and in range."""
        rx, ry, ryaw = robot_xy_yaw
        best, best_d = None, float("inf")
        for tr in self.tracker.confirmed():
            px, py = tr.position
            d = math.hypot(px - rx, py - ry)
            bearing = abs(_wrap(math.atan2(py - ry, px - rx) - ryaw))
            if d <= self.p.lock_max_range and bearing <= self.p.lock_max_bearing and d < best_d:
                best, best_d = tr, d
        if best is None:
            return False
        self.host_id = best.id
        self.appearance = None if best.appearance is None else best.appearance.copy()
        self.last_pos = best.position
        self.last_seen = best.last_update
        self.origin = best.position
        self.lost_since = None
        self.others = set()
        return True

    # ---- per-step update ----

    def update(self, t: float) -> HostEstimate:
        if not self.locked:
            return HostEstimate(HostStatus.NO_HOST)

        track = self.tracker.get(self.host_id) if self.host_id is not None else None
        if track is None:
            self.host_id = None
            if self.lost_since is None:
                self.lost_since = t
            track = self._reidentify(t)
            if track is not None:
                self.host_id = track.id
                self.lost_since = None

        if track is None:
            return HostEstimate(
                HostStatus.LOST,
                position=self.last_pos,
                velocity=self.last_vel,
                last_seen=self.last_seen,
                travelled=self.travelled,
            )

        # host's track exists
        if track.last_update > (self.last_seen or -1):
            self.last_seen = track.last_update
            self._learn_appearance(track)
            hx, hy = track.position
            for other in self.tracker.confirmed():
                # detected in the same camera frame as the host, somewhere else: definitely
                # a different person (the host's own new track after a glitch is not)
                if (other.id != track.id and other.last_update == track.last_update
                        and math.hypot(other.position[0] - hx, other.position[1] - hy) > 1.0):
                    self.others.add(other.id)
        else:
            # host briefly unseen: anyone detected meanwhile further away than the host
            # could have walked is someone else
            reach = 2.0 + 2.0 * max(0.0, t - track.last_update)  # generous: walking + noise
            hx, hy = track.position
            for other in self.tracker.confirmed():
                if (other.id != track.id and other.last_update > track.last_update
                        and math.hypot(other.position[0] - hx, other.position[1] - hy) > reach):
                    self.others.add(other.id)
        pos = track.position
        self.last_pos = pos
        self.last_vel = track.velocity
        if self.origin is not None:
            self.travelled = max(self.travelled, math.hypot(pos[0] - self.origin[0], pos[1] - self.origin[1]))

        visible = t - track.last_update <= self.p.visible_timeout
        stationary, still_for = self._stationary(track, t, visible)
        return HostEstimate(
            HostStatus.TRACKING if visible else HostStatus.COASTING,
            track_id=track.id,
            position=pos,
            velocity=track.velocity,
            position_std=track.position_std(),
            last_seen=self.last_seen,
            stationary=stationary,
            still_for=still_for,
            travelled=self.travelled,
        )

    def reset_travel(self, t: Optional[float] = None):
        """Start a new leg (e.g. after the host said we haven't arrived yet)."""
        self.origin = self.last_pos
        self.travelled = 0.0
        self.still_since = None

    # ---- internals ----

    def _learn_appearance(self, track: Track):
        if track.appearance is None:
            return
        # Only learn when nobody else is close: avoids absorbing a passer-by's colours.
        hx, hy = track.position
        for other in self.tracker.tracks:
            if other.id != track.id:
                ox, oy = other.position
                if math.hypot(ox - hx, oy - hy) < self.p.crowd_radius:
                    return
        if self.appearance is None:
            self.appearance = track.appearance.copy()
        else:
            r = self.p.appearance_rate
            self.appearance = (1 - r) * self.appearance + r * track.appearance
            self.appearance /= max(self.appearance.sum(), 1e-9)

    def _reidentify(self, t: float) -> Optional[Track]:
        """Pick a confirmed track that looks like the host and could plausibly be them.

        Deliberately conservative: when in doubt the host stays lost (and is asked to come
        back) rather than risk following someone else."""
        lost_for = t - (self.last_seen if self.last_seen is not None else t)
        radius = min(self.p.reid_max_radius,
                     self.p.reid_base_radius + self.p.reid_walk_speed * lost_for)
        # centre of the search: where they were heading (up to 2 s of their last velocity)
        horizon = min(lost_for, self.p.reid_predict_time)
        cx = self.last_pos[0] + self.last_vel[0] * horizon
        cy = self.last_pos[1] + self.last_vel[1] * horizon
        scored = []
        for tr in self.tracker.confirmed():
            if tr.id in self.others:
                continue  # was visible together with the host: someone else
            if tr.first_seen < (self.last_seen or 0) - 0.5 and tr.last_update < (self.last_seen or 0):
                continue  # an old track not updated since we lost the host: not them
            sim = appearance_similarity(self.appearance, tr.appearance)
            px, py = tr.position
            d = min(math.hypot(px - self.last_pos[0], py - self.last_pos[1]),
                    math.hypot(px - cx, py - cy))
            if d > radius:
                continue
            scored.append((sim, -d, tr))
        if not scored:
            return None
        scored.sort(key=lambda s: (s[0], s[1]), reverse=True)
        best_sim, _, best = scored[0]
        if self.appearance is not None:
            if best_sim < self.p.reid_min_similarity:
                return None
            if len(scored) > 1 and best_sim - scored[1][0] < self.p.reid_margin:
                return None  # ambiguous: two people look alike, wait
        elif len(scored) > 1:
            return None  # no appearance model and several candidates: wait
        return best

    def _stationary(self, track: Track, t: float, visible: bool) -> Tuple[bool, float]:
        # Only movement restarts the clock: a standing host often drops out of detection
        # for a moment (turning round, partly out of view), which must not reset it.
        if track.speed > self.p.stationary_speed:
            self.still_since = None
            return False, 0.0
        if self.still_since is None:
            self.still_since = t
        still_for = t - self.still_since
        recently_seen = t - track.last_update <= self.p.stationary_seen_within
        stationary = (still_for >= self.p.stationary_time and self.travelled >= self.p.min_travel
                      and recently_seen)
        return stationary, still_for


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))
