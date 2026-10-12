from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from statistics import median

from .datadealer_loader import CombinedFrame, Point3D


class TargetState:
    TRACKING = "tracking"
    STATIC_LOCK = "static_lock"
    HOLD = "hold"
    REACQUIRE = "reacquire"
    LOST = "lost"


@dataclass(frozen=True)
class TrackerConfig:
    x_min_m: float = -1.5
    x_max_m: float = 1.5
    y_min_m: float = 0.15
    y_max_m: float = 3.0
    z_min_m: float = -0.8
    z_max_m: float = 1.7
    min_snr_db: float = 15.0
    cluster_radius_m: float = 0.45
    min_cluster_points: int = 3
    max_cluster_width_x_m: float = 0.5
    max_cluster_depth_y_m: float = 0.5
    track_gate_m: float = 0.5
    reacquire_gate_m: float = 1.0
    switch_confirm_frames: int = 5
    hold_ms: int = 3000


@dataclass(frozen=True)
class TargetFrame:
    frame_index: int
    timestamp_ms: int
    timestamp_text: str
    has_target: bool
    center_x_m: float | None = None
    center_y_m: float | None = None
    center_z_m: float | None = None
    z_min_m: float | None = None
    z_max_m: float | None = None
    height_m: float = 0.0
    width_x_m: float = 0.0
    depth_y_m: float = 0.0
    point_count: int = 0
    motion_count: int = 0
    max_abs_velocity_m_s: float = 0.0
    held: bool = False
    state: str = TargetState.TRACKING

    @property
    def low_posture(self) -> bool:
        return self.has_target and self.center_z_m is not None and self.center_z_m <= 0.75


@dataclass(frozen=True)
class _TrackingStep:
    target: TargetFrame
    selected_cluster: list[Point3D] | None


@dataclass(frozen=True)
class _ClusterSelection:
    cluster: list[Point3D] | None
    freeze_xy: bool = False


@dataclass
class _PendingSwitch:
    center_x_m: float
    center_y_m: float
    frames: int
    cluster: list[Point3D]


class _TrackingRuntime:
    def __init__(self, config: TrackerConfig):
        self.config = config
        self.last_target: TargetFrame | None = None
        self.last_target_time_ms: int | None = None
        self.pending_switch: _PendingSwitch | None = None

    def step(self, frame: CombinedFrame, candidates: list[list[Point3D]]) -> _TrackingStep:
        selection = self._select_cluster(frame, candidates)
        if selection.cluster is not None:
            if selection.freeze_xy and self.last_target is not None:
                target = _target_from_static_core(frame, selection.cluster, self.last_target)
                selected_cluster = None
            else:
                target = _target_from_cluster(frame, selection.cluster, held=False)
                selected_cluster = selection.cluster
            self.last_target = target
            self.last_target_time_ms = frame.timestamp_ms
            self.pending_switch = None
            return _TrackingStep(target=target, selected_cluster=selected_cluster)

        if self._can_hold(frame) and self.last_target is not None:
            return _TrackingStep(target=_held_target_from_last(frame, self.last_target), selected_cluster=None)

        return _TrackingStep(
            target=TargetFrame(
                frame_index=frame.frame_index,
                timestamp_ms=frame.timestamp_ms,
                timestamp_text=frame.timestamp_text,
                has_target=False,
                state=TargetState.REACQUIRE if self.pending_switch is not None else TargetState.LOST,
            ),
            selected_cluster=None,
        )

    def _select_cluster(self, frame: CombinedFrame, candidates: list[list[Point3D]]) -> _ClusterSelection:
        if not candidates:
            self.pending_switch = None
            return _ClusterSelection(None)

        if self.last_target is None or not self.last_target.has_target or self.last_target_time_ms is None:
            self.pending_switch = None
            return _ClusterSelection(_select_target_cluster(candidates, None))

        if _looks_like_static_floor_clutter(self.last_target):
            escape_candidate = _select_static_floor_escape_candidate(candidates, self.last_target, self.config)
            if escape_candidate is not None:
                self.pending_switch = None
                return _ClusterSelection(escape_candidate)

        near_candidates = _clusters_within_gate(candidates, self.last_target, self.config.track_gate_m)
        if near_candidates:
            self.pending_switch = None
            scored_cluster = _select_scored_near_target_cluster(near_candidates, self.last_target, self.config)
            cluster = _select_locked_target_cluster(near_candidates, self.last_target)
            if scored_cluster is not cluster:
                if not _should_keep_locked_cluster_over_scored(cluster, scored_cluster, self.last_target, self.config):
                    return _ClusterSelection(scored_cluster)
            core = _locked_cluster_core(cluster, self.last_target, self.config)
            if core is None:
                return _ClusterSelection(None)
            if _should_use_full_locked_cluster(cluster, core, self.config):
                return _ClusterSelection(cluster)
            return _ClusterSelection(core, freeze_xy=_should_freeze_static_core(core, self.config))

        if self._can_hold(frame):
            self.pending_switch = None
            return _ClusterSelection(None)

        far_candidate = _select_target_cluster(candidates, None)
        return _ClusterSelection(self._confirm_far_candidate(far_candidate))

    def _confirm_far_candidate(self, cluster: list[Point3D]) -> list[Point3D] | None:
        center_x, center_y = _cluster_center_xy(cluster)
        if self.pending_switch is None:
            self.pending_switch = _PendingSwitch(center_x, center_y, 1, cluster)
        elif _xy_distance(center_x, center_y, self.pending_switch.center_x_m, self.pending_switch.center_y_m) <= self.config.track_gate_m:
            self.pending_switch.center_x_m = center_x
            self.pending_switch.center_y_m = center_y
            self.pending_switch.frames += 1
            self.pending_switch.cluster = cluster
        else:
            self.pending_switch = _PendingSwitch(center_x, center_y, 1, cluster)

        if self.pending_switch.frames >= max(self.config.switch_confirm_frames, 1):
            return self.pending_switch.cluster
        return None

    def _can_hold(self, frame: CombinedFrame) -> bool:
        return (
            self.last_target is not None
            and self.last_target_time_ms is not None
            and frame.timestamp_ms - self.last_target_time_ms <= self.config.hold_ms
        )


def track_recording(frames: list[CombinedFrame] | tuple[CombinedFrame, ...], config: TrackerConfig | None = None) -> list[TargetFrame]:
    config = config or TrackerConfig()
    output: list[TargetFrame] = []
    tracker = _TrackingRuntime(config)

    for frame in frames:
        points = _filter_points(frame.all_points, config)
        clusters = _cluster_points(
            points,
            radius_m=config.cluster_radius_m,
            max_width_x_m=config.max_cluster_width_x_m,
            max_depth_y_m=config.max_cluster_depth_y_m,
        )
        candidates = [cluster for cluster in clusters if len(cluster) >= config.min_cluster_points]
        output.append(tracker.step(frame, candidates).target)

    return output


def _filter_points(points: tuple[Point3D, ...], config: TrackerConfig) -> list[Point3D]:
    filtered = []
    for point in points:
        if not (config.x_min_m <= point.x_m <= config.x_max_m):
            continue
        if not (config.y_min_m <= point.y_m <= config.y_max_m):
            continue
        if not (config.z_min_m <= point.z_m <= config.z_max_m):
            continue
        if point.snr_db < config.min_snr_db:
            continue
        filtered.append(point)
    return filtered


def _cluster_points(
    points: list[Point3D],
    *,
    radius_m: float,
    max_width_x_m: float | None = None,
    max_depth_y_m: float | None = None,
) -> list[list[Point3D]]:
    remaining = set(range(len(points)))
    clusters: list[list[Point3D]] = []

    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        cluster_indices = {seed}
        queue = [seed]
        while queue:
            current = queue.pop()
            neighbors = [index for index in sorted(remaining) if _distance(points[current], points[index]) <= radius_m]
            for index in neighbors:
                if not _cluster_fits_xy(points, cluster_indices | {index}, max_width_x_m, max_depth_y_m):
                    continue
                remaining.remove(index)
                cluster_indices.add(index)
                queue.append(index)
        clusters.append([points[index] for index in sorted(cluster_indices)])

    return clusters


def _select_target_cluster(clusters: list[list[Point3D]], last_target: TargetFrame | None) -> list[Point3D]:
    motion_clusters = [cluster for cluster in clusters if any(point.kind == "motion" for point in cluster)]
    if motion_clusters:
        return max(
            motion_clusters,
            key=lambda cluster: (
                sum(1 for point in cluster if point.kind == "motion"),
                len(cluster),
                _height_span(cluster),
            ),
        )

    if last_target is not None and last_target.has_target and last_target.center_x_m is not None and last_target.center_y_m is not None:
        return min(
            clusters,
            key=lambda cluster: (
                _cluster_distance_to_last(cluster, last_target),
                -sum(1 for point in cluster if point.kind == "motion"),
                -len(cluster),
            ),
        )
    return max(
        clusters,
        key=lambda cluster: (
            sum(1 for point in cluster if point.kind == "motion"),
            len(cluster),
            _height_span(cluster),
        ),
    )


def _cluster_center_xy(cluster: list[Point3D]) -> tuple[float, float]:
    xs = [point.x_m for point in cluster]
    ys = [point.y_m for point in cluster]
    return median(xs), median(ys)


def _has_motion_point(cluster: list[Point3D]) -> bool:
    return any(point.kind == "motion" for point in cluster)


def _motion_count(cluster: list[Point3D]) -> int:
    return sum(1 for point in cluster if point.kind == "motion")


def _motion_ratio(cluster: list[Point3D]) -> float:
    if not cluster:
        return 0.0
    return _motion_count(cluster) / len(cluster)


def _xy_distance(first_x: float, first_y: float, second_x: float, second_y: float) -> float:
    return sqrt((first_x - second_x) ** 2 + (first_y - second_y) ** 2)


def _clusters_within_gate(clusters: list[list[Point3D]], last_target: TargetFrame, gate_m: float) -> list[list[Point3D]]:
    return [cluster for cluster in clusters if _cluster_xy_distance_to_last(cluster, last_target) <= gate_m]


def _select_locked_target_cluster(clusters: list[list[Point3D]], last_target: TargetFrame) -> list[Point3D]:
    return min(
        clusters,
        key=lambda cluster: (
            _cluster_xy_distance_to_last(cluster, last_target),
            -_motion_count(cluster),
            -len(cluster),
        ),
    )


def _select_scored_near_target_cluster(
    clusters: list[list[Point3D]],
    last_target: TargetFrame,
    config: TrackerConfig,
) -> list[Point3D]:
    return max(
        clusters,
        key=lambda cluster: (
            _near_target_score(cluster, last_target, config),
            _motion_count(cluster),
            len(cluster),
            -_cluster_xy_distance_to_last(cluster, last_target),
        ),
    )


def _near_target_score(cluster: list[Point3D], last_target: TargetFrame, config: TrackerConfig) -> float:
    motion_count = _motion_count(cluster)
    point_count = len(cluster)
    motion_ratio = _motion_ratio(cluster)
    distance_m = _cluster_xy_distance_to_last(cluster, last_target)
    gate_m = max(config.track_gate_m, 0.001)

    score = 0.0
    score += motion_count * 5.0
    score += min(point_count, 80) * 0.25
    score += motion_ratio * 20.0
    score += min(_height_span(cluster), 1.5) * 4.0
    score -= min(distance_m / gate_m, 2.0) * 15.0

    if point_count < max(config.min_cluster_points * 2, 6):
        score -= 25.0
    if motion_count == 0 and point_count < _static_body_min_points(config):
        score -= 20.0
    return score


def _should_keep_locked_cluster_over_scored(
    locked_cluster: list[Point3D],
    scored_cluster: list[Point3D],
    last_target: TargetFrame,
    config: TrackerConfig,
) -> bool:
    locked_distance_m = _cluster_xy_distance_to_last(locked_cluster, last_target)
    scored_distance_m = _cluster_xy_distance_to_last(scored_cluster, last_target)
    locked_points = len(locked_cluster)
    scored_points = len(scored_cluster)
    scored_motion = _motion_count(scored_cluster)

    if locked_distance_m > max(0.12, config.track_gate_m * 0.2):
        return False
    if scored_distance_m <= locked_distance_m + 0.18:
        return False
    if locked_points < _static_body_min_points(config):
        return False
    if locked_points < scored_points * 4:
        return False
    if scored_motion >= max(6, config.min_cluster_points * 2):
        return False
    return True


def _should_freeze_static_core(core: list[Point3D], config: TrackerConfig) -> bool:
    return not _has_motion_point(core) and len(core) < _static_body_min_points(config)


def _should_use_full_locked_cluster(cluster: list[Point3D], core: list[Point3D], config: TrackerConfig) -> bool:
    if _motion_count(cluster) >= max(6, config.min_cluster_points * 2):
        return True
    if len(cluster) >= _static_body_min_points(config) and len(core) / len(cluster) >= 0.65:
        return True
    return False


def _static_body_min_points(config: TrackerConfig) -> int:
    return max(20, config.min_cluster_points * 6)


def _locked_cluster_core(cluster: list[Point3D], last_target: TargetFrame, config: TrackerConfig) -> list[Point3D] | None:
    if last_target.center_x_m is None or last_target.center_y_m is None:
        return cluster

    half_width = config.max_cluster_width_x_m / 2.0
    half_depth = config.max_cluster_depth_y_m / 2.0
    core = [
        point
        for point in cluster
        if abs(point.x_m - last_target.center_x_m) <= half_width
        and abs(point.y_m - last_target.center_y_m) <= half_depth
    ]
    if core:
        return core
    return None


def _looks_like_static_floor_clutter(target: TargetFrame) -> bool:
    return (
        target.motion_count == 0
        and target.center_z_m is not None
        and target.center_z_m <= 0.2
        and target.height_m <= 0.15
    )


def _select_static_floor_escape_candidate(
    candidates: list[list[Point3D]],
    last_target: TargetFrame,
    config: TrackerConfig,
) -> list[Point3D] | None:
    candidate = _select_target_cluster(candidates, None)
    if _cluster_xy_distance_to_last(candidate, last_target) <= config.track_gate_m:
        return None
    if _looks_like_elevated_body_candidate(candidate):
        return candidate
    return None


def _looks_like_elevated_body_candidate(cluster: list[Point3D]) -> bool:
    if _motion_count(cluster) == 0:
        return False
    zs = [point.z_m for point in cluster]
    center_z = median(zs)
    return center_z >= 0.5 or max(zs) >= 0.8 or _height_span(cluster) >= 0.6


def _held_target_from_last(frame: CombinedFrame, last_target: TargetFrame) -> TargetFrame:
    return TargetFrame(
        frame_index=frame.frame_index,
        timestamp_ms=frame.timestamp_ms,
        timestamp_text=frame.timestamp_text,
        has_target=True,
        center_x_m=last_target.center_x_m,
        center_y_m=last_target.center_y_m,
        center_z_m=last_target.center_z_m,
        z_min_m=last_target.z_min_m,
        z_max_m=last_target.z_max_m,
        height_m=last_target.height_m,
        width_x_m=last_target.width_x_m,
        depth_y_m=last_target.depth_y_m,
        point_count=0,
        motion_count=0,
        max_abs_velocity_m_s=0.0,
        held=True,
        state=TargetState.HOLD,
    )


def _target_from_cluster(frame: CombinedFrame, cluster: list[Point3D], *, held: bool) -> TargetFrame:
    xs = [point.x_m for point in cluster]
    ys = [point.y_m for point in cluster]
    zs = [point.z_m for point in cluster]
    velocities = [abs(point.velocity_m_s) for point in cluster]

    return TargetFrame(
        frame_index=frame.frame_index,
        timestamp_ms=frame.timestamp_ms,
        timestamp_text=frame.timestamp_text,
        has_target=True,
        center_x_m=median(xs),
        center_y_m=median(ys),
        center_z_m=median(zs),
        z_min_m=min(zs),
        z_max_m=max(zs),
        height_m=max(zs) - min(zs),
        width_x_m=max(xs) - min(xs),
        depth_y_m=max(ys) - min(ys),
        point_count=len(cluster),
        motion_count=sum(1 for point in cluster if point.kind == "motion"),
        max_abs_velocity_m_s=max(velocities) if velocities else 0.0,
        held=held,
        state=TargetState.HOLD if held else TargetState.TRACKING,
    )


def _target_from_static_core(frame: CombinedFrame, core: list[Point3D], last_target: TargetFrame) -> TargetFrame:
    zs = [point.z_m for point in core]
    velocities = [abs(point.velocity_m_s) for point in core]
    return TargetFrame(
        frame_index=frame.frame_index,
        timestamp_ms=frame.timestamp_ms,
        timestamp_text=frame.timestamp_text,
        has_target=True,
        center_x_m=last_target.center_x_m,
        center_y_m=last_target.center_y_m,
        center_z_m=median(zs),
        z_min_m=min(zs),
        z_max_m=max(zs),
        height_m=max(zs) - min(zs),
        width_x_m=last_target.width_x_m,
        depth_y_m=last_target.depth_y_m,
        point_count=len(core),
        motion_count=0,
        max_abs_velocity_m_s=max(velocities) if velocities else 0.0,
        held=True,
        state=TargetState.STATIC_LOCK,
    )


def _distance(first: Point3D, second: Point3D) -> float:
    return sqrt((first.x_m - second.x_m) ** 2 + (first.y_m - second.y_m) ** 2 + (first.z_m - second.z_m) ** 2)


def _cluster_distance_to_last(cluster: list[Point3D], last_target: TargetFrame) -> float:
    xs = [point.x_m for point in cluster]
    ys = [point.y_m for point in cluster]
    zs = [point.z_m for point in cluster]
    return sqrt(
        (median(xs) - float(last_target.center_x_m)) ** 2
        + (median(ys) - float(last_target.center_y_m)) ** 2
        + (median(zs) - float(last_target.center_z_m)) ** 2
    )


def _cluster_xy_distance_to_last(cluster: list[Point3D], last_target: TargetFrame) -> float:
    xs = [point.x_m for point in cluster]
    ys = [point.y_m for point in cluster]
    return sqrt(
        (median(xs) - float(last_target.center_x_m)) ** 2
        + (median(ys) - float(last_target.center_y_m)) ** 2
    )


def _height_span(cluster: list[Point3D]) -> float:
    zs = [point.z_m for point in cluster]
    return max(zs) - min(zs)


def _cluster_fits_xy(
    points: list[Point3D],
    indices: set[int],
    max_width_x_m: float | None,
    max_depth_y_m: float | None,
) -> bool:
    if max_width_x_m is None and max_depth_y_m is None:
        return True
    xs = [points[index].x_m for index in indices]
    ys = [points[index].y_m for index in indices]
    if max_width_x_m is not None and max(xs) - min(xs) > max_width_x_m:
        return False
    if max_depth_y_m is not None and max(ys) - min(ys) > max_depth_y_m:
        return False
    return True
