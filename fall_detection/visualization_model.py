from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin
import sys
from pathlib import Path
from statistics import median

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from fall_detection.datadealer_loader import CombinedFrame, Point3D
    from fall_detection.tracking import (
        TargetFrame,
        TargetState,
        TrackerConfig,
        _TrackingRuntime,
        _cluster_center_xy,
        _cluster_points,
        _cluster_xy_distance_to_last,
        _filter_points,
        _motion_count,
        _motion_ratio,
        _near_target_score,
    )
else:
    from .datadealer_loader import CombinedFrame, Point3D
    from .tracking import (
        TargetFrame,
        TargetState,
        TrackerConfig,
        _TrackingRuntime,
        _cluster_center_xy,
        _cluster_points,
        _cluster_xy_distance_to_last,
        _filter_points,
        _motion_count,
        _motion_ratio,
        _near_target_score,
    )


@dataclass(frozen=True)
class PlotBounds:
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_m: float
    z_max_m: float


@dataclass(frozen=True)
class Camera3D:
    azimuth_deg: float = 45.0
    elevation_deg: float = 25.0
    zoom: float = 0.72


@dataclass(frozen=True)
class ClusterView:
    points: tuple[Point3D, ...]
    is_target: bool
    center_x_m: float
    center_y_m: float
    center_z_m: float
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_m: float
    z_max_m: float
    point_count: int
    motion_count: int
    max_abs_velocity_m_s: float


@dataclass(frozen=True)
class DebugCandidate:
    index: int
    center_x_m: float
    center_y_m: float
    point_count: int
    motion_count: int
    motion_ratio: float
    distance_to_previous_m: float | None
    score: float | None
    within_gate: bool
    selected: bool


@dataclass(frozen=True)
class VisualizationFrame:
    frame: CombinedFrame
    filtered_points: tuple[Point3D, ...]
    clusters: tuple[ClusterView, ...]
    target: TargetFrame
    target_cluster: ClusterView | None
    previous_target_x_m: float | None
    previous_target_y_m: float | None
    track_gate_m: float
    debug_candidates: tuple[DebugCandidate, ...]


class VisualizationRuntime:
    def __init__(self, config: TrackerConfig | None = None):
        self.config = config or TrackerConfig()
        self.tracker = _TrackingRuntime(self.config)

    def step(self, frame: CombinedFrame) -> VisualizationFrame:
        previous_target = self.tracker.last_target
        previous_target_x = previous_target.center_x_m if previous_target is not None and previous_target.has_target else None
        previous_target_y = previous_target.center_y_m if previous_target is not None and previous_target.has_target else None
        filtered_points = _filter_points(frame.all_points, self.config)
        raw_clusters = _cluster_points(
            filtered_points,
            radius_m=self.config.cluster_radius_m,
            max_width_x_m=self.config.max_cluster_width_x_m,
            max_depth_y_m=self.config.max_cluster_depth_y_m,
        )
        candidates = [cluster for cluster in raw_clusters if len(cluster) >= self.config.min_cluster_points]
        tracking_step = self.tracker.step(frame, candidates)
        target = tracking_step.target
        selected_cluster = tracking_step.selected_cluster
        debug_candidates = _debug_candidates_from_clusters(candidates, selected_cluster, previous_target, self.config)

        cluster_views = tuple(
            _cluster_view_from_points(cluster, is_target=cluster is selected_cluster)
            for cluster in raw_clusters
            if len(cluster) >= self.config.min_cluster_points
        )
        target_cluster = next((cluster for cluster in cluster_views if cluster.is_target), None)
        if target_cluster is None and selected_cluster is not None:
            target_cluster = _cluster_view_from_points(selected_cluster, is_target=True)
        if target_cluster is None and target.has_target and target.held:
            target_cluster = _held_cluster_view_from_target(target, self.config)
        return VisualizationFrame(
            frame=frame,
            filtered_points=tuple(filtered_points),
            clusters=cluster_views,
            target=target,
            target_cluster=target_cluster,
            previous_target_x_m=previous_target_x,
            previous_target_y_m=previous_target_y,
            track_gate_m=self.config.track_gate_m,
            debug_candidates=debug_candidates,
        )


def build_visualization_frames(
    frames: list[CombinedFrame] | tuple[CombinedFrame, ...],
    config: TrackerConfig | None = None,
) -> list[VisualizationFrame]:
    runtime = VisualizationRuntime(config)
    return [runtime.step(frame) for frame in frames]


def bounds_from_config(config: TrackerConfig | None = None) -> PlotBounds:
    config = config or TrackerConfig()
    return PlotBounds(
        x_min_m=config.x_min_m,
        x_max_m=config.x_max_m,
        y_min_m=config.y_min_m,
        y_max_m=config.y_max_m,
        z_min_m=config.z_min_m,
        z_max_m=config.z_max_m,
    )


def format_target_diagnosis(view: VisualizationFrame) -> str:
    state = getattr(view.target, "state", None)
    if not view.clusters:
        if state == TargetState.REACQUIRE:
            return "诊断: 重捕获中  聚类=0"
        return "诊断: 无有效聚类"
    if view.target_cluster is None:
        if state == TargetState.REACQUIRE:
            return f"诊断: 重捕获中  聚类={len(view.clusters)}"
        return f"诊断: 未选中目标  聚类={len(view.clusters)}"

    target = view.target_cluster
    largest = max(view.clusters, key=lambda cluster: cluster.point_count)
    most_motion = max(view.clusters, key=lambda cluster: (cluster.motion_count, cluster.point_count))
    notes: list[str] = []

    if state == TargetState.STATIC_LOCK:
        notes.append("静态锁定")
    elif state == TargetState.HOLD:
        notes.append("目标保持中")
    elif state == TargetState.REACQUIRE:
        notes.append("重捕获中")
    elif view.target.held:
        if view.target.point_count > 0:
            notes.append("静态锁定")
        else:
            notes.append("目标保持中")
    if largest is not target and largest.point_count >= max(target.point_count * 3, target.point_count + 20):
        notes.append("警告: 最大聚类未选中")
    if most_motion is not target and most_motion.motion_count >= target.motion_count + 6:
        notes.append("警告: 运动最多聚类未选中")
    if target.point_count <= 5 and largest.point_count >= 30:
        notes.append("警告: 小目标可能是运动噪声")

    if not notes:
        notes.append("正常")
    return (
        "诊断: "
        f"目标=点{target.point_count}/动{target.motion_count} "
        f"最大=点{largest.point_count}/动{largest.motion_count} "
        f"运动最多=点{most_motion.point_count}/动{most_motion.motion_count} "
        f"{'；'.join(notes)}"
    )


def format_debug_candidate_label(candidate: DebugCandidate) -> str:
    score = "-" if candidate.score is None else f"{candidate.score:.1f}"
    distance = "-" if candidate.distance_to_previous_m is None else f"{candidate.distance_to_previous_m:.2f}m"
    selected = " 选中" if candidate.selected else ""
    return (
        f"#{candidate.index} 点{candidate.point_count}/动{candidate.motion_count} "
        f"距{distance} 分{score}{selected}"
    )


def target_diagnostic_lines(views: list[VisualizationFrame] | tuple[VisualizationFrame, ...], *, limit: int = 20) -> list[str]:
    rows = []
    for index, view in enumerate(views):
        diagnosis = format_target_diagnosis(view)
        warning = "警告" in diagnosis
        rows.append(
            (
                not warning,
                index,
                f"frame={view.frame.frame_index} t_ms={view.frame.timestamp_ms} {diagnosis}",
            )
        )
    rows.sort(key=lambda row: (row[0], row[1]))
    return [row[2] for row in rows[: max(limit, 0)]]


def project_xy(point: Point3D, bounds: PlotBounds, *, width_px: int, height_px: int, margin_px: int = 24) -> tuple[float, float]:
    plot_w = max(width_px - margin_px * 2, 1)
    plot_h = max(height_px - margin_px * 2, 1)
    x_ratio = _ratio(point.x_m, bounds.x_min_m, bounds.x_max_m)
    y_ratio = _ratio(point.y_m, bounds.y_min_m, bounds.y_max_m)
    return (
        margin_px + x_ratio * plot_w,
        height_px - margin_px - y_ratio * plot_h,
    )


def project_iso_3d(point: Point3D, bounds: PlotBounds, *, width_px: int, height_px: int, margin_px: int = 24) -> tuple[float, float]:
    plot_w = max(width_px - margin_px * 2, 1)
    plot_h = max(height_px - margin_px * 2, 1)
    x_ratio = _ratio(point.x_m, bounds.x_min_m, bounds.x_max_m) - 0.5
    y_ratio = _ratio(point.y_m, bounds.y_min_m, bounds.y_max_m)
    z_ratio = _ratio(point.z_m, bounds.z_min_m, bounds.z_max_m)

    screen_x = width_px / 2 + (x_ratio * 0.85 + (y_ratio - 0.5) * 0.42) * plot_w
    screen_y = height_px - margin_px - y_ratio * plot_h * 0.34 - z_ratio * plot_h * 0.62
    return screen_x, screen_y


def project_rotated_3d(
    point: Point3D,
    bounds: PlotBounds,
    camera: Camera3D,
    *,
    width_px: int,
    height_px: int,
    margin_px: int = 24,
) -> tuple[float, float]:
    azimuth = radians(camera.azimuth_deg)
    elevation = radians(camera.elevation_deg)
    x_mid = (bounds.x_min_m + bounds.x_max_m) / 2.0
    y_mid = (bounds.y_min_m + bounds.y_max_m) / 2.0
    z_mid = (bounds.z_min_m + bounds.z_max_m) / 2.0
    x = point.x_m - x_mid
    y = point.y_m - y_mid
    z = point.z_m - z_mid

    rotated_x = x * cos(azimuth) - y * sin(azimuth)
    rotated_y = x * sin(azimuth) + y * cos(azimuth)
    screen_plane_y = z * cos(elevation) - rotated_y * sin(elevation)

    span = max(
        bounds.x_max_m - bounds.x_min_m,
        bounds.y_max_m - bounds.y_min_m,
        bounds.z_max_m - bounds.z_min_m,
        1.0,
    )
    scale = min(width_px - margin_px * 2, height_px - margin_px * 2) * camera.zoom / span
    return (
        width_px / 2.0 + rotated_x * scale,
        height_px / 2.0 - screen_plane_y * scale,
    )


def _cluster_view_from_points(points: list[Point3D], *, is_target: bool) -> ClusterView:
    xs = [point.x_m for point in points]
    ys = [point.y_m for point in points]
    zs = [point.z_m for point in points]
    velocities = [abs(point.velocity_m_s) for point in points]
    return ClusterView(
        points=tuple(points),
        is_target=is_target,
        center_x_m=median(xs),
        center_y_m=median(ys),
        center_z_m=median(zs),
        x_min_m=min(xs),
        x_max_m=max(xs),
        y_min_m=min(ys),
        y_max_m=max(ys),
        z_min_m=min(zs),
        z_max_m=max(zs),
        point_count=len(points),
        motion_count=sum(1 for point in points if point.kind == "motion"),
        max_abs_velocity_m_s=max(velocities) if velocities else 0.0,
    )


def _debug_candidates_from_clusters(
    candidates: list[list[Point3D]],
    selected_cluster: list[Point3D] | None,
    previous_target: TargetFrame | None,
    config: TrackerConfig,
) -> tuple[DebugCandidate, ...]:
    rows = []
    for index, cluster in enumerate(candidates):
        center_x, center_y = _cluster_center_xy(cluster)
        distance = None
        score = None
        within_gate = False
        if previous_target is not None and previous_target.has_target:
            distance = _cluster_xy_distance_to_last(cluster, previous_target)
            within_gate = distance <= config.track_gate_m
            if within_gate:
                score = _near_target_score(cluster, previous_target, config)
        rows.append(
            DebugCandidate(
                index=index,
                center_x_m=center_x,
                center_y_m=center_y,
                point_count=len(cluster),
                motion_count=_motion_count(cluster),
                motion_ratio=_motion_ratio(cluster),
                distance_to_previous_m=distance,
                score=score,
                within_gate=within_gate,
                selected=_cluster_matches_selection(cluster, selected_cluster),
            )
        )
    return tuple(rows)


def _cluster_matches_selection(cluster: list[Point3D], selected_cluster: list[Point3D] | None) -> bool:
    if selected_cluster is None:
        return False
    if cluster is selected_cluster:
        return True
    cluster_ids = {id(point) for point in cluster}
    selected_ids = {id(point) for point in selected_cluster}
    return bool(selected_ids) and selected_ids <= cluster_ids


def _held_cluster_view_from_target(target: TargetFrame, config: TrackerConfig) -> ClusterView:
    center_x = float(target.center_x_m) if target.center_x_m is not None else 0.0
    center_y = float(target.center_y_m) if target.center_y_m is not None else 0.0
    center_z = float(target.center_z_m) if target.center_z_m is not None else 0.0
    half_width = config.max_cluster_width_x_m / 2.0
    half_depth = config.max_cluster_depth_y_m / 2.0
    z_min = target.z_min_m if target.z_min_m is not None else center_z
    z_max = target.z_max_m if target.z_max_m is not None else center_z
    return ClusterView(
        points=(),
        is_target=True,
        center_x_m=center_x,
        center_y_m=center_y,
        center_z_m=center_z,
        x_min_m=center_x - half_width,
        x_max_m=center_x + half_width,
        y_min_m=center_y - half_depth,
        y_max_m=center_y + half_depth,
        z_min_m=z_min,
        z_max_m=z_max,
        point_count=target.point_count,
        motion_count=target.motion_count,
        max_abs_velocity_m_s=target.max_abs_velocity_m_s,
    )


def _ratio(value: float, low: float, high: float) -> float:
    if high <= low:
        return 0.0
    return min(max((value - low) / (high - low), 0.0), 1.0)


def main() -> None:
    print("visualization_model.py is a library module.")
    print("Run the visual UI from this directory:")
    print(r"  uv run .\visualize_datadealer.py --data-dir ..\datadealer\data")
    print("Or from MRS6240-P2512:")
    print(r"  uv run python -m fall_detection.visualize_datadealer --data-dir datadealer\data")


if __name__ == "__main__":
    main()
