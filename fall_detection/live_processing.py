from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .coordinate_transform import transform_frame_to_world
from .datadealer_loader import CombinedFrame
from .rule_detector import FallRuleConfig, IncrementalFallDetector
from .tracking import TargetFrame, TrackerConfig
from .visualization_model import VisualizationFrame, VisualizationRuntime
from .visualize_datadealer import FrameAlert, TimelineSeries, _build_timeline_series


@dataclass(frozen=True)
class LiveSnapshot:
    views: tuple[VisualizationFrame, ...]
    alerts: tuple[FrameAlert, ...]
    timeline: TimelineSeries


class LiveFrameWindow:
    def __init__(
        self,
        config: TrackerConfig | None = None,
        *,
        max_history: int = 600,
        jump_threshold_m: float = 0.8,
        fall_config: FallRuleConfig | None = None,
        yz_rotation_deg: float = 0.0,
        mount_height_m: float = 0.0,
    ):
        self.runtime = VisualizationRuntime(config or TrackerConfig())
        self.max_history = max(int(max_history), 1)
        self.jump_threshold_m = jump_threshold_m
        self.yz_rotation_deg = float(yz_rotation_deg)
        self.mount_height_m = float(mount_height_m)
        self._views: deque[VisualizationFrame] = deque(maxlen=self.max_history)
        self._alerts: deque[FrameAlert] = deque(maxlen=self.max_history)
        self._timeline = TimelineSeries((), (), (), (), ())
        self._fall_detector = IncrementalFallDetector(fall_config)
        self._previous_alert_target: TargetFrame | None = None

    def push_frame(self, frame: CombinedFrame, *, build_timeline: bool = True) -> LiveSnapshot:
        world_frame = transform_frame_to_world(
            frame,
            yz_rotation_deg=self.yz_rotation_deg,
            mount_height_m=self.mount_height_m,
        )
        view = self.runtime.step(world_frame)
        self._views.append(view)
        self._alerts.append(self._alert_for_view(view))
        return self.snapshot(build_timeline=build_timeline)

    def snapshot(self, *, build_timeline: bool = True) -> LiveSnapshot:
        views = tuple(self._views)
        alerts = tuple(self._alerts)
        if build_timeline:
            self._timeline = _build_timeline_series(views, alerts)
        return LiveSnapshot(views=views, alerts=alerts, timeline=self._timeline)

    def _alert_for_view(self, view: VisualizationFrame) -> FrameAlert:
        jump_m = _target_xy_jump_m(self._previous_alert_target, view.target)
        if jump_m is not None and jump_m < self.jump_threshold_m:
            jump_m = None
        fall_events = self._fall_detector.step(view.target)
        if _target_has_xy(view.target):
            self._previous_alert_target = view.target
        return FrameAlert(jump_m=jump_m, fall_states=tuple(event.state for event in fall_events))


def _target_has_xy(target: TargetFrame) -> bool:
    return bool(target.has_target) and target.center_x_m is not None and target.center_y_m is not None


def _target_xy_jump_m(previous: TargetFrame | None, current: TargetFrame) -> float | None:
    if previous is None or not _target_has_xy(previous) or not _target_has_xy(current):
        return None
    dx = float(current.center_x_m) - float(previous.center_x_m)
    dy = float(current.center_y_m) - float(previous.center_y_m)
    return (dx * dx + dy * dy) ** 0.5
