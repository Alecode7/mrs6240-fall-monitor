from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import ceil

from .datadealer_loader import CombinedFrame, Point3D


@dataclass(frozen=True)
class RawFrameView:
    frame: CombinedFrame
    presence_points: tuple[Point3D, ...]
    motion_points: tuple[Point3D, ...]
    total_count: int
    presence_count: int
    motion_count: int
    max_abs_velocity_m_s: float
    max_z_m: float | None
    p95_z_m: float | None

    @property
    def all_points(self) -> tuple[Point3D, ...]:
        return self.presence_points + self.motion_points

    @property
    def target(self):
        """Keep the display contract explicit: raw frames have no host target."""
        return None


@dataclass(frozen=True)
class RawTimeline:
    frame_indices: tuple[int, ...]
    timestamp_ms: tuple[int, ...]
    max_z_m: tuple[float | None, ...]
    p95_z_m: tuple[float | None, ...]
    motion_counts: tuple[int, ...]
    max_velocity_m_s: tuple[float, ...]


def raw_view_from_frame(frame: CombinedFrame) -> RawFrameView:
    presence_points = tuple(frame.presence_points)
    motion_points = tuple(frame.motion_points)
    points = presence_points + motion_points
    return RawFrameView(
        frame=frame,
        presence_points=presence_points,
        motion_points=motion_points,
        total_count=len(points),
        presence_count=len(presence_points),
        motion_count=len(motion_points),
        max_abs_velocity_m_s=max((abs(point.velocity_m_s) for point in points), default=0.0),
        max_z_m=max((point.z_m for point in points), default=None),
        p95_z_m=_percentile((point.z_m for point in points), 0.95),
    )


class RawFrameHistory:
    def __init__(self, *, max_frames: int = 600):
        self._views: deque[RawFrameView] = deque(maxlen=max(int(max_frames), 1))

    @property
    def views(self) -> tuple[RawFrameView, ...]:
        return tuple(self._views)

    def clear(self) -> None:
        self._views.clear()

    def append(self, view: RawFrameView) -> None:
        existing = next((item for item in self._views if item.frame.frame_index == view.frame.frame_index), None)
        if existing is not None:
            self._views.remove(existing)
        self._views.append(view)

    def timeline(self) -> RawTimeline:
        return RawTimeline(
            frame_indices=tuple(view.frame.frame_index for view in self._views),
            timestamp_ms=tuple(view.frame.timestamp_ms for view in self._views),
            max_z_m=tuple(view.max_z_m for view in self._views),
            p95_z_m=tuple(view.p95_z_m for view in self._views),
            motion_counts=tuple(view.motion_count for view in self._views),
            max_velocity_m_s=tuple(view.max_abs_velocity_m_s for view in self._views),
        )


def _percentile(values, fraction: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    index = max(0, min(len(ordered) - 1, ceil(fraction * len(ordered)) - 1))
    return ordered[index]
