from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from math import hypot

from .tracking import TargetFrame


class FallState(str, Enum):
    SUSPECTED = "suspected_fall"
    CONFIRMED = "confirmed_fall"
    RECOVERED = "recovered"


@dataclass(frozen=True)
class FallRuleConfig:
    drop_window_ms: int = 3500
    confirm_window_ms: int = 7000
    min_drop_height_m: float = 0.80
    motion_velocity_threshold_m_s: float = 0.45
    motion_point_threshold: int = 20
    low_height_m: float = 0.55
    terminal_height_m: float = 0.40
    suspect_min_fresh_samples: int = 3
    confirm_min_fresh_samples: int = 6
    fresh_min_point_count: int = 3
    suspect_low_duration_ms: int = 500
    suspect_low_ratio: float = 0.75
    still_velocity_threshold_m_s: float = 0.18
    still_motion_points_max: int = 1
    low_still_duration_ms: int = 1200
    confirm_low_ratio: float = 0.70
    confirm_min_horizontal_displacement_m: float = 0.0
    rearm_height_m: float = 0.85
    rearm_duration_ms: int = 1000
    # Kept for callers that still construct the old configuration shape.
    confirmed_rearm_ms: int = 5000
    confirmed_rearm_rise_m: float = 0.25


@dataclass(frozen=True)
class FallEvent:
    state: FallState
    frame_index: int
    timestamp_ms: int
    drop_height_m: float
    reason: str


@dataclass
class _FallCandidate:
    frame_index: int
    timestamp_ms: int
    drop_height_m: float
    anchor_x_m: float
    anchor_y_m: float
    max_horizontal_displacement_m: float = 0.0


class IncrementalFallDetector:
    def __init__(self, config: FallRuleConfig | None = None):
        self.config = config or FallRuleConfig()
        self.recent: deque[TargetFrame] = deque()
        self.height_history: deque[tuple[int, float]] = deque()
        self.fresh_height_history: deque[tuple[int, float]] = deque()
        self.candidate: _FallCandidate | None = None
        self.suspected: FallEvent | None = None
        self.confirmed = False
        self.confirmed_at_ms: int | None = None
        self.confirmed_low_z_m: float | None = None
        self.rearm_started_ms: int | None = None

    def step(self, current: TargetFrame) -> list[FallEvent]:
        if not current.has_target or current.center_z_m is None:
            self.height_history.clear()
            self.fresh_height_history.clear()
            if self.confirmed:
                self.rearm_started_ms = None
            return []

        config = self.config
        events: list[FallEvent] = []
        suspected_created = False
        if self.confirmed:
            recovered = self._rearm_confirmed_if_ready(current)
            if recovered is not None:
                events.append(recovered)
            if self.confirmed:
                return events

        self.recent.append(current)
        while self.recent and current.timestamp_ms - self.recent[0].timestamp_ms > config.drop_window_ms:
            self.recent.popleft()

        candidate_low_posture = current.center_z_m <= config.low_height_m
        terminal_low_posture = current.center_z_m <= config.terminal_height_m
        # HOLD has zero points; STATIC_LOCK still carries current static points and refreshes Z.
        fresh_height_sample = current.point_count >= config.fresh_min_point_count
        self.height_history.append((current.timestamp_ms, current.center_z_m))
        if fresh_height_sample:
            self.fresh_height_history.append((current.timestamp_ms, current.center_z_m))
        height_history_window_ms = max(
            config.suspect_low_duration_ms,
            config.low_still_duration_ms,
        )
        # Keep one sample immediately before the window boundary so irregular
        # frame periods can prove that the full duration has elapsed.
        while (
            len(self.height_history) > 1
            and current.timestamp_ms - self.height_history[1][0] > height_history_window_ms
        ):
            self.height_history.popleft()
        while (
            len(self.fresh_height_history) > 1
            and current.timestamp_ms - self.fresh_height_history[1][0] > height_history_window_ms
        ):
            self.fresh_height_history.popleft()

        drop_height = max(frame.center_z_m for frame in self.recent if frame.center_z_m is not None) - current.center_z_m
        motion_triggered = any(
            frame.max_abs_velocity_m_s >= config.motion_velocity_threshold_m_s
            or frame.motion_count >= config.motion_point_threshold
            for frame in self.recent
        )

        candidate_low_ready = (
            candidate_low_posture
            and self._history_covers(current.timestamp_ms, config.suspect_low_duration_ms)
            and self._height_ratio(
                current.timestamp_ms,
                config.suspect_low_duration_ms,
                config.low_height_m,
            )
            >= config.suspect_low_ratio
        )
        if (
            self.candidate is None
            and drop_height >= config.min_drop_height_m
            and motion_triggered
            and candidate_low_ready
        ):
            anchor_x_m, anchor_y_m = self._candidate_anchor(current)
            self.candidate = _FallCandidate(
                frame_index=current.frame_index,
                timestamp_ms=current.timestamp_ms,
                drop_height_m=drop_height,
                anchor_x_m=anchor_x_m,
                anchor_y_m=anchor_y_m,
            )

        if self.candidate is None or self.confirmed:
            return events

        self._update_candidate_horizontal_displacement(current)

        if current.timestamp_ms - self.candidate.timestamp_ms > config.confirm_window_ms:
            if self.suspected is not None:
                events.append(
                    FallEvent(
                        state=FallState.RECOVERED,
                        frame_index=current.frame_index,
                        timestamp_ms=current.timestamp_ms,
                        drop_height_m=max(self.candidate.drop_height_m, drop_height),
                        reason="suspected fall expired",
                    )
                )
            self.candidate = None
            self.suspected = None
            return events

        terminal_suspect_ready = (
            fresh_height_sample
            and terminal_low_posture
            and self._fresh_history_covers(current.timestamp_ms, config.suspect_low_duration_ms)
            and self._fresh_sample_count(current.timestamp_ms, config.suspect_low_duration_ms)
            >= config.suspect_min_fresh_samples
            and self._fresh_height_state_covers(
                current.timestamp_ms,
                config.suspect_low_duration_ms,
                config.terminal_height_m,
            )
            and self._fresh_height_ratio(
                current.timestamp_ms,
                config.suspect_low_duration_ms,
                config.terminal_height_m,
            )
            >= config.suspect_low_ratio
        )
        if self.suspected is None and terminal_suspect_ready:
            self.suspected = FallEvent(
                state=FallState.SUSPECTED,
                frame_index=current.frame_index,
                timestamp_ms=current.timestamp_ms,
                drop_height_m=max(self.candidate.drop_height_m, drop_height),
                reason="terminal height persisted after descent",
            )
            events.append(self.suspected)
            suspected_created = True

        if self.suspected is None:
            return events
        if suspected_created:
            return events

        still = (
            current.max_abs_velocity_m_s <= config.still_velocity_threshold_m_s
            and current.motion_count <= config.still_motion_points_max
        )
        confirm_low_ready = (
            fresh_height_sample
            and terminal_low_posture
            and current.timestamp_ms - self.suspected.timestamp_ms >= config.low_still_duration_ms
            and self._fresh_history_covers(current.timestamp_ms, config.low_still_duration_ms)
            and self._fresh_sample_count(current.timestamp_ms, config.low_still_duration_ms)
            >= config.confirm_min_fresh_samples
            and self._fresh_height_state_covers(
                current.timestamp_ms,
                config.low_still_duration_ms,
                config.terminal_height_m,
            )
            and self._fresh_height_ratio(
                current.timestamp_ms,
                config.low_still_duration_ms,
                config.terminal_height_m,
            )
            >= config.confirm_low_ratio
            and self.candidate.max_horizontal_displacement_m
            >= config.confirm_min_horizontal_displacement_m
        )
        if confirm_low_ready:
            events.append(
                FallEvent(
                    state=FallState.CONFIRMED,
                    frame_index=current.frame_index,
                    timestamp_ms=current.timestamp_ms,
                    drop_height_m=max(self.candidate.drop_height_m, self.suspected.drop_height_m, drop_height),
                    reason=(
                        "low posture stayed still after suspected fall"
                        if still
                        else "low posture persisted with activity after suspected fall"
                    ),
                )
            )
            self.confirmed = True
            self.confirmed_at_ms = current.timestamp_ms
            self.confirmed_low_z_m = current.center_z_m
            self.rearm_started_ms = None
        return events

    def _candidate_anchor(self, current: TargetFrame) -> tuple[float, float]:
        positioned = [
            frame
            for frame in self.recent
            if frame.center_x_m is not None
            and frame.center_y_m is not None
            and frame.center_z_m is not None
        ]
        for frame in reversed(positioned):
            if frame.center_z_m >= self.config.rearm_height_m:
                return float(frame.center_x_m), float(frame.center_y_m)
        if positioned:
            highest = max(positioned, key=lambda frame: float(frame.center_z_m))
            return float(highest.center_x_m), float(highest.center_y_m)
        return float(current.center_x_m or 0.0), float(current.center_y_m or 0.0)

    def _update_candidate_horizontal_displacement(self, current: TargetFrame) -> None:
        if (
            self.candidate is None
            or current.center_x_m is None
            or current.center_y_m is None
        ):
            return
        displacement_m = hypot(
            current.center_x_m - self.candidate.anchor_x_m,
            current.center_y_m - self.candidate.anchor_y_m,
        )
        self.candidate.max_horizontal_displacement_m = max(
            self.candidate.max_horizontal_displacement_m,
            displacement_m,
        )

    def _rearm_confirmed_if_ready(self, current: TargetFrame) -> FallEvent | None:
        config = self.config
        if current.center_z_m < config.rearm_height_m:
            self.rearm_started_ms = None
            return None
        if self.rearm_started_ms is None:
            self.rearm_started_ms = current.timestamp_ms
            return None
        if current.timestamp_ms - self.rearm_started_ms < config.rearm_duration_ms:
            return None

        previous_drop_height = self.suspected.drop_height_m if self.suspected is not None else 0.0
        self.recent.clear()
        self.height_history.clear()
        self.fresh_height_history.clear()
        self.candidate = None
        self.suspected = None
        self.confirmed = False
        self.confirmed_at_ms = None
        self.confirmed_low_z_m = None
        self.rearm_started_ms = None
        return FallEvent(
            state=FallState.RECOVERED,
            frame_index=current.frame_index,
            timestamp_ms=current.timestamp_ms,
            drop_height_m=previous_drop_height,
            reason="confirmed fall target stood up",
        )

    def _history_covers(self, timestamp_ms: int, duration_ms: int) -> bool:
        if duration_ms <= 0:
            return bool(self.height_history)
        window_start_ms = timestamp_ms - duration_ms
        return bool(self.height_history) and self.height_history[0][0] <= window_start_ms

    def _height_ratio(self, timestamp_ms: int, duration_ms: int, threshold_m: float) -> float:
        relevant = [
            center_z_m <= threshold_m
            for sample_ms, center_z_m in self.height_history
            if timestamp_ms - sample_ms <= duration_ms
        ]
        if not relevant:
            return 0.0
        return sum(relevant) / len(relevant)

    def _height_state_covers(self, timestamp_ms: int, duration_ms: int, threshold_m: float) -> bool:
        if duration_ms <= 0:
            return bool(self.height_history) and self.height_history[-1][1] <= threshold_m
        window_start_ms = timestamp_ms - duration_ms
        return any(
            sample_ms <= window_start_ms and center_z_m <= threshold_m
            for sample_ms, center_z_m in self.height_history
        )

    def _fresh_history_covers(self, timestamp_ms: int, duration_ms: int) -> bool:
        if duration_ms <= 0:
            return bool(self.fresh_height_history)
        window_start_ms = timestamp_ms - duration_ms
        return bool(self.fresh_height_history) and self.fresh_height_history[0][0] <= window_start_ms

    def _fresh_sample_count(self, timestamp_ms: int, duration_ms: int) -> int:
        return sum(
            timestamp_ms - sample_ms <= duration_ms
            for sample_ms, _center_z_m in self.fresh_height_history
        )

    def _fresh_height_ratio(self, timestamp_ms: int, duration_ms: int, threshold_m: float) -> float:
        relevant = [
            center_z_m <= threshold_m
            for sample_ms, center_z_m in self.fresh_height_history
            if timestamp_ms - sample_ms <= duration_ms
        ]
        if not relevant:
            return 0.0
        return sum(relevant) / len(relevant)

    def _fresh_height_state_covers(self, timestamp_ms: int, duration_ms: int, threshold_m: float) -> bool:
        if duration_ms <= 0:
            return bool(self.fresh_height_history) and self.fresh_height_history[-1][1] <= threshold_m
        window_start_ms = timestamp_ms - duration_ms
        return any(
            sample_ms <= window_start_ms and center_z_m <= threshold_m
            for sample_ms, center_z_m in self.fresh_height_history
        )


def detect_falls(track: list[TargetFrame] | tuple[TargetFrame, ...], config: FallRuleConfig | None = None) -> list[FallEvent]:
    events: list[FallEvent] = []
    detector = IncrementalFallDetector(config)
    for current in track:
        events.extend(detector.step(current))
    return events
