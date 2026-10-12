from __future__ import annotations

import argparse
from collections import Counter
import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from fall_detection.datadealer_loader import count_points, iter_sample_stems, load_datadealer_recording
    from fall_detection.rule_detector import FallState, detect_falls
    from fall_detection.tracking import TrackerConfig, track_recording
    from fall_detection.visualization_model import (
        build_visualization_frames,
        format_debug_candidate_label,
        format_target_diagnosis,
        target_diagnostic_lines,
    )
else:
    from .datadealer_loader import count_points, iter_sample_stems, load_datadealer_recording
    from .rule_detector import FallState, detect_falls
    from .tracking import TrackerConfig, track_recording
    from .visualization_model import (
        build_visualization_frames,
        format_debug_candidate_label,
        format_target_diagnosis,
        target_diagnostic_lines,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze MRS6240 point-cloud CSV files for fall-detection features.")
    parser.add_argument(
        "--data-dir",
        default="datadealer/data",
        help="Directory containing datadealer CSV files or SDK Software_Kit/05_Data point-cloud conversion CSV files",
    )
    parser.add_argument("--stem", default=None, help="Recording stem, for example 2026_07_03_111005")
    parser.add_argument("--min-snr", type=float, default=15.0)
    parser.add_argument("--cluster-radius", type=float, default=0.45)
    parser.add_argument("--min-cluster-points", type=int, default=3)
    parser.add_argument("--max-cluster-width", type=float, default=0.5)
    parser.add_argument("--max-cluster-depth", type=float, default=0.5)
    parser.add_argument("--track-gate", type=float, default=0.5)
    parser.add_argument("--reacquire-gate", type=float, default=1.0, help="kept for old commands; current tracker uses switch-confirm-frames")
    parser.add_argument("--switch-confirm-frames", type=int, default=5, help="consecutive far-candidate frames required before target switching")
    parser.add_argument("--hold-ms", type=int, default=3000)
    parser.add_argument("--diagnose-targets", action="store_true", help="print target-selection diagnostics for suspicious or early frames")
    parser.add_argument("--diagnose-limit", type=int, default=20, help="maximum diagnostic lines to print")
    parser.add_argument("--batch-summary", action="store_true", help="print one tracking-state summary line per recording in data-dir")
    parser.add_argument("--jump-report", action="store_true", help="print frames where the tracked target center jumps too far")
    parser.add_argument("--jump-threshold", type=float, default=0.8, help="XY jump distance threshold in meters for --jump-report")
    parser.add_argument("--jump-limit", type=int, default=20, help="maximum jump records to print")
    args = parser.parse_args()

    tracker_config = TrackerConfig(
        min_snr_db=args.min_snr,
        cluster_radius_m=args.cluster_radius,
        min_cluster_points=args.min_cluster_points,
        max_cluster_width_x_m=args.max_cluster_width,
        max_cluster_depth_y_m=args.max_cluster_depth,
        track_gate_m=args.track_gate,
        reacquire_gate_m=args.reacquire_gate,
        switch_confirm_frames=args.switch_confirm_frames,
        hold_ms=args.hold_ms,
    )
    data_dir = Path(args.data_dir)
    if args.batch_summary:
        _print_batch_summary(data_dir, args.stem, tracker_config)
        return

    recording = load_datadealer_recording(data_dir, stem=args.stem)
    track = track_recording(recording.frames, tracker_config)
    events = detect_falls(track)
    motion_points, presence_points = count_points(recording.frames)

    tracked = [frame for frame in track if frame.has_target]
    print(f"recording={recording.stem}")
    print(f"frames={len(recording.frames)} duration_s={recording.duration_s:.3f}")
    print(f"raw_points motion={motion_points} presence={presence_points}")
    print(f"tracker target_frames={len(tracked)} coverage={len(tracked) / max(len(track), 1):.1%}")
    if tracked:
        z_values = [frame.center_z_m for frame in tracked if frame.center_z_m is not None]
        y_values = [frame.center_y_m for frame in tracked if frame.center_y_m is not None]
        motion_counts = [frame.motion_count for frame in tracked]
        print(f"target_z_m min={min(z_values):.3f} max={max(z_values):.3f}")
        print(f"target_y_forward_m min={min(y_values):.3f} max={max(y_values):.3f}")
        print(f"target_motion_points max={max(motion_counts)}")

    views = None
    if args.diagnose_targets:
        views = build_visualization_frames(recording.frames, tracker_config)
        lines = target_diagnostic_lines(views, limit=args.diagnose_limit)
        print(f"target_diagnostics={len(lines)}")
        for line in lines:
            print(line)

    if args.jump_report:
        if views is None:
            views = build_visualization_frames(recording.frames, tracker_config)
        _print_jump_report(views, args.jump_threshold, args.jump_limit)

    if not events:
        print("fall_events=0")
        return

    print(f"fall_events={len(events)}")
    for event in events:
        print(
            "event "
            f"state={event.state.value} "
            f"frame={event.frame_index} "
            f"t_ms={event.timestamp_ms} "
            f"drop_m={event.drop_height_m:.3f} "
            f"reason={event.reason}"
        )
    if any(event.state == FallState.CONFIRMED for event in events):
        print("summary=confirmed_fall_detected")
    else:
        print("summary=no_confirmed_fall")


def _print_batch_summary(data_dir: Path, stem: str | None, tracker_config: TrackerConfig) -> None:
    stems = [stem] if stem else list(iter_sample_stems(data_dir))
    print(f"batch_recordings={len(stems)}")
    for item in stems:
        recording = load_datadealer_recording(data_dir, stem=item)
        track = track_recording(recording.frames, tracker_config)
        tracked = [frame for frame in track if frame.has_target]
        state_counts = Counter(frame.state for frame in track)
        max_jump = _max_target_jump_m(track)
        print(
            f"recording={recording.stem} "
            f"frames={len(recording.frames)} "
            f"target_frames={len(tracked)} "
            f"coverage={len(tracked) / max(len(track), 1):.1%} "
            "states "
            f"tracking={state_counts.get('tracking', 0)} "
            f"static_lock={state_counts.get('static_lock', 0)} "
            f"hold={state_counts.get('hold', 0)} "
            f"reacquire={state_counts.get('reacquire', 0)} "
            f"lost={state_counts.get('lost', 0)} "
            f"max_jump_m={max_jump:.3f}"
        )


def _max_target_jump_m(track) -> float:
    max_jump = 0.0
    previous = None
    for target in track:
        if not target.has_target or target.center_x_m is None or target.center_y_m is None:
            continue
        if previous is not None:
            jump = _target_jump_m(previous, target)
            if jump is not None:
                max_jump = max(max_jump, jump)
        previous = target
    return max_jump


def _print_jump_report(views, threshold_m: float, limit: int) -> None:
    jumps = _target_jumps(views, threshold_m)
    shown = jumps[: max(limit, 0)]
    print(f"jump_report threshold_m={threshold_m:.3f} count={len(jumps)} shown={len(shown)}")
    for previous, current, jump_m in shown:
        print(
            "jump "
            f"frame={previous.frame.frame_index}->{current.frame.frame_index} "
            f"t_ms={previous.frame.timestamp_ms}->{current.frame.timestamp_ms} "
            f"jump_m={jump_m:.3f}"
        )
        print(
            "  "
            f"prev=(x={_fmt_m(previous.target.center_x_m)}, y={_fmt_m(previous.target.center_y_m)}, z={_fmt_m(previous.target.center_z_m)}) "
            f"prev_state={previous.target.state} "
            f"prev_points={previous.target.point_count} "
            f"prev_motion={previous.target.motion_count}"
        )
        print(
            "  "
            f"curr=(x={_fmt_m(current.target.center_x_m)}, y={_fmt_m(current.target.center_y_m)}, z={_fmt_m(current.target.center_z_m)}) "
            f"curr_state={current.target.state} "
            f"curr_points={current.target.point_count} "
            f"curr_motion={current.target.motion_count}"
        )
        print(f"  candidates={len(current.debug_candidates)} diagnosis={format_target_diagnosis(current)}")
        for candidate in current.debug_candidates:
            gate = "in_gate" if candidate.within_gate else "out_gate"
            print(f"    {format_debug_candidate_label(candidate)} {gate}")


def _target_jumps(views, threshold_m: float):
    jumps = []
    previous = None
    for current in views:
        if not current.target.has_target or current.target.center_x_m is None or current.target.center_y_m is None:
            continue
        if previous is not None:
            jump = _target_jump_m(previous.target, current.target)
            if jump is not None and jump >= threshold_m:
                jumps.append((previous, current, jump))
        previous = current
    return jumps


def _target_jump_m(previous, current) -> float | None:
    if (
        previous.center_x_m is None
        or previous.center_y_m is None
        or current.center_x_m is None
        or current.center_y_m is None
    ):
        return None
    dx = current.center_x_m - previous.center_x_m
    dy = current.center_y_m - previous.center_y_m
    return (dx * dx + dy * dy) ** 0.5


def _fmt_m(value: float | None) -> str:
    return "-" if value is None else f"{value:.3f}"


if __name__ == "__main__":
    main()
