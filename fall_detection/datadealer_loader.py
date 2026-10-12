from __future__ import annotations

import csv
from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path
from typing import Iterable, Iterator


DATADEALER_MOTION_SUFFIX = "_motion_point_cloud.csv"
DATADEALER_PRESENCE_SUFFIX = "_presence_point_cloud.csv"
SDK_MOTION_SUFFIX = "_Point_cloud_conversion.csv"
SDK_PRESENCE_SUFFIX = "_Point_cloud_MPC_conversion.csv"
SDK_FRAME_PERIOD_MS = 50


@dataclass(frozen=True)
class Point3D:
    """One MRS6240 point in radar-front coordinates.

    Coordinate convention used by this project:
    - z_m: physical height.
    - y_m: radar-facing forward distance.
    - x_m: radar-left is positive.
    """

    x_m: float
    y_m: float
    z_m: float
    velocity_m_s: float
    snr_db: float
    kind: str


@dataclass(frozen=True)
class PointCloudFrame:
    frame_index: int
    timestamp_ms: int
    timestamp_text: str
    kind: str
    points: tuple[Point3D, ...]

    @property
    def point_count(self) -> int:
        return len(self.points)


@dataclass(frozen=True)
class CombinedFrame:
    frame_index: int
    timestamp_ms: int
    timestamp_text: str
    presence_points: tuple[Point3D, ...]
    motion_points: tuple[Point3D, ...]

    @property
    def presence_count(self) -> int:
        return len(self.presence_points)

    @property
    def motion_count(self) -> int:
        return len(self.motion_points)

    @property
    def all_points(self) -> tuple[Point3D, ...]:
        return self.presence_points + self.motion_points


@dataclass(frozen=True)
class DatadealerRecording:
    stem: str
    motion_path: Path
    presence_path: Path
    frames: tuple[CombinedFrame, ...]

    @property
    def duration_s(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        return (self.frames[-1].timestamp_ms - self.frames[0].timestamp_ms) / 1000.0


def parse_pointcloud_row(row: list[str], *, kind: str, frame_index: int) -> PointCloudFrame:
    if len(row) < 2:
        raise ValueError(f"row {frame_index} has fewer than two timestamp columns")

    timestamp_text = row[0]
    timestamp_ms = int(row[1])
    values = [float(value) for value in row[2:] if value != ""]
    if len(values) % 5 != 0:
        raise ValueError(f"row {frame_index} point value count is not divisible by 5: {len(values)}")

    point_count = len(values) // 5
    xs = values[0:point_count]
    ys = values[point_count : 2 * point_count]
    zs = values[2 * point_count : 3 * point_count]
    velocities = values[3 * point_count : 4 * point_count]
    snrs = values[4 * point_count : 5 * point_count]

    points = tuple(
        Point3D(
            x_m=xs[index],
            y_m=ys[index],
            z_m=zs[index],
            velocity_m_s=velocities[index],
            snr_db=snrs[index],
            kind=kind,
        )
        for index in range(point_count)
    )

    return PointCloudFrame(
        frame_index=frame_index,
        timestamp_ms=timestamp_ms,
        timestamp_text=timestamp_text,
        kind=kind,
        points=points,
    )


def iter_pointcloud_csv(path: str | Path, *, kind: str) -> Iterator[PointCloudFrame]:
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        for frame_index, row in enumerate(csv.reader(stream)):
            if not row:
                continue
            yield parse_pointcloud_row(row, kind=kind, frame_index=frame_index)


def iter_sdk_pointcloud_csv(path: str | Path, *, kind: str) -> Iterator[PointCloudFrame]:
    frames: dict[int, list[tuple[int, Point3D]]] = {}
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            return
        fieldnames = {name.strip() for name in reader.fieldnames}
        is_cartesian = {"FrameIndex", "PointIndex", "X", "Y", "Z", "Velocity", "SNR"} <= fieldnames
        is_polar = {"FrameIndex", "PointIndex", "Range", "Azimuth", "Elevation", "Velocity", "SNR"} <= fieldnames
        if not is_cartesian and not is_polar:
            raise ValueError(f"unsupported SDK point-cloud csv header in {path}: {reader.fieldnames}")

        for row_number, row in enumerate(reader, start=2):
            if not row or not row.get("FrameIndex"):
                continue
            frame_number = int(float(_required_cell(row, "FrameIndex", row_number)))
            point_index = int(float(_required_cell(row, "PointIndex", row_number)))
            velocity = float(_required_cell(row, "Velocity", row_number))
            snr = float(_required_cell(row, "SNR", row_number))
            if is_cartesian:
                x = float(_required_cell(row, "X", row_number))
                y = float(_required_cell(row, "Y", row_number))
                z = float(_required_cell(row, "Z", row_number))
            else:
                x, y, z = _polar_to_cartesian(
                    range_m=float(_required_cell(row, "Range", row_number)),
                    azimuth_deg=float(_required_cell(row, "Azimuth", row_number)),
                    elevation_deg=float(_required_cell(row, "Elevation", row_number)),
                )
            frames.setdefault(frame_number, []).append(
                (
                    point_index,
                    Point3D(
                        x_m=x,
                        y_m=y,
                        z_m=z,
                        velocity_m_s=velocity,
                        snr_db=snr,
                        kind=kind,
                    ),
                )
            )

    for frame_number in sorted(frames):
        points = tuple(point for _, point in sorted(frames[frame_number], key=lambda item: item[0]))
        yield PointCloudFrame(
            frame_index=frame_number,
            timestamp_ms=frame_number,
            timestamp_text=f"frame {frame_number}",
            kind=kind,
            points=points,
        )


def load_datadealer_recording(data_dir: str | Path, *, stem: str | None = None) -> DatadealerRecording:
    data_dir = Path(data_dir)
    motion_path, presence_path, resolved_stem = _resolve_recording_paths(data_dir, stem)

    motion_iter = _iter_pointcloud_file(motion_path, kind="motion")
    presence_iter = _iter_pointcloud_file(presence_path, kind="presence")
    motion_by_ts = {frame.timestamp_ms: frame for frame in motion_iter}
    presence_by_ts = {frame.timestamp_ms: frame for frame in presence_iter}
    is_sdk_recording = _is_sdk_pointcloud_file(motion_path) or _is_sdk_pointcloud_file(presence_path)
    timestamps = _recording_timestamps(
        motion_by_ts,
        presence_by_ts,
        fill_missing=is_sdk_recording,
    )
    sdk_first_frame_index = timestamps[0] if is_sdk_recording and timestamps else 0

    frames = []
    for frame_index, source_frame_index in enumerate(timestamps):
        motion = motion_by_ts.get(source_frame_index)
        presence = presence_by_ts.get(source_frame_index)
        timestamp_text = (
            presence.timestamp_text
            if presence is not None
            else motion.timestamp_text
            if motion is not None
            else f"frame {source_frame_index}"
            if is_sdk_recording
            else str(source_frame_index)
        )
        timestamp_ms = (
            _sdk_frame_time_ms(source_frame_index, sdk_first_frame_index)
            if is_sdk_recording
            else source_frame_index
        )
        frames.append(
            CombinedFrame(
                frame_index=frame_index,
                timestamp_ms=timestamp_ms,
                timestamp_text=timestamp_text,
                presence_points=presence.points if presence is not None else (),
                motion_points=motion.points if motion is not None else (),
            )
        )

    return DatadealerRecording(
        stem=resolved_stem,
        motion_path=motion_path,
        presence_path=presence_path,
        frames=tuple(frames),
    )


def _recording_timestamps(
    motion_by_ts: dict[int, PointCloudFrame],
    presence_by_ts: dict[int, PointCloudFrame],
    *,
    fill_missing: bool,
) -> list[int]:
    observed = sorted(set(motion_by_ts) | set(presence_by_ts))
    if not observed or not fill_missing:
        return observed
    return list(range(observed[0], observed[-1] + 1))


def _sdk_frame_time_ms(source_frame_index: int, first_frame_index: int) -> int:
    return (source_frame_index - first_frame_index) * SDK_FRAME_PERIOD_MS


def iter_sample_stems(data_dir: str | Path) -> Iterator[str]:
    data_dir = Path(data_dir)
    seen = set()
    for recording_dir in _iter_candidate_stem_dirs(data_dir):
        for suffix in (DATADEALER_MOTION_SUFFIX, SDK_MOTION_SUFFIX):
            for path in sorted(recording_dir.glob(f"*{suffix}")):
                stem = path.name[: -len(suffix)]
                if stem in seen:
                    continue
                seen.add(stem)
                yield stem


def _resolve_recording_paths(data_dir: Path, stem: str | None) -> tuple[Path, Path, str]:
    if stem is None:
        stems = list(iter_sample_stems(data_dir))
        if not stems:
            raise FileNotFoundError(f"no *_motion_point_cloud.csv found in {data_dir}")
        stem = stems[0]

    expected_names = (
        (f"{stem}{DATADEALER_MOTION_SUFFIX}", f"{stem}{DATADEALER_PRESENCE_SUFFIX}"),
        (f"{stem}{SDK_MOTION_SUFFIX}", f"{stem}{SDK_PRESENCE_SUFFIX}"),
    )
    for recording_dir in _candidate_recording_dirs(data_dir, stem):
        for motion_name, presence_name in expected_names:
            motion_path = recording_dir / motion_name
            presence_path = recording_dir / presence_name
            if motion_path.exists() and presence_path.exists():
                return motion_path, presence_path, stem

    expected = " or ".join(f"{motion} + {presence}" for motion, presence in expected_names)
    raise FileNotFoundError(f"point cloud csv pair not found in {data_dir}: {expected}")


def _iter_candidate_stem_dirs(data_dir: Path) -> Iterator[Path]:
    if not data_dir.exists():
        return
    yield data_dir
    for child in sorted(data_dir.iterdir()):
        if child.is_dir():
            yield child


def _candidate_recording_dirs(data_dir: Path, stem: str) -> Iterator[Path]:
    yield data_dir
    session_dir = data_dir / stem
    if session_dir != data_dir:
        yield session_dir


def _iter_pointcloud_file(path: Path, *, kind: str) -> Iterator[PointCloudFrame]:
    if _is_sdk_pointcloud_file(path):
        yield from iter_sdk_pointcloud_csv(path, kind=kind)
    else:
        yield from iter_pointcloud_csv(path, kind=kind)


def _is_sdk_pointcloud_file(path: Path) -> bool:
    return path.name.endswith(SDK_MOTION_SUFFIX) or path.name.endswith(SDK_PRESENCE_SUFFIX)


def _required_cell(row: dict[str, str], key: str, row_number: int) -> str:
    value = row.get(key)
    if value is None or value == "":
        raise ValueError(f"missing {key} at row {row_number}")
    return value


def _polar_to_cartesian(*, range_m: float, azimuth_deg: float, elevation_deg: float) -> tuple[float, float, float]:
    azimuth = radians(azimuth_deg)
    elevation = radians(elevation_deg)
    horizontal = range_m * cos(elevation)
    return (
        horizontal * sin(azimuth),
        horizontal * cos(azimuth),
        range_m * sin(elevation),
    )


def count_points(frames: Iterable[CombinedFrame]) -> tuple[int, int]:
    motion = 0
    presence = 0
    for frame in frames:
        motion += frame.motion_count
        presence += frame.presence_count
    return motion, presence
