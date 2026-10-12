from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin
from statistics import median
from typing import Iterable

from .datadealer_loader import CombinedFrame, Point3D


@dataclass(frozen=True)
class GroundPlane:
    """Ground height model in the pitched and translated coordinate system."""

    x_slope: float
    y_slope: float
    intercept_m: float
    point_count: int
    rmse_m: float

    def height_at(self, x_m: float, y_m: float) -> float:
        return self.x_slope * x_m + self.y_slope * y_m + self.intercept_m


def rotate_yz_point(point: Point3D, rotation_deg: float) -> Point3D:
    if abs(rotation_deg) < 1e-9:
        return point
    cos_angle = cos(radians(rotation_deg))
    sin_angle = sin(radians(rotation_deg))
    return _rotate_yz_point_with_trig(point, cos_angle, sin_angle)


def rotate_yz_frame(frame: CombinedFrame, rotation_deg: float) -> CombinedFrame:
    if abs(rotation_deg) < 1e-9:
        return frame
    cos_angle = cos(radians(rotation_deg))
    sin_angle = sin(radians(rotation_deg))
    return CombinedFrame(
        frame_index=frame.frame_index,
        timestamp_ms=frame.timestamp_ms,
        timestamp_text=frame.timestamp_text,
        presence_points=tuple(_rotate_yz_point_with_trig(point, cos_angle, sin_angle) for point in frame.presence_points),
        motion_points=tuple(_rotate_yz_point_with_trig(point, cos_angle, sin_angle) for point in frame.motion_points),
    )


def transform_point_to_world(
    point: Point3D,
    *,
    yz_rotation_deg: float = 0.0,
    mount_height_m: float = 0.0,
    ground_plane: GroundPlane | None = None,
) -> Point3D:
    if abs(yz_rotation_deg) < 1e-9 and abs(mount_height_m) < 1e-9 and ground_plane is None:
        return point

    rotated = rotate_yz_point(point, yz_rotation_deg)
    z_m = rotated.z_m + float(mount_height_m)
    if ground_plane is not None:
        z_m -= ground_plane.height_at(rotated.x_m, rotated.y_m)
    return Point3D(
        x_m=rotated.x_m,
        y_m=rotated.y_m,
        z_m=z_m,
        velocity_m_s=rotated.velocity_m_s,
        snr_db=rotated.snr_db,
        kind=rotated.kind,
    )


def transform_frame_to_world(
    frame: CombinedFrame,
    *,
    yz_rotation_deg: float = 0.0,
    mount_height_m: float = 0.0,
    ground_plane: GroundPlane | None = None,
) -> CombinedFrame:
    if abs(yz_rotation_deg) < 1e-9 and abs(mount_height_m) < 1e-9 and ground_plane is None:
        return frame
    return CombinedFrame(
        frame_index=frame.frame_index,
        timestamp_ms=frame.timestamp_ms,
        timestamp_text=frame.timestamp_text,
        presence_points=tuple(
            transform_point_to_world(
                point,
                yz_rotation_deg=yz_rotation_deg,
                mount_height_m=mount_height_m,
                ground_plane=ground_plane,
            )
            for point in frame.presence_points
        ),
        motion_points=tuple(
            transform_point_to_world(
                point,
                yz_rotation_deg=yz_rotation_deg,
                mount_height_m=mount_height_m,
                ground_plane=ground_plane,
            )
            for point in frame.motion_points
        ),
    )


def calibrate_ground_plane(
    frames: Iterable[CombinedFrame],
    *,
    yz_rotation_deg: float = 0.0,
    mount_height_m: float = 0.0,
    frame_count: int = 200,
    ground_band_m: float = 0.25,
    min_snr_db: float = 10.0,
    min_points: int = 30,
    max_slope: float = 0.35,
) -> GroundPlane:
    if frame_count <= 0:
        raise ValueError("ground calibration frame_count must be positive")
    if ground_band_m <= 0:
        raise ValueError("ground calibration band must be positive")
    if min_points < 3:
        raise ValueError("ground calibration requires at least three points")

    candidates: list[tuple[float, float, float]] = []
    for index, frame in enumerate(frames):
        if index >= frame_count:
            break
        world_frame = transform_frame_to_world(
            frame,
            yz_rotation_deg=yz_rotation_deg,
            mount_height_m=mount_height_m,
        )
        for point in world_frame.presence_points:
            if point.snr_db < min_snr_db or abs(point.z_m) > ground_band_m:
                continue
            candidates.append((point.x_m, point.y_m, point.z_m))

    if len(candidates) < min_points:
        raise ValueError(
            f"ground calibration found {len(candidates)} candidate points; "
            f"need at least {min_points}. Check mount height, pitch angle, or ground band."
        )

    sampled = _limit_calibration_points(candidates, max_points=12000)
    coefficients = _fit_plane(sampled)
    inliers = _plane_inliers(sampled, coefficients, ground_band_m)
    if len(inliers) < min_points:
        raise ValueError(
            f"ground calibration kept {len(inliers)} inliers; need at least {min_points}"
        )
    coefficients = _fit_plane(inliers)
    inliers = _plane_inliers(inliers, coefficients, ground_band_m)
    if len(inliers) >= min_points:
        coefficients = _fit_plane(inliers)

    x_slope, y_slope, intercept = coefficients
    if (x_slope * x_slope + y_slope * y_slope) ** 0.5 > max_slope:
        raise ValueError(
            "ground calibration slope is too large; check pitch angle, mount height, or calibration scene"
        )
    rmse = _plane_rmse(inliers, coefficients)
    return GroundPlane(
        x_slope=x_slope,
        y_slope=y_slope,
        intercept_m=intercept,
        point_count=len(inliers),
        rmse_m=rmse,
    )


def _rotate_yz_point_with_trig(point: Point3D, cos_angle: float, sin_angle: float) -> Point3D:
    y_m = point.y_m * cos_angle + point.z_m * sin_angle
    z_m = point.z_m * cos_angle - point.y_m * sin_angle
    return Point3D(
        x_m=point.x_m,
        y_m=y_m,
        z_m=z_m,
        velocity_m_s=point.velocity_m_s,
        snr_db=point.snr_db,
        kind=point.kind,
    )


def _limit_calibration_points(
    points: list[tuple[float, float, float]],
    *,
    max_points: int,
) -> list[tuple[float, float, float]]:
    if len(points) <= max_points:
        return points
    step = len(points) / max_points
    return [points[min(int(index * step), len(points) - 1)] for index in range(max_points)]


def _fit_plane(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    count = float(len(points))
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    sum_z = sum(point[2] for point in points)
    sum_xx = sum(point[0] * point[0] for point in points)
    sum_xy = sum(point[0] * point[1] for point in points)
    sum_yy = sum(point[1] * point[1] for point in points)
    sum_xz = sum(point[0] * point[2] for point in points)
    sum_yz = sum(point[1] * point[2] for point in points)
    matrix = [
        [sum_xx, sum_xy, sum_x, sum_xz],
        [sum_xy, sum_yy, sum_y, sum_yz],
        [sum_x, sum_y, count, sum_z],
    ]
    return _solve_three_by_three(matrix)


def _solve_three_by_three(matrix: list[list[float]]) -> tuple[float, float, float]:
    rows = [row[:] for row in matrix]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row_index: abs(rows[row_index][column]))
        if abs(rows[pivot][column]) < 1e-10:
            raise ValueError("ground calibration points do not span a usable plane")
        rows[column], rows[pivot] = rows[pivot], rows[column]
        pivot_value = rows[column][column]
        rows[column] = [value / pivot_value for value in rows[column]]
        for row_index in range(3):
            if row_index == column:
                continue
            factor = rows[row_index][column]
            rows[row_index] = [
                value - factor * pivot_entry
                for value, pivot_entry in zip(rows[row_index], rows[column])
            ]
    return rows[0][3], rows[1][3], rows[2][3]


def _plane_inliers(
    points: list[tuple[float, float, float]],
    coefficients: tuple[float, float, float],
    ground_band_m: float,
) -> list[tuple[float, float, float]]:
    residuals = [abs(_plane_residual(point, coefficients)) for point in points]
    median_residual = median(residuals)
    median_deviation = median(abs(value - median_residual) for value in residuals)
    robust_sigma = max(median_deviation * 1.4826, 0.01)
    threshold = min(ground_band_m, max(0.03, robust_sigma * 3.0))
    return [
        point
        for point in points
        if abs(_plane_residual(point, coefficients)) <= threshold
    ]


def _plane_residual(
    point: tuple[float, float, float],
    coefficients: tuple[float, float, float],
) -> float:
    x_slope, y_slope, intercept = coefficients
    x_m, y_m, z_m = point
    return z_m - (x_slope * x_m + y_slope * y_m + intercept)


def _plane_rmse(
    points: list[tuple[float, float, float]],
    coefficients: tuple[float, float, float],
) -> float:
    if not points:
        return 0.0
    squared_error = sum(_plane_residual(point, coefficients) ** 2 for point in points)
    return (squared_error / len(points)) ** 0.5
