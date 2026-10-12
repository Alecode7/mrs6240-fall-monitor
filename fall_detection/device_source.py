from __future__ import annotations

import importlib
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Iterable

from .datadealer_loader import CombinedFrame, Point3D
from .live_source import LatestFrameBuffer


POINT_CLOUD_DATA_TYPE = 4
DEFAULT_SNR_DB = 30.0


@dataclass(frozen=True)
class HifDeviceConfig:
    transport: str = "spi"
    spi_index: int = 0
    spi_freq_mode: int = 0
    uart_com: str = "COM12"
    uart_baud_rate: str = "1000000"
    dll_path: str | Path | None = None


class HifDevicePointCloudSource:
    def __init__(
        self,
        *,
        config: HifDeviceConfig | None = None,
        transport: str | None = None,
        spi_index: int | None = None,
        spi_freq_mode: int | None = None,
        uart_com: str | None = None,
        uart_baud_rate: str | None = None,
        dll_path: str | Path | None = None,
        api_factory: Callable[[], object] | None = None,
    ):
        base = config or HifDeviceConfig()
        self.config = HifDeviceConfig(
            transport=transport or base.transport,
            spi_index=base.spi_index if spi_index is None else spi_index,
            spi_freq_mode=base.spi_freq_mode if spi_freq_mode is None else spi_freq_mode,
            uart_com=uart_com or base.uart_com,
            uart_baud_rate=uart_baud_rate or base.uart_baud_rate,
            dll_path=dll_path if dll_path is not None else base.dll_path,
        )
        self.api_factory = api_factory
        self.api: object | None = None
        self.buffer: LatestFrameBuffer | None = None
        self._running = False

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self, buffer: LatestFrameBuffer) -> None:
        if self._running:
            return
        self.buffer = buffer
        self.api = self._create_api()
        self.api.callback += self._data_callback
        if self.config.transport == "uart":
            opened = self.api.OpenUartDevice(self.config.uart_com, self.config.uart_baud_rate)
        else:
            opened = self.api.OpenSpiDevice(self.config.spi_index, self.config.spi_freq_mode)
        if not opened:
            self.api.callback -= self._data_callback
            self.api = None
            raise RuntimeError(f"failed to open {self.config.transport} device")
        self.api.StartCollectingData()
        self._running = True

    def stop(self) -> None:
        if self.api is None:
            self._running = False
            return
        try:
            self.api.StopCollectingData()
        finally:
            try:
                if self.config.transport == "uart":
                    self.api.CloseUartDevice()
                else:
                    self.api.CloseSpiDevice(self.config.spi_index)
            finally:
                self.api.callback -= self._data_callback
                self._running = False

    def join(self, timeout_s: float | None = None) -> None:
        _ = timeout_s

    def _create_api(self) -> object:
        if self.api_factory is not None:
            return self.api_factory()
        dll_path = Path(self.config.dll_path) if self.config.dll_path is not None else _default_hif_dll_path()
        if not dll_path.exists():
            raise FileNotFoundError(f"HifMsgDataCollectionLib.dll not found: {dll_path}")
        dll_dir = str(dll_path.parent)
        if dll_dir not in sys.path:
            sys.path.append(dll_dir)
        try:
            import clr
        except ImportError as exc:
            raise RuntimeError("pythonnet is required to use HifMsgDataCollectionLib.dll") from exc
        clr.AddReference(dll_path.stem)
        module = importlib.import_module(dll_path.stem)
        return module.HifMsgDataCollectionApi()

    def _data_callback(self, _sender, receive_data) -> None:
        if self.buffer is None or int(getattr(receive_data, "dataType", -1)) != POINT_CLOUD_DATA_TYPE:
            return
        payload = _payload_bytes(receive_data)
        point_clouds = self.api.ConvertPointCloudData(
            payload,
            getattr(receive_data, "frameIndex", 0),
            getattr(receive_data, "frameLen", 0),
        )
        raw_frame_index = getattr(receive_data, "frameIndex", 0)
        raw_timestamp = getattr(receive_data, "timeStamp", raw_frame_index)
        timestamp_ms = _coerce_timestamp_ms(raw_timestamp, fallback=_coerce_timestamp_ms(raw_frame_index, fallback=0))
        frame_index = _coerce_int(raw_frame_index, fallback=timestamp_ms)
        frame = point_clouds_to_combined_frame(
            point_clouds or (),
            frame_index=frame_index,
            timestamp_ms=timestamp_ms,
            timestamp_text=str(raw_timestamp),
        )
        self.buffer.publish(frame)


def point_clouds_to_combined_frame(
    point_clouds: Iterable[object],
    *,
    frame_index: int,
    timestamp_ms: int,
    timestamp_text: str | None = None,
) -> CombinedFrame:
    motion_points: list[Point3D] = []
    presence_points: list[Point3D] = []
    for point_cloud in point_clouds:
        kind = _point_cloud_kind(point_cloud)
        points = _points_from_point_cloud(point_cloud, kind=kind)
        if kind == "presence":
            presence_points.extend(points)
        else:
            motion_points.extend(points)
    return CombinedFrame(
        frame_index=frame_index,
        timestamp_ms=timestamp_ms,
        timestamp_text=timestamp_text or f"t{timestamp_ms}",
        presence_points=tuple(presence_points),
        motion_points=tuple(motion_points),
    )


def _points_from_point_cloud(point_cloud: object, *, kind: str) -> list[Point3D]:
    point_count = int(getattr(point_cloud, "pointNum", 0))
    xs = _series(point_cloud, "x", point_count, default=0.0)
    ys = _series(point_cloud, "y", point_count, default=0.0)
    zs = _series(point_cloud, "z", point_count, default=0.0)
    velocities = _series_any(point_cloud, ("velocity", "vel", "v_m_s", "w"), point_count, default=0.0)
    snrs = _series_any(point_cloud, ("snr", "snr_db", "u"), point_count, default=DEFAULT_SNR_DB)
    return [
        Point3D(
            x_m=xs[index],
            y_m=ys[index],
            z_m=zs[index],
            velocity_m_s=velocities[index],
            snr_db=snrs[index],
            kind=kind,
        )
        for index in range(point_count)
    ]


def _series_any(point_cloud: object, names: tuple[str, ...], point_count: int, *, default: float) -> list[float]:
    for name in names:
        if hasattr(point_cloud, name):
            return _series(point_cloud, name, point_count, default=default)
    return [default] * point_count


def _series(point_cloud: object, name: str, point_count: int, *, default: float) -> list[float]:
    values = getattr(point_cloud, name, None)
    if values is None:
        return [default] * point_count
    return [float(values[index]) if index < len(values) else default for index in range(point_count)]


def _point_cloud_kind(point_cloud: object) -> str:
    signal_name = str(getattr(point_cloud, "signalName", "")).lower()
    if "mpc" in signal_name or "presence" in signal_name:
        return "presence"
    return "motion"


def _payload_bytes(receive_data) -> object:
    payload = getattr(receive_data, "payloadData", None)
    if payload is not None:
        return payload
    return getattr(receive_data, "pointCloudRawDataOrStringData", None)


def _coerce_timestamp_ms(value: object, *, fallback: int) -> int:
    if value is None:
        return fallback
    if isinstance(value, (int, float)):
        return int(value)
    text = str(value).strip()
    if not text:
        return fallback
    try:
        return int(float(text))
    except ValueError:
        pass
    for fmt in ("%Y-%m-%d-%H:%M:%S.%f", "%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d-%H:%M:%S", "%Y-%m-%d %H:%M:%S"):
        try:
            parsed = datetime.strptime(text, fmt).replace(tzinfo=timezone.utc)
            return int(parsed.timestamp() * 1000)
        except ValueError:
            continue
    return fallback


def _coerce_int(value: object, *, fallback: int) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return fallback


def _default_hif_dll_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "SDK"
        / "Software_Kit"
        / "03_Tool"
        / "HifMsgDataCollectionLib"
        / "HifMsgDataCollectionLib.dll"
    )
