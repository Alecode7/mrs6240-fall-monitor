from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Mapping


CAPTURE_SCHEMA_VERSION = 2
PROTOCOL_VERSION = "fall-v2.0"
DEFAULT_RADAR_PROFILE = "mrs6240-50ms-r256-d16"
SESSION_SCENARIOS = (
    "normal",
    "crouch",
    "object_assisted",
    "fall",
    "mixed",
    "blind_test",
)
ANNOTATION_CONTEXTS = (
    "plain",
    "held_stool",
    "held_board",
    "carried_object",
    "kneeling",
    "floor_activity",
    "other",
)
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


class CaptureProfileError(ValueError):
    pass


@dataclass(frozen=True)
class CaptureProfile:
    mount_height_m: float
    pitch_deg: float
    subject_id: str
    environment_id: str
    scenario: str
    protocol_version: str = PROTOCOL_VERSION
    radar_profile: str = DEFAULT_RADAR_PROFILE

    def __post_init__(self) -> None:
        height = _finite_float(self.mount_height_m, "mount_height_m")
        pitch = _finite_float(self.pitch_deg, "pitch_deg")
        subject = _safe_id(self.subject_id, "subject_id")
        environment = _safe_id(self.environment_id, "environment_id")
        scenario = str(self.scenario).strip()
        protocol = str(self.protocol_version).strip()
        radar = str(self.radar_profile).strip()
        if not 0.5 <= height <= 5.0:
            raise CaptureProfileError("mount_height_m must be within 0.5..5.0")
        if not -90.0 <= pitch <= 90.0:
            raise CaptureProfileError("pitch_deg must be within -90..90")
        if scenario not in SESSION_SCENARIOS:
            raise CaptureProfileError(f"unsupported scenario: {scenario}")
        if not protocol:
            raise CaptureProfileError("protocol_version must not be empty")
        if not radar:
            raise CaptureProfileError("radar_profile must not be empty")
        object.__setattr__(self, "mount_height_m", height)
        object.__setattr__(self, "pitch_deg", pitch)
        object.__setattr__(self, "subject_id", subject)
        object.__setattr__(self, "environment_id", environment)
        object.__setattr__(self, "scenario", scenario)
        object.__setattr__(self, "protocol_version", protocol)
        object.__setattr__(self, "radar_profile", radar)

    def to_metadata(self) -> dict[str, object]:
        return {
            "capture_schema_version": CAPTURE_SCHEMA_VERSION,
            "mount_height_m": self.mount_height_m,
            "pitch_deg": self.pitch_deg,
            "subject_id": self.subject_id,
            "environment_id": self.environment_id,
            "scenario": self.scenario,
            "protocol_version": self.protocol_version,
            "radar_profile": self.radar_profile,
        }

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "CaptureProfile":
        return cls(
            mount_height_m=values.get("mount_height_m", ""),
            pitch_deg=values.get("pitch_deg", ""),
            subject_id=str(values.get("subject_id", "")),
            environment_id=str(values.get("environment_id", "")),
            scenario=str(values.get("scenario", "")),
            protocol_version=str(values.get("protocol_version", "")),
            radar_profile=str(values.get("radar_profile", "")),
        )


def format_annotation_note(context: str) -> str:
    normalized = str(context).strip()
    if normalized not in ANNOTATION_CONTEXTS:
        raise CaptureProfileError(f"unsupported annotation context: {normalized}")
    return f"annotation_app;context={normalized}"


def parse_annotation_context(note: str) -> str:
    for field in str(note).split(";"):
        key, separator, value = field.partition("=")
        if separator and key.strip() == "context":
            normalized = value.strip()
            return normalized if normalized in ANNOTATION_CONTEXTS else "unknown"
    return "unknown"


def _finite_float(value: object, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise CaptureProfileError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise CaptureProfileError(f"{name} must be finite")
    return result


def _safe_id(value: object, name: str) -> str:
    result = str(value).strip()
    if not _SAFE_ID.fullmatch(result):
        raise CaptureProfileError(
            f"{name} must use 1..64 letters, numbers, dot, underscore, or dash"
        )
    return result


DEFAULT_CAPTURE_PROFILE = CaptureProfile(
    mount_height_m=1.63,
    pitch_deg=30.0,
    subject_id="P01",
    environment_id="room-a",
    scenario="mixed",
)
