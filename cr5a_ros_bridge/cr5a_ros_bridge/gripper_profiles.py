"""Persistent, validated PGEA gripper force/speed profiles.

User profiles are stored outside the source tree so rebuilding or updating the
ROS workspace does not overwrite them.
"""
from __future__ import annotations

import os
import re
import uuid
from pathlib import Path

import yaml


PROFILE_ID_RE = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$")
MAX_PROFILES = 50
DEFAULT_GRIPPER_PROFILES = [
    {"id": "gentle", "name": "Gentle", "force_percent": 30, "speed_percent": 25},
    {"id": "balanced", "name": "Balanced", "force_percent": 60, "speed_percent": 50},
    {"id": "firm", "name": "Firm", "force_percent": 80, "speed_percent": 30},
]


def resolve_profiles_path(gripper_config: dict) -> Path:
    configured = os.getenv("CR5A_GRIPPER_PROFILES_FILE") or gripper_config.get("profiles_file")
    if configured:
        return Path(str(configured)).expanduser()
    config_home = Path(os.getenv("XDG_CONFIG_HOME", "~/.config")).expanduser()
    return config_home / "cr5a_ros_bridge" / "gripper_profiles.yaml"


def validate_profile(payload: dict, *, allow_missing_id: bool = False) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("profile must be an object")

    profile_id = payload.get("id")
    if profile_id is None and allow_missing_id:
        profile_id = uuid.uuid4().hex
    if not isinstance(profile_id, str) or not PROFILE_ID_RE.fullmatch(profile_id):
        raise ValueError("profile id must contain 1–64 letters, numbers, underscores or hyphens")

    name = payload.get("name")
    if not isinstance(name, str) or not name.strip() or len(name.strip()) > 48:
        raise ValueError("profile name must contain 1–48 characters")

    def percent(key: str, minimum: int) -> int:
        value = payload.get(key)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{key} must be an integer")
        if not minimum <= value <= 100:
            raise ValueError(f"{key} must be between {minimum} and 100")
        return value

    return {
        "id": profile_id,
        "name": name.strip(),
        "force_percent": percent("force_percent", 20),
        "speed_percent": percent("speed_percent", 1),
    }


class GripperProfileStore:
    """Small YAML-backed store with atomic writes and strict value validation."""

    def __init__(self, path: Path, defaults: list[dict], default_profile_id: str = "balanced"):
        self.path = Path(path).expanduser()
        self.defaults = [validate_profile(item) for item in defaults]
        self.default_profile_id = default_profile_id
        self.profiles = list(self.defaults)
        self.active_profile_id = default_profile_id
        self._load()

    def _validate_collection(self, profiles: list[dict]) -> list[dict]:
        if not isinstance(profiles, list) or not profiles or len(profiles) > MAX_PROFILES:
            raise ValueError(f"profiles must be a list containing 1–{MAX_PROFILES} entries")
        normalized = [validate_profile(item) for item in profiles]
        ids = [item["id"] for item in normalized]
        names = [item["name"].casefold() for item in normalized]
        if len(ids) != len(set(ids)):
            raise ValueError("profile ids must be unique")
        if len(names) != len(set(names)):
            raise ValueError("profile names must be unique")
        return normalized

    def _load(self) -> None:
        if not self.path.exists():
            if self.default_profile_id not in {p["id"] for p in self.profiles}:
                self.active_profile_id = self.profiles[0]["id"]
            return
        try:
            with self.path.open("r", encoding="utf-8") as handle:
                document = yaml.safe_load(handle) or {}
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
            raise ValueError(f"Cannot read gripper profile file {self.path}: {exc}") from exc
        if not isinstance(document, dict) or document.get("version") != 1:
            raise ValueError(f"Unsupported gripper profile file format: {self.path}")
        profiles = self._validate_collection(document.get("profiles"))
        active_id = document.get("active_profile_id")
        if active_id not in {p["id"] for p in profiles}:
            raise ValueError("active_profile_id must refer to an existing profile")
        self.profiles = profiles
        self.active_profile_id = active_id

    def get(self, profile_id: str) -> dict | None:
        return next((dict(item) for item in self.profiles if item["id"] == profile_id), None)

    def active(self) -> dict:
        profile = self.get(self.active_profile_id)
        if profile is None:
            raise ValueError("The active gripper profile no longer exists")
        return profile

    def _write(self, profiles: list[dict], active_profile_id: str) -> None:
        normalized = self._validate_collection(profiles)
        if active_profile_id not in {p["id"] for p in normalized}:
            raise ValueError("active_profile_id must refer to an existing profile")
        document = {
            "version": 1,
            "active_profile_id": active_profile_id,
            "profiles": normalized,
        }
        temporary = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_name(
                self.path.name + f".{os.getpid()}.{uuid.uuid4().hex}.tmp"
            )
            with temporary.open("w", encoding="utf-8") as handle:
                os.fchmod(handle.fileno(), 0o600)
                yaml.safe_dump(document, handle, sort_keys=False, allow_unicode=True)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.path)
        except OSError as exc:
            try:
                temporary.unlink(missing_ok=True)
            except Exception:
                pass
            raise ValueError(f"Cannot save gripper profiles to {self.path}: {exc}") from exc
        self.profiles = normalized
        self.active_profile_id = active_profile_id

    def upsert(self, payload: dict) -> dict:
        profile = validate_profile(payload, allow_missing_id=True)
        current = [dict(item) for item in self.profiles]
        existing_index = next(
            (index for index, item in enumerate(current) if item["id"] == profile["id"]),
            None,
        )
        if existing_index is None:
            current.append(profile)
        else:
            current[existing_index] = profile
        self._write(current, self.active_profile_id)
        return dict(profile)

    def delete(self, profile_id: str) -> None:
        if profile_id == self.active_profile_id:
            raise ValueError("Apply another profile before deleting the active profile")
        current = [dict(item) for item in self.profiles if item["id"] != profile_id]
        if len(current) == len(self.profiles):
            raise KeyError(f"Unknown gripper profile: {profile_id}")
        self._write(current, self.active_profile_id)

    def set_active(self, profile_id: str) -> dict:
        profile = self.get(profile_id)
        if profile is None:
            raise KeyError(f"Unknown gripper profile: {profile_id}")
        self._write(self.profiles, profile_id)
        return profile

    def snapshot(self, runtime_synced: bool = False) -> dict:
        return {
            "profiles": [dict(item) for item in self.profiles],
            "active_profile_id": self.active_profile_id,
            "storage_path": str(self.path),
            "runtime_synced": bool(runtime_synced),
        }
