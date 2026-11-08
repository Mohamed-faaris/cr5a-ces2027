from cr5a_ros_bridge.gripper_profiles import (
    DEFAULT_GRIPPER_PROFILES,
    GripperProfileStore,
)


def test_profiles_load_defaults_then_persist_user_edits(tmp_path):
    path = tmp_path / "config" / "gripper_profiles.yaml"
    store = GripperProfileStore(path, DEFAULT_GRIPPER_PROFILES)
    assert store.active()["id"] == "balanced"

    created = store.upsert({
        "name": "Soft insert",
        "force_percent": 24,
        "speed_percent": 15,
    })
    store.set_active(created["id"])

    reloaded = GripperProfileStore(path, DEFAULT_GRIPPER_PROFILES)
    assert reloaded.active() == created
    assert reloaded.snapshot()["storage_path"] == str(path)


def test_profiles_reject_invalid_percentages_and_duplicate_names(tmp_path):
    store = GripperProfileStore(tmp_path / "profiles.yaml", DEFAULT_GRIPPER_PROFILES)
    try:
        store.upsert({
            "name": "Invalid force",
            "force_percent": 19,
            "speed_percent": 50,
        })
        assert False, "Expected invalid force to be rejected"
    except ValueError as exc:
        assert "force_percent" in str(exc)

    try:
        store.upsert({
            "name": "gentle",
            "force_percent": 30,
            "speed_percent": 30,
        })
        assert False, "Expected duplicate name to be rejected"
    except ValueError as exc:
        assert "unique" in str(exc)


def test_active_profile_cannot_be_deleted(tmp_path):
    store = GripperProfileStore(tmp_path / "profiles.yaml", DEFAULT_GRIPPER_PROFILES)
    try:
        store.delete("balanced")
        assert False, "Expected active profile delete to be rejected"
    except ValueError as exc:
        assert "active profile" in str(exc)
