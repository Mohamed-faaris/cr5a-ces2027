from pathlib import Path

import yaml


def test_default_config_has_required_sections():
    config_path = Path(__file__).parents[1] / "config" / "bridge.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert {"api", "ros", "moveit", "robot", "auth", "rosbridge"} <= set(config)
    assert config["api"]["host"] == "127.0.0.1"
    assert config["moveit"]["group_name"] == "cr5_group"
    assert config["moveit"]["end_effector_link"] == "Link6"
    assert any(item["topic"] == "/joint_states" for item in config["ros"]["auto_subscribe"])
