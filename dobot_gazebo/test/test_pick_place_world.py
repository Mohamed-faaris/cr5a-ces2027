import xml.etree.ElementTree as ET
import pytest

from dobot_gazebo.scripts.generate_pick_place_world import OBJECTS, SHELVES, generate_world


@pytest.mark.parametrize("profile", list(SHELVES))
@pytest.mark.parametrize("object_kind", list(OBJECTS))
def test_generated_gazebo_world_contains_shelf_platform_and_massful_object(tmp_path, profile, object_kind):
    path = generate_world(profile, object_kind, tmp_path / f"{profile}_{object_kind}.world", level=0)
    root = ET.parse(path).getroot()
    world = root.find("world")
    models = {item.get("name"): item for item in world.findall("model")}
    assert {"pick_shelf", "place_platform", "pick_object"} <= set(models)
    mass = float(models["pick_object"].find("link/inertial/mass").text)
    assert mass == pytest.approx(OBJECTS[object_kind][2])
    shelf_link = models["pick_shelf"].find("link")
    assert len(shelf_link.findall("collision")) == SHELVES[profile][3] + 4


def test_world_generator_rejects_unknown_profile_and_level(tmp_path):
    with pytest.raises(ValueError):
        generate_world("unknown", "coke", tmp_path / "bad.world")
    with pytest.raises(ValueError):
        generate_world("compact", "coke", tmp_path / "bad.world", level=8)
