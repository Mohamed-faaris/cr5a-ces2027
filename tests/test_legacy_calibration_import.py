import json
import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from import_legacy_calibration_dataset import convert_dataset


class LegacyCalibrationImportTests(unittest.TestCase):
    def make_dataset(self, root: Path, with_extrinsic: bool = True, with_board_pose: bool = False) -> None:
        (root / "samples").mkdir(parents=True)
        (root / "poses").mkdir()
        (root / "results").mkdir()
        matrix = [[1, 0, 0, 1.0], [0, 1, 0, 2.0], [0, 0, 1, 3.0], [0, 0, 0, 1]]
        board_detection = {"reprojection_rms_px": 0.8, "tag_ids": [1, 2, 3], "tag_family": "aruco_dict_5x5", "board_type": "charuco"}
        if with_board_pose:
            board_detection["camera_T_board"] = {"matrix": [[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}
        sample = {
            "pose_id": 7,
            "frames": {"base": "base_link", "tcp": "Link6", "camera": "d415_color_optical_frame"},
            "image": "images/sample_007.png",
            "base_to_tcp": {"matrix": matrix},
            "board_detection": board_detection,
            "timestamp": {"sec": 123, "nanosec": 456},
        }
        (root / "samples" / "sample_007.json").write_text(json.dumps(sample))
        (root / "poses" / "calibration_poses.jsonl").write_text(
            json.dumps({"id": 7, "type": "joint", "positions": [0, 1, 2, 3, 4, 5]}) + "\n"
        )
        if with_extrinsic:
            (root / "results" / "hand_eye_transform.yaml").write_text(
                "calibrated: true\nframes:\n  parent: Link6\n  child: d415_color_optical_frame\n"
                "transform_parent_to_child:\n  matrix:\n"
                "  - [1, 0, 0, 0.1]\n  - [0, 1, 0, 0]\n"
                "  - [0, 0, 1, 0]\n  - [0, 0, 0, 1]\n"
            )

    def test_composes_camera_pose_and_preserves_sample_metadata(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            self.make_dataset(root)
            bundle = convert_dataset(root, include_thumbnails=False)
            pose = bundle["poses"][0]
            self.assertEqual(pose["pose_frame"], "camera")
            self.assertEqual(pose["position_m"], [1.1, 2.0, 3.0])
            self.assertEqual(pose["tcp_position_m"], [1.0, 2.0, 3.0])
            self.assertEqual(pose["joints"]["joint6"], 5.0)
            self.assertEqual(pose["reprojection_rms_px"], 0.8)
            self.assertEqual(bundle["frames"]["camera"], "d415_color_optical_frame")
            self.assertIsNone(pose["image_data_url"])

    def test_falls_back_to_tcp_when_no_handeye_result_exists(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            self.make_dataset(root, with_extrinsic=False)
            bundle = convert_dataset(root, include_thumbnails=False)
            pose = bundle["poses"][0]
            self.assertEqual(pose["pose_frame"], "tcp")
            self.assertEqual(pose["position_m"], [1.0, 2.0, 3.0])
            self.assertIn("TCP origin", bundle["pose_reference"])

    def test_imports_cartesian_plan_poses_separately_from_observations(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            self.make_dataset(root)
            plan = root / "generated.jsonl"
            plan.write_text(json.dumps({
                "id": 3, "type": "cartesian",
                "pose": {"translation_m": [0.1, 0.2, 0.3], "quaternion_xyzw": [0, 0, 0, 1]},
                "tilt_deg": 15.0,
            }) + chr(10))
            bundle = convert_dataset(root, include_thumbnails=False, pose_plan_path=plan)
            self.assertEqual(len(bundle["poses"]), 1)
            self.assertEqual(len(bundle["planned_poses"]), 1)
            self.assertEqual(bundle["planned_poses"][0]["position_m"], [0.1, 0.2, 0.3])
            self.assertEqual(bundle["planned_poses"][0]["kind"], "planned")
            self.assertEqual(bundle["summary"]["planned_pose_count"], 1)
            self.assertTrue(any("preview-only" in warning for warning in bundle["warnings"]))

    def test_composes_and_exports_a_base_frame_charuco_board_pose(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            self.make_dataset(root, with_board_pose=True)
            bundle = convert_dataset(root, include_thumbnails=False)
            board = bundle["board"]
            self.assertEqual(board["frame_id"], "base_link")
            self.assertEqual(board["position_m"], [1.6, 2.0, 3.0])
            self.assertEqual(board["pose_sample_count"], 1)
            self.assertEqual(board["geometry"]["columns"], 24)
            self.assertEqual(board["geometry"]["rows"], 17)
            self.assertAlmostEqual(board["geometry"]["width_m"], 0.74)
            self.assertAlmostEqual(board["geometry"]["height_m"], 0.53)
            self.assertTrue(any("visual reference" in warning for warning in bundle["warnings"]))

    def test_board_pose_is_not_guessed_without_calibrated_camera_extrinsic(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            self.make_dataset(root, with_extrinsic=False, with_board_pose=True)
            bundle = convert_dataset(root, include_thumbnails=False)
            self.assertIsNone(bundle["board"]["position_m"])
            self.assertEqual(bundle["board"]["pose_sample_count"], 0)
            self.assertTrue(any("pose is unavailable" in warning for warning in bundle["warnings"]))

    def test_rejects_missing_sample_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(ValueError, "Missing samples directory"):
                convert_dataset(Path(temp), include_thumbnails=False)


if __name__ == "__main__":
    unittest.main()
