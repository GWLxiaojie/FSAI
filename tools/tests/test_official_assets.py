import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve().parents[1] / "official_assets.py"


class OfficialAssetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("official_assets", SCRIPT)
        cls.assets = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.assets)

    def test_git_object_integrity_checks_content_and_size(self):
        payload = b"official fixture\n"
        descriptor = {"size": len(payload), "git_blob_sha1": hashlib.sha1(
            b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()}
        self.assets.verify_blob(payload, descriptor)
        for corrupt in (payload + b"x", payload.replace(b"official", b"modified")):
            with self.assertRaises(ValueError):
                self.assets.verify_blob(corrupt, descriptor)

    def test_cache_path_rejects_escape_and_absolute_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(self.assets.cache_path(root, "Movie/model.glb"), root / "Movie/model.glb")
            for path in ("../escape", "/etc/passwd", "Movie/../../escape"):
                with self.assertRaises(ValueError):
                    self.assets.cache_path(root, path)
            (root / "link").symlink_to(root.parent, target_is_directory=True)
            with self.assertRaises(ValueError):
                self.assets.cache_path(root, "link/outside")

    def test_lock_uses_official_owner_immutable_commit_and_bounded_files(self):
        lock = self.assets.read_lock(SCRIPT.parent.parent / "simulator/official_assets.lock.json")
        self.assertEqual(lock["repository"], "FS-AI/FS-AI_IMechE_ADS-DV_HiL")
        self.assertEqual(len(lock["files"]), 14)
        self.assertRegex(lock["commit"], r"^[0-9a-f]{40}$")
        self.assertLess(sum(item["size"] for item in lock["files"]), 5_000_000)

    def test_vehicle_uses_five_native_meshes_and_six_moving_joints(self):
        # A tiny GLB metadata fixture isolates the URDF assembly contract;
        # the full official meshes are hash-checked and rendered separately.
        import json
        import struct
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            component = root / "MovieNX/data/3D/Vehicles/Components/FSAI_RaceCar"
            component.mkdir(parents=True)
            for name, count in (("Body", 7), ("Wheel", 5)):
                document = {"meshes": [{"primitives": [{"indices": 0}] * count}],
                            "accessors": [{"count": 3}], "materials": [{}] * count, "images": [{}]}
                body = json.dumps(document).encode()
                body += b" " * (-len(body) % 4)
                payload = struct.pack("<4sIIII", b"glTF", 2, 20 + len(body), len(body), 0x4e4f534a) + body
                (component / f"FSAI_{name}.glb").write_bytes(payload)
            audit = self.assets.write_vehicle_description(root, root)
            robot = ET.parse(root / "ads_dv_official.urdf")
            self.assertEqual(len(robot.findall(".//mesh")), 5)
            self.assertFalse(robot.findall(".//box"))
            self.assertEqual({joint.attrib["name"] for joint in robot.findall("joint")
                              if joint.attrib["type"] != "fixed"},
                             {"steer_fl", "steer_fr", "spin_fl", "spin_fr", "spin_rl", "spin_rr"})
            self.assertEqual(audit["body"]["primitives"], 7)
            self.assertEqual(audit["wheel"]["primitives"], 5)
            self.assertFalse(audit["physics_changed"])


if __name__ == "__main__":
    unittest.main()
