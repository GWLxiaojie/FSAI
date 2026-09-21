"""Synthetic topology fixtures; optional official asset checks are opt-in."""
import importlib.util
import math
import os
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "official_track.py"


def module():
    spec = importlib.util.spec_from_file_location("official_track", MODULE_PATH)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class OfficialTrackTest(unittest.TestCase):
    def test_triangle_boundary_extraction_preserves_closed_geometry(self):
        api = module()
        vertices = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
        loops = api.boundary_loops(vertices, [(0, 1, 2), (0, 2, 3)])
        self.assertEqual(len(loops), 1)
        self.assertEqual(len(loops[0]), 4)
        self.assertEqual(set(loops[0]), set(vertices))
        self.assertAlmostEqual(api.perimeter(loops[0]), 4)

    def test_known_circular_corridor_preserves_direction_width_and_length(self):
        api = module()
        def circle(radius):
            return [(radius * math.cos(i * math.tau / 256),
                     radius * math.sin(i * math.tau / 256), 0) for i in range(256)]
        points, report = api.derive_centerline([circle(8), circle(12)], 20 * math.pi,
                                               [10, 0], math.pi / 2, 0.25)
        self.assertGreater(points[1][1], points[0][1])
        self.assertLess(abs(report["derived_length_m"] - 20 * math.pi), 0.03)
        self.assertTrue(all(abs(p[2] - 4) < 0.002 for p in points))
        self.assertNotEqual(points[0][:2], points[-1][:2])
        self.assertLess(math.dist(points[0][:2], points[-1][:2]), 0.251)

    def test_rejects_nonmanifold_and_wrong_route_length(self):
        api = module()
        with self.assertRaisesRegex(ValueError, "non-manifold"):
            api.boundary_loops([(0, 0, 0)] * 5, [(0, 1, 2), (0, 1, 3), (0, 1, 4)])
        with self.assertRaisesRegex(ValueError, "exactly two"):
            api.derive_centerline([], 100, [0, 0], 0)

    def test_extracts_official_s_t_without_inventing_xy(self):
        api = module()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "TestRun"
            path.write_text("Traffic.N = 1\nTraffic.0.Template.FName = TrafficCone_Small_Blue\n"
                            "Traffic.0.StartPos.Type = Route\nTraffic.0.StartPos.ObjId = 1\n"
                            "Traffic.0.StartPos = -2 2.0\n")
            cones, start = api.read_traffic(path)
        self.assertEqual(cones, [{"source_id": 0, "template": "TrafficCone_Small_Blue",
                                  "tag": "blue", "s_m": -2.0, "t_m": 2.0}])
        self.assertIsNone(start)

    def test_route_wrap_and_positive_left_offset(self):
        api = module()
        square = [(0, 0, 4), (10, 0, 4), (10, 10, 4), (0, 10, 4)]
        self.assertEqual(api.route_position(square, 40, 5, 2), (5, 2, 0))
        x, y, heading = api.route_position(square, 40, -5, 2)
        self.assertEqual((x, y), (2, 5))
        self.assertAlmostEqual(heading, -math.pi / 2)

    def test_control_start_reorders_without_changing_polygon(self):
        api = module()
        square = [(0, 0, 4), (10, 0, 4), (10, 10, 4), (0, 10, 4)]
        reordered = api.reorder_at_position(square, (10, 3))
        self.assertEqual(reordered[0], (10, 3, 4))
        self.assertEqual(reordered[1:], [(10, 10, 4), (0, 10, 4), (0, 0, 4), (10, 0, 4)])
        self.assertAlmostEqual(api.perimeter(reordered), api.perimeter(square))
        self.assertNotEqual(reordered[0], reordered[-1])
        at_vertex = api.reorder_at_position(square, (10, 0))
        self.assertEqual(len(at_vertex), 4)
        self.assertEqual(at_vertex[0], (10, 0, 4))
        self.assertAlmostEqual(api.perimeter(at_vertex), 40)

    def test_rejects_invalid_source_traffic(self):
        api = module()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "TestRun"
            prefix = ("Traffic.0.Template.FName = TrafficCone_Small_Blue\n"
                      "Traffic.0.StartPos.Type = Route\nTraffic.0.StartPos.ObjId = 1\n")
            path.write_text(prefix + "Traffic.0.StartPos = nan 2\n")
            with self.assertRaisesRegex(ValueError, "finite"):
                api.read_traffic(path)
            path.write_text(prefix.replace("Route", "World") + "Traffic.0.StartPos = 0 2\n")
            with self.assertRaisesRegex(ValueError, "Route 1"):
                api.read_traffic(path)

    def test_command_rejects_output_outside_ignored_dependencies(self):
        api = module()
        with tempfile.TemporaryDirectory() as directory:
            code = api.main(["--road", "missing", "--terrain", "missing", "--test-run", "missing",
                             "--route-lateral-offset-m", "0", "--output", directory])
        self.assertEqual(code, 2)

    @unittest.skipUnless(os.environ.get("FSAI_OFFICIAL_HIL_ROOT"), "official external assets not requested")
    def test_external_official_sprint(self):
        api = module()
        root = Path(os.environ["FSAI_OFFICIAL_HIL_ROOT"])
        road = root / "Data/Road/FS_autonomous_Sprint_LTS.rd5"
        terrain = root / "Movie/3D/Terrain/FS_autonomous_Sprint_LTS.obj"
        length, start, heading = api.read_road(road)
        vertices, faces = api.read_obj(terrain)
        points, report = api.derive_centerline(api.boundary_loops(vertices, faces), length, start, heading)
        cones, vehicle_start = api.read_traffic(root / "Data/TestRun/FS_autonomous_Sprint_LTS")
        self.assertEqual(len(cones), 366)
        self.assertEqual(vehicle_start, (719.3, 0.0))
        self.assertLess(abs(report["derived_length_m"] - length), 0.2)
        self.assertLess(report["node0_projection_error_m"], 0.001)


if __name__ == "__main__":
    unittest.main()
