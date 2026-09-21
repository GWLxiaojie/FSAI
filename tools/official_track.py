#!/usr/bin/env python3
"""Derive a closed reference from the official Sprint terrain's road opening.

No CarMaker PointList interpolation is reconstructed. Official TestRun cone s/t
positions are retained; world XY conversion is explicitly derived. Assets stay
under the simulator repository's ignored .dependencies directory.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import sys


def perimeter(points):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:] + points[:1]))


def boundary_loops(vertices, faces):
    edges = Counter(tuple(sorted((a, b))) for face in faces
                    for a, b in zip(face, face[1:] + face[:1]))
    if any(count > 2 for count in edges.values()):
        raise ValueError("non-manifold triangle mesh")
    adjacent = defaultdict(list)
    for (a, b), count in edges.items():
        if count == 1:
            adjacent[a].append(b)
            adjacent[b].append(a)
    if any(len(neighbors) != 2 for neighbors in adjacent.values()):
        raise ValueError("boundary is not a collection of simple closed loops")
    loops, seen = [], set()
    for start in sorted(adjacent):
        if start in seen:
            continue
        indices, previous, current = [], None, start
        while current not in seen:
            seen.add(current)
            indices.append(current)
            candidates = [item for item in adjacent[current] if item != previous]
            previous, current = current, min(candidates)
        if current != start or len(indices) < 3:
            raise ValueError("boundary failed to close")
        loops.append([vertices[index] for index in indices])
    return loops


def read_obj(path):
    vertices, faces = [], []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v":
            point = tuple(map(float, fields[1:4]))
            if len(point) != 3 or not all(map(math.isfinite, point)):
                raise ValueError("OBJ vertex must contain three finite coordinates")
            vertices.append(point)
        elif fields[0] == "f":
            if len(fields) != 4:
                raise ValueError("only triangulated source OBJ is supported")
            face = tuple(int(field.split("/")[0]) - 1 for field in fields[1:])
            if len(set(face)) != 3 or any(index < 0 or index >= len(vertices) for index in face):
                raise ValueError("invalid OBJ face index")
            faces.append(face)
    if not faces:
        raise ValueError("OBJ has no triangles")
    return vertices, faces


def closest_on_loop(point, points):
    best = None
    for index, (a, b) in enumerate(zip(points, points[1:] + points[:1])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        squared = dx * dx + dy * dy
        if squared == 0:
            raise ValueError("repeated boundary point")
        weight = max(0, min(1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / squared))
        projected = (a[0] + weight * dx, a[1] + weight * dy)
        candidate = (math.dist(point[:2], projected), index, weight, projected)
        if best is None or candidate[0] < best[0]:
            best = candidate
    return best


def resample(points, spacing):
    length = perimeter(points)
    count = math.ceil(length / spacing)
    samples, index, traveled = [], 0, 0.0
    for i in range(count):
        target = i * length / count
        while True:
            a, b = points[index], points[(index + 1) % len(points)]
            segment = math.dist(a[:2], b[:2])
            if segment <= 0:
                raise ValueError("repeated centerline point")
            if traveled + segment >= target:
                break
            traveled += segment
            index += 1
        weight = (target - traveled) / segment
        samples.append(tuple(a[k] + weight * (b[k] - a[k]) for k in range(3)))
    return samples


def derive_centerline(loops, expected_length, start_xy, start_heading, spacing=0.5):
    if not all(map(math.isfinite, [expected_length, *start_xy, start_heading, spacing])) or spacing <= 0:
        raise ValueError("geometry parameters must be finite and spacing positive")
    if expected_length <= 0 or spacing > expected_length / 10:
        raise ValueError("invalid route length or insufficient sampling")
    planar = [loop for loop in loops if max(abs(p[2]) for p in loop) < 0.001]
    if len(planar) != 2:
        raise ValueError("expected exactly two planar road-opening boundary loops")
    inner, outer = sorted(planar, key=perimeter)
    lengths = [perimeter(inner), perimeter(outer)]
    if abs(sum(lengths) / 2 - expected_length) > expected_length * 0.005:
        raise ValueError("mesh boundary lengths disagree with official route length")
    centers = []
    for point in inner:
        distance, _, _, projected = closest_on_loop(point, outer)
        centers.append(((point[0] + projected[0]) / 2, (point[1] + projected[1]) / 2, distance))
    widths = [point[2] for point in centers]
    if min(widths) <= 0 or max(widths) / min(widths) > 1.03:
        raise ValueError("road opening is not a uniform-width offset corridor")
    distance, index, weight, projected = closest_on_loop(start_xy, centers)
    if distance > 0.05:
        raise ValueError("derived reference misses official Node0 by more than 5 cm")
    width = centers[index][2] + weight * (centers[(index + 1) % len(centers)][2] - centers[index][2])
    anchor = (*projected, width)
    rotated = [anchor] + centers[index + 1:] + centers[:index + 1]
    rotated = [point for i, point in enumerate(rotated)
               if i == 0 or math.dist(point[:2], rotated[i - 1][:2]) > 1e-8]
    if math.dist(rotated[0][:2], rotated[-1][:2]) < 1e-8:
        rotated.pop()
    direction = (math.cos(start_heading), math.sin(start_heading))
    tangent = (rotated[1][0] - rotated[0][0], rotated[1][1] - rotated[0][1])
    if sum(a * b for a, b in zip(direction, tangent)) < 0:
        rotated = rotated[:1] + list(reversed(rotated[1:]))
    result = resample(rotated, spacing)
    length = perimeter(result)
    if abs(length - expected_length) > expected_length * 0.005:
        raise ValueError("derived centerline length disagrees with official route length")
    return result, {"boundary_lengths_m": lengths, "derived_length_m": length,
                    "official_route_length_m": expected_length,
                    "mesh_opening_width_range_m": [min(widths), max(widths)],
                    "node0_projection_error_m": distance, "sample_count": len(result)}


def read_road(path):
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if " = " in line:
            key, value = line.split(" = ", 1)
            fields[key] = value
    if fields.get("nLinks") != "1" or fields.get("nRoutes") != "1":
        raise ValueError("only the single-link official Sprint route is supported")
    node = list(map(float, fields["Link.0.Node0"].split()))
    end = list(map(float, fields["Link.0.Node1"].split()))
    if len(node) != 4 or len(end) != 4 or math.dist(node[:2], end[:2]) > 0.001:
        raise ValueError("official road link is not closed")
    return float(fields["Route.0.Length"]), node[:2], math.radians(node[3])


def read_traffic(path):
    fields = dict(line.split(" = ", 1) for line in path.read_text(encoding="utf-8").splitlines()
                  if " = " in line)
    colors = {"TrafficCone_Large_Orange": "big_orange", "TrafficCone_Small_Blue": "blue",
              "TrafficCone_Small_Yellow": "yellow", "TrafficCone_Small_Orange": "orange"}
    indices = sorted(int(match.group(1)) for key in fields
                     if (match := re.fullmatch(r"Traffic\.(\d+)\.Template\.FName", key)))
    if not indices or indices != list(range(len(indices))):
        raise ValueError("TestRun traffic indices must be contiguous from zero")
    cones = []
    for index in indices:
        prefix = f"Traffic.{index}."
        template = fields[prefix + "Template.FName"]
        if template not in colors:
            raise ValueError(f"unsupported official traffic template: {template}")
        if fields.get(prefix + "StartPos.Type") != "Route" or fields.get(prefix + "StartPos.ObjId") != "1":
            raise ValueError("traffic must be positioned relative to official Route 1")
        position = list(map(float, fields[prefix + "StartPos"].split()))
        if len(position) != 2 or not all(map(math.isfinite, position)):
            raise ValueError("traffic s/t must contain two finite metres values")
        cones.append({"source_id": index, "template": template, "tag": colors[template],
                      "s_m": position[0], "t_m": position[1]})
    start = None
    if "Vehicle.StartPos" in fields:
        if fields.get("Vehicle.StartPos.Type") != "Route" or fields.get("Vehicle.StartPos.ObjId") != "1":
            raise ValueError("vehicle must start relative to official Route 1")
        start = tuple(map(float, fields["Vehicle.StartPos"].split()))
        if len(start) != 2 or not all(map(math.isfinite, start)):
            raise ValueError("invalid vehicle route start")
    return cones, start


def route_position(points, official_length, s, t):
    """Map official wrapped s proportionally onto derived polyline; t is left."""
    distance = (s % official_length) * perimeter(points) / official_length
    for a, b in zip(points, points[1:] + points[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if distance <= length:
            weight = distance / length
            return (a[0] + weight * dx - t * dy / length,
                    a[1] + weight * dy + t * dx / length, math.atan2(dy, dx))
        distance -= length
    raise ValueError("route position interpolation failed")


def reorder_at_position(points, position):
    """Split the nearest segment at the projected start; retain all original edges."""
    _, index, weight, projected = closest_on_loop(position, points)
    following = (index + 1) % len(points)
    if weight == 0:
        return points[index:] + points[:index]
    if weight == 1:
        return points[following:] + points[:following]
    width = points[index][2] + weight * (points[following][2] - points[index][2])
    return [(*projected, width)] + points[index + 1:] + points[:index + 1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--road", required=True, type=Path)
    parser.add_argument("--terrain", required=True, type=Path)
    parser.add_argument("--test-run", required=True, type=Path)
    parser.add_argument("--route-lateral-offset-m", required=True, type=float,
                        help="explicit route-to-mesh-center assumption; Sprint evidence supports 0")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--spacing", type=float, default=0.5)
    args = parser.parse_args(argv)
    try:
        dependency_root = Path(__file__).resolve().parents[1] / ".dependencies"
        if not args.output.resolve().is_relative_to(dependency_root.resolve()):
            raise ValueError("output must be inside this repository's .dependencies directory")
        expected_length, start, heading = read_road(args.road)
        vertices, faces = read_obj(args.terrain)
        points, report = derive_centerline(boundary_loops(vertices, faces), expected_length,
                                           start, heading, args.spacing)
        if not math.isfinite(args.route_lateral_offset_m):
            raise ValueError("route offset must be finite")
        cones, vehicle_start = read_traffic(args.test_run)
        if vehicle_start is None:
            raise ValueError("TestRun lacks official vehicle start")
        blue_offsets = {cone["t_m"] for cone in cones if cone["tag"] == "blue"}
        yellow_offsets = {cone["t_m"] for cone in cones if cone["tag"] == "yellow"}
        if len(blue_offsets) != 1 or len(yellow_offsets) != 1:
            raise ValueError("only constant official blue/yellow cone corridor offsets are supported")
        blue, yellow = blue_offsets.pop(), yellow_offsets.pop()
        if blue <= 0 or yellow >= 0:
            raise ValueError("expected blue on positive t and yellow on negative t")
        corridor_width = blue - yellow
        if args.route_lateral_offset_m == 0:
            route_points = [(point[0], point[1], corridor_width) for point in points]
        else:
            route_points = [(*route_position(points, expected_length, i * expected_length / len(points),
                                             args.route_lateral_offset_m)[:2], corridor_width)
                            for i in range(len(points))]
        start_pose = route_position(points, expected_length, vehicle_start[0],
                                    vehicle_start[1] + args.route_lateral_offset_m)
        control_points = reorder_at_position(route_points, start_pose[:2])
        gate_pose = route_position(points, expected_length, 0, args.route_lateral_offset_m)
        args.output.mkdir(parents=True, exist_ok=True)
        with (args.output / "centerline.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["x_m", "y_m", "width_m"])
            writer.writerows(route_points)
        with (args.output / "control_centerline.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["x_m", "y_m", "width_m"])
            writer.writerows(control_points)
        with (args.output / "traffic_route.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["source_id", "template", "tag", "s_m", "t_m"])
            writer.writeheader()
            writer.writerows(cones)
        with (args.output / "cones.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["tag", "x", "y", "direction", "x_variance", "y_variance", "xy_covariance"])
            for cone in cones:
                pose = route_position(points, expected_length, cone["s_m"],
                                      cone["t_m"] + args.route_lateral_offset_m)
                writer.writerow([cone["tag"], *pose, 0, 0, 0])
        def pose_yaml(pose):
            return f"  x_m: {pose[0]}\n  y_m: {pose[1]}\n  yaw_rad: {pose[2]}\n"
        (args.output / "track.yaml").write_text(
            "schema_version: 1\nname: official_sprint\nmission: track_drive\nframe_id: map\n"
            "vehicle_start:\n" + pose_yaml(start_pose) + "start_gate:\n" + pose_yaml(gate_pose)
            + f"  width_m: {corridor_width}\nfinish_gate:\n" + pose_yaml(gate_pose)
            + f"  width_m: {corridor_width}\nrules_profile: fsai_default\nseed: 1\n"
            + "geometry_status: derived_from_official_mesh\ncones_status: official_route_positions_derived_world_xy\n",
            encoding="utf-8")
        report.update({"schema_version": 1, "geometry_status": "derived_from_official_mesh_boundaries",
                       "cones_status": "official_TestRun_s_t_mapped_to_derived_world_xy", "closed": True,
                       "cone_count": len(cones), "cone_color_counts": dict(Counter(cone["tag"] for cone in cones)),
                       "width_definition": "official blue t minus yellow t; terrain opening width reported separately",
                       "cone_corridor_width_m": corridor_width,
                       "cone_corridor_width_source": "TestRun blue t=+2m minus yellow t=-2m; not terrain opening",
                       "exported_centerline_length_m": perimeter(route_points),
                       "control_centerline_length_m": perimeter(control_points),
                       "control_start_projection_error_m": math.dist(control_points[0][:2], start_pose[:2]),
                       "control_centerline_definition": "same closed polyline rotated at vehicle-start projection; one segment split, no reshaping",
                       "vehicle_start_official_s_t_m": vehicle_start,
                       "route_lateral_offset_m_assumption": args.route_lateral_offset_m,
                       "conversion_assumptions": ["Route 1 follows terrain road opening center plus explicit offset",
                           "positive route t is left of travel; negative s wraps around closed route",
                           "official s is proportionally mapped to mesh-derived polyline arc length",
                           "zero CSV covariance is deterministic geometry, not a measured accuracy claim"],
                       "route_offset_evidence": "LaneR0 width4 and centerline lateral shift2 support route midpoint at zero; not a CarMaker XY export",
                       "lateral_sign_evidence": {
                           "source": "https://github.com/FS-AI/FS-AI_IMechE_ADS-DV_HiL/blob/1fe77bed411ba621a3187bc2eb9ca13c7d371832/Data/Script/generate_cones.tcl",
                           "meaning": "official script labels blue/yellow left/right and uses positive/negative t respectively",
                           "note": "script example uses 1.5m; actual Sprint TestRun source uses 2m and is authoritative for this corridor"},
                       "route_length_scale": report["derived_length_m"] / expected_length,
                       "method": "triangle boundary topology; inner-to-outer closest projection midpoints; linear resampling",
                       "coordinates": "official CarMaker world XY in metres; unshifted; heading from Road Node0",
                       "start_heading_rad": heading,
                       "inputs": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in (args.road, args.terrain, args.test_run)}})
        (args.output / "provenance.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0
    except (ValueError, KeyError, OSError) as error:
        print(f"official-track: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
