#!/usr/bin/env python3
"""Local-only importer for pinned IMechE FS-AI assets."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import struct
import tempfile
import time
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_REPOSITORY = "FS-AI/FS-AI_IMechE_ADS-DV_HiL"


def read_lock(path):
    lock = json.loads(Path(path).read_text(encoding="utf-8"))
    if lock.get("schema_version") != 1 or lock.get("repository") != OFFICIAL_REPOSITORY:
        raise ValueError("only the pinned official FS-AI HiL repository is accepted")
    if not re.fullmatch(r"[0-9a-f]{40}", lock.get("commit", "")):
        raise ValueError("official source must be pinned to a full commit")
    seen = set()
    for entry in lock["files"]:
        cache_path(ROOT / ".dependencies", entry["path"])
        if entry["path"] in seen or not 0 < entry["size"] <= 50_000_000:
            raise ValueError("duplicate or unbounded asset entry")
        if not re.fullmatch(r"[0-9a-f]{40}", entry.get("git_blob_sha1", "")):
            raise ValueError("missing official Git object hash")
        seen.add(entry["path"])
    return lock


def verify_blob(payload, descriptor):
    header = b"blob " + str(len(payload)).encode("ascii") + b"\0"
    if len(payload) != descriptor["size"] or hashlib.sha1(header + payload).hexdigest() != descriptor["git_blob_sha1"]:
        raise ValueError("download does not match the pinned official Git blob")


def cache_path(root, relative):
    root = Path(root).resolve()
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("asset path must be relative and may not traverse parents")
    result = root / relative
    if not result.resolve().is_relative_to(root):
        raise ValueError("asset cache symlink escapes its root")
    return result


def fetch_sources(lock, cache):
    def fetch(entry):
        destination = cache_path(cache, entry["path"])
        if destination.is_file():
            verify_blob(destination.read_bytes(), entry)
            return destination
        url = f"https://raw.githubusercontent.com/{lock['repository']}/{lock['commit']}/{entry['path']}"
        for attempt in range(3):
            try:
                request = Request(url, headers={"User-Agent": "FSAI-official-assets/1"})
                with urlopen(request, timeout=45) as response:
                    payload = response.read(entry["size"] + 1)
                verify_blob(payload, entry)
                break
            except (OSError, ValueError):
                if attempt == 2:
                    raise
                time.sleep(attempt + 1)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as temporary:
            temporary.write(payload)
            temporary_path = Path(temporary.name)
        temporary_path.replace(destination)
        print(f"Verified official asset: {entry['path']}", flush=True)
        return destination

    with ThreadPoolExecutor(max_workers=4) as pool:
        return list(pool.map(fetch, lock["files"]))


def inspect_glb(path):
    payload = Path(path).read_bytes()
    magic, version, size = struct.unpack_from("<4sII", payload)
    if magic != b"glTF" or version != 2 or size != len(payload):
        raise ValueError("official vehicle is not a complete glTF 2 binary")
    length, kind = struct.unpack_from("<II", payload, 12)
    if kind != 0x4e4f534a:
        raise ValueError("GLB must begin with its JSON scene description")
    document = json.loads(payload[20:20 + length])
    primitives = [part for mesh in document["meshes"] for part in mesh["primitives"]]
    if any(part.get("mode", 4) != 4 for part in primitives):
        raise ValueError("expected triangle primitives in the official vehicle")
    triangles = sum(document["accessors"][part["indices"]]["count"] // 3 for part in primitives)
    return {"primitives": len(primitives), "triangles": triangles,
            "materials": len(document.get("materials", [])),
            "embedded_images": len(document.get("images", [])),
            "sha256": hashlib.sha256(payload).hexdigest()}


def write_vehicle_description(cache, output):
    """Use byte-identical official GLBs with their scene transforms/textures.

    RViz Humble's Assimp 5 loader supports embedded GLB textures. The extra
    visual Rx(+90deg) converts glTF Y-up to ROS Z-up; it is not mesh decimation.
    Official MovieNX wheel-carrier locations are visual-only: physics is unchanged.
    """
    component = cache / "MovieNX/data/3D/Vehicles/Components/FSAI_RaceCar"
    body = component / "FSAI_Body.glb"
    wheel = component / "FSAI_Wheel.glb"
    audit = {"body": inspect_glb(body), "wheel": inspect_glb(wheel)}
    if audit["body"]["primitives"] != 7 or audit["wheel"]["primitives"] != 5:
        raise ValueError("official vehicle primitive inventory changed")
    robot = ET.Element("robot", name="official_fsai_ads_dv_2026")
    base = ET.SubElement(robot, "link", name="base_footprint")

    def visual(link, filename, xyz, rpy):
        element = ET.SubElement(link, "visual")
        ET.SubElement(element, "origin", xyz=xyz, rpy=rpy)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "mesh", filename=filename.as_uri(), scale="1 1 1")

    def joint(name, kind, parent, child, xyz="0 0 0", axis=None):
        element = ET.SubElement(robot, "joint", name=name, type=kind)
        ET.SubElement(element, "parent", link=parent)
        ET.SubElement(element, "child", link=child)
        ET.SubElement(element, "origin", xyz=xyz, rpy="0 0 0")
        if axis:
            ET.SubElement(element, "axis", xyz=axis)
        if kind == "revolute":
            ET.SubElement(element, "limit", lower="-0.523598775598", upper="0.523598775598",
                          effort="100", velocity="0.39")

    cg_x = 1.3287
    visual(base, body, f"{-cg_x} 0 0", f"{math.pi / 2} 0 0")
    for suffix, x, y in (("fl", 2.093, .476), ("fr", 2.093, -.476),
                         ("rl", .557, .476), ("rr", .557, -.476)):
        carrier = "carrier_" + suffix
        ET.SubElement(robot, "link", name=carrier)
        front = suffix.startswith("f")
        joint(("steer_" if front else "mount_") + suffix,
              "revolute" if front else "fixed", "base_footprint", carrier,
              f"{x - cg_x:.10f} {y} 0.257", "0 0 1" if front else None)
        link = ET.SubElement(robot, "link", name="wheel_" + suffix)
        joint("spin_" + suffix, "continuous", carrier, "wheel_" + suffix, axis="0 1 0")
        visual(link, wheel, "0 0 0", f"{math.pi / 2} 0 {math.pi if suffix.endswith('r') else 0}")
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(output / "ads_dv_official.urdf", encoding="utf-8", xml_declaration=True)
    audit["visual_cg_shift_x_m"] = -cg_x
    audit["physics_changed"] = False
    audit["format_conversion"] = "none: original GLBs, scene nodes, all primitives and embedded textures retained"
    return audit


def write_road_surface(points, filename, paved_width):
    """Visual ribbon sampled from the official terrain-derived reference.

    The original mesh is the surrounding terrain and omits CarMaker's procedural
    road. Fill that opening using the same derived layout; no new track is drawn.
    """
    rows = ["# Derived visual surface of the official Sprint layout; metres, ROS Z-up",
            "mtllib road.mtl", "o official_sprint_road", "usemtl asphalt"]
    for i, point in enumerate(points):
        before, after = points[i - 1], points[(i + 1) % len(points)]
        dx, dy = after[0] - before[0], after[1] - before[1]
        scale = paved_width / (2 * math.hypot(dx, dy))
        nx, ny = -dy * scale, dx * scale
        rows.extend([f"v {point[0] + nx:.9f} {point[1] + ny:.9f} -0.005",
                     f"v {point[0] - nx:.9f} {point[1] - ny:.9f} -0.005"])
    for i in range(len(points)):
        a, b = 2 * i + 1, 2 * i + 2
        c, d = 2 * ((i + 1) % len(points)) + 1, 2 * ((i + 1) % len(points)) + 2
        rows.extend([f"f {a} {b} {c}", f"f {b} {d} {c}"])
    filename.write_text("\n".join(rows) + "\n", encoding="utf-8")
    (filename.parent / "road.mtl").write_text(
        "newmtl asphalt\nKa 0.22 0.22 0.22\nKd 0.26 0.26 0.26\nKs 0 0 0\nd 1\n",
        encoding="utf-8")


def prepare(lock, cache, output):
    import official_track
    output.mkdir(parents=True, exist_ok=True)
    result = official_track.main([
        "--road", str(cache / "Data/Road/FS_autonomous_Sprint_LTS.rd5"),
        "--terrain", str(cache / "Movie/3D/Terrain/FS_autonomous_Sprint_LTS.obj"),
        "--test-run", str(cache / "Data/TestRun/FS_autonomous_Sprint_LTS"),
        "--route-lateral-offset-m", "0", "--output", str(output / "track"),
    ])
    if result:
        raise ValueError("official track conversion failed")
    audit = write_vehicle_description(cache, output)
    with (output / "track/centerline.csv").open(newline="", encoding="utf-8") as stream:
        points = [(float(row["x_m"]), float(row["y_m"])) for row in csv.DictReader(stream)]
    provenance = json.loads((output / "track/provenance.json").read_text())
    paved_width = sum(provenance["mesh_opening_width_range_m"]) / 2
    write_road_surface(points, output / "road.obj", paved_width)
    terrain_source = cache / "Movie/3D/Terrain/FS_autonomous_Sprint_LTS.obj"
    terrain = terrain_source.read_text(encoding="utf-8")
    # Retain every original terrain vertex/face. Only its external IPG .mtex
    # reference is replaced by a neutral local material supported by RViz.
    terrain = re.sub(r"(?m)^mtllib .*$", "mtllib terrain.mtl", terrain)
    (output / "terrain.obj").write_text(terrain, encoding="utf-8")
    (output / "terrain.mtl").write_text(
        "newmtl Terrain_Area\nKa 0.25 0.30 0.23\nKd 0.40 0.46 0.34\nKs 0 0 0\nd 1\n"
        "newmtl Background\nKa 0.25 0.30 0.23\nKd 0.40 0.46 0.34\nKs 0 0 0\nd 1\n",
        encoding="utf-8")
    cone_dir = cache / "Movie/TrafficCones"
    visuals = {"road_mesh_uri": (output / "road.obj").as_uri(),
               "terrain_mesh_uri": (output / "terrain.obj").as_uri(), "cone_mesh_uris": {
        name: (cone_dir / ("TrafficCone_" + filename + ".obj")).as_uri()
        for name, filename in (("blue", "Small_Blue"), ("yellow", "Small_Yellow"),
                               ("orange", "Small_Orange"), ("big_orange", "Large_Orange"))}}
    # JSON is a YAML subset and preserves quoted file URIs safely.
    (output / "track/visuals.yaml").write_text(json.dumps(visuals, indent=2) + "\n", encoding="utf-8")
    state = {"source_repository": lock["repository"], "source_commit": lock["commit"],
             "vehicle": audit, "track": provenance,
             "road_visual": "derived surface of official layout; neutral asphalt because IPG .mtex renderer data is not portable",
             "terrain_visual": "all original OBJ vertices/faces retained; only unavailable IPG texture replaced by neutral material",
             "output_root": str(output), "code_hash": code_hash()}
    state["output_hashes"] = {
        str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(output.rglob("*")) if path.is_file() and path.name != "asset_manifest.json"}
    (output / "asset_manifest.json").write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    print(f"Official assets ready: {output}", flush=True)


def code_hash():
    digest = hashlib.sha256()
    for path in (Path(__file__), ROOT / "tools/official_track.py", ROOT / "simulator/official_assets.lock.json"):
        digest.update(path.read_bytes())
    return digest.hexdigest()


def prepared(output):
    try:
        state = json.loads((output / "asset_manifest.json").read_text())
        return (state["output_root"] == str(output) and state["code_hash"] == code_hash() and
                all(hashlib.sha256(cache_path(output, path).read_bytes()).hexdigest() == digest
                    for path, digest in state["output_hashes"].items()))
    except (OSError, KeyError, ValueError):
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fetch-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        lock = read_lock(ROOT / "simulator/official_assets.lock.json")
        cache = ROOT / ".dependencies/official/hil"
        fetch_sources(lock, cache)
        if not args.fetch_only:
            output = ROOT / ".dependencies/official/generated"
            if not prepared(output):
                prepare(lock, cache, output)
            else:
                print(f"Verified prepared official assets: {output}")
        return 0
    except Exception as error:
        print(f"official-assets: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
