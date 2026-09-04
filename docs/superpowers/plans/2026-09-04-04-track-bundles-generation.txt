# Track Bundles and Centerline Generation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付可验证的 `TrackDefinition`、EUFS CSV 导入、确定性 TrackBundle 导出和有序中心线生成流程。

**Architecture:** 所有输入先归一化为不可变 `TrackDefinition`。
CSV 负责锥桶几何，YAML 负责元数据和初始位姿，运行时、传感器、比赛裁判和可视化只读取规范对象。

**Tech Stack:** C++20, Eigen3, yaml-cpp, GoogleTest, ROS 2 Humble, map_lib, Python 3.10 CLI wrappers.

**Spec:** `docs/superpowers/specs/2026-09-04-eufs-sim2-team-simulator-design.md`

## Global Constraints

EUFS CSV 表头必须严格为 `tag,x,y,direction,x_variance,y_variance,xy_covariance`。
中心线 CSV 表头必须严格为 `x,y`。
普通位置单位是 m，`car_start.direction` 单位是 rad。
沿行驶方向观察时左侧为蓝锥，右侧为黄锥。
`track.yaml` 是元数据、初始位姿和门定义的事实来源。
`cones.csv` 是 bundle 内锥桶几何的事实来源。
legacy `car_start` 只在导入边界读取。
同一输入和 seed 必须产生逐字节相同的规范输出。
运行时轨道必须是不可变对象。
热切换必须通过完整 context 原子 reset。

---

## File Map

```text
simulator/src/fsai_track_tools/
  include/fsai_track_tools/types.hpp
  include/fsai_track_tools/eufs_csv.hpp
  include/fsai_track_tools/track_bundle.hpp
  include/fsai_track_tools/validator.hpp
  include/fsai_track_tools/centerline_generator.hpp
  include/fsai_track_tools/canonical_writer.hpp
  src/
  test/
  scripts/fsai-track
simulator/src/fsai_interfaces/msg/TrackRevision.msg
simulator/src/fsai_sim2_adapter/
  include/fsai_sim2_adapter/track_adapter.hpp
  src/track_adapter.cpp
simulator/src/fsai_bringup/tracks/
```

### Task 1: Define TrackDefinition and strict EUFS CSV import

**Files:**

- Create: `simulator/src/fsai_track_tools/package.xml`
- Create: `simulator/src/fsai_track_tools/CMakeLists.txt`
- Create: `simulator/src/fsai_track_tools/include/fsai_track_tools/types.hpp`
- Create: `simulator/src/fsai_track_tools/include/fsai_track_tools/eufs_csv.hpp`
- Create: `simulator/src/fsai_track_tools/src/eufs_csv.cpp`
- Create: `simulator/src/fsai_track_tools/test/eufs_csv_test.cpp`
- Create: `simulator/src/fsai_track_tools/test/data/minimal_legacy.csv`

**Interfaces:**

- Consumes: EUFS-compatible CSV path.
- Produces: `LegacyTrackInput LoadEufsCsv(path)` and stable cone records.

- [ ] **Step 1: Write failing parser tests**

```cpp
TEST(EufsCsv, LoadsConesAndLegacyStart) {
  auto input = LoadEufsCsv(DataPath("minimal_legacy.csv"));
  EXPECT_EQ(input.cones.size(), 4u);
  EXPECT_DOUBLE_EQ(input.legacy_start->yaw_rad, 0.25);
}

TEST(EufsCsv, ReportsLineFieldAndValue) {
  EXPECT_THAT(
      CaptureError("non_finite.csv"),
      HasSubstr("non_finite.csv:3:x:nan"));
}

TEST(EufsCsv, RejectsWrongHeader) {
  EXPECT_THROW(LoadEufsCsv(DataPath("wrong_header.csv")), TrackFormatError);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_track_tools --ctest-args -R eufs_csv_test --output-on-failure`

Expected: FAIL because the package and parser are absent.

- [ ] **Step 3: Implement exact canonical types and parser**

```cpp
enum class ConeColour : std::uint8_t {
  kBlue, kYellow, kOrange, kBigOrange, kUnknown
};

struct Cone final {
  std::string stable_id;
  ConeColour colour;
  Eigen::Vector2d position_m;
  Eigen::Matrix2d covariance_m2;
};

struct Pose2d final {
  Eigen::Vector2d position_m;
  double yaw_rad;
};

struct TrackDefinition final {
  std::uint32_t schema_version;
  std::string name;
  Mission mission;
  std::string frame_id;
  Pose2d vehicle_start;
  Gate start_gate;
  Gate finish_gate;
  std::vector<Cone> cones;
  std::string rules_profile;
  std::uint64_t seed;
  std::string revision_hash;
};
```

Accept only documented tags.
Require seven fields per row and finite numeric values.
Allow exactly one legacy `car_start`.
Preserve source row index and derive the initial stable ID from canonical colour, canonical coordinates and occurrence index.

- [ ] **Step 4: Run parser tests**

Expected: Valid fixtures pass and every malformed fixture reports path, line, field and value.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_track_tools
git commit -m "feat: parse EUFS track files"
```

### Task 2: Load TrackBundle metadata and normalize legacy files

**Files:**

- Create: `simulator/src/fsai_track_tools/include/fsai_track_tools/track_bundle.hpp`
- Create: `simulator/src/fsai_track_tools/src/track_bundle.cpp`
- Create: `simulator/src/fsai_track_tools/test/track_bundle_test.cpp`
- Create: `simulator/src/fsai_track_tools/test/data/valid_bundle/cones.csv`
- Create: `simulator/src/fsai_track_tools/test/data/valid_bundle/track.yaml`

**Interfaces:**

- Consumes: Bundle directory or legacy CSV path.
- Produces: `TrackDefinition LoadTrack(path)`.

- [ ] **Step 1: Write failing source-of-truth tests**

```cpp
TEST(TrackBundle, UsesYamlStartInsteadOfGeneratedCsvRow) {
  auto track = LoadTrack(DataPath("valid_bundle"));
  EXPECT_EQ(track.vehicle_start, Pose2dAt(1.0, 2.0, 0.3));
}

TEST(TrackBundle, RejectsConflictingEditableStartData) {
  EXPECT_THROW(LoadTrack(DataPath("conflicting_bundle")), TrackFormatError);
}

TEST(LegacyTrack, NormalizesCarStartIntoDefinition) {
  auto track = LoadTrack(DataPath("minimal_legacy.csv"));
  EXPECT_EQ(track.vehicle_start, Pose2dAt(0.0, 0.0, 0.25));
}
```

- [ ] **Step 2: Run the focused test**

Run: `colcon test --base-paths simulator/src --packages-select fsai_track_tools --ctest-args -R track_bundle_test --output-on-failure`

Expected: FAIL because the bundle loader is absent.

- [ ] **Step 3: Implement schema version 1**

Require `track.yaml` fields `schema_version`, `name`, `mission`, `frame_id`, `units`, `vehicle_start`, `start_gate`, `finish_gate`, `rules_profile`, `generation` and `seed`.
Require `units: SI` and `frame_id: map` in version 1.
For bundle loading, treat a CSV `car_start` row as generated compatibility data and require it to equal YAML start within 1e-9.
For a standalone legacy CSV, import `car_start` into the definition and use an explicit `legacy_eufs_v1` rules profile.
Reject unknown YAML keys.

- [ ] **Step 4: Run bundle tests**

Expected: Bundle and legacy fixtures yield semantically equal definitions where geometry and start match.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_track_tools
git commit -m "feat: load canonical track bundles"
```

### Task 3: Validate geometry and rules profiles

**Files:**

- Create: `simulator/src/fsai_track_tools/include/fsai_track_tools/validator.hpp`
- Create: `simulator/src/fsai_track_tools/src/validator.cpp`
- Create: `simulator/src/fsai_track_tools/test/validator_test.cpp`
- Create: `simulator/src/fsai_track_tools/config/rules/fs_2026.yaml`

**Interfaces:**

- Consumes: Mutable candidate TrackDefinition and rules profile.
- Produces: `ValidationReport ValidateTrack(...)` and accepted immutable definition.

- [ ] **Step 1: Write failing geometry tests**

```cpp
TEST(TrackValidator, RejectsDuplicateStableIds) {
  auto track = ValidTrack();
  track.cones[1].stable_id = track.cones[0].stable_id;
  EXPECT_THAT(ValidateTrack(track, rules).errors,
              Contains(FieldError("cones[1].stable_id")));
}

TEST(TrackValidator, RejectsInvalidWidthAndSpacing) {
  EXPECT_FALSE(ValidateTrack(TooNarrowTrack(), rules).ok());
  EXPECT_FALSE(ValidateTrack(OverSpacedTrack(), rules).ok());
}

TEST(TrackValidator, RequiresCompleteGates) {
  EXPECT_FALSE(ValidateTrack(MissingFinishGate(), rules).ok());
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_track_tools --ctest-args -R validator_test --output-on-failure`

Expected: FAIL because validation is absent.

- [ ] **Step 3: Implement explicit checks**

Check finite coordinates and covariance.
Require covariance to be symmetric positive semidefinite.
Require stable IDs to be unique and non-empty.
Validate cone spacing, estimated local track width, gate width and mission-specific gate requirements against `fs_2026.yaml`.
Reject self-intersecting left and right boundaries.
Report every error in deterministic path order.
Return an immutable shared definition only when the report has no errors.

- [ ] **Step 4: Run all validation fixtures**

Expected: Each invalid fixture triggers its named rule and the valid fixture has an empty report.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_track_tools
git commit -m "feat: validate track geometry"
```

### Task 4: Generate boundaries from ordered centerlines

**Files:**

- Create: `simulator/src/fsai_track_tools/include/fsai_track_tools/centerline_generator.hpp`
- Create: `simulator/src/fsai_track_tools/src/centerline_generator.cpp`
- Create: `simulator/src/fsai_track_tools/test/centerline_generator_test.cpp`
- Create: `simulator/src/fsai_track_tools/test/data/circle_centerline.csv`
- Create: `simulator/src/fsai_track_tools/test/data/open_centerline.csv`
- Create: `simulator/src/fsai_track_tools/test/data/circle_generation.yaml`

**Interfaces:**

- Consumes: Strict `x,y` CSV and versioned generation YAML.
- Produces: `TrackDefinition GenerateTrack(Centerline, GenerationConfig)`.

- [ ] **Step 1: Write failing generation tests**

```cpp
TEST(CenterlineGenerator, AssignsColoursByTravelDirection) {
  auto track = GenerateCircle();
  EXPECT_THAT(track.LeftBoundary(), Each(HasColour(ConeColour::kBlue)));
  EXPECT_THAT(track.RightBoundary(), Each(HasColour(ConeColour::kYellow)));
}

TEST(CenterlineGenerator, ProducesRequestedWidth) {
  auto track = GenerateCircle();
  EXPECT_NEAR(MeanBoundaryDistance(track), 3.0, 1e-6);
}

TEST(CenterlineGenerator, RejectsClosedFileWithOpenEnds) {
  EXPECT_THROW(GenerateTrack(OpenPoints(), ClosedConfig()), TrackGeometryError);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_track_tools --ctest-args -R centerline_generator_test --output-on-failure`

Expected: FAIL because the generator is absent.

- [ ] **Step 3: Implement the deterministic algorithm**

Parse exactly two finite columns named `x` and `y`.
Input order defines travel direction.
For closed tracks require first and last points within 1e-6 m and remove the duplicated last point.
Build a periodic cubic Hermite curve for closed tracks and a clamped cubic Hermite curve for open tracks.
Derive tangents from centered finite differences.
Sample every segment at exactly 1024 equal parameter intervals, accumulate arc length, and linearly interpolate target cone stations.
Offset half the configured width along the normalized left normal.
Place blue cones left and yellow cones right.
Place big-orange gate cones from the configured gate width and start station.
Reject zero tangents, self-intersection, boundary inversion, invalid spacing and curvature radius below the rules profile.

- [ ] **Step 4: Run circle, S-curve and open-track tests**

Compare the generated circle radius, width and spacing against analytic values.
Run equal inputs twice and compare cone IDs and coordinates exactly.
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_track_tools
git commit -m "feat: generate tracks from centerlines"
```

### Task 5: Write canonical bundles and revision hashes

**Files:**

- Create: `simulator/src/fsai_track_tools/include/fsai_track_tools/canonical_writer.hpp`
- Create: `simulator/src/fsai_track_tools/src/canonical_writer.cpp`
- Create: `simulator/src/fsai_track_tools/test/canonical_writer_test.cpp`

**Interfaces:**

- Consumes: Validated TrackDefinition.
- Produces: `WriteTrackBundle(track, directory)` and `TrackRevisionHash(track)`.

- [ ] **Step 1: Write failing byte-stability tests**

```cpp
TEST(CanonicalWriter, ProducesByteIdenticalBundles) {
  WriteTrackBundle(track, OutputPath("a"));
  WriteTrackBundle(track, OutputPath("b"));
  EXPECT_EQ(ReadBytes("a/cones.csv"), ReadBytes("b/cones.csv"));
  EXPECT_EQ(ReadBytes("a/track.yaml"), ReadBytes("b/track.yaml"));
}

TEST(CanonicalWriter, WritesCompatibleCarStart) {
  auto rows = ReadCsv(OutputPath("a/cones.csv"));
  EXPECT_EQ(rows.back().tag, "car_start");
  EXPECT_DOUBLE_EQ(rows.back().direction, track.vehicle_start.yaw_rad);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_track_tools --ctest-args -R canonical_writer_test --output-on-failure`

Expected: FAIL because canonical output is absent.

- [ ] **Step 3: Implement canonical serialization**

Sort cones by colour enum, stable ID and coordinates.
Write UTF-8, LF newlines and a final newline.
Use `std::to_chars` with `max_digits10` for all doubles.
Write YAML keys in the schema order defined by the test fixture.
Write `car_start` as a generated final CSV row.
Hash canonical schema version, metadata, gates and sorted cone records with SHA-256.
Do not include `preview.png` in the revision hash.

- [ ] **Step 4: Run byte and round-trip tests**

Load, write, reload and require semantic equality plus matching revision hash.
Expected: PASS on repeated output directories.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_track_tools
git commit -m "feat: write deterministic track bundles"
```

### Task 6: Add track CLI and reference bundles

**Files:**

- Create: `simulator/src/fsai_track_tools/scripts/fsai-track`
- Create: `simulator/src/fsai_track_tools/test/test_cli.py`
- Create: `simulator/src/fsai_bringup/tracks/reference_circle/centerline.csv`
- Create: `simulator/src/fsai_bringup/tracks/reference_circle/generation.yaml`
- Create: `simulator/src/fsai_bringup/tracks/reference_circle/cones.csv`
- Create: `simulator/src/fsai_bringup/tracks/reference_circle/track.yaml`
- Create: `docs/tracks.md`

**Interfaces:**

- Consumes: `fsai-track import`, `generate` and `validate` arguments.
- Produces: Validated bundles and optional generated preview.

- [ ] **Step 1: Write failing CLI tests**

```python
import re

def test_generate_then_validate(tmp_path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    run("fsai-track", "generate", "circle_centerline.csv",
        "circle_generation.yaml", "--output", str(first))
    run("fsai-track", "generate", "circle_centerline.csv",
        "circle_generation.yaml", "--output", str(second))
    first_result = run("fsai-track", "validate", str(first))
    second_result = run("fsai-track", "validate", str(second))
    assert re.fullmatch(r"valid schema=1 hash=[0-9a-f]{64}\n",
                        first_result.stdout)
    assert first_result.stdout == second_result.stdout

def test_invalid_track_returns_two():
    result = run("fsai-track", "validate", "invalid_bundle", check=False)
    assert result.returncode == 2
    assert "cones.csv:3:x:nan" in result.stderr
```

- [ ] **Step 2: Run CLI tests**

Run: `pytest simulator/src/fsai_track_tools/test/test_cli.py -q`

Expected: FAIL because the CLI is absent.

- [ ] **Step 3: Implement commands and documentation**

`import` converts a legacy EUFS CSV into a bundle.
`generate` consumes centerline CSV and generation YAML.
`validate` prints schema and hash on success and deterministic errors on failure.
`preview` may generate PNG but its output is never required at runtime.
Document the exact file headers and one command for each workflow.

- [ ] **Step 4: Regenerate and verify the reference bundle**

Generate twice into separate directories and compare `cones.csv` and `track.yaml` byte for byte.
Validate the committed reference bundle.
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_track_tools simulator/src/fsai_bringup/tracks docs/tracks.md
git commit -m "feat: add custom track workflow"
```

### Task 7: Adapt TrackDefinition to EUFS and atomically switch tracks

**Files:**

- Create: `simulator/src/fsai_interfaces/msg/TrackRevision.msg`
- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/track_adapter.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/track_adapter.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/track_adapter_test.cpp`
- Modify: `simulator/src/fsai_sim2_adapter/src/simulation_context.cpp`
- Modify: `simulator/src/fsai_bringup/launch/upstream_compatibility.launch.py`

**Interfaces:**

- Consumes: Immutable TrackDefinition and SimulationContextFactory.
- Produces: EUFS `type::Track` conversion and atomic load service.

- [ ] **Step 1: Write failing conversion and failed-switch tests**

```cpp
TEST(TrackAdapter, PreservesConeGeometryAndColour) {
  auto converted = ToEufsTrack(track);
  EXPECT_EQ(FromEufsTrack(converted), track.ConeGeometry());
}

TEST(TrackSwitch, InvalidCandidateKeepsCurrentRevision) {
  auto before = owner.Current().TrackRevision();
  EXPECT_FALSE(owner.RequestTrackSwitch(InvalidBundle()));
  EXPECT_EQ(owner.Current().TrackRevision(), before);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R track_adapter_test --output-on-failure`

Expected: FAIL because the adapter is absent.

- [ ] **Step 3: Implement conversion and context replacement**

Convert only accepted definitions.
Preserve stable IDs in the project-side map even though EUFS map messages do not carry them.
Load and validate candidate bundles outside the step lock.
Create a fresh context with reset plant, sensors, race, safety, queues, RNG and simulation time.
Swap at the next outer-step boundary.
Publish `TrackRevision` only after success.

- [ ] **Step 4: Run service and replay tests**

Switch from reference circle to a second valid track and require time zero, initial vehicle pose, empty sensor queues and new hash.
Attempt an invalid switch and require no observable change.
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_interfaces simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup
git commit -m "feat: load tracks atomically"
```

## Plan Completion Gate

Legacy CSV and TrackBundle inputs must normalize to the documented model.
Centerline generation must pass analytic geometry checks.
Repeated generation must produce byte-identical CSV and YAML.
Invalid hot switches must preserve the running context.
Camera, LiDAR, RaceDirector and Foxglove must consume the same immutable TrackDefinition.
