# Read-Only EUFS Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立可重复的 Ubuntu 22.04 ROS 2 Humble 工作区，并通过自有 `FsaiSimulationNode` 组合未经修改的 EUFS sim2。

**Architecture:** vcstool 下载固定 revision 的 EUFS 依赖，并立即禁用官方 remote 的 push。
`fsai_sim2_adapter` 拥有 core factory、plugin registry、runner 和可替换 `SimulationContext`，EUFS 源码保持只读。

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, C++20, ament_cmake, colcon, vcstool, rosdep, GoogleTest, launch_testing.

**Spec:** `docs/superpowers/specs/2026-09-04-eufs-sim2-team-simulator-design.md`

## Global Constraints

不得修改或提交 `simulator/src/eufs_sim2`。
不得修改 `refs/eufs_sim2`。
EUFS remote 必须命名为 `upstream`，push URL 必须为 `DISABLED`。
正式入口必须是 `FsaiSimulationNode`。
兼容模式必须使用原始 `EufsCore` 证明组合接口有效。
`PreUpdate -> Core Step -> PostUpdate` 顺序必须保留。
实时和无头模式必须共享相同步进代码。
每个任务必须使用测试先行流程。

---

## File Map

```text
.gitignore
THIRD_PARTY_NOTICES.md
simulator/fsai_sim.repos
simulator/dependencies.lock.yaml
tools/prepare_eufs_checkout.sh
tools/bootstrap_ubuntu.sh
tools/build.sh
tools/test.sh
tools/run_sim.sh
tools/tests/
simulator/src/fsai_sim2_adapter/
  include/fsai_sim2_adapter/core_factory.hpp
  include/fsai_sim2_adapter/plugin_registry.hpp
  include/fsai_sim2_adapter/simulation_context.hpp
  include/fsai_sim2_adapter/simulation_runner.hpp
  include/fsai_sim2_adapter/simulation_node.hpp
  src/
  test/
simulator/src/fsai_bringup/
  launch/upstream_compatibility.launch.py
  test/test_upstream_compatibility.launch.py
```

### Task 1: Pin dependencies and enforce read-only EUFS remotes

**Files:**

- Create: `simulator/fsai_sim.repos`
- Create: `simulator/dependencies.lock.yaml`
- Create: `THIRD_PARTY_NOTICES.md`
- Create: `tools/prepare_eufs_checkout.sh`
- Create: `tools/tests/test_dependency_contract.sh`
- Modify: `.gitignore`

**Interfaces:**

- Consumes: Dependency revisions listed in the master plan.
- Produces: A valid vcstool manifest and `prepare_eufs_checkout.sh PATH`.

- [ ] **Step 1: Write the failing dependency contract**

```bash
#!/usr/bin/env bash
set -euo pipefail
vcs validate < simulator/fsai_sim.repos
grep -q "9f5df79a03725ea7d10542fc2ce8224d90836560" simulator/fsai_sim.repos
grep -q "3508bec2c3d77e0ff16f08794675d4f7b52479b7" simulator/fsai_sim.repos
grep -q "1919b36062850c9ba4553d1833a9b517c61c2e86" simulator/fsai_sim.repos
grep -q "ec83a141f188e8a4c39a381f4666485d8cc83e20" simulator/fsai_sim.repos
grep -q "9e918686c9e9292c613f321e6fd85e3a5d87cd87" simulator/fsai_sim.repos
test -x tools/prepare_eufs_checkout.sh
```

- [ ] **Step 2: Run the test and verify failure**

Run: `bash tools/tests/test_dependency_contract.sh`

Expected: FAIL because the manifest is absent.

- [ ] **Step 3: Create the manifest, lock, notices and remote guard**

Use the exact repository URLs and revisions from the master plan.
`prepare_eufs_checkout.sh` must accept exactly one checkout path, verify HEAD is `9f5df79a03725ea7d10542fc2ce8224d90836560`, rename `origin` to `upstream`, verify the fetch URL is `https://gitlab.com/eufs/public/eufs_sim2.git`, and run:

```bash
git -C "$1" remote set-url --push upstream DISABLED
```

The script must reject a dirty EUFS checkout.
Add generated dependency directories, `build`, `install`, `log`, `artifacts` and `*.mcap` to `.gitignore`.
Record each dependency revision and reviewed license in `dependencies.lock.yaml` and `THIRD_PARTY_NOTICES.md`.

- [ ] **Step 4: Verify a temporary import**

Run:

```bash
fsai_vcs_check_dir="$(mktemp -d)"
mkdir -p "${fsai_vcs_check_dir}/src"
vcs import "${fsai_vcs_check_dir}/src" < simulator/fsai_sim.repos
tools/prepare_eufs_checkout.sh "${fsai_vcs_check_dir}/src/eufs_sim2"
git -C "${fsai_vcs_check_dir}/src/eufs_sim2" remote get-url --push upstream
bash tools/tests/test_dependency_contract.sh
```

Expected: The push URL output is `DISABLED` and all tests pass.

- [ ] **Step 5: Commit**

```bash
git add .gitignore THIRD_PARTY_NOTICES.md simulator/fsai_sim.repos simulator/dependencies.lock.yaml tools
git commit -m "build: pin read-only EUFS dependencies"
```

### Task 2: Add idempotent Ubuntu build and test entry points

**Files:**

- Create: `tools/bootstrap_ubuntu.sh`
- Create: `tools/build.sh`
- Create: `tools/test.sh`
- Create: `tools/run_sim.sh`
- Create: `tools/tests/test_entry_points.sh`

**Interfaces:**

- Consumes: `fsai_sim.repos` and `prepare_eufs_checkout.sh`.
- Produces: Stable repository-root bootstrap, build, test and run commands.

- [ ] **Step 1: Write the failing entry-point test**

```bash
#!/usr/bin/env bash
set -euo pipefail
tools/bootstrap_ubuntu.sh --help
tools/build.sh --help
tools/test.sh --help
tools/run_sim.sh --help
grep -q 'VERSION_ID="22.04"' tools/bootstrap_ubuntu.sh
grep -q 'ros-humble-desktop' tools/bootstrap_ubuntu.sh
grep -q 'prepare_eufs_checkout.sh' tools/bootstrap_ubuntu.sh
```

- [ ] **Step 2: Run the test**

Run: `bash tools/tests/test_entry_points.sh`

Expected: FAIL because the scripts do not exist.

- [ ] **Step 3: Implement the four commands**

`bootstrap_ubuntu.sh` must reject non-22.04 systems.
It must install the official Humble apt source, `ros-humble-desktop`, `python3-vcstool`, `python3-rosdep`, `python3-colcon-common-extensions`, `clang-tidy` and `lcov`.
It must make repeated runs safe by checking keys, sources, packages, rosdep initialization and existing repositories.
After `vcs import` it must run `prepare_eufs_checkout.sh` before rosdep.
`build.sh` must source `/opt/ros/humble/setup.bash` and run colcon with fixed directories and `RelWithDebInfo`.
`test.sh` must run `colcon test` followed by `colcon test-result --verbose`.
`run_sim.sh` must require `--vehicle`, `--track` and `--scenario`.

- [ ] **Step 4: Run contract and clean build**

Run:

```bash
bash tools/tests/test_entry_points.sh
tools/build.sh
```

Expected: PASS on Ubuntu 22.04 with all imported EUFS packages built.

- [ ] **Step 5: Commit**

```bash
git add tools
git commit -m "build: add Ubuntu simulator entry points"
```

### Task 3: Compose the original EufsCore from the external package

**Files:**

- Create: `simulator/src/fsai_sim2_adapter/package.xml`
- Create: `simulator/src/fsai_sim2_adapter/CMakeLists.txt`
- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/simulation_node.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/simulation_node.cpp`
- Create: `simulator/src/fsai_sim2_adapter/src/main.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/eufs_core_composition_test.cpp`
- Create: `simulator/src/fsai_bringup/package.xml`
- Create: `simulator/src/fsai_bringup/CMakeLists.txt`
- Create: `simulator/src/fsai_bringup/launch/upstream_compatibility.launch.py`
- Create: `simulator/src/fsai_bringup/test/test_upstream_compatibility.launch.py`

**Interfaces:**

- Consumes: `eufs::sim2::SimulationBase`, `eufs::sim2::core::EufsCore` and EUFS public plugins.
- Produces: `FsaiSimulationNode` executable and compatibility launch.

- [ ] **Step 1: Write failing direct composition and launch tests**

```cpp
TEST(EufsComposition, AdvancesOriginalCoreThroughSimulationBase) {
  auto core = std::make_unique<eufs::sim2::core::EufsCore>(TestParams());
  eufs::sim2::SimulationBase simulation(std::move(core));
  simulation.Step(5ms);
  EXPECT_EQ(simulation.GetCore().GetTime(), 5ms);
}
```

The launch test must wait at most 20 seconds for two `/clock` messages and assert the second timestamp is greater than the first.

- [ ] **Step 2: Run focused tests**

Run:

```bash
tools/build.sh
colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter fsai_bringup --event-handlers console_direct+
```

Expected: FAIL because both project packages are missing.

- [ ] **Step 3: Implement the initial composition node**

`FsaiSimulationNode` must derive from `rclcpp::Node`.
For compatibility mode it must load EUFS vehicle parameters, create `EufsCore`, pass it to `SimulationBase`, register the selected public EUFS plugins, step at 5 ms, and publish `/clock`.
Do not include or modify any source below the imported `eufs_sim2` directory.
The launch file must set all paths through package shares and must not depend on `EUFS_MASTER`.

- [ ] **Step 4: Run tests and verify dependency cleanliness**

Run:

```bash
tools/build.sh
tools/test.sh
git -C simulator/src/eufs_sim2 status --porcelain
```

Expected: Tests pass and the final command prints nothing.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup
git commit -m "feat: compose read-only EUFS core"
```

### Task 4: Add project-owned CoreFactory and PluginRegistry

**Files:**

- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/core_factory.hpp`
- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/plugin_registry.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/core_factory.cpp`
- Create: `simulator/src/fsai_sim2_adapter/src/plugin_registry.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/core_factory_test.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/plugin_registry_test.cpp`
- Modify: `simulator/src/fsai_sim2_adapter/src/simulation_node.cpp`

**Interfaces:**

- Consumes: EUFS `CoreSimulationBase` and `Plugin`.
- Produces: `CoreFactory::Create` and `PluginRegistry::Create`.

- [ ] **Step 1: Write failing registry tests**

```cpp
TEST(CoreFactory, RejectsDuplicateAndUnknownKeys) {
  CoreFactory factory;
  factory.Register("eufs", MakeEufsCreator());
  EXPECT_THROW(factory.Register("eufs", MakeEufsCreator()), std::invalid_argument);
  EXPECT_THROW(factory.Create("missing", CoreConfig{}), std::invalid_argument);
}

TEST(PluginRegistry, UsesLongestBoundaryPrefix) {
  PluginRegistry registry;
  registry.Register("sensor", MakeNamedPlugin("generic"));
  registry.Register("sensor.camera", MakeNamedPlugin("camera"));
  EXPECT_EQ(registry.Create("sensor.camera.front")->Name(), "camera");
}
```

- [ ] **Step 2: Run the focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R 'core_factory|plugin_registry' --output-on-failure`

Expected: FAIL because the registries are undefined.

- [ ] **Step 3: Implement exact contracts**

```cpp
struct CoreConfig final {
  std::filesystem::path parameter_file;
};

class CoreFactory final {
 public:
  using Creator = std::function<std::unique_ptr<
      eufs::sim2::core::CoreSimulationBase>(const CoreConfig&)>;
  void Register(std::string key, Creator creator);
  std::unique_ptr<eufs::sim2::core::CoreSimulationBase> Create(
      std::string_view key, const CoreConfig& config) const;
};

class PluginRegistry final {
 public:
  using Creator = std::function<std::unique_ptr<
      eufs::sim2::plugin::Plugin>(std::string)>;
  void Register(std::string prefix, Creator creator);
  std::unique_ptr<eufs::sim2::plugin::Plugin> Create(
      std::string_view instance_name) const;
};
```

Use ordered maps.
A plugin prefix matches only the whole name or a dot boundary.
Validate every configured core and plugin before creating ROS entities.
Register `eufs` and all selected EUFS public plugins in one composition function.

- [ ] **Step 4: Run unit and compatibility tests**

Run:

```bash
colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --event-handlers console_direct+
colcon test --base-paths simulator/src --packages-select fsai_bringup --event-handlers console_direct+
```

Expected: PASS with unchanged compatibility topics.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim2_adapter
git commit -m "feat: add simulator composition registries"
```

### Task 5: Add deterministic runner modes

**Files:**

- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/simulation_runner.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/simulation_runner.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/simulation_runner_test.cpp`
- Modify: `simulator/src/fsai_sim2_adapter/src/simulation_node.cpp`
- Modify: `simulator/src/fsai_bringup/launch/upstream_compatibility.launch.py`

**Interfaces:**

- Consumes: `SimulationBase::Step(Duration)`.
- Produces: `SimulationRunner::StepOnce`, `RunSteps` and `RunMode`.

- [ ] **Step 1: Write failing fixed-step tests**

```cpp
TEST(SimulationRunner, AdvancesExactSteps) {
  SimulationRunner runner(simulation, 5ms);
  runner.RunSteps(3);
  EXPECT_EQ(fake_core.StepDurations(), std::vector{5ms, 5ms, 5ms});
  EXPECT_EQ(fake_core.GetTime(), 15ms);
}

TEST(SimulationRunner, ModesProduceEqualTrace) {
  EXPECT_EQ(RunTrace(RunMode::kRealtime),
            RunTrace(RunMode::kAsFastAsPossible));
}
```

- [ ] **Step 2: Run the test**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R simulation_runner_test --output-on-failure`

Expected: FAIL because `SimulationRunner` is absent.

- [ ] **Step 3: Implement deterministic stepping**

```cpp
enum class RunMode { kRealtime, kAsFastAsPossible };

class SimulationRunner final {
 public:
  SimulationRunner(eufs::sim2::SimulationBase& simulation, Duration outer_step);
  void StepOnce();
  void RunSteps(std::size_t count);
 private:
  eufs::sim2::SimulationBase& simulation_;
  Duration outer_step_;
};
```

`StepOnce` must be the only direct call site of `SimulationBase::Step`.
Wall pacing must remain in `FsaiSimulationNode` and must not change `outer_step`.
Reject zero, negative and fractional-nanosecond timing parameters.

- [ ] **Step 4: Run both launch modes**

Run:

```bash
ros2 launch fsai_bringup upstream_compatibility.launch.py run_mode:=realtime max_steps:=20
ros2 launch fsai_bringup upstream_compatibility.launch.py run_mode:=as_fast_as_possible max_steps:=20
```

Expected: Both finish at exactly 100 ms and produce equal state traces.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup
git commit -m "feat: add deterministic simulator runner"
```

### Task 6: Add atomic SimulationContext reset

**Files:**

- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/simulation_context.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/simulation_context.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/simulation_context_test.cpp`
- Modify: `simulator/src/fsai_sim2_adapter/src/simulation_node.cpp`
- Modify: `simulator/src/fsai_bringup/test/test_upstream_compatibility.launch.py`

**Interfaces:**

- Consumes: CoreFactory, PluginRegistry and SimulationRunner.
- Produces: `SimulationContextFactory::Create` and `FsaiSimulationNode::RequestReset`.

- [ ] **Step 1: Write failing transactional reset tests**

```cpp
TEST(SimulationContext, FailedReplacementKeepsCurrentContext) {
  auto original = MakeContext();
  ContextOwner owner(std::move(original));
  EXPECT_THROW(owner.Replace(FailingConfig()), ConfigurationError);
  EXPECT_EQ(owner.Current().CoreTime(), 25ms);
}

TEST(SimulationContext, SuccessfulResetRestoresObservableState) {
  ContextOwner owner(MakeAdvancedContext());
  owner.Replace(InitialConfig());
  EXPECT_EQ(owner.Current().CoreTime(), 0ns);
  EXPECT_EQ(owner.Current().TrackRevision(), 0u);
  EXPECT_EQ(owner.Current().ASState(), ASState::OFF);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R simulation_context_test --output-on-failure`

Expected: FAIL because context replacement is missing.

- [ ] **Step 3: Implement prepare-then-swap reset**

`SimulationContextFactory::Create` must fully validate configuration, construct core, simulation, runner and plugins, and return an owning context.
`ContextOwner::Replace` must construct the candidate before locking the step boundary.
It must swap only after construction succeeds and before the next `StepOnce`.
The reset service must publish success only after the swap.
No partially initialized publisher, subscription or state may become observable.

- [ ] **Step 4: Verify reset replay and EUFS cleanliness**

Run the compatibility E2E, advance 200 steps, reset, replay the same commands and compare traces byte for byte.
Then run `git -C simulator/src/eufs_sim2 status --short`.

Expected: Replay matches and EUFS is clean.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup
git commit -m "feat: add atomic simulation context reset"
```

### Task 7: Add Humble CI and Jazzy compile guard

**Files:**

- Create: `.github/workflows/humble.yml`
- Create: `.github/workflows/jazzy.yml`
- Create: `tools/ci_humble.sh`
- Create: `tools/ci_jazzy.sh`
- Create: `tools/tests/test_ci_contract.sh`

**Interfaces:**

- Consumes: Bootstrap, build and test entry points.
- Produces: Full Humble gate and Jazzy build guard without write permissions.

- [ ] **Step 1: Write the failing CI contract**

```bash
#!/usr/bin/env bash
set -euo pipefail
grep -q "ubuntu-22.04" .github/workflows/humble.yml
grep -q "ROS_DISTRO: humble" .github/workflows/humble.yml
grep -q "ubuntu-24.04" .github/workflows/jazzy.yml
grep -q "ROS_DISTRO: jazzy" .github/workflows/jazzy.yml
grep -q "contents: read" .github/workflows/humble.yml
grep -q "prepare_eufs_checkout.sh" tools/ci_humble.sh
```

- [ ] **Step 2: Run the contract**

Run: `bash tools/tests/test_ci_contract.sh`

Expected: FAIL because workflows are missing.

- [ ] **Step 3: Implement workflows**

Humble must import locked dependencies, enforce read-only EUFS remotes, run rosdep, build every package and run all tests.
Jazzy must build and test pure libraries plus compatible adapters, without full scenario E2E.
Both workflows must set `permissions: contents: read`.
Do not add retries or remote write tokens.

- [ ] **Step 4: Run final Plan 1 verification**

Run:

```bash
bash tools/tests/test_dependency_contract.sh
bash tools/tests/test_entry_points.sh
bash tools/tests/test_ci_contract.sh
tools/ci_humble.sh
git -C simulator/src/eufs_sim2 status --short
```

Expected: All checks pass and EUFS status is empty.

- [ ] **Step 5: Commit**

```bash
git add .github tools
git commit -m "ci: validate read-only simulator integration"
```

## Plan Completion Gate

The compatibility node must run 200 fixed 5 ms steps and finish at exactly 1 second.
Realtime and as-fast-as-possible traces must match.
Reset and replay must match byte for byte.
`simulator/src/eufs_sim2` must remain unmodified.
No remote push is part of this plan.
