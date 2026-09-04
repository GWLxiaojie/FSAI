# Complete FSAI Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Ubuntu 22.04 和 ROS 2 Humble 上交付可复现、可配置、可发布到用户独立 GitHub 仓库的完整 FSAI 仿真器。

**Architecture:** EUFS sim2 是固定 revision 的只读依赖，本项目的 `FsaiSimulationNode` 是唯一正式组合入口。
无 ROS 的 `fsai_sim_core` 独占车辆状态推进，传感器、赛道、比赛裁判、工件和 Foxglove 通过版本化 DTO 消费已接受状态。

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, C++20, CMake, ament, colcon, Eigen3, yaml-cpp, Python 3.10, pytest, launch_testing, MCAP, Foxglove Bridge.

**Spec:** `docs/superpowers/specs/2026-09-04-eufs-sim2-team-simulator-design.md`

## Global Constraints

正式运行平台是 Ubuntu 22.04 LTS 和 ROS 2 Humble。
Jazzy 只作为 Ubuntu 24.04 编译护栏。
EUFS sim2 必须保持只读，并固定到 `9f5df79a03725ea7d10542fc2ce8224d90836560`。
EUFS 官方 remote 只允许 fetch，push URL 必须是 `DISABLED`。
不得向 EUFS 创建 issue、pull request 或 merge request。
用户项目的远程仓库只在用户明确授权后配置或 push。
`fsai_sim_core` 公开头文件不得依赖 ROS 2、EUFS、文件 I/O、GUI 或 wall clock。
车辆状态只能由 `HybridIntegrator` 推进。
正式 plant 内部步长是 1 ms，外层步长是 5 ms。
连续动力学默认使用固定步 RK4。
正式动态自行车使用参数化 Pacejka B、C、D、E 侧向力。
摩擦制动命令是范围 `[0, 1]` 的无量纲值。
第一版只生成语义级 Camera 和 LiDAR 锥桶观测。
赛道输入只支持 EUFS CSV 和有序 `x,y` 中心线 CSV。
相同 revision、配置、步长和 seed 必须产生相同结果。
无人驾驶节点不得订阅 `/ground_truth/*`。
Foxglove 只能读取输出。
每项功能必须先有失败测试，再实现，再验证，再提交。
Markdown 中每个完整句子必须独占一个物理行。
提交消息不得添加 agent co-author。

---

## Repository Layout

```text
FSAI/
├── .github/workflows/
├── THIRD_PARTY_NOTICES.md
├── docs/superpowers/
├── simulator/
│   ├── fsai_sim.repos
│   └── src/
│       ├── eufs_sim2/                 generated, ignored, read-only
│       ├── vehicle_models/            generated, ignored, read-only
│       ├── state_lib/                 generated, ignored, read-only
│       ├── map_lib/                   generated, ignored, read-only
│       ├── eufs_msgs/                 generated, ignored, read-only
│       ├── fsai_sim_core/
│       ├── fsai_sim2_adapter/
│       ├── fsai_sensor_models/
│       ├── fsai_interfaces/
│       ├── fsai_track_tools/
│       ├── fsai_race/
│       ├── fsai_description/
│       └── fsai_bringup/
└── tools/
```

## Fixed Dependency Revisions

| Repository | Revision |
|---|---|
| `eufs_sim2` | `9f5df79a03725ea7d10542fc2ce8224d90836560` |
| `vehicle_models` | `3508bec2c3d77e0ff16f08794675d4f7b52479b7` |
| `state_lib` | `ec83a141f188e8a4c39a381f4666485d8cc83e20` |
| `map_lib` | `1919b36062850c9ba4553d1833a9b517c61c2e86` |
| `eufs_msgs` | `9e918686c9e9292c613f321e6fd85e3a5d87cd87` |
| `eufs-gmock-matchers` | `7ef83d030746c6a31bcf4f888d4121fcf4b7e8a9` |
| `eufs-logger` | `375ea1d8f8885af66809129e444624ba13353fa7` |
| `Open-Car-Dynamics` | `94f8fb187fb0ed22bba1d809bd74f66d1ff75af4` |

## Stable Cross-Plan Interfaces

```cpp
namespace fsai::sim {

using Duration = std::chrono::nanoseconds;
using SimTime = std::chrono::nanoseconds;

struct Command final {
  double steering_angle_rad{};
  double front_axle_torque_nm{};
  double rear_axle_torque_nm{};
  double friction_brake_ratio{};
};

struct StepResult final {
  PlantState next_state;
  GroundTruth ground_truth;
  std::vector<SimulationEvent> events;
  Diagnostics diagnostics;
};

class Plant final {
 public:
  StepResult Update(
      const PlantState& state,
      const Command& command,
      Duration dt) const;
};

struct SimulationSnapshot final {
  SimTime sim_time;
  PlantState plant;
  SensorSuiteState sensors;
  RaceDirectorState race;
  SafetyState safety;
  TrackRevision track;
};

}
```

`Plant::Update` must be deterministic for identical state, command, duration and immutable parameters.
`SimulationSnapshot` must include every value that can affect the next step.
`TrackDefinition` must be immutable after validation.
`Scenario` must name vehicle, track, mission, seed, timing mode and duration limit.

## Plan Sequence

1. `2026-09-04-01-workspace-readonly-eufs.md` establishes the reproducible workspace and external composition seam.
2. `2026-09-04-02-bicycle-plant-actuation.md` delivers the phase-one vehicle plant and safety chain.
3. `2026-09-04-03-sensors-interfaces.md` delivers Ground Truth history, sensor models and ROS interfaces.
4. `2026-09-04-04-track-bundles-generation.md` delivers custom file and centerline track workflows.
5. `2026-09-04-05-race-visualization-e2e.md` delivers race control, artifacts, Foxglove and the full-lap E2E.
6. `2026-09-04-06-four-wheel-double-track.md` upgrades the internal dynamics backend without changing external contracts.

Plans 3 and 4 may begin in parallel after Plan 2 freezes `PlantState`, `GroundTruth` and `Command`.
Plan 3 cone-sensor task must wait until Plan 4 freezes `TrackDefinition`.
Plan 5 consumes the final interfaces from Plans 3 and 4.
Plan 6 begins only after the phase-one full-lap E2E is stable.

## Specification Coverage

| Specification sections | Primary implementation plan | Acceptance evidence |
|---|---|---|
| 1 to 3, scope and non-goals | Master plan | Repository layout and global constraints remain inside the approved boundary |
| 4 to 5, upstream policy and EUFS integration | Plan 1 | Pinned manifest, disabled push URL, clean upstream checkout and composition tests |
| 6 to 11, modules, state, timing, bicycle dynamics, actuation and safety | Plan 2 | Core unit tests, deterministic replay and vehicle scenario suite |
| 12, Ground Truth and sensors | Plan 3 | Timestamp, interpolation, noise, delay, dropout and snapshot tests |
| 13, track input and generation | Plan 4 | EUFS import, centerline generation, canonical hash and atomic switch tests |
| 14 to 16, race control, ROS interfaces and visualization | Plan 5 | RaceDirector tests, interface tests, Foxglove layout and full-lap E2E |
| 17, configuration governance | Plans 2 to 5 | Versioned strict loaders, hashes and provenance in reports |
| 18 to 19, diagnostics, repeatability and artifacts | Plan 5 | Ordered events, reports, MCAP and replay equality checks |
| 20 to 23, platforms, migration, tests and CI | Plans 1 and 5 | Ubuntu 22.04 Humble CI plus Ubuntu 24.04 Jazzy compile guard |
| 24 to 26, delivery order, definition of done and approved decisions | Master plan and all six plans | Sequential gates, final clean-clone test and publication authorization gate |
| Phase-two four-wheel double-track extension | Plan 6 | Conservation, convergence, low-speed, snapshot and backend compatibility tests |

## Acceptance Gates

Plan 1 ends when an unmodified EUFS checkout can be composed through `FsaiSimulationNode` and both pacing modes produce the same trace.
Plan 2 ends when stationary, acceleration, circle, braking, timeout and EBS scenarios pass.
Plan 3 ends when every sensor reproduces sampling, noise, bias, delay, dropout and snapshot behavior.
Plan 4 ends when EUFS files and centerline inputs produce the same canonical `TrackDefinition` and deterministic bundle output.
Plan 5 ends when a controller completes one lap without Ground Truth subscriptions and produces a reproducible report and MCAP.
Plan 6 ends when four-wheel and OCD backends pass conservation, convergence, low-speed and handling comparisons.

## Global Verification

```bash
tools/bootstrap_ubuntu.sh
tools/build.sh
tools/test.sh
tools/run_scenario.sh simulator/src/fsai_bringup/scenarios/autocross_regression_01.yaml
tools/assert_replay_equal.sh simulator/artifacts/run_a simulator/artifacts/run_b
```

Humble CI executes all lint, unit, scenario and E2E checks.
Jazzy CI compiles and links core libraries and compatible adapters without claiming runtime support.
No CI workflow receives write permission to EUFS or to the user repository.
