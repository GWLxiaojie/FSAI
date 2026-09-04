# 基于 EUFS sim2 的车队可配置 FSAI 仿真器设计

日期：2026-09-04。

状态：已完成方案讨论，等待最终书面审阅。

## 1. 摘要

本项目将基于 EUFS sim2 建设一个不依赖 Isaac Sim 的完整 Formula Student AI 仿真器。
EUFS sim2 提供 ROS 2 编排、插件生命周期、赛道、比赛状态机和 Foxglove 集成。
车队自己的车辆动力学、执行器、传感器、参数和接口放在独立模块中。
车辆状态只能由独立的 C++ plant 推进。
ROS 2、赛道裁判、传感器和可视化不能反向拥有车辆动力学。
第一版使用带噪声的语义级锥桶观测，不生成原始相机图像或 LiDAR 点云。
第一版正式支持 Ubuntu 22.04 LTS 和 ROS 2 Humble。
CI 额外执行 Ubuntu 24.04 和 ROS 2 Jazzy 编译检查，为后续迁移保留护栏。

详细上游调研见 [EUFS 与其他 FSAI 仿真器调研](../../../research/eufs_simulator_architecture_research.md)。
车辆物理基线见 [车辆动力学仿真计划](../../../vehicle_model_simulation_plan.txt)。

## 2. 目标

本项目必须允许车队通过配置接入自己的车辆，而不需要修改 EUFS sim2 的平台代码。
本项目必须支持动态自行车和后续四轮双轨车辆模型。
本项目必须覆盖转向、驱动、摩擦制动、再生制动、EBS 和控制命令超时。
本项目必须提供 IMU、GNSS、轮速、转角、OSS、Camera 锥桶和 LiDAR 锥桶观测。
本项目必须支持 acceleration、skidpad、autocross、trackdrive 和人工测试场景。
本项目必须支持 EUFS 兼容 CSV 赛道和中心线自动生成赛道。
本项目必须支持无头批量运行和 Foxglove 交互运行。
本项目必须保证相同版本、配置、步长和随机 seed 产生相同结果。
本项目必须允许同一套车辆核心以后迁移到 ROS 2 Jazzy，而不修改车辆方程。

## 3. 非目标

第一版不生成相机像素。
第一版不生成 LiDAR 原始点云。
第一版不提供图形化赛道编辑器。
第一版不使用 Isaac Sim、Gazebo、Unreal 或 Unity 推进车辆状态。
第一版不模拟行人、交通流、城市路网或非 Formula Student 场景。
第一版不承诺严格实时 HiL。
第一版不接入真实 CAN、EBS 或 USB 硬件。
第一版不将未经实车数据验证的高自由度模型称为高精度模型。
本项目不直接复制 GPL-2.0 的 FSDS 实现。

## 4. 上游策略与仓库边界

`refs/eufs_sim2` 保持为只读的完整上游参考仓库。
开发代码不能直接写入 `refs/eufs_sim2`。
车队应建立自己的 EUFS sim2 fork。
车队 fork 使用 `origin` 指向车队仓库，并使用 `upstream` 指向 EUFS 官方仓库。
车队 fork 只包含对通用平台扩展能力的修改。
车辆参数、传感器参数、车队 ROS 话题和车队 CAN 语义不能写入 EUFS sim2 fork。

建议工作区结构如下。

```text
FSAI/
├── docs/
├── refs/
│   └── eufs_sim2/
├── research/
└── simulator/
    ├── fsai_sim.repos
    ├── tools/
    └── src/
        ├── eufs_sim2/
        ├── fsai_sim_core/
        ├── fsai_sim2_adapter/
        ├── fsai_sensor_models/
        ├── fsai_interfaces/
        ├── fsai_description/
        └── fsai_bringup/
```

`fsai_sim.repos` 固定所有依赖仓库及其提交。
构建过程不能在没有更新 manifest 的情况下拉取依赖仓库的最新分支。
EUFS sim2、`vehicle_models`、`state_lib`、`map_lib` 和 `eufs_msgs` 都必须固定 revision。

## 5. EUFS sim2 fork 的允许修改

fork 增加 `CoreFactory`，用于按配置创建 `EufsCore` 或 `FsaiCoreAdapter`。
fork 将当前硬编码插件创建逻辑替换为可验证的 `PluginRegistry`。
fork 将车辆步进与 wall timer 解耦，使实时和无头模式共享相同的核心步进代码。
fork 为核心接口和插件接口增加显式 schema version。
fork 恢复自动化测试，并覆盖插件顺序、重置和错误路径。
fork 保持 EUFS 的 MIT 许可证和版权声明。
fork 不包含任何特定年份车辆的硬编码物理常数。

EUFS sim2 的 `PreUpdate -> Core Step -> PostUpdate` 生命周期作为平台基础保留。
`PreUpdate` 只允许处理会影响当前步的命令、状态机和赛道控制。
`PostUpdate` 只允许读取已经接受的状态并生成传感器、日志、裁判和可视化输出。

## 6. 核心模块

### 6.1 `fsai_sim_core`

`fsai_sim_core` 是无 ROS、无文件 I/O、无 GUI 和无 wall clock 的 C++20 库。
它只依赖数值计算、参数 DTO 和自己的强类型接口。
它拥有执行器、车辆动力学、轮胎接触、积分器和 Ground Truth 构建。

核心公开入口为以下语义。

```cpp
StepResult update(
    const PlantState& state,
    const Command& command,
    Duration dt);
```

`StepResult` 包含下一时刻 `PlantState`、构建 Ground Truth 所需的力学解、离散事件和诊断量。
对于相同的状态、命令、步长、参数和 RNG 状态，`update` 必须返回相同结果。
所有影响下一步结果的变量必须存在于可序列化状态中。
影响结果的隐藏缓存不允许保存在不可见的模型对象成员中。

### 6.2 `fsai_sim2_adapter`

`FsaiCoreAdapter` 实现 EUFS sim2 的核心仿真接口。
它把 EUFS 控制输入转换为 FSAI `Command`。
它调用 `fsai_sim_core` 推进车辆。
它把 `StepResult` 映射为 EUFS 插件可以读取的车辆状态、轮速和车辆力。
它不能重新计算轮胎力或覆盖 plant 已接受的状态。

### 6.3 `fsai_sensor_models`

`fsai_sensor_models` 从 Ground Truth 历史生成车载级传感器观测。
它拥有各传感器的采样调度、安装外参、噪声、bias、量化、延迟、丢包和 RNG 状态。
传感器状态不属于 `PlantState`，但必须进入完整的 `SimulationSnapshot`。

### 6.4 `fsai_interfaces`

`fsai_interfaces` 包含核心 DTO 与 ROS 2 消息之间的转换。
EUFS 兼容接口和车队接口通过不同 adapter 暴露。
任何 ROS 2 类型都不能进入 `fsai_sim_core` 的公开头文件。

### 6.5 `fsai_description`

`fsai_description` 保存 URDF、网格、碰撞轮廓和传感器安装外参。
车辆几何参数与动力学参数必须引用同一个车辆 profile。
URDF 只能用于显示和坐标变换，不能成为车辆动力学的状态所有者。

### 6.6 `fsai_bringup`

`fsai_bringup` 保存 launch、车辆 profile、赛道 bundle、场景和 Foxglove layout。
用户通过车辆名和赛道名启动仿真。
用户不需要修改 C++ 才能切换车辆或赛道。

## 7. 车辆状态与输入契约

核心 `Command` 使用实际车辆控制语义。

```text
steering_angle_cmd       前轴中心等效道路轮角，单位 rad
front_axle_torque_cmd    前轴车轮侧总扭矩，单位 Nm
rear_axle_torque_cmd     后轴车轮侧总扭矩，单位 Nm
friction_brake_cmd       无量纲摩擦制动请求，范围为 [0, 1]
```

正扭矩表示驱动车辆前进。
负扭矩只表示真实 VCU 允许的再生制动。
摩擦制动与再生制动由执行器或 VCU 模型仲裁，不能重复施加制动力。
真实车辆接口使用制动压力时，车队 adapter 必须根据车辆 profile 中经过标定的映射将压力转换为归一化请求。
压力单位和压力到制动力矩的映射不能进入核心 `Command` 契约。

`PlantState` 至少包含以下内容。

```text
schema_version
backend_id
parameter_hash
chassis: X, Y, yaw, u, v, yaw_rate
actuator: actual steering, actual torques, actual brake, delay and filter state
wheels: omega_FL, omega_FR, omega_RL, omega_RR, contact and brake modes
backend: versioned backend-specific continuous state
```

不同 `backend_id` 的隐藏连续状态不能盲目互相恢复。
加载 snapshot 时必须同时验证 schema version、backend revision 和 parameter hash。

## 8. 每步计算顺序

每个外层仿真步执行以下顺序。

```text
1. 读取已经到达的控制命令
2. 更新 AS 状态机、RES、EBS 和命令 watchdog
3. 生成当前步的安全有效 Command
4. 更新 ActuatorModel
5. 由 HybridIntegrator 推进车辆状态
6. 构建已接受状态的 Ground Truth
7. 将 Ground Truth 写入时间历史
8. 更新 RaceDirector
9. 按采样时刻生成传感器观测
10. 释放已经到达发布时间的延迟消息
11. 发布 ROS 2、日志和 Foxglove 数据
```

步骤顺序是外部可观察行为的一部分。
插件注册顺序不能改变这一语义。
某步计算失败时不能发布一半更新后的状态。

## 9. 时间模型

第一版 plant 内部目标频率为 1000 Hz。
EUFS sim2 外层默认调度频率为 200 Hz。
每个 5 ms 外层步由 `HybridIntegrator` 拆成五个 1 ms 内部子步。
连续动力学默认使用固定步 RK4。
停车、制动保持和接触模式变化由混合事件逻辑处理。

系统支持 `realtime` 和 `as_fast_as_possible` 两种运行模式。
两种模式必须调用完全相同的 plant 和传感器代码。
运行模式只能改变 wall clock 节奏，不能改变 `sim_time`、步长或采样时刻。
ROS `/clock` 是全部 ROS 节点的时间来源。

## 10. 车辆动力学路线

### 10.1 第一阶段动态自行车

第一阶段使用动态自行车与一致的低速运动学处理。
状态覆盖世界位置、航向、车体纵横向速度和横摆角速度。

```text
X_dot   = u cos(yaw) - v sin(yaw)
Y_dot   = u sin(yaw) + v cos(yaw)
yaw_dot = r
u_dot   = r v  + sum(Fx_body) / mass
v_dot   = -r u + sum(Fy_body) / mass
r_dot   = sum(Mz) / Iz
```

前后轴侧偏角从轮胎接地点速度计算。
正式动态自行车后端使用参数化 Pacejka B、C、D、E 模型计算侧向力。
线性轮胎模型只作为单元测试和解析结果对照后端，不能作为第一版正式场景的默认模型。
纵向力来自驱动扭矩、摩擦制动、再生制动、滚动阻力和空气阻力。
静止时不能因为转角命令产生不存在的轮胎力。
负纵向速度不能通过积分后简单截断掩盖数值或物理错误。

第一阶段用于钉死坐标系、符号、执行器、时间、ROS 闭环和场景级测试。
第一阶段不宣称具有最终 handling 精度。

### 10.2 第二阶段四轮双轨

第二阶段保持 `Command`、`PlantState` 和 `StepResult` 的外部语义不变。
内部 `DynamicsBackend` 替换为逐轮双轨模型。
每个车轮分别计算轮心速度、纵滑率、侧偏角、法向载荷、纵向力、侧向力和角速度。

```text
omega_dot =
  (drive_torque - brake_torque - tyre_force_x * effective_radius)
  / wheel_inertia
```

`SteeringGeometry` 从等效道路轮角生成左右道路轮角。
`ContactSolver` 拥有 `STICK`、`TRANSITION`、`SLIP` 和 `BRAKE_HOLD`。
`HybridIntegrator` 是唯一允许推进 chassis 和 wheel 状态的组件。
Open-Car-Dynamics 只能通过 `OcdDoubleTrackBackend` 作为连续动力学后端接入。
同一时刻只能有一个模块拥有轮胎力、轮荷和气动力。

## 11. 执行器与安全链路

控制命令先经过命令超时检查和 AS 状态机。
车辆未进入 `DRIVING` 时不能施加驱动扭矩。
命令超时后撤销驱动扭矩并执行配置定义的安全制动。
EBS 一旦触发，普通控制命令不能解除它。

`ActuatorModel` 负责以下行为。

- 转角饱和和转角速率限制。
- 驱动扭矩和制动命令饱和。
- 经过实车数据确认的纯延迟。
- 经过实车数据确认的一阶响应。
- 摩擦制动与再生制动仲裁。
- 实际执行器状态的序列化。

未经测试辨识的动态参数必须标记为未标定参数，并不能作为高保真结论的依据。

## 12. Ground Truth 与传感器

Ground Truth 只从已经接受的最终状态和力解构建。
Ground Truth 与带噪观测使用不同的数据路径。

IMU 使用质心物理加速度，而不是车体系速度状态的直接导数。

```text
a_x_CG = u_dot - r v
a_y_CG = v_dot + r u
```

每个传感器执行以下处理顺序。

```text
Ground Truth history
  -> sample-time interpolation
  -> mounting transform
  -> ideal observation
  -> bias, noise, scale, quantization and saturation
  -> dropout and failure
  -> delay queue
  -> ROS 2 publication
```

消息 header 使用采样时间。
消息发布时间单独由延迟队列决定。
相同 seed 必须产生相同噪声、丢包和延迟序列。

第一版传感器如下。

| 传感器 | 默认频率 | 第一版误差模型 |
|---|---:|---|
| IMU | 200 Hz | 白噪声、bias random walk、安装误差和饱和 |
| 轮速 | 100 Hz | 编码器量化、延迟和丢脉冲 |
| 转角 | 100 Hz | 偏置、量化和延迟 |
| GNSS | 10 Hz | 位置噪声、慢变偏置和失锁状态 |
| OSS | 100 Hz | 纵横向速度噪声和安装外参 |
| Camera 锥桶 | 20 Hz | 视场、距离、漏检、颜色混淆和位置噪声 |
| LiDAR 锥桶 | 20 Hz | 视场、距离、漏检和距离角度噪声 |

Camera 和 LiDAR 分别发布自己的锥桶观测。
模拟融合输出是可选输出，不能替代单传感器输出。
Ground Truth 只能发布到明确的 debug namespace。

## 13. 赛道格式与自定义流程

第一版支持 EUFS 兼容 CSV 导入和中心线自动生成。
第一版不提供图形化赛道编辑器。

每条正式赛道保存为一个 `TrackBundle`。

```text
tracks/
└── custom_autocross_01/
    ├── cones.csv
    ├── track.yaml
    └── preview.png
```

`preview.png` 是可选的生成工件，不能作为加载输入或赛道事实来源。

`cones.csv` 使用以下 EUFS 兼容表头。

```csv
tag,x,y,direction,x_variance,y_variance,xy_covariance
```

支持的 tag 为 `blue`、`yellow`、`orange`、`big_orange`、`unknown` 和 `car_start`。
普通锥桶的 `x` 和 `y` 使用米。
`car_start` 的 `direction` 使用 rad，并表示车辆初始 yaw。

规范内存模型 `TrackDefinition` 保存锥桶几何、赛道元数据、车辆初始位姿、起终点门和稳定 ID。
在 `TrackBundle` 中，`cones.csv` 是锥桶几何的事实来源。
`track.yaml` 是赛道名、mission、坐标系、单位、车辆初始位姿、起终点门、规则版本、生成方式和 seed 的事实来源。
EUFS CSV exporter 根据 `track.yaml` 生成兼容的 `car_start` 行，避免在 bundle 中分别维护两份初始位姿。
旧 EUFS CSV 中的 `car_start` 只由 legacy importer 读取，并在保存时归一化到 `track.yaml`。
加载完成后，所有运行时组件只能读取不可变的 `TrackDefinition`，不能继续直接读取两个源文件。
赛道 hash 根据规范序列化后的完整 `TrackDefinition` 计算。

加载器必须验证以下内容。

- 表头和字段数量正确。
- 所有数值都是有限值。
- tag 属于支持集合。
- 起点和起终点门完整。
- 锥桶间距和赛道宽度满足选定规则 profile。
- 锥桶 ID 在导入后稳定且唯一。
- 赛道 hash 与场景报告一致。

中心线生成器接受表头严格为 `x,y` 的有序 CSV 点列。
赛道宽度、锥桶间距、起点位置、规则 profile 和 seed 由生成配置 YAML 提供。

```yaml
schema_version: 1
track_name: custom_autocross_01
mission: autocross
closed: true
track_width_m: 3.0
cone_spacing_m: 3.0
start:
  x_m: 0.0
  y_m: 0.0
  yaw_rad: 0.0
rules_profile: fs_2026
seed: 20260904
```

`closed: true` 要求中心线首尾形成闭环。
`closed: false` 保留有序开放中心线，并要求生成配置给出明确的终点门。
输入点顺序定义赛道行驶方向。
生成器按弧长重采样中心线。
生成器通过中心线切向的法向偏移生成左右边界。
沿行驶方向观察时，左边界生成 `blue` 锥桶，右边界生成 `yellow` 锥桶。
起终点门根据规则 profile 生成 `big_orange` 锥桶。
生成器检查自交、过小曲率半径、边界翻转和不合法间距。
生成器输出完整 `TrackBundle`，而不是只在内存中生成临时地图。
相同输入和 seed 必须生成逐字节一致的 `cones.csv` 与语义一致的 `track.yaml`。

运行时可以从文件或 ROS 2 `SetTrack` 消息加载赛道。
热切换赛道必须执行原子 reset。
运行中的赛道随机扰动必须使用场景 seed，不能使用 `std::random_device` 直接决定结果。

## 14. 场景与 RaceDirector

每次运行由版本化 `Scenario` 描述。

```yaml
scenario:
  schema_version: 1
  name: autocross_regression_01
  mission: autocross
  vehicle: our_team_2026
  track: custom_autocross_01
  seed: 20260904
  duration_limit: 120.0

simulation:
  mode: realtime
  plant_step_seconds: 0.001
  outer_step_seconds: 0.005
```

`RaceDirector` 负责起跑、结束、计圈、计时、碰锥、出界、超时、DNF、RES 和 EBS 事件。
`RaceDirector` 只能读取已接受车辆状态和赛道事件。
需要停车时，`RaceDirector` 必须通过正常安全命令链触发 EBS。
`RaceDirector` 不能直接覆盖车辆位置、速度或轮速。

reset 必须同时恢复 plant、执行器、传感器、延迟队列、RNG、RaceDirector、AS 状态机、赛道、计圈和仿真时间。
reset 成功后不能保留上一次运行的可观察状态。

## 15. ROS 2 接口

第一版正式使用 Ubuntu 22.04 和 ROS 2 Humble。
EUFS adapter 保持 `/cmd`、`eufs_msgs` 和现有状态机接口可用。
车队 adapter 将来映射车队实际 ROS 2 或 CAN 语义，但不会改变核心 DTO。

接口按用途隔离到以下 namespace。

```text
/clock
/sim/control/*
/sim/state/*
/sensors/*
/ground_truth/*
/race/*
/diagnostics/*
```

连续数据使用 topic。
短时同步操作使用 service。
需要进度和取消语义的长操作使用 action。
所有名称允许通过 ROS remapping 适配 EUFS 或车队栈。

EUFS 的加速度命令只能在兼容 adapter 中转换为物理扭矩命令。
该转换必须记录所使用的质量、轮胎半径、驱动布局和补偿策略。
该兼容路径不能被描述为真实 VCU 模型。

CI 的闭环测试必须检查无人驾驶节点没有订阅 `/ground_truth/*`。

## 16. Foxglove 可视化

Foxglove 通过 Foxglove Bridge 读取 ROS 2 数据。
Foxglove 不参与车辆积分、传感器生成或比赛裁判。

第一版 layout 显示以下内容。

- 车辆 URDF 和四轮转角。
- 赛道和锥桶颜色。
- 车辆真实轨迹。
- Camera 和 LiDAR 视场。
- 单传感器锥桶观测。
- Ground Truth 与带噪观测对比。
- 速度、加速度、横摆角速度和轮速。
- 请求与实际转角、扭矩和制动状态。
- 轮胎力、滑移和接触模式诊断。
- AS 状态、mission、圈数、碰锥、出界和计时。

无头模式不启动 Foxglove Bridge，但输出与交互模式相同的 MCAP 和场景报告。
交互模式与无头模式不能使用不同的物理或传感器代码路径。

## 17. 配置与参数治理

车辆通过 profile 配置。

```text
vehicles/
└── our_team_2026/
    ├── vehicle.yaml
    ├── actuators.yaml
    ├── tyres.yaml
    ├── aero.yaml
    ├── sensors.yaml
    ├── interfaces.yaml
    └── robot.urdf.xacro
```

每个物理参数必须具有单位、来源、有效范围和校准状态。
加载后参数对象不可变。
派生参数由代码从最小独立参数集计算。
参数 hash 写入每次 snapshot、MCAP metadata 和场景报告。
配置 schema 变化必须增加 schema version 并提供明确迁移器。

## 18. 错误处理与诊断

配置缺失、单位错误、schema 不匹配或参数越界时拒绝启动。
赛道错误必须报告文件、行号、字段和值。
车辆状态出现 NaN 或无限值时立即终止当前场景。
数值失败时保存最后有效 snapshot、失败命令、参数 hash、track hash 和 seed。
失败步不能发布部分更新后的 Ground Truth 或传感器消息。
命令超时进入安全制动，并在诊断与场景报告中记录事件。
传感器故障、丢包和恢复都必须生成带仿真时间的事件。
插件抛出异常时，runner 必须将场景标记为失败并执行受控停止。

## 19. 可重复性与工件

每次场景运行输出以下工件。

```text
run.json
events.jsonl
simulation.mcap
final_snapshot.bin
resolved_vehicle_params.yaml
resolved_scenario.yaml
```

`run.json` 记录代码 revision、依赖 revision、参数 hash、track hash、seed、平台和最终结果。
`events.jsonl` 记录状态机、碰锥、出界、故障和安全事件。
MCAP 记录 ROS 2 输出与 Ground Truth debug 数据。
回放工具必须能够从 resolved 配置和 snapshot 重现失败附近的仿真状态。

## 20. 平台和开发环境

正式运行平台为 Ubuntu 22.04 LTS 和 ROS 2 Humble。
支持架构为 amd64 和 arm64。
主要构建使用 C++20、CMake、ament 和 colcon。
Python 工具以 Ubuntu 22.04 的 Python 3.10 为基线。
本项目不提供 macOS 原生 ROS 2 构建支持。

仓库提供以下入口。

```text
tools/bootstrap_ubuntu.sh
tools/build.sh
tools/test.sh
tools/run_sim.sh
```

`bootstrap_ubuntu.sh` 必须检查系统版本，并可安全重复执行。
`bootstrap_ubuntu.sh` 使用 apt 和 rosdep 安装依赖，不能修改不相关的用户配置。
`build.sh` 必须支持干净构建和增量构建。
`run_sim.sh` 必须要求显式或有记录的默认车辆与场景。

## 21. Humble 到 Jazzy 的迁移护栏

`fsai_sim_core` 在 Humble 和 Jazzy 中使用相同源码。
ROS 2 依赖只能出现在 adapter、plugin、bringup 和 interface 包中。
ROS 时间只在边界转换为核心 `Duration` 和 `TimePoint`。
核心配置、车辆参数、赛道和 snapshot schema 与 ROS 2 发行版无关。
代码不使用已经在 Humble 标记为废弃的 API。
CI 在 Ubuntu 24.04 和 ROS 2 Jazzy 上执行编译、链接和核心测试。
Jazzy 在第一版不是正式运行平台，因此 CI 不要求完整场景 E2E。
迁移时允许修改 ROS adapter 和构建清单，但不允许修改车辆方程来解决 ROS 兼容问题。

## 22. 测试策略

### 22.1 静态检查

所有 C++ 目标必须通过格式检查、编译警告、clang-tidy 和 include 自包含检查。
所有 Python 工具必须通过格式、lint、类型检查和单元测试。
所有 ROS package 必须通过 `ament_lint`。
任何已知 flaky test 都必须修复或删除，不能通过无限重试隐藏。

### 22.2 单元测试

单元测试覆盖执行器饱和、速率、延迟和状态恢复。
单元测试覆盖轮胎力符号、车辆方程和积分器收敛。
单元测试覆盖轮速、转向几何和低速接触行为。
单元测试覆盖每种传感器的采样、噪声、bias、延迟、丢包和 snapshot 恢复。
单元测试覆盖 CSV 导入、赛道验证、中心线生成和确定性输出。
单元测试覆盖 RaceDirector 的所有状态转移和非法转移。

### 22.3 场景测试

场景测试覆盖静止无假轮胎力。
场景测试覆盖直线加速、定转角圆周、制动停车和命令超时。
场景测试覆盖 EBS、碰锥、出界、计圈和任务结束。
场景测试覆盖相同 seed 的逐步重放一致性。
场景测试覆盖不同 outer step 下的物理收敛和传感器采样一致性。

### 22.4 E2E 测试

E2E 测试必须以用户实际启动方式运行完整 ROS 2 launch。
E2E 测试必须加载车辆 profile 和赛道 bundle。
E2E 测试必须经过 AS 状态机进入 `DRIVING`。
E2E 测试必须运行真实无人驾驶节点或正式测试控制器。
E2E 测试必须验证无人驾驶节点未订阅 Ground Truth。
E2E 测试必须验证最终报告、MCAP 和受控退出。

首个完整验收目标如下。

> 同一套无人驾驶软件不订阅 Ground Truth，在自定义赛道上通过模拟 Camera 和 LiDAR 锥桶观测完成一圈，并输出可重复的计时与事件报告。

Foxglove layout 发布前必须在支持的 Ubuntu 显示环境进行人工视觉检查。
车辆、锥桶、TF、视场和传感器观测必须在同一坐标系中正确对齐。
明显的模型穿插、轮角错误、比例错误或锥桶颜色错误必须在发布前修复。

## 23. CI 矩阵

Ubuntu 22.04 + ROS 2 Humble 执行全部 lint、单元、场景和 E2E 测试。
Ubuntu 24.04 + ROS 2 Jazzy 执行编译、链接和无 ROS 核心测试。
amd64 是完整 CI 的主架构。
arm64 至少执行依赖解析、编译和核心测试。
依赖 manifest 更新必须单独显示 revision 差异，并触发全部测试。

## 24. 交付顺序

第一步是建立 Ubuntu 22.04 + Humble 的可重复工作区和 CI。
第二步是使车队 fork 的原始 `EufsCore` 通过完整启动 smoke test。
第三步是引入 `CoreFactory`、`PluginRegistry` 和可测试 runner。
第四步是接入动态自行车 `FsaiCoreAdapter` 与执行器安全链。
第五步是接入 Ground Truth 历史和第一版传感器。
第六步是接入 `TrackBundle`、CSV 导入和中心线生成器。
第七步是接入 RaceDirector、报告、MCAP 和 Foxglove layout。
第八步是完成整圈 E2E 验收。
第九步是在外部接口不变的前提下升级四轮双轨模型。

每一步都必须具备独立退出标准。
不能因为后续阶段计划存在而跳过当前阶段的测试和验证。

## 25. 完成定义

第一版完成必须同时满足以下条件。

- 一条命令可以在 Ubuntu 22.04 + Humble 上完成依赖安装和构建。
- 一条命令可以选择车辆和赛道启动仿真。
- 动态自行车、执行器、安全状态机和传感器全部通过测试。
- EUFS CSV 和中心线生成赛道都能进入相同 `TrackDefinition`。
- 无头模式和交互模式产生相同的物理与传感器序列。
- Foxglove 正确显示车辆、锥桶、轨迹、视场、观测和比赛状态。
- 完整自主栈能够在不订阅 Ground Truth 的情况下完成一圈。
- 相同 revision、配置和 seed 的结果可以重放。
- Jazzy CI 编译护栏通过。
- 所有 lint、测试和 E2E 检查无失败且无已知 flaky test。

## 26. 已确认的设计决定

本项目采用维护型 EUFS sim2 fork，而不是在上游参考目录直接开发。
车辆核心独立于 ROS 2 和 EUFS sim2。
第一阶段采用动态自行车，第二阶段升级四轮双轨。
第一版采用语义级 Camera 和 LiDAR 锥桶观测。
Foxglove 是第一版可视化前端。
赛道第一版支持文件导入和中心线生成，不提供图形编辑器。
正式运行平台是 Ubuntu 22.04 + ROS 2 Humble。
Jazzy 只作为第一版 CI 迁移护栏。
Isaac Sim、Gazebo 和其他渲染器不参与第一版车辆状态计算。
