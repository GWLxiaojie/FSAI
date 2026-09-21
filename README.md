# FSAI ADS-DV Simulator

车队维护的 Formula Student AI 模拟器，正式运行平台为 **Ubuntu 22.04 + ROS 2 Humble**。
EUFS sim2 是固定提交的只读依赖；`fsai_sim_core` 独占车辆状态推进，Gazebo/Isaac Sim 不参与积分。
默认车辆为官方 ADS-DV 几何基准，质量、轮胎、惯量及执行器动态仍保留未实车标定标识。
参数出处和验证流程见 [ADS-DV 标定说明](docs/calibration.md)。
基础修复记录见 [验收报告](docs/validation-2026-09-21.md)；官方车辆外观、赛道导入和10圈自检见 [跑圈说明与验收](docs/official-ten-laps.md)。

## 安装与测试

```bash
git clone https://github.com/GWLxiaojie/FSAI.git
cd FSAI
tools/bootstrap_ubuntu.sh
tools/build.sh
tools/test.sh
python3 -B -m unittest discover -s tools/tests -p 'test_*.py' -v
```

Bootstrap 需要 sudo 安装系统依赖，导入锁定的源码，并禁止向 EUFS sim2 push。
Humble修复、官方车辆外观与10圈自检已整合到 `main`，默认克隆即可使用，无需切换功能分支。
已有依赖目录若脏或提交不匹配会报错，不会覆盖你的改动。
构建只包含 `fsai_bringup` 的依赖闭包；锁定的 Open-Car-Dynamics 是后续后端参考，不是当前运行核心。
脚本会加载 Humble 和本工作区环境，默认每包构建并发为2，避免头文件较多的依赖耗尽内存。
可用 `FSAI_BUILD_JOBS=4 tools/build.sh` 调整。
`tools/test.sh` 在仅限本机的独立 DDS domain 171 中运行，避免触发正在运行的其他模拟器服务。
需要时设置 `FSAI_TEST_DOMAIN_ID`。
跨进程命令行验收可在加载工作区后运行 `python3 -B tools/tests/ros_control_smoke.py`。
该测试使用本机独立DDS domain 174，自动启动并清理自己的模拟器进程。

构建对整个工作区统一启用 `EIGEN_INITIALIZE_MATRICES_BY_ZERO`，修复锁定的上游 map_lib 在 std::reduce 中使用未初始化 Eigen 向量种子的问题。
该策略没有修改上游源码或跳过原始测试，定义也通过车队库的CMake接口传给下游。
在外部工作区直接编译这些EUFS头文件时应保持同一初始化策略。
`pybind11_conversions` 是 map_lib 的必要源码依赖，已经加入锁文件。
许可证出处及未声明项见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 一键官方赛道低速跑10圈

完成依赖安装后，在项目目录执行：

```bash
./tools/run_official_laps.sh
```

脚本自动下载并校验 IMechE FS-AI 官方 HiL 的 Sprint/LTS 赛道和 ADS-DV 原始外观模型，增量构建后打开 RViz，以2.5 m/s低速跑10圈并停车，约49分钟仿真时间。
车辆使用原始 GLB 车身、四个轮胎及内嵌贴图，不使用方盒替代；当前动力学保持不变。
这是官方仓库提供的 Sprint 布局，**不是已经核实年份的某届赛事实测地图**。路线由官方道路网格提取，转换精度与假设见跑圈说明。

```bash
./tools/run_official_laps.sh --fast    # 无窗口、加速执行完整10圈验证
./tools/run_official_laps.sh --no-gui  # 无窗口、实时运行
```

每次在 `artifacts/` 生成独立JSON报告，只有完成10圈、保持赛道余量并停车才通过。
默认仅限本机 DDS domain 183；录制/观察时需设置相同 `ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1`。
关闭窗口或按 Ctrl-C 是中止运行，不是完成验收。官方数据只保留在本地被忽略的 `.dependencies/official/`，不会上传到本仓库。

跑圈使用车辆真值和已知中心线的参考控制器，用于模拟器回归和演示；**不代表依靠相机/LiDAR感知的完整无人驾驶系统，也不验证实车动力学精度**。

## 自动场景

```bash
tools/run_sim.sh --vehicle ads_dv --track skidpad_small --scenario straight_acceleration
tools/run_sim.sh --vehicle ads_dv --track skidpad_small --scenario brake_to_stop
```

这两个场景按仿真时间准备5秒，执行已声明的驱动/制动命令，并在20秒仿真时间结束。
默认 `as_fast_as_possible` 适用于内部预定命令；相同代码、配置、seed和时间网格产生相同轨迹。
实时演示及RViz：

```bash
tools/run_sim.sh --vehicle ads_dv --track skidpad_small \
  --scenario straight_acceleration --run-mode realtime --visualize
```

`skidpad_small` 当前是用于启动和锥桶观测测试的短直线夹道，不代表赛事标准八字绕环几何。
场景名、车辆名和赛道名会实际解析对应 YAML/CSV，缺失或非法配置导致启动失败。
`--max-steps N` 可限制步数。
详细参数可在加载 `install/setup.bash` 后通过 `ros2 launch fsai_bringup simulator.launch.py --show-args` 查看。

## 外部控制

```bash
tools/run_sim.sh --vehicle ads_dv --track skidpad_small --scenario manual
```

`manual` 使用实时模式，启动时为 OFF，时限120秒。
另一终端执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /set_mission eufs_msgs/srv/SetMission '{mission: 5}'
# READY 后等待至少5秒仿真时间，再执行 /go。
ros2 service call /go std_srvs/srv/Trigger '{}'
python3 tools/drive_command.py --torque 30 --steering 0 --duration 2
```

物理命令采用道路轮角(rad)、前后轴车轮侧总扭矩(N m)、摩擦制动请求([0,1])。
消息必须携带 `/clock` 时间戳与严格递增的 sequence；过期、未来、乱序或非法命令被拒绝。
默认100ms仿真时间收不到新命令便撤销驱动并施加超时制动。
`/ebs` 在任何任务状态锁存急停，`/reset` 恢复原始位姿、OFF、时间、命令序号状态和传感器随机种子。
自动场景与外部命令互斥。
工具参数和完整流程见 [控制命令说明](docs/drive-command.md)。

## 模型与接口

默认动态自行车、Pacejka侧向力、按轴摩擦圆、低速无滑动约束及零速制动保持。
执行器和底盘按1ms内部步推进，外层默认5ms，连续部分使用RK4。
下压力按静态轴荷比例分配，四轮速度由各轮位置和滚动几何推导。
再生和摩擦制动共用轮胎附着预算，仍不代表实车VCU的已标定制动分配策略。

| 输出 | 类型/语义 |
| --- | --- |
| `/clock` | 仿真时间 |
| `/fsai/actuator_state` | 实际执行器量与车体速度 |
| `/sim/state/as_state` | OFF / READY / DRIVING / EMERGENCY_BRAKE / FINISHED |
| `/ground_truth/odom` | nav_msgs/Odometry，map → base_footprint |
| `/sensors/imu` | 理想平面 IMU，含重力；默认位于车体参考点 |
| `/sensors/gnss` | 合成局部切平面GNSS，以纬度0/经度0为原点 |
| `/sensors/wheel_speeds` | EUFS WheelSpeedsStamped，轮速rev/s，转角rad |
| `/sensors/camera/cones`、`/sensors/lidar/cones` | 带seed噪声的语义锥桶观测，不是图像/点云 |
| `/ground_truth/track_markers`、`/tf` | 可视化锥桶和车辆位姿 |
| `/joint_states` | 官方外观的前轮转向和四轮滚动关节 |
| `/sim/reference/status`、`/sim/reference/markers` | 参考自检模式的圈数、误差和状态 |

完整接口、采样频率、配置与兼容边界见 [运行时说明](docs/runtime.md)。
控制算法应使用传感器接口，不使用 `/ground_truth/*` 作为实车可用测量。
当前模型不含逐轮转动惯量/滑移、动态载荷转移、ABS或真实电池/电机功率曲线。
传感器不是实车级延迟、外参和误差模型；其适用范围与参数需要独立验证。
通过程序测试不代表实车精度已达标，也不构成完整无人驾驶跑圈验收。

## EUFS兼容入口

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fsai_bringup upstream_compatibility.launch.py
```

此入口保留原始EUFS核心与插件。
正式FSAI核心使用独立物理安全链，不加载会覆盖物理命令的EUFS控制/状态机插件。
若必须使用EUFS加速度命令，在FSAI车辆 `interfaces.yaml` 中显式开启兼容订阅；物理安全状态和watchdog仍生效。
其加速度至扭矩换算是兼容近似，不能冒充实车VCU。

## 记录与回放

安装 `ros-humble-rosbag2-storage-mcap` 后，可在另一个同DDS domain的终端先启动录制，再启动实时场景：

```bash
tools/record_sim.sh artifacts/run_01
# Ctrl-C 正常结束录制后：
source install/setup.bash
ros2 bag info artifacts/run_01
```

录制使用仿真时钟，包含控制、传感器及真值主题，已有输出目录会被拒绝以避免覆盖。
高速内部场景可能在DDS发现完成前已经结束，录制时应使用实时模式。
诊断回放前停止模拟器和控制器，避免录制的命令重新进入活动控制链。
只重放传感器与真值供可视化时，可通过 `ros2 bag play --topics ... --clock` 显式选择主题。
包回放是ROS消息重放，不等同于完整物理状态snapshot恢复。

## 无ROS核心验证

```bash
cmake -S simulator/src/fsai_sim_core -B build/standalone \
  -DFSAI_SIM_CORE_STANDALONE=ON
cmake --build build/standalone -j2
ctest --test-dir build/standalone --output-on-failure
```

需要C++20编译器、CMake、Eigen3、OpenSSL和GTest。
macOS只支持这部分库测试；不能将跳过ROS的结果称为整机运行通过。

## 目录

- `simulator/src/fsai_sim_core`：无ROS车辆核心和物理回归。
- `simulator/src/fsai_sim2_adapter`：组合节点、命令安全链、状态输出、配置加载。
- `simulator/src/fsai_bringup`：launch、ADS-DV参数、赛道、场景和RViz配置。
- `simulator/src/fsai_interfaces`：物理命令/执行器消息。
- `tools/calibration`：实测与模拟CSV对齐、误差门限与JSON报告。
- `simulator/fsai_sim.repos`、`simulator/dependencies.lock.yaml`：只读依赖锁。
