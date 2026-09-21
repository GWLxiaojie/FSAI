# Ubuntu 22.04 / Humble 修复验收

本轮基于 `b38e86d69b20a94f329870052068fa1d4f32bef6` 完成软件修复，工作分支为 `fix/humble-simulator`。
实现及验收结合 Astra 的物理、ROS 和标定工具审查。
源码与构建结果位于 `/home/muhan/ymh_data/FSAI/ads-dv-simulator`。
GitHub发布目标为 `GWLxiaojie/FSAI` 的 `fix/humble-simulator` 分支，EUFS依赖保持只读。

## 已验证结果

| 检查 | 实测结果 |
| --- | --- |
| 平台 | Ubuntu 22.04.5、GCC 11.4、ROS 2 Humble |
| 完整运行依赖闭包构建 | 13个包成功，含未经修改的EUFS依赖和新增pybind11_conversions |
| 全工作区测试 | `tools/test.sh` 成功；colcon汇总437项，0 errors / 0 failures / 0 skipped |
| 无ROS核心回归 | 8个测试程序、40个GTest用例通过 |
| 工具测试 | CSV标定评估8项、命令工具8项，共16项通过 |
| 脚本和依赖 | 入口合同、CI合同、依赖锁与只读保护测试通过；shell语法及git diff检查通过 |
| ROS组合测试 | OFF/READY/DRIVING、watchdog、EBS、时间戳/序号拒绝、reset及种子恢复通过 |
| 模式一致性 | 同一输入/seed，两种模式的4000个已接受物理状态样本和400帧相机消息逐项相等 |
| 实际物理launch | 实时6秒测试确认5秒READY后车辆前进，初始位姿沿夹道方向；完整20秒场景正常退出 |
| 原EUFS兼容launch | 20步、100ms仿真成功，原插件完成初始化 |
| 非法启动参数 | `run_mode:=invalid` 导致节点及ros2 launch非零退出 |
| 跨进程DDS控制 | 独立CLI发送30Nm，实测车速达到0.445049m/s；结束撤扭矩并制动；EBS锁存；reset回OFF/零状态；约7.8秒墙钟完成 |
| MCAP | 官方storage插件实际写入、正常结束并读回ROS消息；验证READY阶段静止和驱动阶段速度增长 |
| RViz | X11/xcb兼容模式创建OpenGL4.5窗口，加载车辆描述，随场景正常结束；未把自动截图当作视觉验收 |

437是colcon聚合CTest/GTest等结果后的报告口径，并不等于437个独立物理工况。
模式一致性测试通过真实ROS节点和消息回调推进相同输入，隔离每次试验的订阅并等待最终消息收取；不把DDS发现之前的构造时刻t=0消息当作已接受积分步。
墙钟节奏另外通过实际实时launch与加速launch验证。

## 审查问题处理

- 环境加载兼容ROS生成脚本，修正colcon全局参数位置；运行脚本加载本工作区。
- 修正无效package manifest、EUFS类型别名、力状态访问及namespaced库链接，补齐导出依赖。
- 物理核心不再更新时间戳接收状态，真实watchdog可触发；命令时间戳、序号、非法值不再被忽略。
- FSAI安全链独立管理OFF/READY/DRIVING/EBS/reset，EUFS兼容入口保留原插件，避免命令类型冲突。
- 赛道、场景、接口profile实际加载；补齐初始位姿、时限、seed、内部脚本和执行器反馈。
- 轮速输出统一为EUFS原有rev/s语义；新增传感器及实际轴力映射。
- 修复静止再生倒车、制动保持蠕动、低速速度定义、事件时刻、非有限参数输入。
- 下压力参与轴荷/附着，前轴纵力按转角投影，左右轮速来自轮心速度和滚动几何。
- 再生与摩擦制动共用按轴附着包络；执行器和底盘共用1ms时间网格。
- backend revision升为2，旧动力学snapshot被明确拒绝。
- 新增ADS-DV参数来源、CSV误差门限工具、严格时间戳控制工具、RViz入口和独立MCAP录制脚本。

## 上游兼容修复

完整测试额外发现锁定的map_lib向量版AlignSVD使用无初值的std::reduce。
其默认归约种子是未初始化的Eigen::Vector2d，原始测试稳定产生错误平移。
项目通过 `cmake/eufs_compatibility.cmake` 对整个工作区统一启用官方支持的 `EIGEN_INITIALIZE_MATRICES_BY_ZERO`。
原始上游断言随后通过，没有修改依赖源码、放宽断言或跳过失败测试。
车队库也导出同一编译定义，下游编译须保持一致。
所有锁定依赖工作树在验收后仍干净。

## 数值与实车准确性

真实内部步长减半测试采用解析数值压力工况 `du/dt=-u²`、u(0)=30、T=0.05s。
1ms、500µs、250µs误差分别约为5.05e-8、3.16e-9、1.97e-10，按四阶收敛。
生产默认仍为1ms；此试验证明积分算法行为，不是实车气动参数校准。

ADS-DV官方文档确认轴距1.53m、前后轮距1.20m和后驱布局。
质量300kg、偏航惯量172.44kg·m²、有效滚动半径0.2525m及轮胎/气动/扭矩映射仍是EUFS参考值，保留 `reference_unvalidated`。
没有收到足以完成动态标定的实测数据，因此未宣称整车物理精度已验收。
标定工具和参数来源见 [calibration.md](calibration.md)。

本阶段仍是动态自行车，未交付完整四轮转动/滑移、载荷转移、真实电机电池模型、实车级传感器延迟外参、赛事裁判或无人驾驶闭环跑圈。
这些能力与当前已修复的程序错误分开管理，不能靠单元测试通过来替代实测验证。

## 本机依赖与验证边界

本机安装系统包需要交互sudo，验收采用项目 `.dependencies/root` 中解包的官方ROS geodesy、MCAP vendor/storage二进制。
下载内容的SHA256与官方软件包索引一致；该目录被git忽略，不混入源码或上游checkout。
入口脚本自动识别这一可选本地prefix，当前目录可直接启动。
新机器正常执行bootstrap/rosdep安装；录制额外需要 `ros-humble-rosbag2-storage-mcap`。
本机未在空白VM执行sudo bootstrap；发布后通过GitHub Actions的全新Ubuntu镜像验证安装链。
远端检查额外修复了ROS软件源符号链接识别、rosdep不支持的旧选项，以及Eigen的rosdep键。
Jazzy工作流已在GitHub完成真实安装、核心编译和测试，但不代表完整Jazzy运行支持。
最新Humble运行状态以[该分支的GitHub Actions](https://github.com/GWLxiaojie/FSAI/actions?query=branch%3Afix%2Fhumble-simulator)为准。
Qt offscreen不支持此RViz/GLX窗口路径；使用图形会话的xcb模式启动成功。

## 复现

```bash
cd /home/muhan/ymh_data/FSAI/ads-dv-simulator
tools/build.sh
tools/test.sh
python3 -B -m unittest discover -s tools/tests -p 'test_*.py' -v
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B tools/tests/ros_control_smoke.py
tools/run_sim.sh --scenario straight_acceleration --run-mode realtime --visualize
```

完整ROS接口与范围见 [runtime.md](runtime.md)，外部驾驶流程见 [drive-command.md](drive-command.md)。
