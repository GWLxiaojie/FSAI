# 用仿真时钟发送物理命令

`tools/drive_command.py` 发布 `fsai_interfaces/msg/ActuationCommand`，从 `/clock` 获取
消息时间戳，进程内 `sequence` 严格递增。它不访问 CAN/实车 API，也不自动选择任务或进入 DRIVING。
请在模拟器使用的 ROS domain 中运行，确保所选 topic 对应模拟器。

完成 Humble 工作区构建后，从仓库根目录运行：

```bash
tools/run_sim.sh --vehicle ads_dv --track skidpad_small --scenario manual
```

另一个终端加载消息定义并选择任务：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /set_mission eufs_msgs/srv/SetMission '{mission: 5}'
```

任务进入 READY 后至少等待 **5 秒仿真时间**，然后执行 `/go`，确认服务返回成功。
`mission: 5` 是 manual；其他任务枚举见 `eufs_msgs/srv/SetMission`。

```bash
ros2 service call /go std_srvs/srv/Trigger '{}'
python3 tools/drive_command.py --torque 30 --brake 0 --steering 0 \
  --duration 2 --rate 50 --topic /fsai/actuation_command
```

`manual` 场景允许外部命令。`straight_acceleration`、`brake_to_stop` 自动场景拥有内部命令源，
不能同时用本工具控制。默认 OFF/READY 状态不会接受驱动命令；本工具不读取命令接受反馈，
发布成功不代表执行器执行成功，应查看模拟器日志和执行器反馈。

参数含义：`--torque` 为后轴 N m，负数请求再生制动；前轴始终为零。
`--steering` 为等效道路轮弧度，左转为正，输入必须在 `(-π/2, π/2)`；
`--brake` 为 `[0,1]` 摩擦制动请求。实际转角、扭矩与速率限制由车辆 profile 决定。
所有数字必须有限，时长/超时必须为正，发布频率在 `(0,1000]` Hz。

`--duration` 按仿真时间计时，`--rate` 按墙钟 Hz 调度，默认 50 Hz。
消息 stamp 始终是最近收到的 `/clock`，不会把墙钟时间填入 ROS 仿真时间。
默认 `--clock-timeout 5` 在 5 秒墙钟内未收到时钟或时钟不再推进时失败退出。
时钟回退（例如 reset）立即停止正常命令，结束进程；重置后重新执行任务准备和本工具。
不应在高速加速仿真中假定固定 50 Hz 墙钟发送能够满足 100 ms 仿真 watchdog；
交互控制使用 realtime 模式，并确保发送间隔和 DDS 延迟低于当前 profile 的命令 timeout。

sequence 从本机 Unix 纳秒时间初始化，此后每次发布加一，时钟回退不会重置序号。
这不是多发布者仲裁机制：同一模拟器只运行一个命令发布者；跨主机时间差、系统时钟大幅调整或
已有更大的序号可能使后续进程命令被拒绝，使用 `/reset` 建立新的模拟器会话后重新准备任务。

正常结束、Ctrl-C、时钟失败时尽力发送一次零扭矩、零转角、满摩擦制动命令，并短暂处理 DDS。
该消息没有可靠的执行确认，ROS 已关闭或 reset 后可能无法接收，最终由模拟器 watchdog 撤销旧命令。
watchdog 使用仿真时间；暂停仿真期间不会凭墙钟计时触发。需要锁存 EBS 时显式调用：

```bash
ros2 service call /ebs std_srvs/srv/Trigger '{}'
```

`/reset` (`std_srvs/srv/Trigger`) 恢复 OFF 并清空命令时序，需要重新选择任务并进入 DRIVING。
退出码 0 为时长完成/正常 ROS 关闭，130 为捕获 Ctrl-C，2 为参数、依赖、时钟或运行错误。

边界自检不依赖 ROS 安装：

```bash
python3 -B -m unittest discover -s tools/tests -p test_drive_command.py -v
```

测试注入时钟和发布器，验证真实调度入口的仿真时长、递增序号、超时、回退、结束制动与参数拒绝。
ROS 消息生成、DDS 传输和模拟器接受状态还需整机运行验收。
