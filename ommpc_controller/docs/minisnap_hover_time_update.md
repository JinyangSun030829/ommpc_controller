# 稳定悬停启动、执行校验与时间压缩

本次不加入避障、走廊、负载或飞行器动力学约束，不改变原航点顺序和消息格式。

## 启动约定

ROS 规划节点明确只支持 `dev_order=4` 的稳定悬停启动，起始位置取新鲜里程计，起终点 v/a/j 固定为零。
数学核心仍支持 order=3/4 和非零边界，供离线使用；这不表示 ROS 飞行流程允许运动中重规划。

`stable_hover.h` 供规划节点、控制器共用。默认连续 1 秒速度模长 ≤0.10 m/s、位置窗口包围盒对角线 ≤0.05 m。
该包围盒指标是任意两点距离的保守上界。状态样本间隔超过0.20秒、时间倒退、非有限数据、运动超限或控制器离开可启动 HOVER 都重置窗口。
重复读取同一时间戳不刷新样本，也不能积累稳定时间。真实加速度未直接测量；这是零导数悬停模型的适用性代理判据。

规划请求在未稳定时被拒绝，必须稳定后重新发送；并非自动排队等待。后台开始和发布前也检查稳定状态。
规划节点只依据里程计判断低运动状态；已起飞/HOVER/OFFBOARD/ARMED 的最终许可由控制器强制检查。

控制器仅在稳定且已起飞 HOVER/OFFBOARD/ARMED 时接收新轨迹。只接受8系数、有限零v/a/j起点。
READY 默认2秒过期，即使尚未开启 COMMAND 也会清除。激活前检查新鲜状态、稳定窗口、飞行许可及起点位置误差≤0.10m。
短暂不稳定时保持 HOVER；READY过期、空间失配时清除并要求重新规划。不开启任何自动解锁、起飞、轨迹热替换或重定位轨迹操作。
状态提供器不再将零时间戳伪装为当前时间；非有限状态不能激活轨迹。

## 参数

规划节点私有参数 `planning/hover/{duration,max_speed,max_position_span,max_state_age}` 默认 `1.0/0.10/0.05/0.20`。
控制器相同判据为 `trajectory/hover/{duration,max_speed,max_position_span,max_state_age}`。
控制器额外参数 `trajectory/max_ready_age=2.0`、`trajectory/max_start_position_error=0.10`、`trajectory/start_boundary_tolerance=1e-5`。
两端判据默认一致；若调参须同步。控制器原 `control/state_timeout` 更严格时仍然生效。
既有控制器 YAML 保留不动，新参数使用 C++ 默认值；可通过 YAML 或 rosparam 覆盖。

## 时间求解

1. 每段数值保护下限改为0.001秒，不再把0.1秒当作每个航点的停留要求。
2. 单段静止起停加入七次曲线V/A/J形状常数时间种子。
3. 多段静止路径使用整条弧长的对称七阶段jerk受限S形时间种子，再反求航点时刻。
   常态转弯采用离散曲率80%分位估计巡航速度，孤立尖角加入局部时间预算；≥150°反转保留旧保守种子。
   这些仅是初始化，不能视为实际多项式曲率或硬速度约束。
4. 零边界采用精确整体延时至可行，避免密集起停段局部延时引发邻段jerk峰值传播。
5. 默认再尝试兼容旧时间种子及旧可行性策略，两个完整验证的可行结果选较短者。
   `planning/compare_legacy_seed=false` 可关闭对照，减少额外求解；备选失败保留主可行解。
6. 依据整段保守上界尝试整体压缩；相同时间倍率α下V/A/J分别按α⁻¹/α⁻²/α⁻³变化。
7. 对具有余量的连续段组尝试5%局部压缩，失败减小步长。每次重新求解并检查所有段、边界、连续性和物理系数。
   只有完整通过的候选才提交，否则保留最后一个可行解。最后再完整检查一次。

`planning/compress_time=true`、`compression_max_trials=32`、`compression_budget_ms=50.0`、`compression_step=0.05`。
预算只限制压缩阶段，是软预算：一个已开始的候选和最终完整验证必须结束，初始可行性及备选种子比较不包含其中。
这不是全局最短时间证明，也不保证所有路径对点数完全不敏感。返回的V/A/J是浮点Bernstein保守界，不是形式化区间认证。

## 结果高亮

求解成功输出独立中文结果块，亮青色加粗，显示总时长、耗时、可行性轮数、备选种子、整体倍率、局部压缩计数及V/A/J上界。
初始化仍用普通日志，警告/错误保持原级别。原始多项式和绘图JSON不含ANSI字符。
`planning/result_color=auto|always|never`：直接执行默认auto；launch默认always以适应roslaunch管道。
重定向终端输出到文件时使用 `roslaunch ommpc_controller minisnap_3D.launch result_color:=never`，避免捕获彩色转义。

## 验证

独立编译及测试，不连接实际飛行ROS图：

```bash
cmake -S tests -B /tmp/minisnap_hover_test -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS_NODE=ON
cmake --build /tmp/minisnap_hover_test -j2
(cd /tmp/minisnap_hover_test && ctest --output-on-failure)
python3 tests/ros_isolated_integration.py /tmp/minisnap_hover_test/trajectory_generator_node /tmp/minisnap_hover_test/trajectory_manager_ros_test
python3 tests/test_minisnap_plot.py
```

测试包括冻结时间戳、样本中断、位置跳变、速度超限、许可失效、零导数起点、READY过期/起点失配清除、正常激活，
以及30/60/120/245点同圆的点密度对比、时间不增加、试验预算和V/A/J整段上界回归。

2026-09-16离线结果（初始位置取路径起点，v/a/j=0，限制V/A/J=3/2/4）：

| 同一半径1.5m单圈航点数 | 改进前总时长（秒） | 改进后总时长（秒） |
| --- | --- | --- |
| 30 | 17.986548 | 8.180964 |
| 60 | 24.154471 | 8.438814 |
| 120 | 30.620759 | 8.711582 |
| 245 | 33.163587 | 8.712519 |

不同点数的新总时长最大/最小比1.064975，回归测试限制不超过1.20。
原fly_cir setup=0的245段多圈几何：177.508970秒→约174.463423秒，最终V/A/J保守界约1.705251/1.959093/2.906404。
245段完整规划含两种种子比较和时间压缩此次约30–40ms；软耗时预算及系统负载可能使结果略有差异。
备份：功能包 `backups/minisnap_hover_before_20260916_181539.tar.gz`，并单独备份了旧规划节点、控制器及数学库。
未启动正式飞行节点；隔离ROS校验不能替代仿真和实飞验收。
