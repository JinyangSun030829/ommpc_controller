# Minisnap 重构说明（2026-09-16）

本文记录首次重构及其历史基准。后续稳定悬停启动、执行校验和时间压缩已实现，现阶段行为与参数以
[稳定悬停及时间优化说明](minisnap_hover_time_update.md) 为准；以下177.51秒等数值属于时间压缩前的结果。

## 架构判断

原始数学公式可以形成合法的固定时间、航点插值问题，但实现架构不适合长轨迹或在线状态更新：

- 一个1042行节点文件同时负责参数、状态、输入、时间分配、优化、显示和发布，全部依赖可变全局变量。
- 数学求解器把分段块结构展开成稠密矩阵，重复显式求逆，不能扩展到大量段。
- ROS回调直接执行全部规划，计算期间里程计也停止更新。
- 时间表基于零初速及固定30/40/30比例，而优化边界使用真实初速，两者不一致。
- 配置的速度、加速度没有成为轨迹验证条件，也没有jerk限制；min_order参数实际未使用。
- 可视化每个采样点按值复制整条系数矩阵，发布还必须等待显示计算完成。

重构后的最小职责划分：

```text
trajectory_generator_node.cpp  ROS输入、坐标约定、状态快照、最新任务队列、发布/显示
              |
trajectory_planner.cpp         航点清理、初始时间表、局部retiming、动力学与边界验证
              |
trajectory_generator.cpp       归一化Hermite映射、稀疏导数系统、三轴求解、轨迹求值/界
```

前两层数学模块不依赖ROS，组成minisnap_core库；离线测试无需ROS master。
节点层一个ROS回调线程、一个规划工作线程，队列只保留最新待处理请求。
新请求会取消旧规划，即使新请求无效，也不会随后意外发布旧结果。

## 数学求解器

- 支持order=3的五次minimum-jerk和order=4的七次minimum-snap。
- 每段采用tau=t/T归一化时间，局部Hermite映射通过6/8阶线性方程生成。
- 局部代价按T^(1-2r)换算，固定位置及起终点导数后，只装配内部自由导数系统。
- 相邻航点耦合产生块三对角稀疏结构；内存规模随段数线性增长。
- 对角均衡后进行一次SimplicialLDLT分解，一次求解三列右端项。
- 检查正定性、数值有限性及相对残差，不添加隐藏正则项改变原优化问题。
- 位置先减去起始位置，降低大坐标下固定位置项的抵消误差。
- 兼容保留SolveQPClosedForm接口和物理时间、降幂系数布局。

内部归一化系数用于Horner求值及导数检查；发布时转换为旧协议：

```text
[段数, 每轴系数数, 各段时间, 每段x降幂系数/y降幂系数/z降幂系数]
```

因此现有trajectory_manager和minisnap_plot无需改变解析格式。

## 时间规划与验证

1. 合并距离不超过duplicate_distance的连续近重复点；不删除非连续重复点，不改变fly_cir绕圈几何。
2. 起始速度使用实际三维速度，考虑第一段方向变化；终点速度/加速度为0。
3. 根据局部转角/间距估计曲率，限制初始标量速度表；前向/后向扫描处理加减速。
4. 用梯形/三角速度时间和jerk相关过渡时间初始化段时间，取消固定30/40/30规则及不匹配阶数的单段经验常数。
5. 优化后通过向量Bernstein控制点的凸包上界、自适应de Casteljau细分，检查整段速度/加速度/jerk范数。
6. 检查航点误差、显式连续性以及起终点导数误差，阈值1e-5。
7. 优先仅延长违反限制的段，再重新优化并重新检查所有段。
8. 零起始速度/加速度时，局部调整超过8轮后允许全局时间放大作为后备；非零起始导数不使用这一后备。
9. 超过迭代/总时间预算或仍不满足限制时，抛出明确错误，节点不发布。

特别说明：固定非零初速时，统一放大所有时间并不意味着速度必然下降。
测试中初速3m/s、路径0/2/4/6m的七次轨迹，用统一放大可能出现速度回弹；局部延长制动段后两轮通过，时长约3.39s。
这仍是固定时间优化外层的可行性启发式，不是带动力学不等式的时间最优求解器，也不保证所有可行路线都能收敛。

Bernstein验证是带浮点误差裕量的保守数值检查，不是严格区间算术形式证明。
独立密集采样测试用于交叉检查其上界不会漏掉峰值；发布不依赖这些采样测试。

## 状态和坐标约定

- Odometry.pose属于header.frame_id；按Odometry消息规范，twist属于child_frame_id。
- 默认odom/twist_frame=child，先用pose四元数将速度旋转到pose坐标系。
- 如果实际上游把twist发布在pose坐标系，可明确配置pose，不应重复旋转。
- planning/frame_id默认world。节点默认不接受其他pose/waypoint坐标系，也不自动执行TF变换。
- launch显式设置odom_frame_alias=map，保留原rviz_quad将MAVROS map数值当作world数值的约定。
  这是身份映射假设而不是TF转换；map/world若不重合，必须清空该alias并修正上游坐标转换，不能直接实飞。
- 节点检查里程计时间戳、接收新鲜度、四元数和数值有效性。
- 工作线程开始时取得最新完整状态；发布前再次检查计算耗时、位置和速度漂移。
- ROS入口起始加速度仍按0建模，snap模式边界jerk也为0；没有从噪声速度差分中伪造实际加速度。
  核心planner接口允许调用者提供有效的initial_acceleration。

## 参数

| 参数 | 默认值 | 意义 |
|---|---:|---|
| planning/dev_order | 4 | 3=jerk，4=snap，必须是整数；min_order已删除 |
| planning/vel | 3.0 | 三维速度范数上限，m/s |
| planning/acc | 2.0 | 三维加速度范数上限，m/s²，不包括重力项 |
| planning/max_jerk | 4.0 | 三维jerk范数上限，m/s³；初始调试值，非硬件认证值 |
| planning/seed_safety_ratio | 0.9 | 初始速度/jerk时间表裕量 |
| planning/limit_margin | 0.98 | 最终验证裕量；固定边界恰在限值时允许该边界值 |
| planning/min_segment_time | 0.1 | 段时间下界，不等价于动力学限值 |
| planning/duplicate_distance | 0.0001 | 连续近重复点合并距离，m |
| planning/max_segments | 2000 | 最大输入目标数/优化段数 |
| planning/max_iterations | 30 | retiming最大求解轮数 |
| planning/max_total_time | 3600 | 最大总时长，s |
| planning/bound_tolerance | 0.002 | 保守界细分停止的相对精度 |
| planning/bound_max_depth | 12 | 最大界细分深度；到达深度仍返回保守上界 |
| odom/max_age | 0.5 | 时间戳和接收状态允许年龄，s |
| planning/max_result_age | 0.5 | 计算加待发布最大耗时，s |
| planning/max_start_position_error | 0.15 | 发布前起点位置漂移上限，m |
| planning/max_start_velocity_error | 0.3 | 发布前起始速度漂移上限，m/s |
| vis/enabled | true | 可关闭显示，轨迹发布不依赖显示 |
| vis/sample_dt | 0.05 | 可视化基础采样周期，s |
| vis/max_points | 2000 | 轨迹显示点数上限，包含最终端点 |

launch保留现有vis_traj_width=0.05及其他RViz/waypoint节点设置。
可用dev_order:=3回到minimum-jerk，max_vel/max_acc/max_jerk可通过launch参数覆盖。

## 已验证结果

2026-09-16，远端Ubuntu、GCC9.4、Release/O3，结果会随负载和硬件变化。

| 项目 | 旧求解器 | 新求解器 |
|---|---:|---:|
| 同一245段、order=3、固定0.8s/段，仅求解 | 2554ms | 0.46ms |
| 上述独立进程峰值RSS | 128804KB | 4480KB |

旧优化版本此次能结束，并未复现永久死锁；实测证明的是主回调会被秒级阻塞，不能将所有“卡死”都断言为无限循环。

完整原fly_cir setup=0几何、起点(0,0,2)、order=4、限制3/2/4：

- 245段，2轮求解，包含时间分配及整段导数界检查约2.9ms。
- 总时长约177.51s。
- V/A/J上界约1.688/1.947/2.912。
- 稀疏自由系统非零项6570，而非1960×1960全局矩阵。

通过的离线测试：独立稠密KKT对照（两阶数、多组随机边界）、短单段/多段、非零初速/加速度、
初速恰为限速、反向/侧向起步、急转弯、连续重复点、悬停、极短段、混合段时间、坐标平移、
原245段多圈输入、245段密集一圈输入、每阶数50组随机移动路径、取消、NaN、无效参数和停车距离保护。

隔离ROS测试：私人loopback master、模拟里程计、话题格式及物理系数、速度坐标旋转、显示点数上限、
空/NaN/坐标不一致输入拒绝、停车距离不足拒绝、陈旧里程计拒绝、最新任务替换。
测试脚本不连接用户现有master，不发送解锁、起飞或控制命令。

## 构建与复测

从功能包目录执行独立测试：

```bash
cmake -S tests -B /tmp/minisnap_test_build -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS_NODE=ON
cmake --build /tmp/minisnap_test_build -j2
/tmp/minisnap_test_build/trajectory_generator_offline_test
python3 tests/ros_isolated_integration.py /tmp/minisnap_test_build/trajectory_generator_node
```

正式catkin工作区可按原build_OMMPC.sh构建，也可只构建所需目标：

```bash
cmake --build /home/yundrone/Sunray/build/MPC_basic --target trajectory_generator_node trajectory_generator_offline_test -- -j2
```

## 未扩展的范围与实飞前检查

- 保留fly_cir原几何和setup默认值；setup=0仍为45°间隔、约30.5圈，不是245点的一圈圆。
- 不自动删减普通航点，不实现滚动时域或改变航点顺序。稀疏求解已解决当前245段性能问题。
- 仍然只是航点插值，不含障碍物/走廊/绳索/推力/倾角/电机约束；多项式可能偏离航点连线或发生空间超调。
- “路径不足停车距离”是保守的拒绝策略，不能据此声称所有通过请求都有足够无障碍制动空间。
- 动力学限值检查针对参考轨迹，不保证真实飞机一定满足这些值。
- 当前控制器TrajectoryManager只在HOVER且无READY/ACTIVE轨迹时接收，激活时钟由FSM决定。
  节点异步并不代表支持飞行中的热替换；旧协议没有起始执行时间/状态确认，READY后长时间等待的失配仍须由控制器侧处理。
- 已完成离线/隔离测试不等于完成仿真或实飞验证。实飞前核对坐标身份映射、速度坐标、起始加速度模型、
  限值与飞行器能力，并在安全空间中先做仿真和低速短路线测试。

## 备份与恢复

部署前逐字节核对本轮读取的6个原文件，若远端被再次修改则停止部署；新文件若已存在也不覆盖。
原源码/launch/构建文件备份位于功能包backups/minisnap_before_时间戳.tar.gz，旧节点二进制也单独保留。
需要恢复时可在确认后解压该备份到功能包目录，并重新构建；新增数学/测试文件不必删除，旧CMake不会使用它们。
本轮不会自动恢复、删除用户文件或启动正式launch。
