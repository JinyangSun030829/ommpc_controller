# minisnap_3D.launch 参数说明

本文对应远程电脑当前的配置（核对日期：2026-09-17），不是旧备份中的默认值。

文件位置：`/home/yundrone/Sunray/External_Module/MPC_basic/src/ommpc_controller/ommpc_controller/launch/minisnap_3D.launch`。

说明依据：`src/trajectory_generator_node.cpp`、`src/trajectory_planner.cpp`、`include/stable_hover.h`，以及当前 ROS 环境中 `waypoint_generator` 的源码。本文只解释配置，不修改 launch 或控制逻辑。

## 1. 这个 launch 做什么

加载无人机 URDF，启动机器人模型发布、航点输入、minimum-snap 轨迹规划、无人机实时可视化和 RViz。**它本身不启动飞行控制器，也不负责解锁、切换 OFFBOARD、起飞或降落。**

主要数据流：

```text
/goal → waypoint_generator → /waypoint_generator/waypoints
                                      ↓
                   trajectory_generator_node
                  （里程计 + 稳定悬停 + 接收权限）
                                      ↓
                         /planning/poly_trajectory
                                      ↓
                          控制器状态机决定执行
```

规划器会在求解开始时读取最新有效位置，把它作为轨迹起点；当前 ROS 规划接口以稳定悬停为前提，起点和终点的速度、加速度、jerk 均设为零。中间航点不要求逐点停车。求解成功、轨迹已发布，不等于控制器已经开始执行。

## 2. 可从 roslaunch 命令传入的 arg

`arg` 是 launch 的输入变量，下面对应的 `<param>` 才是节点实际读取的 ROS 参数。

| arg 名称 | 当前默认值 | 作用 |
| --- | --- | --- |
| `dev_order` | `4` | 优化目标的导数阶数。4 表示最小化 snap（位置四阶导数）的平方积分，生成七次多项式。**当前 ROS 节点只接受 4**，不能因为底层求解器支持其他阶数就直接改成 3。 |
| `max_vel` | `5.2` | 传给 `planning/vel`，参考轨迹速度上限，单位 m/s。不是保证无人机实际速度一定不超过此值。 |
| `max_acc` | `4.0` | 传给 `planning/acc`，参考轨迹加速度上限，单位 m/s²。不是归一化推力、总拉力或包含重力的比力上限。 |
| `max_jerk` | `4.0` | 传给 `planning/max_jerk`，参考轨迹加速度变化率上限，单位 m/s³。该值是调试配置，不是已经认证的硬件极限。 |
| `vis_enabled` | `true` | 是否发布规划轨迹和航点的可视化 Marker；关闭不影响多项式轨迹发布，也不会关闭整个 RViz 或 `rviz_quad`。 |
| `result_color` | `always` | 规划结果和接收提示的终端颜色：`always` 总是启用；`auto` 根据终端及 `NO_COLOR` 环境变量判断；`never` 禁用。不改变轨迹数值。 |
| `planning_frame` | `world` | 传给 `planning/frame_id`，规划数据所使用的坐标系名称。只改名字不会进行坐标转换。 |
| `odom_frame_alias` | `map` | 额外允许里程计的 `header.frame_id` 为 `map`，并把其数值视为与规划坐标系一致。**这是相同坐标系的别名许可，不会查询 TF 或变换坐标。** |
| `window_sec` | `20.0` | 原实时 DOB 绘图窗口长度，单位 s。当前绘图节点被注释，因此这个 arg 虽然存在，但没有实际消费者，不生效。 |

例如覆盖速度上限：

```bash
roslaunch ommpc_controller minisnap_3D.launch max_vel:=3.0 max_acc:=2.0
```

## 3. 轨迹优化与时间分配参数

这些参数属于 `/trajectory_generator_node` 私有命名空间。例如 `planning/vel` 的完整路径是 `/trajectory_generator_node/planning/vel`。

| 参数 | 当前值 | 中文说明 |
| --- | --- | --- |
| `planning/vel` | `$(arg max_vel)`，默认 `5.2` | 三维速度向量的模长上限，不是 xyz 每个分量各自的上限。规划器对整段多项式进行保守上界检查；超限时延长轨迹时间并重新求解。 |
| `planning/acc` | `$(arg max_acc)`，默认 `4.0` | 三维加速度向量模长上限。影响初始时间分配，以及最终可行性检查和时间调整。 |
| `planning/dev_order` | `$(arg dev_order)`，默认 `4` | minimum-snap 优化阶数；当前 ROS 节点要求为 4。相邻多项式段的位置、速度、加速度和 jerk 连续。 |
| `planning/max_jerk` | `$(arg max_jerk)`，默认 `4.0` | 三维 jerk 向量模长上限，控制加速度变化的快慢；通常越小，起停和转弯越平缓，轨迹可能越长。 |
| `planning/min_segment_time` | `0.001` | 单段轨迹时间的数值下限，单位 s。防止极短段造成求解数值问题，**不是每个航点的悬停时间，也不是控制周期**。 |
| `planning/compress_time` | `true` | 得到满足约束的可行轨迹后，尝试整体或局部缩短段时间。每次候选都会重新求解并检查，只有通过检查才替换已有可行解；关闭后仍进行必要的超限延时处理。 |
| `planning/compression_budget_ms` | `50.0` | 时间压缩阶段的计算预算，单位 ms。是软预算：单次候选求解/检查可能超过预算，程序不会为了赶时间跳过验证。**不是整个规划过程的总耗时上限。** |
| `planning/compression_max_trials` | `32` | 压缩候选的最大尝试次数；与计算预算共同限制压缩搜索。达到限制后保留最后通过检查的可行轨迹。 |

补充：launch 没有显式设置 `planning/limit_margin`，当前节点默认使用 `0.98`。因此零导数起止条件下，规划器通常按配置 V/A/J 上限的 98% 进行验收，为数值和约束留出余量。这不是 `flight_box.yaml` 的空间安全余量。

这些参数只约束规划出的参考轨迹；实际跟踪误差、推力/倾角限制和区域保护仍由控制器及其他配置负责。当前规划器没有因为这些参数就自动获得避障或飞行区域约束。

## 4. 稳定悬停与控制器接收权限

| 参数 | 当前值 | 中文说明 |
| --- | --- | --- |
| `planning/hover/duration` | `1.0` | 开始允许接收航点前，持续满足稳定悬停条件的时间，单位 s。程序同时检查消息时间和本机接收时间的覆盖长度；重复读取旧样本不能累计成稳定悬停。不是轨迹结束后的停留时间。 |
| `planning/hover/max_speed` | `0.10` | 稳定悬停允许的最大三维速度模长，单位 m/s。超过时清空悬停判断窗口，重新累计。 |
| `planning/hover/max_position_span` | `0.05` | 悬停判断窗口内的位置活动范围上限，单位 m。代码使用三维位置包围盒对角线长度作为保守跨度，不是“相对某个目标点的距离误差”。 |
| `planning/hover/max_state_age` | `0.20` | 悬停判断允许的状态延迟和相邻样本间隔，单位 s。样本过期、间隔过大或时钟倒退会使判断失效；它比普通里程计有效性检查更严格。 |
| `planning/require_controller_ready` | `true` | 是否要求控制器同时明确允许接收轨迹。开启时，规划器自身悬停就绪还不够，还必须收到新鲜的控制器 `ALLOW` 状态；关闭为独立规划模式，不保证控制器可执行。 |
| `planning/controller_status_max_age` | `1.0` | 控制器权限状态允许的最大年龄，单位 s，按状态文本中携带的 wall time 判断。状态无效、过期或发布者不存在时，开启权限检查的规划器会拒绝航点。 |
| `planning/controller_status_topic` | `/traj_tracking_controller/trajectory_admission_status` | 控制器轨迹接收权限话题，类型 `std_msgs/String`。当前解析格式为 `ALLOW\|时间\|原因` 或 `BLOCKED\|时间\|原因`。必须配置到真正提供该协议的控制器。 |
| `planning/status_interval` | `1.0` | 正常接收状态提示的刷新间隔，单位 s。允许/禁止状态改变时会提前打印；不改变里程计频率、求解频率或控制频率。 |

注意：当前这个权限话题由 `traj_tracking_controller_fsm.cpp` 发布。核对的 `ommpc_example.cpp` 没有发布同名权限状态，所以**只启动 OMMPC 时，当前 `require_controller_ready=true` 配置会因缺少权限状态而阻止规划器接收**。仅把话题名改成一个不存在的 OMMPC 话题不能解决问题；应另行决定接入权限协议，或明确使用独立规划模式。本文不修改该配置。

航点被拒绝时不会缓存等待就绪；需要在就绪后重新发送。规划期间/发布前也会复核接收条件，因此请求已接收仍不代表结果一定会发布。

## 5. 坐标系、里程计和结果有效性

| 参数 | 当前值 | 中文说明 |
| --- | --- | --- |
| `planning/frame_id` | `$(arg planning_frame)`，默认 `world` | 规划坐标系。输入航点 Path 的 `header.frame_id` 必须匹配；单个航点非空的 frame_id 也必须匹配。也是规划可视化 Marker 的 frame_id。程序不做隐式 TF 转换。 |
| `planning/odom_frame_alias` | `$(arg odom_frame_alias)`，默认 `map` | 接受里程计 frame_id 的额外别名。仅当 `map` 与 `world` 的原点、方向和单位确实一致时才可这样配置；若不一致，应取消别名并在外部完成坐标转换。 |
| `odom/twist_frame` | `child` | 里程计线速度所属坐标系。`child`：按机体/子坐标系速度处理，用里程计姿态旋转到 pose/header 坐标系；`pose`：认为速度已经在 pose/header 坐标系，直接使用。只支持这两个字符串。 |
| `odom/max_age` | `0.5` | 普通里程计有效性的最大年龄，单位 s，同时检查 ROS 消息时间和本机 wall time 接收延迟。用于拒绝旧状态；悬停门限还有更严格的 `hover/max_state_age=0.20`。 |
| `planning/max_result_age` | `0.5` | 从工作线程开始本次规划到准备发布结果的最大耗时，单位 s。超过后丢弃结果，防止发送基于旧起点的轨迹；不是规划轨迹的飞行总时长上限。 |
| `planning/max_start_position_error` | `0.15` | 发布结果时，当前位置与本次求解起点位置之间允许的最大三维距离，单位 m。偏离超过此值就丢弃结果，需要重新发送航点。不是控制器执行时的跟踪误差上限。 |
| `planning/max_start_velocity_error` | `0.3` | 发布结果时，当前速度与本次求解起点实际速度快照之间允许的最大差值模长，单位 m/s。不是速度上限，也不是把规划起点速度设成测量速度。 |
| `planning/result_color` | `$(arg result_color)`，默认 `always` | 终端提示和求解摘要的颜色模式，仅影响显示。可选 `auto`、`always`、`never`。 |

launch 未设置 `odom/topic`，当前节点默认订阅 `/uav1/mavros/local_position/odom`。这里 `odom/twist_frame=child` 针对的是 **odom 消息的 twist**，不要直接套用到之前讨论的 `velocity_local` 话题。

## 6. 规划结果可视化

| 参数 | 当前值 | 中文说明 |
| --- | --- | --- |
| `vis/enabled` | `$(arg vis_enabled)`，默认 `true` | 是否发布规划轨迹和航点 Marker。多项式控制消息会先发布，可视化不是飞行控制输入。 |
| `vis/sample_dt` | `0.05` | 绘制参考轨迹时的采样时间间隔，单位 s。越小线条通常越细密；不是控制器执行轨迹的采样周期。 |
| `vis/max_points` | `2000` | 规划轨迹线的最大绘制点数。长轨迹时自动增大采样间隔，实际使用 `max(sample_dt, 总时长/(max_points-1))`，并补上终点。 |
| `vis/vis_traj_width` | `0.05` | Marker 的尺寸设置，单位 m；用于轨迹/航点连线线宽和航点球的尺寸，不影响轨迹优化。 |

规划可视化话题为 `/trajectory_generator_node/vis_trajectory` 和 `/trajectory_generator_node/vis_waypoint_path`。当前参考轨迹线颜色在源码中固定为深紫色，不由 `result_color` 控制。

## 7. 其他参数、话题映射和启动属性

| 配置项 | 当前设置 | 中文说明 |
| --- | --- | --- |
| 全局 `robot_description` | `config/drone.urdf` 的文件内容 | 加载无人机模型供机器人模型显示和 `robot_state_publisher` 使用；不是飞行控制器的质量、惯量或增益配置。 |
| `~waypoints` remap | `/waypoint_generator/waypoints` | 把规划器的私有航点输入连接到航点生成节点，消息类型为 `nav_msgs/Path`。 |
| `~goal` remap | `/goal` | 航点生成节点接收人工目标的输入话题，消息类型为 `geometry_msgs/PoseStamped`。 |
| `/waypoint_generator/waypoint_type` | `manual` | 人工航点输入模式：目标 z>0 时追加航点；-1<z≤0 时删除最后一个航点；z≤-1 时发送已累计的航点并清空。这个负高度是输入操作指令，不是要求无人机飞到负高度。 |
| 规划节点 `required` | `true` | 该节点退出时，roslaunch 会停止本次 launch 启动的其他节点。它不是“必须有新轨迹才能飞行”的控制许可。 |
| 节点 `output` | `screen` | 将节点输出显示到终端，不改变计算和控制逻辑。 |
| RViz 的 `args` | `-d .../config/test_rviz.rviz` | 加载指定 RViz 布局文件。当前实际命令**没有 `-f world`**，Fixed Frame 由布局文件或界面设置决定；不能把旧注释当成已执行的参数。 |

`robot_state_publisher` 和 `rviz_quad.py` 在这个 launch 中没有额外配置私有参数。`rviz_quad` 按脚本自身默认值显示实际无人机轨迹并发布 TF；避免让其他节点同时发布同一对父子坐标系 TF。

## 8. 当前被注释的实时绘图参数

以下 `<node>` 整体位于 XML 注释中，当前不会启动，也不会设置其中参数。

| 项目 | 注释中的值 | 原用途 |
| --- | --- | --- |
| `plot_rate` arg | `20.0` | 实时绘图界面刷新频率，单位 Hz；原注释说明其不影响话题记录频率。当前 arg 自身也被注释。 |
| `dob_realtime_plot/window_sec` | `$(arg window_sec)`，默认 `20.0` | 实时绘图窗口显示的时间长度，单位 s。 |
| `dob_realtime_plot/plot_rate` | `$(arg plot_rate)`，默认 `20.0` | 传给实时绘图脚本的界面刷新频率。 |
| `dob_realtime_plot/max_points` | `5000` | 原实时绘图脚本的点数限制；不是上面的规划可视化 `vis/max_points`。 |
| 绘图节点 `required` | `false` | 即使启用绘图节点，它退出也不会因这一属性导致整个 launch 退出。 |

若以后启用这段配置，还需核对 `dob_realtime_plot.py` 当时的实现和可用性；上述参数目前都不参与轨迹规划或无人机控制。

## 9. 修改时需要注意

- 规划器参数在节点构造时读取，**运行中修改 rosparam 不会自动让这些参数重新加载**；与刚新增的运行时 LAND 许可开关不同。需要修改配置后重启规划器，或在启动命令中覆盖对应 arg。
- V/A/J 上限不等于飞控的推力、倾角或区域边界限制，必须与控制器实际能力匹配。
- 坐标系别名只在数值坐标确实一致时使用，不能用改 frame_id 代替坐标变换。
- 稳定悬停判断是低速、小位置跨度的代理条件，不包含实际加速度/jerk 测量验证。
- 规划结果被丢弃或航点被拒绝时，先排查权限、悬停窗口、里程计新鲜度和规划耗时，再重新发送航点。
