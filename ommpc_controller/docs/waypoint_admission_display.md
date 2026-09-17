# 航点接收实时状态

规划器从里程计到达时开始持续检测，不需要先发送航点。终端默认每秒打印
`[航点接收: 允许/禁止]`，权限切换时立即打印（检测周期0.1s）。同时显示速度、
稳定累计时间、位置跨度、状态延迟和控制器拒绝原因。

`minisnap_3D.launch` 默认联动控制器。控制器每0.2s发布带绝对wall时间戳的权限心跳；
没有控制器、旧版本无心跳、心跳过期、控制器非HOVER、起飞未完成、状态失效、
非OFFBOARD/未解锁、安全锁定、已有READY/正在跟踪都会显示禁止接收。
规划器自己仍独立检查稳定悬停，不能以控制器权限替代里程计安全检查。
独立规划器测试可设置 `planning/require_controller_ready=false`；显示会注明独立模式。

保持满足条件，允许接收就持续有效，没有“必须几秒内发送第一条航点”的倒计时。
满1秒指的是连续稳定悬停，不是接收窗口。求解后控制器的READY轨迹有效期仍为2秒，
且执行仍需COMMAND；这里未延长READY有效期，也未改为收到航点就强制执行。
建议在稳定HOVER中先启用COMMAND，再等显示允许后发送航点；符合执行条件时FSM
将自动激活READY轨迹，避免求解后再手动切COMMAND赶2秒窗口。

航点回调会明确打印 `[航点已接收]` 或 `[航点未接收]`（未接收不会缓存）。
规划输出后打印 `[轨迹已发送]`；求解成功/已发送均不表示无人机已开始跟踪，
实际执行看控制器POLY_TRAJ状态转换。

参数：`planning/status_interval=1.0`、`planning/controller_status_max_age=1.0`、
`planning/controller_status_topic=/traj_tracking_controller/trajectory_admission_status`。
话题：`/trajectory_generator_node/waypoint_admission_status` 可用 `rostopic echo` 查看。
控制器心跳格式：`ALLOW|wall_epoch_seconds|reason` 或 `BLOCKED|wall_epoch_seconds|reason`；
心跳时间过期后，即使收到latched旧消息也不能放行。

仅新增状态显示及接收权限联动，不缓存不安全航点、不降低稳定门槛、不自动切飞控模式。
