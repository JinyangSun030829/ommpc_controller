# 连续多项式执行与参考点显示

本次修复：自然轨迹结束进入 BRAKE 时不再清除 COMMAND 会话。一次 COMMAND
启用后，轨迹结束 -> 有界停止 -> 持续稳定悬停 -> HOVER；之后新接收的 READY
轨迹经状态、起点、零导数检查后自动启动。显式 HOVER、LAND 仍关闭 COMMAND，
LAND/BRAKE-for-LAND 禁止抢回控制；失效状态/模式安全锁仍保留。

仍不支持运动途中排队/无悬停拼接：等规划器显示 `[航点接收: 允许]` 再发送
下一组航点。READY 的 2 秒有效期是起点新鲜度保护，未延长；COMMAND 未启用
或状态条件在求解期间失效时仍可能过期，必须重新发送。

实际当前控制参考点发布为 geometry_msgs/PointStamped，与 ommpc_example.cpp
统一使用绝对话题 `/uav1/reference_point`，不随节点名称/命名空间变化。
在新鲜控制循环中以控制频率发布
（默认 100 Hz），覆盖 HOVER/TAKEOFF/POLY_TRAJ/BRAKE/LAND；状态失效且停止
正常控制时不伪造新点。有启用 bag 时同步写入该话题。

RViz 添加 PointStamped 显示，选择上述话题，Fixed Frame 设为 `world`。
两套控制器应切换运行，不应同时控制同一架无人机；同时发布此话题也会使
RViz 混合显示两套参考点。录制 bag 使用发布器实际话题名，兼容 ROS remap。
私有参数 `visualization/reference_frame` 默认 `world`，仅声明参考点坐标系，
不进行坐标变换；设成其他名称之前需确认参考点本就属于该坐标系或提供正确 TF。

规划器航点接收滚动状态/常规进度与 `[CTRL]` 行使用
sunray_logger.h 的 LOG_CYAN `ESC[36m`；求解成功块保留普通 LOG_GREEN
`ESC[32m`，与滚动状态区分。
无粗体、下划线、闪烁控制码。使用独立常量以免引入公共 Logger 的全局静态
实现及 Logger::info 的强制加粗/下划线。WARN/ERROR 保持原警告等级和样式。
规划器 `planning/result_color`、控制器 `control/status_color` 可取
`always/auto/never`；控制器默认 always，auto 仅终端且未设置 NO_COLOR 时着色。

回归覆盖：两条轨迹只调用一次 COMMAND、结束保留会话、复用已经满足的停止
悬停窗口、显式 HOVER 关闭会话、LAND 优先级/连续停止、PointStamped 内容、
普通 cyan 输出、READY 过期与拒绝非悬停起点。测试使用独立回环 ROS master，
不调用真实 ARM/DISARM，不发送姿态控制指令。不是实机飞行验证。
