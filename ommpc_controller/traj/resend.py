#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

# 导入目标话题的消息类型 (请确保环境中有 traj_utils 功能包并已 source)
from traj_utils.msg import PolyTraj

# 全局变量定义
traj_pub = None
current_traj_id = 1

def trajectory_callback(msg):
    global current_traj_id
    if len(msg.data) < 2:
        rospy.logerr("收到的轨迹消息为空或格式错误！")
        return

    # 1. 解析基本维度信息
    num_segments = int(msg.data[0])
    poly_coeff_num = int(msg.data[1])

    # 验证数据长度是否合法
    expected_size = 2 + num_segments + (num_segments * 3 * poly_coeff_num)
    if len(msg.data) != expected_size:
        rospy.logerr(f"轨迹长度不匹配！期望: {expected_size} 实际: {len(msg.data)}")
        return

    # 2. 解析每一段的时间
    segment_times = np.array(msg.data[2 : 2 + num_segments])
    total_time = np.sum(segment_times)

    # 3. 解析多项式系数矩阵
    coeff_start_idx = 2 + num_segments
    coeffs = np.array(msg.data[coeff_start_idx:]).reshape((num_segments, 3 * poly_coeff_num))

    # 4. 转换为 PolyTraj 格式
    poly_msg = PolyTraj()
    poly_msg.drone_id = 0  # 对应 /drone_0_planning
    poly_msg.traj_id = current_traj_id
    current_traj_id += 1
    poly_msg.start_time = rospy.Time.now()
    
    # 阶数 = 系数个数 - 1 (例如 8 个系数对应 7 阶多项式)
    poly_msg.order = poly_coeff_num - 1
    
    # 提取 X, Y, Z 的系数并展平为一维列表
    # 根据原代码逻辑，每段的系数按照 X, Y, Z 顺序排列
    coef_x = coeffs[:, 0 : poly_coeff_num].flatten().tolist()
    coef_y = coeffs[:, poly_coeff_num : 2 * poly_coeff_num].flatten().tolist()
    coef_z = coeffs[:, 2 * poly_coeff_num : 3 * poly_coeff_num].flatten().tolist()
    
    poly_msg.coef_x = coef_x
    poly_msg.coef_y = coef_y
    poly_msg.coef_z = coef_z
    poly_msg.duration = segment_times.tolist()

    # 5. 发布轨迹消息给控制器
    if traj_pub is not None:
        traj_pub.publish(poly_msg)
        rospy.loginfo(f"已成功将轨迹转发至 /drone_0_planning/trajectory, Traj ID: {poly_msg.traj_id}, 总时长: {total_time:.2f}s")


def main():
    global traj_pub
    rospy.init_node('trajectory_forwarder_node', anonymous=True)
    
    # 订阅原有的 Float64MultiArray 话题
    rospy.Subscriber('/planning/poly_trajectory', Float64MultiArray, trajectory_callback)
    
    # 定义 PolyTraj 控制器所需的发布者
    traj_pub = rospy.Publisher('/drone_0_planning/trajectory', PolyTraj, queue_size=10)
    
    rospy.loginfo("轨迹转发节点已启动 (纯数据转发模式)，等待接收轨迹...")
    rospy.spin()

if __name__ == '__main__':
    main()