#!/usr/bin/env python
import rospy
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def send_3d_path():
    rospy.init_node('send_3d_path_node')
    # 直接发布给你的 C++ 节点（跳过 Rviz 鼠标点击）
    pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=10)
    rospy.sleep(1.0)

    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    points = []
    
    # --- 参数设定 ---
    radius = 1.5       # 半径 1.5 米
    z_start = 2

    setup=0
    if setup==1:
        # points.append((-6, 0, 2))
        points.append((-5, 0, 2))
        points.append((-4,0, 2))
        points.append((-3,0,2))
        points.append((-2, 0,2))
        points.append((-1, 0,2))
        points.append((0, 0,2))
        points.append((1, 0,2))
        points.append((2, 0,2))
        points.append((3, 0,2))
        points.append((4, 0,2))
        points.append((5, 0,2))
        # points.append((6, 0,2))
    elif setup==-1:
        # points.append((5, 0, 2))
        points.append((4,0, 2))
        points.append((3,0,2))
        points.append((2, 0,2))
        points.append((1, 0,2))
        points.append((0, 0,2))
        points.append((-1, 0,2))
        points.append((-2, 0,2))
        points.append((-3, 0,2))
        points.append((-4, 0,2))
        points.append((-5, 0,2))
        points.append((-6, 0,2))
    elif setup==0:
        for i in range(1, 35):
            theta = (i /16.0) * 4.0 * math.pi
            x = radius * math.sin(theta)
            y = radius * math.cos(theta)
            z = z_start
            points.append((x, y, z))
        points.append((0,0,2))

    else:
        pass
    # points.append((0, 0,2))

    for pt in points:
        p = PoseStamped()
        p.pose.position.x = pt[0]
        p.pose.position.y = pt[1]
        p.pose.position.z = pt[2] # Z 大于 0
        path.poses.append(p)

    pub.publish(path)
    rospy.loginfo("成功发送 3D 航点！")

if __name__ == '__main__':
    send_3d_path()