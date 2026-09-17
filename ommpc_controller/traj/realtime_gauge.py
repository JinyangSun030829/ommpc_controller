#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from collections import deque
from threading import Lock

import rospy
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.collections import PolyCollection
import matplotlib.animation as animation

from nav_msgs.msg import Odometry

# 预定义配置
DEFAULT_UAV_NAME = "uav1"
DEFAULT_PARAMETERS = {
    "filter_alpha": 0.2,
    "max_speed": 15.0,
    "history_size": 50,
    "state_timeout": 1.0,
    # Standard MAVROS odom reports linear velocity in the child/body frame.
    # Set false only if the upstream odom already contains world-frame velocity.
    "velocity_in_body": True,
}
QUATERNION_EPSILON = 1e-6
COLORS = [
    (0.0, "navy"), (0.15, "dodgerblue"), (0.3, "limegreen"),
    (0.45, "yellowgreen"), (0.55, "gold"), (0.7, "orange"),
    (0.85, "red"), (1.0, "darkred")
]
ANGLES = np.linspace(-np.pi/6, np.pi + np.pi/6, 11)
THETA = np.linspace(-np.pi/6, np.pi + np.pi/6, 500)

def normalize_quaternion(x, y, z, w):
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite quaternion")
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm < QUATERNION_EPSILON:
        raise ValueError("invalid quaternion norm")
    return tuple(value / norm for value in values)


def rotate_velocity_to_world(velocity, quaternion):
    """Rotate a child/body-frame vector with the odom body-to-world quaternion."""
    x, y, z, w = normalize_quaternion(*quaternion)
    vx, vy, vz = velocity
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def quaternion_to_euler(x, y, z, w):
    """Return roll, pitch and yaw in radians without an additional TF dependency."""
    x, y, z, w = normalize_quaternion(x, y, z, w)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw

class RealTimeSpeedGauge:
    def __init__(self, uav_name=DEFAULT_UAV_NAME):
        self.odom_topic = rospy.get_param(
            "~odom_topic", f"/{uav_name}/mavros/local_position/odom"
        )
        self.alpha = float(rospy.get_param(
            "~filter_alpha", DEFAULT_PARAMETERS["filter_alpha"]
        ))
        self.max_speed = float(rospy.get_param(
            "~max_speed", DEFAULT_PARAMETERS["max_speed"]
        ))
        history_size = int(rospy.get_param(
            "~history_size", DEFAULT_PARAMETERS["history_size"]
        ))
        self.state_timeout = float(rospy.get_param(
            "~state_timeout", DEFAULT_PARAMETERS["state_timeout"]
        ))
        self.velocity_in_body = rospy.get_param(
            "~velocity_in_body", DEFAULT_PARAMETERS["velocity_in_body"]
        )
        if not math.isfinite(self.alpha) or not 0 < self.alpha <= 1:
            raise ValueError("~filter_alpha must be in (0, 1]")
        if not math.isfinite(self.max_speed) or self.max_speed <= 0:
            raise ValueError("~max_speed must be finite and positive")
        if history_size <= 0:
            raise ValueError("~history_size must be positive")
        if not math.isfinite(self.state_timeout) or self.state_timeout <= 0:
            raise ValueError("~state_timeout must be finite and positive")

        self.data_lock = Lock()
        self.last_valid_received_at = None
        self.current_speed = 0.0
        self.filtered_speed = 0.0
        self.speed_history = deque(maxlen=history_size)
        
        # 增加状态锁，用于迟滞判断避免UI闪烁
        self.is_dangerous = False 
        
        # 共享数据字典，用于给 UI 线程提供最新数据
        self.state_data = {
            'pos': [0.0, 0.0, 0.0],
            'vel': [0.0, 0.0, 0.0],
            'euler': [0.0, 0.0, 0.0] 
        }
        
        self._last_max_speed = -1.0
        self._last_cur_str = ""
        
        # 初始化颜色映射
        cdict = {'red': [], 'green': [], 'blue': []}
        for pos, color in COLORS:
            r, g, b = plt.cm.colors.to_rgb(color)
            cdict['red'].append((pos, r, r))
            cdict['green'].append((pos, g, g))
            cdict['blue'].append((pos, b, b))
        self.cmap = LinearSegmentedColormap('smooth_cmap', cdict, N=1024)
        self.base_colors = self.cmap(np.linspace(0, 1, len(THETA)-1))
        
        # 初始化界面
        self.init_ui_layout()
        
        # 订阅话题
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        rospy.loginfo(
            "[GAUGE] Waiting for %s; velocity_in_body=%s, displayed velocity=WORLD/ENU.",
            self.odom_topic, self.velocity_in_body,
        )

    def odom_callback(self, msg):
        try:
            position = msg.pose.pose.position
            velocity = msg.twist.twist.linear
            orientation = msg.pose.pose.orientation
            pos = (position.x, position.y, position.z)
            vel = (velocity.x, velocity.y, velocity.z)
            if not all(math.isfinite(value) for value in pos + vel):
                raise ValueError("non-finite position or velocity")
            quaternion = normalize_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            if self.velocity_in_body:
                vel = rotate_velocity_to_world(vel, quaternion)
            raw_speed = math.hypot(*vel)
            if not math.isfinite(raw_speed):
                raise ValueError("overflowing velocity magnitude")
            euler = tuple(math.degrees(angle)
                          for angle in quaternion_to_euler(*quaternion))
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "[GAUGE] Ignoring invalid odometry: %s", error)
            return

        # ROS callbacks never modify Matplotlib artists. The GUI reads one
        # coherent snapshot, including the history, under the same lock.
        with self.data_lock:
            self.filtered_speed = self.alpha * raw_speed + (1.0 - self.alpha) * self.filtered_speed
            self.current_speed = self.filtered_speed
            self.speed_history.append(self.current_speed)
            if self.current_speed > self.max_speed * 0.95:
                self.max_speed = self.current_speed * 1.2
            self.state_data = {'pos': pos, 'vel': vel, 'euler': euler}
            self.last_valid_received_at = time.monotonic()
        rospy.loginfo_once("[GAUGE] Valid MAVROS Odometry received; dashboard updating.")

    def init_ui_layout(self):
        self.fig = plt.figure(figsize=(10, 6), dpi=100)
        self.fig.patch.set_facecolor('#1e1e1e')

        # === 左侧：仪表盘 ===
        self.ax = self.fig.add_axes([0.02, 0.05, 0.55, 0.9], projection='polar')
        self.ax.set_theta_zero_location('W')
        self.ax.set_theta_direction(-1)
        self.ax.set_ylim(0, 1.2)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.spines['polar'].set_visible(False)
        self.ax.set_facecolor("none")

        for i in range(len(THETA)-1):
            self.ax.fill_between([THETA[i], THETA[i+1]], 1.08, 1.2, color="#333333")
        
        self.texts = {}
        for idx, angle in enumerate(ANGLES):
            self.ax.plot([angle, angle], [1.1, 1.2], color="white", lw=2)
            self.texts[f'tick_{idx}'] = self.ax.text(angle, 1.3, "", color="white", fontsize=11, ha='center', va='center')

        self.texts.update({
            'max_label': self.ax.text(-np.pi/2, 0.1, "Max Vel(m/s)", color="red", fontsize=14, ha='center', va='top', fontweight="bold"),
            'max_value': self.ax.text(-np.pi/2, 0.3, f"{self.max_speed:.1f}", color="red", fontsize=18, ha='center', va='top', fontweight="bold"),
            'cur_label': self.ax.text(np.pi/2, 0.3, "Cur Vel(m/s)", color="cyan", fontsize=14, ha='center', va='center', fontweight="bold"),
            'cur_value': self.ax.text(np.pi/2, 0.1, "0.0", color="cyan", fontsize=22, ha='center', va='center', fontweight="bold"),
            'status': self.ax.text(-np.pi/2, 0.6, "Waiting Data...", fontsize=14, ha='center', fontweight="bold", color="gray")
        })

        verts = [[(THETA[i], 0.8), (THETA[i], 1), (THETA[i+1], 1), (THETA[i+1], 0.8)] for i in range(len(THETA)-1)]
        self.poly = PolyCollection(verts, facecolors=self.base_colors)
        self.ax.add_collection(self.poly)
        self.pointer = self.ax.plot([0, 0], [0.8, 1.25], color=self.cmap(0), lw=3)[0]

        # === 右侧：数据面板 ===
        self.ax_data = self.fig.add_axes([0.6, 0.1, 0.35, 0.8])
        self.ax_data.axis('off')
        
        self.ax_data.set_xlim(0, 1)
        self.ax_data.set_ylim(0, 1)
        
        self.ax_data.text(0.05, 0.95, "UAV TELEMETRY", color="white", fontsize=16, fontweight="bold", ha="left")
        self.ax_data.plot([0.05, 0.95], [0.92, 0.92], color="cyan", lw=2) 
        
        # 文本初始化
        text_opts = {'color': 'white', 'fontsize': 14, 'family': 'monospace', 'va': 'top'}
        self.data_texts = {
            'pos': self.ax_data.text(0.05, 0.85, "POS:\nWaiting...", **text_opts),
            'vel': self.ax_data.text(0.05, 0.55, "VEL:\nWaiting...", **text_opts),
            'att': self.ax_data.text(0.05, 0.25, "ATT:\nWaiting...", **text_opts)
        }
        
        self.fig.canvas.manager.set_window_title('UAV Telemetry Dashboard')

    def update_frame(self, frame):
        if rospy.is_shutdown():
            plt.close(self.fig)
            return []
        with self.data_lock:
            speed = self.current_speed
            max_speed = self.max_speed
            history = tuple(self.speed_history)
            state = self.state_data.copy()
            last_received = self.last_valid_received_at
        fresh = (last_received is not None and
                 time.monotonic() - last_received <= self.state_timeout)
        # 1. 仪表盘更新
        norm_speed = max(0.0, min(speed / max_speed, 1.0))
        pointer_angle = -np.pi/6 + norm_speed * (np.pi + np.pi/3)
        
        colors = self.base_colors.copy()
        active_idx = int(norm_speed * (len(THETA)-1))
        colors[:active_idx, 3] = 0.8  
        colors[active_idx:, 3] = 0.0  
        self.poly.set_facecolors(colors)
        
        self.pointer.set_xdata([pointer_angle, pointer_angle])
        self.pointer.set_color(self.cmap(norm_speed))
        
        cur_str = f"{speed:.1f}"
        if cur_str != self._last_cur_str:
            self.texts['cur_value'].set_text(cur_str)
            self._last_cur_str = cur_str
            
        avg_speed = np.mean(history) if history else speed
        
        # ==========================================
        # 🚨 关键修复：加入状态迟滞 (Hysteresis) 防止闪烁
        # ==========================================
        if fresh and not self.is_dangerous:
            # 触发条件：速度突增超过平均值的 30%，并且绝对速度大于 1.5m/s (防噪点)
            if speed > avg_speed * 1.3 and speed > 1.5:
                self.is_dangerous = True
        elif fresh:
            # 解除条件：速度平稳回落到平均值的 105% 以下
            if speed < avg_speed * 1.05:
                self.is_dangerous = False

        # This is a speed-spike indicator, not a flight/trajectory safety verdict.
        if not fresh:
            status = "Waiting ODOM..." if last_received is None else "ODOM stale"
            status_color = "gray" if last_received is None else "orange"
        else:
            status = "Speed Spike" if self.is_dangerous else "Speed Stable"
            status_color = "red" if self.is_dangerous else "limegreen"
        self.texts['status'].set_text(status)
        self.texts['status'].set_color(status_color)
        # ==========================================
        
        if abs(max_speed - self._last_max_speed) > 0.01:
            self.texts['max_value'].set_text(f"{max_speed:.1f}")
            for idx in range(len(ANGLES)):
                self.texts[f'tick_{idx}'].set_text(f"{idx * max_speed / (len(ANGLES)-1):.1f}")
            self._last_max_speed = max_speed

        # 2. 实时刷新右侧遥测数据
        pos = state['pos']
        vel = state['vel']
        att = state['euler']
        
        self.data_texts['pos'].set_text(f"POSITION (m)\n X: {pos[0]:>6.2f}\n Y: {pos[1]:>6.2f}\n Z: {pos[2]:>6.2f}")
        self.data_texts['vel'].set_text(f"WORLD VEL (m/s)\n VX: {vel[0]:>6.2f}\n VY: {vel[1]:>6.2f}\n VZ: {vel[2]:>6.2f}")
        self.data_texts['att'].set_text(f"ATTITUDE (deg)\n R: {att[0]:>6.1f}°\n P: {att[1]:>6.1f}°\n Y: {att[2]:>6.1f}°")

        return [self.poly, self.pointer] + list(self.texts.values()) + list(self.data_texts.values())

    def start(self):
        self.ani = animation.FuncAnimation(
            self.fig, self.update_frame, interval=30, blit=False, cache_frame_data=False
        )
        plt.show()

if __name__ == "__main__":
    rospy.init_node('uav_telemetry_dashboard', anonymous=True)
    uav_name_param = rospy.get_param('~uav_name', DEFAULT_UAV_NAME)
    
    try:
        gauge = RealTimeSpeedGauge(uav_name=uav_name_param)
        gauge.start() 
    except rospy.ROSInterruptException:
        pass
    except (TypeError, ValueError) as error:
        rospy.logfatal("[GAUGE] Invalid configuration: %s", error)
