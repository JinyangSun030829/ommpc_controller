#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OMMPC ROS bag analysis

Topics:
  /uav1/mavros/setpoint_raw/attitude      mavros_msgs/AttitudeTarget
  /ommpc_controller/reference_position    geometry_msgs/PointStamped
  /uav1/mavros/vision_pose/pose           geometry_msgs/PoseStamped
  /uav1/mavros/local_position/odom        nav_msgs/Odometry
  /uav1/mavros/imu/data                   sensor_msgs/Imu
  /uav1/mavros/state                      mavros_msgs/State
  /drone_0_planning/trajectory            traj_utils/PolyTraj

Outputs:
  1. Reference and vision-pose 3-D position trajectory
  2. Reference and vision-pose X/Y/Z positions
  3. Reference velocity (differentiated from reference position) and actual velocity
  4. Actual attitude converted from quaternion to Euler angles
  5. Actual angular velocity and commanded body rates
  6. Thrust and body-rate control commands
  7. IMU linear acceleration
  8. MAVROS connection/arming/mode state
  9. Position, velocity and angular-rate MAE/RMSE
  10. Three timestamped figures saved under 3D/, ODOM/ and CMD/
  11. One metrics CSV containing only Position, Velocity and Angular-rate MAE/RMSE
"""

import csv
import datetime
import math
import os
import sys
import warnings
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag


# ==============================================================================
# Configuration
# ==============================================================================

BAG_PATH = "/home/yundrone/Downloads/2026-09-09-19-05-52.bag"
OUTPUT_DIR = "~/uav_analysis_results"

TOPICS = {
    "attitude_cmd": "/uav1/mavros/setpoint_raw/attitude",
    "reference_position": "/ommpc_controller/reference_position",
    "vision_pose": "/uav1/mavros/vision_pose/pose",
    "odom": "/uav1/mavros/local_position/odom",
    "imu": "/uav1/mavros/imu/data",
    "state": "/uav1/mavros/state",
    "trajectory": "/drone_0_planning/trajectory",
}

# Analysis interval relative to the first odometry sample.
START_TIME =40
END_TIME = 100
# In the supplied controller simulation code vel_in_body_ is true. Therefore,
# set this to True if odom.twist.twist.linear is body-frame velocity and must be
# rotated into the world/ENU frame. Set it to False if the odometry velocity is
# already expressed in the world frame.
ODOM_LINEAR_VELOCITY_IN_BODY = True

# Actual angular velocity source. "imu" is recommended; if IMU data is absent,
# the code automatically falls back to odom.twist.twist.angular.
ANGULAR_RATE_SOURCE = "imu"

# A positive delay compares actual rate at t against command at t-delay.
# Leave at 0.0 unless you have identified the command-to-response latency.
RATE_COMMAND_DELAY_S = 0.0

# Differentiate a lightly smoothed reference position to reduce derivative noise.
# Use 1 to disable smoothing. Prefer an odd integer.
REF_POSITION_SMOOTH_WINDOW = 5

# False makes every metric use exactly the same START_TIME/END_TIME interval as
# the plots. Samples lacking a synchronized reference or measurement are still
# excluded automatically. Set True only when an additional armed+OFFBOARD
# restriction is explicitly desired.
METRICS_ONLY_OFFBOARD_ARMED = False

# Euler angle display.
EULER_IN_DEGREES = True
UNWRAP_EULER_ANGLES = True

SAVE_ALIGNED_CSV = False
SHOW_FIGURES = True

warnings.filterwarnings("ignore", message=".*Glyph.*missing from current font.*")
plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial", "Helvetica"]
plt.rcParams["axes.unicode_minus"] = False


# ==============================================================================
# Utility functions
# ==============================================================================

def message_time(msg, bag_time):
    """Prefer a non-zero ROS header timestamp; otherwise use bag record time."""
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        stamp = msg.header.stamp
        if stamp.to_sec() > 0.0:
            return stamp.to_sec()
    return bag_time.to_sec()


def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    """Quaternion [w, x, y, z] to a 3x3 rotation matrix."""
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1.0e-12:
        return np.eye(3)

    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz),
         2.0 * (qx * qy - qz * qw),
         2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw),
         1.0 - 2.0 * (qx * qx + qz * qz),
         2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw),
         2.0 * (qy * qz + qx * qw),
         1.0 - 2.0 * (qx * qx + qy * qy)],
    ])


def quaternion_to_euler(qw, qx, qy, qz):
    """Quaternion [w, x, y, z] to roll, pitch, yaw in radians."""
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1.0e-12:
        return 0.0, 0.0, 0.0

    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def sort_and_deduplicate(t, values):
    """
    Sort by timestamp and retain the last value for duplicate timestamps.
    values must be shaped [N] or [N, D].
    """
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)

    if t.size == 0:
        return t, values

    order = np.argsort(t, kind="stable")
    t = t[order]
    values = values[order]

    # Reverse before unique so the last occurrence in the original sorted data wins.
    _, reverse_indices = np.unique(t[::-1], return_index=True)
    keep = t.size - 1 - reverse_indices
    keep.sort()

    return t[keep], values[keep]


def interpolate_vector(t_src, values_src, t_dst, width=None):
    """Linearly interpolate scalar/vector data. Outside overlap becomes NaN."""
    t_src, values_src = sort_and_deduplicate(t_src, values_src)
    t_dst = np.asarray(t_dst, dtype=float)

    if t_src.size == 0:
        if width is None:
            width = 1 if np.asarray(values_src).ndim <= 1 else np.asarray(values_src).shape[1]
        out = np.full((t_dst.size, width), np.nan)
        return out[:, 0] if width == 1 else out

    values_src = np.asarray(values_src, dtype=float)
    scalar = values_src.ndim == 1
    if scalar:
        values_src = values_src[:, None]

    out = np.full((t_dst.size, values_src.shape[1]), np.nan)
    for axis in range(values_src.shape[1]):
        out[:, axis] = np.interp(
            t_dst,
            t_src,
            values_src[:, axis],
            left=np.nan,
            right=np.nan,
        )

    return out[:, 0] if scalar else out


def sample_previous(t_src, values_src, t_dst, default_value):
    """Zero-order hold: use the most recent source value at every target time."""
    t_src = np.asarray(t_src, dtype=float)
    values_src = np.asarray(values_src)

    if t_src.size == 0:
        return np.full(t_dst.shape, default_value, dtype=values_src.dtype if values_src.size else type(default_value))

    order = np.argsort(t_src, kind="stable")
    t_src = t_src[order]
    values_src = values_src[order]

    indices = np.searchsorted(t_src, t_dst, side="right") - 1
    valid = indices >= 0
    out = np.full(t_dst.shape, default_value, dtype=values_src.dtype)
    out[valid] = values_src[indices[valid]]
    return out


def moving_average_valid(data, window):
    """Moving average for a 1-D finite array with edge padding."""
    data = np.asarray(data, dtype=float)
    if window <= 1 or data.size < 3:
        return data.copy()

    window = int(window)
    if window % 2 == 0:
        window += 1
    window = min(window, data.size if data.size % 2 == 1 else data.size - 1)
    if window <= 1:
        return data.copy()

    left = window // 2
    right = window - 1 - left
    padded = np.pad(data, (left, right), mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(padded, kernel, mode="valid")


def differentiate_reference_position(t, ref_position, smooth_window):
    """
    Differentiate synchronized reference position. NaNs outside the valid overlap
    are preserved.
    """
    t = np.asarray(t, dtype=float)
    ref_position = np.asarray(ref_position, dtype=float)
    velocity = np.full_like(ref_position, np.nan)

    valid = np.all(np.isfinite(ref_position), axis=1) & np.isfinite(t)
    valid_indices = np.flatnonzero(valid)
    if valid_indices.size < 3:
        return velocity

    # Split into contiguous valid blocks.
    split_points = np.where(np.diff(valid_indices) > 1)[0] + 1
    blocks = np.split(valid_indices, split_points)

    for block in blocks:
        if block.size < 3:
            continue
        t_block = t[block]
        if np.any(np.diff(t_block) <= 0.0):
            continue

        for axis in range(3):
            p = moving_average_valid(ref_position[block, axis], smooth_window)
            velocity[block, axis] = np.gradient(p, t_block, edge_order=2)

    return velocity


def finite_rows(*arrays):
    mask = None
    for array in arrays:
        arr = np.asarray(array)
        current = np.all(np.isfinite(arr), axis=1) if arr.ndim > 1 else np.isfinite(arr)
        mask = current if mask is None else (mask & current)
    return mask


def compute_vector_metrics(reference, actual, mask):
    """Return per-axis metrics and a 3-D vector error summary."""
    reference = np.asarray(reference, dtype=float)
    actual = np.asarray(actual, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    mask &= finite_rows(reference, actual)

    if np.count_nonzero(mask) == 0:
        return {
            "count": 0,
            "axis_mae": np.full(3, np.nan),
            "axis_rmse": np.full(3, np.nan),
            "vector_mae": np.nan,
            "vector_rmse": np.nan,
        }

    error = reference[mask] - actual[mask]
    error_norm = np.linalg.norm(error, axis=1)

    return {
        "count": int(error.shape[0]),
        "axis_mae": np.mean(np.abs(error), axis=0),
        "axis_rmse": np.sqrt(np.mean(np.square(error), axis=0)),
        "vector_mae": float(np.mean(error_norm)),
        "vector_rmse": float(np.sqrt(np.mean(np.sum(np.square(error), axis=1)))),
    }


def add_trajectory_markers(axes, intervals, time_origin):
    """Mark planner trajectory start times on time-series figures."""
    for interval in intervals:
        start_rel = interval["start"] - time_origin
        for axis in np.atleast_1d(axes):
            axis.axvline(start_rel, linestyle=":", linewidth=0.9, alpha=0.45)


def save_metrics_csv(path, metric_groups):
    """
    Save exactly the tracking metrics printed in the terminal.

    CSV columns:
      Group, Axis, Samples, MAE, RMSE, Unit
    """
    axis_names = ["X", "Y", "Z"]

    with open(path, "w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "Group",
            "Axis",
            "Samples",
            "MAE",
            "RMSE",
            "Unit",
        ])

        for group_name, group in metric_groups.items():
            metrics = group["metrics"]
            unit = group["unit"]

            for axis_index, axis_name in enumerate(axis_names):
                writer.writerow([
                    group_name,
                    axis_name,
                    metrics["count"],
                    f"{metrics['axis_mae'][axis_index]:.6f}",
                    f"{metrics['axis_rmse'][axis_index]:.6f}",
                    unit,
                ])

            writer.writerow([
                group_name,
                "3D vector",
                metrics["count"],
                f"{metrics['vector_mae']:.6f}",
                f"{metrics['vector_rmse']:.6f}",
                unit,
            ])


# ==============================================================================
# Bag parsing
# ==============================================================================

def parse_bag(bag_path):
    raw = {
        "ref": {"t": [], "position": []},
        "vision": {"t": [], "position": []},
        "odom": {
            "t": [],
            "position": [],
            "velocity_raw": [],
            "velocity_world": [],
            "quaternion": [],
            "euler": [],
            "angular_rate": [],
        },
        "imu": {
            "t": [],
            "angular_rate": [],
            "linear_acceleration": [],
            "quaternion": [],
        },
        "cmd": {
            "t": [],
            "body_rate": [],
            "thrust": [],
        },
        "state": {
            "t": [],
            "connected": [],
            "armed": [],
            "mode": [],
        },
        "trajectory": [],
    }

    topic_list = list(TOPICS.values())
    topic_counts = {topic: 0 for topic in topic_list}

    print(f"Reading bag: {bag_path}")
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=topic_list):
            topic_counts[topic] += 1
            t_sec = message_time(msg, bag_time)

            if topic == TOPICS["reference_position"]:
                raw["ref"]["t"].append(t_sec)
                raw["ref"]["position"].append([
                    msg.point.x,
                    msg.point.y,
                    msg.point.z,
                ])

            elif topic == TOPICS["vision_pose"]:
                raw["vision"]["t"].append(t_sec)
                raw["vision"]["position"].append([
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ])

            elif topic == TOPICS["odom"]:
                p = np.array([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ], dtype=float)

                q = np.array([
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                ], dtype=float)

                v_raw = np.array([
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ], dtype=float)

                if ODOM_LINEAR_VELOCITY_IN_BODY:
                    rotation = quaternion_to_rotation_matrix(*q)
                    v_world = rotation @ v_raw
                else:
                    v_world = v_raw.copy()

                odom_rate = np.array([
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z,
                ], dtype=float)

                euler = np.array(quaternion_to_euler(*q), dtype=float)

                raw["odom"]["t"].append(t_sec)
                raw["odom"]["position"].append(p)
                raw["odom"]["velocity_raw"].append(v_raw)
                raw["odom"]["velocity_world"].append(v_world)
                raw["odom"]["quaternion"].append(q)
                raw["odom"]["euler"].append(euler)
                raw["odom"]["angular_rate"].append(odom_rate)

            elif topic == TOPICS["imu"]:
                raw["imu"]["t"].append(t_sec)
                raw["imu"]["angular_rate"].append([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ])
                raw["imu"]["linear_acceleration"].append([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ])
                raw["imu"]["quaternion"].append([
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ])

            elif topic == TOPICS["attitude_cmd"]:
                raw["cmd"]["t"].append(t_sec)
                raw["cmd"]["body_rate"].append([
                    msg.body_rate.x,
                    msg.body_rate.y,
                    msg.body_rate.z,
                ])
                raw["cmd"]["thrust"].append(msg.thrust)

            elif topic == TOPICS["state"]:
                raw["state"]["t"].append(t_sec)
                raw["state"]["connected"].append(bool(msg.connected))
                raw["state"]["armed"].append(bool(msg.armed))
                raw["state"]["mode"].append(str(msg.mode))

            elif topic == TOPICS["trajectory"]:
                start = msg.start_time.to_sec() if hasattr(msg, "start_time") else t_sec
                durations = list(msg.duration) if hasattr(msg, "duration") else []
                total_duration = float(np.sum(durations)) if durations else 0.0
                raw["trajectory"].append({
                    "message_time": t_sec,
                    "start": start,
                    "end": start + total_duration,
                    "traj_id": int(msg.traj_id) if hasattr(msg, "traj_id") else -1,
                    "order": int(msg.order) if hasattr(msg, "order") else -1,
                    "piece_count": len(durations),
                    "duration": total_duration,
                })

    print("\nTopic message counts:")
    for topic, count in topic_counts.items():
        print(f"  {topic:<48} {count}")

    if len(raw["odom"]["t"]) == 0:
        raise RuntimeError(f"No odometry data found on {TOPICS['odom']}")
    if len(raw["vision"]["t"]) == 0:
        raise RuntimeError(f"No vision-pose data found on {TOPICS['vision_pose']}")
    if len(raw["ref"]["t"]) == 0:
        raise RuntimeError(f"No reference-position data found on {TOPICS['reference_position']}")

    return raw


# ==============================================================================
# Time alignment and metrics
# ==============================================================================

def prepare_aligned_data(raw):
    t_odom, odom_position = sort_and_deduplicate(
        raw["odom"]["t"], raw["odom"]["position"]
    )
    _, odom_velocity_world = sort_and_deduplicate(
        raw["odom"]["t"], raw["odom"]["velocity_world"]
    )
    _, odom_velocity_raw = sort_and_deduplicate(
        raw["odom"]["t"], raw["odom"]["velocity_raw"]
    )
    _, odom_euler = sort_and_deduplicate(
        raw["odom"]["t"], raw["odom"]["euler"]
    )
    _, odom_rate = sort_and_deduplicate(
        raw["odom"]["t"], raw["odom"]["angular_rate"]
    )

    # sort_and_deduplicate must select identical timestamps for all odometry fields.
    # To be robust against malformed bags, interpolate non-position fields explicitly.
    odom_velocity_world = interpolate_vector(
        raw["odom"]["t"], raw["odom"]["velocity_world"], t_odom
    )
    odom_velocity_raw = interpolate_vector(
        raw["odom"]["t"], raw["odom"]["velocity_raw"], t_odom
    )
    odom_euler = interpolate_vector(
        raw["odom"]["t"], raw["odom"]["euler"], t_odom
    )
    odom_rate = interpolate_vector(
        raw["odom"]["t"], raw["odom"]["angular_rate"], t_odom
    )
    # Vision pose is the ground-truth position source. It is interpolated onto
    # the odometry time grid so position, velocity, attitude and command plots
    # share one set of timestamps and one analysis interval.
    vision_position = interpolate_vector(
        raw["vision"]["t"], raw["vision"]["position"], t_odom, width=3
    )

    t_origin = t_odom[0]
    t_relative = t_odom - t_origin

    interval_mask = t_relative >= START_TIME
    if END_TIME is not None:
        interval_mask &= t_relative <= END_TIME

    if np.count_nonzero(interval_mask) < 3:
        raise RuntimeError("Selected START_TIME/END_TIME interval contains fewer than 3 odometry samples.")

    t_odom = t_odom[interval_mask]
    t_relative = t_relative[interval_mask]
    odom_position = odom_position[interval_mask]
    vision_position = vision_position[interval_mask]
    odom_velocity_world = odom_velocity_world[interval_mask]
    odom_velocity_raw = odom_velocity_raw[interval_mask]
    odom_euler = odom_euler[interval_mask]
    odom_rate = odom_rate[interval_mask]

    ref_position = interpolate_vector(
        raw["ref"]["t"], raw["ref"]["position"], t_odom
    )
    # Reference velocity is obtained by differentiating the synchronized
    # reference position with respect to time.
    ref_velocity = differentiate_reference_position(
        t_odom,
        ref_position,
        REF_POSITION_SMOOTH_WINDOW,
    )

    imu_rate = interpolate_vector(
        raw["imu"]["t"], raw["imu"]["angular_rate"], t_odom, width=3
    )
    imu_acceleration = interpolate_vector(
        raw["imu"]["t"], raw["imu"]["linear_acceleration"], t_odom, width=3
    )

    command_sample_times = t_odom - RATE_COMMAND_DELAY_S
    cmd_body_rate = interpolate_vector(
        raw["cmd"]["t"], raw["cmd"]["body_rate"], command_sample_times, width=3
    )
    cmd_thrust = interpolate_vector(
        raw["cmd"]["t"], raw["cmd"]["thrust"], command_sample_times
    )

    if ANGULAR_RATE_SOURCE.lower() == "imu" and np.any(np.isfinite(imu_rate)):
        actual_angular_rate = imu_rate
        actual_rate_label = "IMU angular rate"
    else:
        actual_angular_rate = odom_rate
        actual_rate_label = "Odometry angular rate"

    connected = sample_previous(
        raw["state"]["t"],
        np.asarray(raw["state"]["connected"], dtype=int),
        t_odom,
        0,
    ).astype(bool)
    armed = sample_previous(
        raw["state"]["t"],
        np.asarray(raw["state"]["armed"], dtype=int),
        t_odom,
        0,
    ).astype(bool)
    mode = sample_previous(
        raw["state"]["t"],
        np.asarray(raw["state"]["mode"], dtype=object),
        t_odom,
        "UNKNOWN",
    )

    if UNWRAP_EULER_ANGLES:
        for axis in range(3):
            valid = np.isfinite(odom_euler[:, axis])
            if np.count_nonzero(valid) > 1:
                odom_euler[valid, axis] = np.unwrap(odom_euler[valid, axis])

    if EULER_IN_DEGREES:
        odom_euler = np.rad2deg(odom_euler)

    active_mask = np.ones(t_odom.shape, dtype=bool)
    if METRICS_ONLY_OFFBOARD_ARMED:
        requested_active_mask = armed & (mode == "OFFBOARD")
        if np.count_nonzero(requested_active_mask) > 0:
            active_mask = requested_active_mask
        else:
            print(
                "Warning: no armed+OFFBOARD samples were found in the selected interval; "
                "metrics will use all valid samples."
            )

    aligned = {
        "t_abs": t_odom,
        "t": t_relative - START_TIME,
        "time_origin": t_origin + START_TIME,
        "ref_position": ref_position,
        "actual_position": vision_position,
        "odom_position": odom_position,
        "position_source": TOPICS["vision_pose"],
        "ref_velocity": ref_velocity,
        "actual_velocity": odom_velocity_world,
        "actual_velocity_raw": odom_velocity_raw,
        "actual_euler": odom_euler,
        "actual_angular_rate": actual_angular_rate,
        "actual_rate_label": actual_rate_label,
        "cmd_body_rate": cmd_body_rate,
        "cmd_thrust": cmd_thrust,
        "imu_acceleration": imu_acceleration,
        "connected": connected,
        "armed": armed,
        "mode": mode,
        "active_mask": active_mask,
    }

    return aligned


def evaluate_metrics(data):
    active = data["active_mask"]

    position_metrics = compute_vector_metrics(
        data["ref_position"],
        data["actual_position"],
        active,
    )
    velocity_metrics = compute_vector_metrics(
        data["ref_velocity"],
        data["actual_velocity"],
        active,
    )
    angular_rate_metrics = compute_vector_metrics(
        data["cmd_body_rate"],
        data["actual_angular_rate"],
        active,
    )

    return {
        "Position": {"metrics": position_metrics, "unit": "m"},
        "Velocity": {"metrics": velocity_metrics, "unit": "m/s"},
        "Angular rate": {"metrics": angular_rate_metrics, "unit": "rad/s"},
    }


def print_metrics(metric_groups):
    axis_names = ["X", "Y", "Z"]
    print("\n" + "=" * 86)
    print("Tracking metrics")
    print("=" * 86)
    print(f"{'Group':<16} {'Axis':<12} {'Samples':>10} {'MAE':>16} {'RMSE':>16} {'Unit':>10}")
    print("-" * 86)

    for group_name, group in metric_groups.items():
        metrics = group["metrics"]
        for axis_index, axis_name in enumerate(axis_names):
            print(
                f"{group_name if axis_index == 0 else '':<16} "
                f"{axis_name:<12} "
                f"{metrics['count']:>10d} "
                f"{metrics['axis_mae'][axis_index]:>16.6f} "
                f"{metrics['axis_rmse'][axis_index]:>16.6f} "
                f"{group['unit']:>10}"
            )

        print(
            f"{'':<16} "
            f"{'3D vector':<12} "
            f"{metrics['count']:>10d} "
            f"{metrics['vector_mae']:>16.6f} "
            f"{metrics['vector_rmse']:>16.6f} "
            f"{group['unit']:>10}"
        )
        print("-" * 86)


# ==============================================================================
# Plotting
# ==============================================================================

def finish_axes(axes, xlabel="Time (s)"):
    for axis in np.atleast_1d(axes):
        axis.grid(True, linestyle=":", alpha=0.6)
        axis.set_xlabel(xlabel)


def save_figure(figure, directory, filename):
    """Save one figure as a high-resolution PNG."""
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / filename
    figure.savefig(path, dpi=300, bbox_inches="tight")
    print(f"  Saved figure:      {path}")
    return path


def plot_3d_trajectory(data, output_dir, timestamp):
    """
    Figure 1:
      Reference and actual 3-D position trajectory.

    Saved as:
      OUTPUT_DIR/3D/3D_<timestamp>.png
    """
    figure = plt.figure(figsize=(10, 8))
    axis = figure.add_subplot(111, projection="3d")

    axis.plot(
        data["ref_position"][:, 0],
        data["ref_position"][:, 1],
        data["ref_position"][:, 2],
        "--",
        linewidth=1.8,
        label="Reference",
    )
    axis.plot(
        data["actual_position"][:, 0],
        data["actual_position"][:, 1],
        data["actual_position"][:, 2],
        linewidth=1.8,
        label="Actual (vision pose)",
    )

    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_zlabel("Z (m)")
    axis.set_title("UAV 3-D trajectory")
    axis.legend(loc="best")
    axis.grid(True)

    # Make the three spatial axes visually comparable when possible.
    finite_ref = data["ref_position"][np.all(np.isfinite(data["ref_position"]), axis=1)]
    finite_actual = data["actual_position"][np.all(np.isfinite(data["actual_position"]), axis=1)]
    if finite_ref.size and finite_actual.size:
        all_position = np.vstack((finite_ref, finite_actual))
        xyz_min = np.min(all_position, axis=0)
        xyz_max = np.max(all_position, axis=0)
        center = 0.5 * (xyz_min + xyz_max)
        half_range = 0.5 * np.max(xyz_max - xyz_min)
        if half_range > 1.0e-9:
            axis.set_xlim(center[0] - half_range, center[0] + half_range)
            axis.set_ylim(center[1] - half_range, center[1] + half_range)
            axis.set_zlim(center[2] - half_range, center[2] + half_range)

    figure.tight_layout()
    return figure, save_figure(
        figure,
        output_dir / "3D",
        f"3D_{timestamp}.png",
    )


def plot_odom_states(data, trajectory_intervals, output_dir, timestamp):
    """
    Figure 2:
      Twelve state plots in one large 4x3 figure:
        Row 1: reference position and actual position X/Y/Z
        Row 2: reference velocity and actual velocity X/Y/Z
        Row 3: Euler roll/pitch/yaw
        Row 4: angular velocity Wx/Wy/Wz

    The angular-rate row also overlays the body-rate command, making the
    actual response and commanded rate directly comparable.

    Saved as:
      OUTPUT_DIR/ODOM/ODOM_<timestamp>.png
    """
    t = data["t"]
    xyz_names = ["X", "Y", "Z"]
    attitude_names = ["Roll", "Pitch", "Yaw"]
    rate_names = ["Wx", "Wy", "Wz"]
    angle_unit = "deg" if EULER_IN_DEGREES else "rad"

    figure, axes = plt.subplots(
        4,
        3,
        figsize=(18, 16),
        sharex="col",
    )

    # Row 1: position
    for axis_index, axis_name in enumerate(xyz_names):
        axis = axes[0, axis_index]
        axis.plot(
            t,
            data["ref_position"][:, axis_index],
            "--",
            linewidth=1.5,
            label="Reference",
        )
        axis.plot(
            t,
            data["actual_position"][:, axis_index],
            linewidth=1.5,
            label="Vision pose",
        )
        axis.set_title(f"Position {axis_name}: reference vs vision pose")
        axis.set_ylabel("Position (m)")
        axis.legend(loc="best")

    # Row 2: velocity
    for axis_index, axis_name in enumerate(xyz_names):
        axis = axes[1, axis_index]
        axis.plot(
            t,
            data["ref_velocity"][:, axis_index],
            "--",
            linewidth=1.5,
            label="Reference from d(position)/dt",
        )
        axis.plot(
            t,
            data["actual_velocity"][:, axis_index],
            linewidth=1.5,
            label="Actual",
        )
        axis.set_title(f"Velocity {axis_name}: d(ref position)/dt vs ODOM")
        axis.set_ylabel("Velocity (m/s)")
        axis.legend(loc="best")

    # Row 3: Euler attitude converted from odometry quaternion
    for axis_index, attitude_name in enumerate(attitude_names):
        axis = axes[2, axis_index]
        axis.plot(
            t,
            data["actual_euler"][:, axis_index],
            linewidth=1.5,
            label="Actual",
        )
        axis.set_title(f"Attitude {attitude_name}")
        axis.set_ylabel(f"Angle ({angle_unit})")
        axis.legend(loc="best")

    # Row 4: actual angular velocity and commanded angular velocity
    for axis_index, rate_name in enumerate(rate_names):
        axis = axes[3, axis_index]
        axis.plot(
            t,
            data["cmd_body_rate"][:, axis_index],
            "--",
            linewidth=1.5,
            label="Command",
        )
        axis.plot(
            t,
            data["actual_angular_rate"][:, axis_index],
            linewidth=1.5,
            label=data["actual_rate_label"],
        )
        axis.set_title(f"Angular rate {rate_name}")
        axis.set_ylabel("Angular rate (rad/s)")
        axis.set_xlabel("Time (s)")
        axis.legend(loc="best")

    # Draw planner trajectory start markers on all twelve time-series axes.
    add_trajectory_markers(
        axes.reshape(-1),
        trajectory_intervals,
        data["time_origin"],
    )

    for axis in axes.reshape(-1):
        axis.grid(True, linestyle=":", alpha=0.6)

    figure.suptitle(
        "UAV states: reference/vision position, ODOM velocity, attitude and angular rate",
        fontsize=16,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))

    return figure, save_figure(
        figure,
        output_dir / "ODOM",
        f"ODOM_{timestamp}.png",
    )


def plot_control_commands(data, output_dir, timestamp):
    """
    Figure 3:
      Four control-input plots:
        1. Normalized thrust
        2. Commanded body rate Wx
        3. Commanded body rate Wy
        4. Commanded body rate Wz

    Saved as:
      OUTPUT_DIR/CMD/CMD_<timestamp>.png
    """
    t = data["t"]
    rate_names = ["Wx", "Wy", "Wz"]

    figure, axes = plt.subplots(
        4,
        1,
        figsize=(14, 13),
        sharex=True,
    )

    axes[0].plot(
        t,
        data["cmd_thrust"],
        linewidth=1.6,
        label="Normalized thrust command",
    )
    axes[0].set_title("Normalized thrust command")
    axes[0].set_ylabel("Thrust")
    axes[0].legend(loc="best")

    for axis_index, rate_name in enumerate(rate_names):
        axis = axes[axis_index + 1]
        axis.plot(
            t,
            data["cmd_body_rate"][:, axis_index],
            linewidth=1.6,
            label=f"Command {rate_name}",
        )
        axis.set_title(f"Commanded body rate {rate_name}")
        axis.set_ylabel("Angular rate (rad/s)")
        axis.legend(loc="best")

    axes[-1].set_xlabel("Time (s)")

    for axis in axes:
        axis.grid(True, linestyle=":", alpha=0.6)

    figure.suptitle(
        "UAV control inputs",
        fontsize=16,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))

    return figure, save_figure(
        figure,
        output_dir / "CMD",
        f"CMD_{timestamp}.png",
    )


def plot_and_save(data, trajectory_intervals, output_dir, timestamp):
    """
    Create exactly three figures and save each one in its corresponding folder.
    """
    figures = []

    figure_3d, path_3d = plot_3d_trajectory(
        data,
        output_dir,
        timestamp,
    )
    figures.append(figure_3d)

    figure_odom, path_odom = plot_odom_states(
        data,
        trajectory_intervals,
        output_dir,
        timestamp,
    )
    figures.append(figure_odom)

    figure_cmd, path_cmd = plot_control_commands(
        data,
        output_dir,
        timestamp,
    )
    figures.append(figure_cmd)

    return figures, {
        "3D": path_3d,
        "ODOM": path_odom,
        "CMD": path_cmd,
    }


def save_aligned_csv(path, data):
    headers = [
        "time",
        "ref_px", "ref_py", "ref_pz",
        "actual_px", "actual_py", "actual_pz",
        "ref_vx", "ref_vy", "ref_vz",
        "actual_vx", "actual_vy", "actual_vz",
        "roll", "pitch", "yaw",
        "actual_wx", "actual_wy", "actual_wz",
        "cmd_wx", "cmd_wy", "cmd_wz",
        "cmd_thrust",
        "imu_ax", "imu_ay", "imu_az",
        "connected", "armed", "mode",
    ]

    with open(path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(headers)

        for index in range(data["t"].size):
            writer.writerow([
                data["t"][index],
                *data["ref_position"][index],
                *data["actual_position"][index],
                *data["ref_velocity"][index],
                *data["actual_velocity"][index],
                *data["actual_euler"][index],
                *data["actual_angular_rate"][index],
                *data["cmd_body_rate"][index],
                data["cmd_thrust"][index],
                *data["imu_acceleration"][index],
                int(data["connected"][index]),
                int(data["armed"][index]),
                data["mode"][index],
            ])


def save_trajectory_summary(path, intervals):
    with open(path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "traj_id",
            "order",
            "piece_count",
            "message_time",
            "start_time",
            "end_time",
            "duration",
        ])
        for item in intervals:
            writer.writerow([
                item["traj_id"],
                item["order"],
                item["piece_count"],
                item["message_time"],
                item["start"],
                item["end"],
                item["duration"],
            ])


# ==============================================================================
# Main
# ==============================================================================

def main():
    bag_path = os.path.expanduser(BAG_PATH)
    output_dir = Path(os.path.expanduser(OUTPUT_DIR))
    output_dir.mkdir(parents=True, exist_ok=True)

    if not os.path.isfile(bag_path):
        raise FileNotFoundError(f"Bag file does not exist: {bag_path}")

    raw = parse_bag(bag_path)
    data = prepare_aligned_data(raw)
    metrics = evaluate_metrics(data)
    print(
        f"\nPosition MAE/RMSE actual source: {data['position_source']}\n"
        f"Metric/plot interval: START_TIME={START_TIME}s, END_TIME={END_TIME}s "
        "relative to the first odometry sample."
    )
    print_metrics(metrics)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    metrics_path = output_dir / f"ommpc_all_metrics_{timestamp}.csv"

    plt.close("all")
    figures, figure_paths = plot_and_save(
        data,
        raw["trajectory"],
        output_dir,
        timestamp,
    )

    save_metrics_csv(metrics_path, metrics)

    print("\nSaved results:")
    print(f"  3-D figure:        {figure_paths['3D']}")
    print(f"  ODOM figure:       {figure_paths['ODOM']}")
    print(f"  CMD figure:        {figure_paths['CMD']}")
    print(f"  Metrics CSV:       {metrics_path}")

    if SHOW_FIGURES:
        plt.show()
    else:
        for figure in figures:
            plt.close(figure)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"\nAnalysis failed: {error}", file=sys.stderr)
        sys.exit(1)
