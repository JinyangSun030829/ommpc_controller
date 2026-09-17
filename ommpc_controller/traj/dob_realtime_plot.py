#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Real-time DOB plotting node for ROS1.

Expected topics:
  /ommpc_controller/dob/disturbance_raw           geometry_msgs/Vector3Stamped
  /ommpc_controller/dob/disturbance_filtered      geometry_msgs/Vector3Stamped
  /ommpc_controller/dob/disturbance_comp          geometry_msgs/Vector3Stamped
  /ommpc_controller/dob/disturbance_dot           geometry_msgs/Vector3Stamped
  /ommpc_controller/dob/nominal_acceleration      geometry_msgs/Vector3Stamped
  /ommpc_controller/dob/velocity_error            geometry_msgs/Vector3Stamped
  /ommpc_controller/dob/status                    std_msgs/Float64MultiArray

status.data:
  [0] enable
  [1] initialized
  [2] input_valid
  [3] delayed_thrustacc [m/s^2]
  [4] observer_dt [s]
  [5] ramp
"""

import threading
from collections import deque
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray


class VectorBuffer:
    """Thread-safe time history for a 3D vector."""

    def __init__(self, max_points: int) -> None:
        self.t = deque(maxlen=max_points)
        self.x = deque(maxlen=max_points)
        self.y = deque(maxlen=max_points)
        self.z = deque(maxlen=max_points)

    def append(self, t: float, x: float, y: float, z: float) -> None:
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

    def copy(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return (
            np.asarray(self.t, dtype=float),
            np.asarray(self.x, dtype=float),
            np.asarray(self.y, dtype=float),
            np.asarray(self.z, dtype=float),
        )


class DobRealtimePlot:
    def __init__(self) -> None:
        self.lock = threading.Lock()

        self.window_sec = float(rospy.get_param("~window_sec", 20.0))
        self.plot_rate = float(rospy.get_param("~plot_rate", 20.0))
        self.max_points = int(rospy.get_param("~max_points", 5000))

        self.topic_names = {
            "raw": rospy.get_param(
                "~raw_topic",
                "/ommpc_controller/dob/disturbance_raw",
            ),
            "filtered": rospy.get_param(
                "~filtered_topic",
                "/ommpc_controller/dob/disturbance_filtered",
            ),
            "comp": rospy.get_param(
                "~comp_topic",
                "/ommpc_controller/dob/disturbance_comp",
            ),
            "dot": rospy.get_param(
                "~dot_topic",
                "/ommpc_controller/dob/disturbance_dot",
            ),
            "nominal_acc": rospy.get_param(
                "~nominal_acc_topic",
                "/ommpc_controller/dob/nominal_acceleration",
            ),
            "velocity_error": rospy.get_param(
                "~velocity_error_topic",
                "/ommpc_controller/dob/velocity_error",
            ),
            "status": rospy.get_param(
                "~status_topic",
                "/ommpc_controller/dob/status",
            ),
        }

        self.buffers: Dict[str, VectorBuffer] = {
            name: VectorBuffer(self.max_points)
            for name in (
                "raw",
                "filtered",
                "comp",
                "dot",
                "nominal_acc",
                "velocity_error",
            )
        }

        self.first_stamp = None
        self.last_time = 0.0

        self.status = {
            "enable": 0.0,
            "initialized": 0.0,
            "input_valid": 0.0,
            "delayed_thrustacc": 0.0,
            "observer_dt": 0.0,
            "ramp": 0.0,
        }

        for key in (
            "raw",
            "filtered",
            "comp",
            "dot",
            "nominal_acc",
            "velocity_error",
        ):
            rospy.Subscriber(
                self.topic_names[key],
                Vector3Stamped,
                self._vector_callback,
                callback_args=key,
                queue_size=100,
                tcp_nodelay=True,
            )

        rospy.Subscriber(
            self.topic_names["status"],
            Float64MultiArray,
            self._status_callback,
            queue_size=100,
            tcp_nodelay=True,
        )

        self._build_figure()

    def _relative_time(self, stamp_sec: float) -> float:
        if stamp_sec <= 0.0:
            stamp_sec = rospy.get_time()

        if self.first_stamp is None:
            self.first_stamp = stamp_sec

        return stamp_sec - self.first_stamp

    def _vector_callback(self, msg: Vector3Stamped, key: str) -> None:
        stamp_sec = msg.header.stamp.to_sec()
        with self.lock:
            t = self._relative_time(stamp_sec)
            self.buffers[key].append(
                t,
                msg.vector.x,
                msg.vector.y,
                msg.vector.z,
            )
            self.last_time = max(self.last_time, t)

    def _status_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            rospy.logwarn_throttle(
                2.0,
                "DOB status topic has fewer than 6 elements.",
            )
            return

        with self.lock:
            self.status["enable"] = msg.data[0]
            self.status["initialized"] = msg.data[1]
            self.status["input_valid"] = msg.data[2]
            self.status["delayed_thrustacc"] = msg.data[3]
            self.status["observer_dt"] = msg.data[4]
            self.status["ramp"] = msg.data[5]

    def _build_figure(self) -> None:
        self.fig, self.axes = plt.subplots(
            3,
            3,
            figsize=(18, 10),
            sharex=True,
        )
        self.fig.canvas.manager.set_window_title("OMMPC DOB Real-time Monitor")

        # Row 1: raw / filtered / compensated comparison for each axis
        self.compare_lines = {}
        axis_names = ("x", "y", "z")
        for col, axis_name in enumerate(axis_names):
            ax = self.axes[0, col]
            self.compare_lines[(axis_name, "raw")], = ax.plot(
                [], [], label="raw"
            )
            self.compare_lines[(axis_name, "filtered")], = ax.plot(
                [], [], label="filtered"
            )
            self.compare_lines[(axis_name, "comp")], = ax.plot(
                [], [], label="comp"
            )
            ax.set_title(
                f"Disturbance {axis_name.upper()}: raw / filtered / comp"
            )
            ax.set_ylabel("m/s²")
            ax.grid(True)
            ax.legend(loc="upper right")

        # Row 2: complete XYZ vectors
        self.vector_lines = {}
        row2_specs = (
            ("raw", "Raw disturbance", "m/s²"),
            ("filtered", "Filtered disturbance", "m/s²"),
            ("comp", "Applied compensation", "m/s²"),
        )
        for col, (key, title, unit) in enumerate(row2_specs):
            ax = self.axes[1, col]
            for component in axis_names:
                self.vector_lines[(key, component)], = ax.plot(
                    [], [], label=component
                )
            ax.set_title(title)
            ax.set_ylabel(unit)
            ax.grid(True)
            ax.legend(loc="upper right")

        # Row 3: derivative / nominal acceleration / velocity error
        row3_specs = (
            ("dot", "Filtered disturbance derivative", "m/s³"),
            ("nominal_acc", "Observer nominal acceleration", "m/s²"),
            ("velocity_error", "Observer velocity error", "m/s"),
        )
        for col, (key, title, unit) in enumerate(row3_specs):
            ax = self.axes[2, col]
            for component in axis_names:
                self.vector_lines[(key, component)], = ax.plot(
                    [], [], label=component
                )
            ax.set_title(title)
            ax.set_xlabel("Time [s]")
            ax.set_ylabel(unit)
            ax.grid(True)
            ax.legend(loc="upper right")

        self.fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.95])
        self.fig.canvas.mpl_connect("close_event", self._on_close)

    @staticmethod
    def _component(data, component: str) -> np.ndarray:
        if component == "x":
            return data[1]
        if component == "y":
            return data[2]
        return data[3]

    def _update(self, _frame):
        with self.lock:
            snapshot = {
                key: buffer.copy()
                for key, buffer in self.buffers.items()
            }
            status = dict(self.status)
            last_time = self.last_time

        # First row: compare raw, filtered, comp on the same component
        for axis_name in ("x", "y", "z"):
            for key in ("raw", "filtered", "comp"):
                data = snapshot[key]
                self.compare_lines[(axis_name, key)].set_data(
                    data[0],
                    self._component(data, axis_name),
                )

        # Remaining vector plots
        for key in (
            "raw",
            "filtered",
            "comp",
            "dot",
            "nominal_acc",
            "velocity_error",
        ):
            data = snapshot[key]
            for component in ("x", "y", "z"):
                self.vector_lines[(key, component)].set_data(
                    data[0],
                    self._component(data, component),
                )

        xmin = max(0.0, last_time - self.window_sec)
        xmax = max(self.window_sec, last_time)

        for ax in self.axes.flat:
            ax.set_xlim(xmin, xmax)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        enable = int(status["enable"] > 0.5)
        initialized = int(status["initialized"] > 0.5)
        input_valid = int(status["input_valid"] > 0.5)

        self.fig.suptitle(
            "DOB Status | "
            f"enable={enable}  "
            f"initialized={initialized}  "
            f"input_valid={input_valid}  "
            f"delayed_thrust={status['delayed_thrustacc']:.3f} m/s²  "
            f"dt={status['observer_dt']:.4f} s  "
            f"ramp={status['ramp']:.3f}",
            fontsize=13,
        )

        return list(self.compare_lines.values()) + list(
            self.vector_lines.values()
        )

    def _on_close(self, _event) -> None:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("DOB plot window closed")

    def run(self) -> None:
        interval_ms = max(10, int(1000.0 / max(self.plot_rate, 1.0)))
        self.animation = FuncAnimation(
            self.fig,
            self._update,
            interval=interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()


def main() -> None:
    rospy.init_node("dob_realtime_plot", anonymous=False)

    try:
        plotter = DobRealtimePlot()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logfatal("DOB real-time plot failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
