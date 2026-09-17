#!/usr/bin/env python3
"""Compare calibration with the real ROS planner on a PRIVATE loopback master.

Starts no controller/MAVROS, sends no actuator commands; temp records auto-clean.
Usage: python3 circle_calculator_ros_test.py GENERATOR_BINARY CALCULATOR_SCRIPT
"""
import importlib.util
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client


def main():
    if len(sys.argv) != 3:
        raise RuntimeError("provide generator binary and calculator script")
    generator, script = map(Path, sys.argv[1:])
    spec = importlib.util.spec_from_file_location("calculator", script)
    calculator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(calculator)
    package = script.resolve().parent.parent
    limits, options = calculator.load_configuration(package)
    circle = calculator.circle_requirements(1.5, 2, 21, limits["gravity"])
    probe_binary = calculator.find_probe(package)
    settings, _ = calculator.calibrate(circle, options,
        lambda s: calculator.run_probe(probe_binary, 1.5, s, options))
    settings = [float(s) for s in calculator.format_settings(settings)]
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    if port == 11311:
        raise RuntimeError("refusing standard flight ROS port")
    uri = "http://127.0.0.1:%d" % port
    os.environ["ROS_MASTER_URI"] = uri
    os.environ["ROS_IP"] = "127.0.0.1"
    os.environ.pop("ROS_HOSTNAME", None)
    processes, stop = [], threading.Event()
    thread = None
    rospy = None
    with tempfile.TemporaryDirectory(prefix="circle_calculator_ros_") as temporary:
        os.environ["ROS_HOME"] = temporary
        os.environ["ROS_LOG_DIR"] = temporary + "/log"
        with open(temporary + "/master.log", "w") as master_log, \
             open(temporary + "/planner.log", "w") as planner_log:
            try:
                processes.append(subprocess.Popen(["roscore", "-p", str(port)],
                    stdout=master_log, stderr=subprocess.STDOUT))
                deadline = time.monotonic() + 15
                while True:
                    try:
                        if xmlrpc.client.ServerProxy(uri).getPid("circle_calculator_test")[0] == 1:
                            break
                    except (OSError, xmlrpc.client.Error):
                        pass
                    if time.monotonic() > deadline:
                        raise RuntimeError("private master startup timeout")
                    time.sleep(.05)
                import rospy as ros
                rospy = ros
                from nav_msgs.msg import Odometry, Path as PathMessage
                from geometry_msgs.msg import PoseStamped
                from std_msgs.msg import Float64MultiArray, String
                rospy.init_node("circle_calculator_test", disable_signals=True)
                prefix = "/circle_calibration_planner/"
                for key, value in options.items():
                    rospy.set_param(prefix + "planning/" + key, value)
                for key, value in zip(("vel", "acc", "max_jerk"), settings):
                    rospy.set_param(prefix + "planning/" + key, value)
                rospy.set_param(prefix + "planning/compress_time", False)
                rospy.set_param(prefix + "planning/require_controller_ready", False)
                rospy.set_param(prefix + "planning/result_color", "never")
                rospy.set_param(prefix + "vis/enabled", False)
                rospy.set_param(prefix + "odom/topic", "/circle_calculator_test/odom")
                processes.append(subprocess.Popen([str(generator), "__name:=circle_calibration_planner",
                    "~waypoints:=/circle_calculator_test/waypoints",
                    "/planning/poly_trajectory:=/circle_calculator_test/trajectory"],
                    stdout=planner_log, stderr=subprocess.STDOUT))
                results, admission = [], []
                subscribers = [rospy.Subscriber("/circle_calculator_test/trajectory",
                    Float64MultiArray, lambda msg: results.append(msg)),
                    rospy.Subscriber(prefix + "waypoint_admission_status", String,
                                     lambda msg: admission.append(msg.data))]
                odom_pub = rospy.Publisher("/circle_calculator_test/odom", Odometry, queue_size=1)
                waypoint_pub = rospy.Publisher("/circle_calculator_test/waypoints", PathMessage, queue_size=1)
                def broadcast():
                    while not stop.is_set():
                        msg = Odometry()
                        msg.header.frame_id = "world"
                        msg.child_frame_id = "base_link"
                        msg.header.stamp = rospy.Time.now()
                        msg.pose.pose.position.y = 1.5
                        msg.pose.pose.position.z = 2
                        msg.pose.pose.orientation.w = 1
                        odom_pub.publish(msg)
                        stop.wait(.01)
                thread = threading.Thread(target=broadcast)
                thread.start()
                deadline = time.monotonic() + 10
                while not admission or "[航点接收: 允许]" not in admission[-1]:
                    if time.monotonic() > deadline:
                        raise RuntimeError("planner did not reach stable hover admission")
                    time.sleep(.01)
                message = PathMessage()
                message.header.frame_id = "world"
                message.header.stamp = rospy.Time.now()
                for i in range(1, calculator.CIRCLE_TARGETS + 1):
                    p = PoseStamped()
                    angle = i * 2 * math.pi / calculator.WAYPOINTS_PER_TURN
                    p.pose.position.x = 1.5 * math.sin(angle)
                    p.pose.position.y = 1.5 * math.cos(angle)
                    p.pose.position.z = 2
                    message.poses.append(p)
                p = PoseStamped(); p.pose.position.z = 2
                message.poses.append(p)
                waypoint_pub.publish(message)
                deadline = time.monotonic() + 10
                while not results:
                    if time.monotonic() > deadline:
                        raise RuntimeError("no polynomial result from actual planner")
                    time.sleep(.01)
                values = results[-1].data
                segments, count = int(values[0]), int(values[1])
                assert segments == 35 and count == 8
                assert len(values) == 2 + segments + 3 * count * segments
                def speed_at(segment, t):
                    offset = 2 + segments + segment * 3 * count
                    vector = []
                    for axis in range(3):
                        velocity = 0
                        for k in range(count - 1):
                            velocity = velocity * t + values[offset + axis * count + k] * (count - 1 - k)
                        vector.append(velocity)
                    return math.sqrt(sum(x*x for x in vector))
                length, duration = 0, 0
                for segment in range(8, 24):
                    t = values[2 + segment]; h = t / 128
                    duration += t
                    for k in range(129):
                        length += h / 3 * (1 if k in (0, 128) else (4 if k % 2 else 2)) * speed_at(segment, k*h)
                mean = length / duration
                expected = calculator.run_probe(probe_binary, 1.5, settings, options)
                assert abs(mean - 2) / 2 <= calculator.SPEED_RELATIVE_TOLERANCE
                assert abs(mean - expected["mean_speed"]) < 1e-7
                assert abs(sum(values[2:2+segments]) - expected["total_time"]) < 1e-7
                print("PASS actual ROS planner vs offline calculator: mean=%.9f m/s, duration=%.9f s" %
                      (mean, sum(values[2:2+segments])))
            finally:
                stop.set()
                if thread:
                    thread.join(timeout=3)
                if rospy:
                    rospy.signal_shutdown("private test complete")
                for process in reversed(processes):
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill(); process.wait(timeout=5)
                # Log output on failure before TemporaryDirectory removes records.
                if sys.exc_info()[0] is not None:
                    planner_log.flush()
                    print(Path(temporary + "/planner.log").read_text(), file=sys.stderr)


if __name__ == "__main__":
    main()
