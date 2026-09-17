#!/usr/bin/env python3
"""Launch a PRIVATE loopback ROS master. Never attach to a flight ROS graph.

Usage: python3 ros_isolated_integration.py /absolute/path/to/test/node
All traffic is confined to /minisnap_test/* on the private master. Processes
started by this script are terminated in finally; logs remain in its temp dir.
"""
import math
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client


def main():
    if len(sys.argv) not in (2, 3):
        raise RuntimeError("provide the standalone test node executable")
    binary = os.path.abspath(sys.argv[1])
    if not os.path.isfile(binary):
        raise RuntimeError("node executable not found")
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    uri = "http://127.0.0.1:%d" % port
    os.environ["ROS_MASTER_URI"] = uri
    os.environ["ROS_IP"] = "127.0.0.1"
    os.environ.pop("ROS_HOSTNAME", None)
    directory = tempfile.mkdtemp(prefix="minisnap_ros_test_")
    os.environ["ROS_HOME"] = directory
    environment = os.environ.copy()
    processes = []
    logfiles = []
    stopped = threading.Event()
    broadcaster = None
    try:
        master_log = open(os.path.join(directory, "master.log"), "w")
        logfiles.append(master_log)
        processes.append(subprocess.Popen(["roscore", "-p", str(port)], env=environment,
                                          stdout=master_log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            try:
                if xmlrpc.client.ServerProxy(uri).getPid("minisnap_test")[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                pass
            if processes[0].poll() is not None:
                raise RuntimeError("private ROS master failed")
            time.sleep(.1)
        else:
            raise RuntimeError("private ROS master startup timed out")

        # Import only AFTER overriding the master environment.
        import rospy
        from nav_msgs.msg import Odometry, Path
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Float64MultiArray, String
        from visualization_msgs.msg import Marker
        rospy.init_node("minisnap_isolated_test", disable_signals=True)
        node_log = open(os.path.join(directory, "node.log"), "w")
        logfiles.append(node_log)
        processes.append(subprocess.Popen([
            binary, "__name:=minisnap_integration_node",
            "~waypoints:=/minisnap_test/waypoints",
            "/planning/poly_trajectory:=/minisnap_test/trajectory",
            "~vis_trajectory:=/minisnap_test/visualization",
            "_odom/topic:=/minisnap_test/odom", "_planning/frame_id:=world",
            "_planning/max_segments:=10000", "_planning/max_total_time:=10000.0",
            "_planning/dev_order:=4", "_planning/result_color:=always", "_vis/max_points:=32"],
            env=environment, stdout=node_log, stderr=subprocess.STDOUT))
        state_lock = threading.Lock()
        state = {"p": (0., 0., 2.), "v": (0., 0., 0.), "yaw": 0., "enabled": True}
        outputs, visuals, admission_states = [], [], []
        output_lock = threading.Lock()

        def output(msg):
            with output_lock:
                outputs.append(msg)

        def visual(msg):
            with output_lock:
                visuals.append(msg)
        def admission(msg):
            with output_lock:
                admission_states.append(msg.data)

        odom_pub = rospy.Publisher("/minisnap_test/odom", Odometry, queue_size=1)
        path_pub = rospy.Publisher("/minisnap_test/waypoints", Path, queue_size=1)
        subscribers = [rospy.Subscriber("/minisnap_test/trajectory", Float64MultiArray, output),
                       rospy.Subscriber("/minisnap_test/visualization", Marker, visual)]
        subscribers.append(rospy.Subscriber("/minisnap_integration_node/waypoint_admission_status", String, admission))

        def broadcast():
            while not stopped.is_set():
                with state_lock:
                    snapshot = dict(state)
                if snapshot["enabled"]:
                    msg = Odometry()
                    msg.header.frame_id = "world"
                    msg.child_frame_id = "base_link"
                    msg.header.stamp = rospy.Time.now()
                    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = snapshot["p"]
                    msg.pose.pose.orientation.z = math.sin(snapshot["yaw"] / 2)
                    msg.pose.pose.orientation.w = math.cos(snapshot["yaw"] / 2)
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = snapshot["v"]
                    odom_pub.publish(msg)
                stopped.wait(.01)

        broadcaster = threading.Thread(target=broadcast)
        broadcaster.start()
        deadline = time.monotonic() + 10
        while path_pub.get_num_connections() < 1 or odom_pub.get_num_connections() < 1:
            if time.monotonic() > deadline:
                raise RuntimeError("isolated node subscriptions timed out")
            time.sleep(.02)
        time.sleep(1.2)  # full default stable-hover window
        with output_lock:
            assert admission_states and "[航点接收: 允许]" in admission_states[-1], "no proactive admission before first waypoint"
        print("ROS proactive admission before any waypoint PASS")

        def count():
            with output_lock:
                return len(outputs)

        def send(targets, frame="world"):
            msg = Path()
            msg.header.frame_id = frame
            msg.header.stamp = rospy.Time.now()
            for p in targets:
                pose = PoseStamped()
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
                msg.poses.append(pose)
            path_pub.publish(msg)

        def wait_result(previous):
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                with output_lock:
                    if len(outputs) > previous:
                        return outputs[-1]
                time.sleep(.01)
            raise RuntimeError("validated trajectory not received; see " + directory)

        def evaluate(msg, segment, local, derivative=0):
            s, n = int(msg.data[0]), int(msg.data[1])
            values = []
            offset = 2 + s + segment * 3 * n
            for axis in range(3):
                value = 0.
                for j in range(n - derivative):
                    power = n - 1 - j
                    coefficient = msg.data[offset + axis * n + j]
                    multiplier = 1
                    for k in range(derivative):
                        multiplier *= power - k
                    value = value * local + coefficient * multiplier
                values.append(value)
            return tuple(values)

        def check_wire(msg, endpoint, velocity=(0., 0., 0.)):
            s, n = int(msg.data[0]), int(msg.data[1])
            assert n == 8 and s >= 1
            assert len(msg.data) == 2 + s + 3 * n * s
            assert all(math.isfinite(x) for x in msg.data)
            assert all(x > 0 for x in msg.data[2:2+s])
            actual = evaluate(msg, 0, 0., 1)
            assert max(abs(x-y) for x, y in zip(actual, velocity)) < 1e-6, (actual, velocity)
            actual = evaluate(msg, s-1, msg.data[2+s-1])
            assert max(abs(x-y) for x, y in zip(actual, endpoint)) < 1e-5
            for segment in range(s):
                for k in range(21):
                    for d, limit in [(1, 3.), (2, 2.), (3, 4.)]:
                        value = evaluate(msg, segment, msg.data[2+segment]*k/20, d)
                        assert math.sqrt(sum(x*x for x in value)) <= limit + 1e-6

        previous = count()
        send([(.1, 0, 2), (.2, 0, 2), (.4, 0, 2)])
        check_wire(wait_result(previous), (.4, 0, 2))
        time.sleep(.05)
        with output_lock:
            assert visuals and visuals[-1].type == Marker.LINE_STRIP
            assert 2 <= len(visuals[-1].points) <= 32
            colour = visuals[-1].color
            assert all(abs(actual - expected) < 1e-6 for actual, expected in
                       zip((colour.r, colour.g, colour.b, colour.a), (0.45, 0.15, 0.65, 1.0))), "planned trace must be fixed dark purple"
            assert not visuals[-1].colors, "planned trace must not use the actual UAV speed gradient"
        print("ROS wire format, limits and bounded visualization PASS")

        with state_lock:
            state.update(v=(.4, 0, 0), yaw=math.pi/2)
        time.sleep(.1)
        previous = count()
        send([(1., 0, 2), (2., 0, 2)])
        time.sleep(.2)
        assert count() == previous, "moving start was accepted"
        print("ROS moving-start rejection PASS")
        with state_lock:
            state.update(v=(.05, 0, 0), yaw=math.pi/2)
        time.sleep(1.2)
        previous = count()
        send([(1., 0, 2)])
        check_wire(wait_result(previous), (1., 0, 2))
        print("ROS low-speed hover proxy, zero-v/a/j boundary PASS")
        with state_lock:
            state.update(v=(0., 0, 0), yaw=0.)
        # Position drift resets the window even with reported zero velocity.
        with state_lock:
            state["p"] = (.2, 0., 2.)
        time.sleep(.1)
        previous = count()
        send([(1., 0, 2)])
        time.sleep(.2)
        assert count() == previous, "position jump did not reset hover stability"
        with state_lock:
            state["p"] = (0., 0., 2.)
        time.sleep(1.2)

        for targets, frame in [([], "world"), ([(math.nan, 0, 2)], "world"),
                                ([(1., 0, 2)], "invalid_frame")]:
            previous = count()
            send(targets, frame)
            time.sleep(.2)
            assert count() == previous, "invalid request was published"
        with state_lock:
            state.update(v=(2., 0, 0))
        time.sleep(.1)
        previous = count()
        send([(.1, 0, 2)])
        time.sleep(.2)
        assert count() == previous, "insufficient stopping route was published"
        with state_lock:
            state.update(v=(0., 0, 0), enabled=False)
        time.sleep(.7)
        previous = count()
        send([(1., 0, 2)])
        time.sleep(.2)
        assert count() == previous, "stale odometry accepted"
        with state_lock:
            state["enabled"] = True
        time.sleep(1.2)
        print("ROS empty/NaN/frame/braking/freshness rejection PASS")

        # Work must be superseded before a long result can be published.
        previous = count()
        send([(i*.01, 0, 2) for i in range(1, 5001)])
        time.sleep(.005)
        send([(.3, 0, 2)])
        check_wire(wait_result(previous), (.3, 0, 2))
        time.sleep(.4)
        assert count() == previous + 1, "obsolete planning result was published"
        print("ROS latest-request cancellation PASS")
        with open(os.path.join(directory, "node.log"), encoding="utf-8") as stream:
            console = stream.read()
        assert "\033[32m" in console and "MINIMUM-SNAP 求解成功" in console, "normal green result highlighting missing"
        assert "\033[36m[航点接收:" in console, "rolling admission status must use normal cyan"
        assert "\033[1;36m" not in console and "\033[5m" not in console, "bold/blinking result styling remains"
        print("ROS normal-cyan rolling status and distinct green result block PASS")
        if len(sys.argv) == 3:
            manager = subprocess.run([os.path.abspath(sys.argv[2])], env=environment,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15, text=True)
            print(manager.stdout)
            assert manager.returncode == 0, "manager isolated tests failed"
        print("ALL ISOLATED ROS TESTS PASS; logs=" + directory)
        return 0
    finally:
        stopped.set()
        if broadcaster:
            broadcaster.join(timeout=2)
        # Only terminate Popen handles created by this script, never arbitrary
        # ROS processes or the user's existing master.
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        for logfile in logfiles:
            logfile.close()


if __name__ == "__main__":
    sys.exit(main())
