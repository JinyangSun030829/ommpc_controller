#!/usr/bin/env python3
"""Start our own loopback master; never connect tests to the flight master."""
import os
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

def main():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    uri = "http://127.0.0.1:%d" % port
    environment = os.environ.copy()
    environment.update(ROS_MASTER_URI=uri, ROS_IP="127.0.0.1",
                       ROS_HOME=tempfile.mkdtemp(prefix="fsm_safety_ros_"))
    environment.pop("ROS_HOSTNAME", None)
    log_path = os.path.join(environment["ROS_HOME"], "master.log")
    with open(log_path, "w") as log:
        master = subprocess.Popen(["roscore", "-p", str(port)], env=environment,
                                  stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+20
            while time.monotonic() < deadline:
                try:
                    if xmlrpc.client.ServerProxy(uri).getPid("fsm_test_runner")[0] == 1:
                        break
                except (OSError, xmlrpc.client.Error):
                    pass
                if master.poll() is not None:
                    raise RuntimeError("isolated master failed")
                time.sleep(.1)
            else:
                raise RuntimeError("isolated master timeout")
            for binary in sys.argv[1:]:
                subprocess.run([os.path.abspath(binary)], env=environment, check=True, timeout=30)
        finally:
            master.terminate()
            try:
                master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                master.kill()
                master.wait()
    print("PRIVATE MASTER TESTS COMPLETE; log:", log_path)

if __name__ == "__main__":
    main()
