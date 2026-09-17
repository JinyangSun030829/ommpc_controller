#!/usr/bin/env python3
"""Only run through run_isolated_cpp_tests.py on its private loopback master."""
import os
import subprocess
import threading
import time
import tempfile

def main():
    uri=os.environ.get("ROS_MASTER_URI", "")
    assert uri.startswith("http://127.0.0.1:") and ":11311" not in uri, "private master required"
    import rospy
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String, Float64MultiArray
    rospy.init_node("admission_private_test", disable_signals=True)
    binary=os.environ["ADMISSION_TEST_NODE"]
    logdir=tempfile.mkdtemp(prefix="admission_node_test_")
    log=open(os.path.join(logdir,"node.log"),"w")
    process=subprocess.Popen([binary,"__name:=admission_test_node",
        "~waypoints:=/minisnap_test/admission_waypoints",
        "/planning/poly_trajectory:=/minisnap_test/admission_trajectory",
        "~waypoint_admission_status:=/minisnap_test/admission_status",
        "_odom/topic:=/minisnap_test/admission_odom", "_planning/frame_id:=world",
        "_planning/require_controller_ready:=true",
        "_planning/controller_status_topic:=/minisnap_test/fsm_status",
        "_planning/controller_status_max_age:=0.5", "_planning/status_interval:=0.5",
        "_planning/result_color:=never","_vis/enabled:=false"], stdout=log,stderr=subprocess.STDOUT)
    lock=threading.Lock(); state={"allowed":False,"reason":"FSM=LAND","heartbeat":True,"stamp_offset":0,"speed":0.0}
    statuses=[]; outputs=[]; stopped=threading.Event()
    def status(m):
        with lock: statuses.append(m.data)
    def output(m):
        with lock: outputs.append(m)
    subs=[rospy.Subscriber("/minisnap_test/admission_status",String,status),
          rospy.Subscriber("/minisnap_test/admission_trajectory",Float64MultiArray,output)]
    odom=rospy.Publisher("/minisnap_test/admission_odom",Odometry,queue_size=1)
    fsm=rospy.Publisher("/minisnap_test/fsm_status",String,queue_size=1,latch=True)
    path=rospy.Publisher("/minisnap_test/admission_waypoints",Path,queue_size=1)
    def broadcast():
        count=0
        while not stopped.is_set():
            with lock: s=state.copy()
            m=Odometry(); m.header.stamp=rospy.Time.now(); m.header.frame_id="world"; m.child_frame_id="base_link"
            m.pose.pose.position.z=2; m.pose.pose.orientation.w=1; m.twist.twist.linear.x=s["speed"]; odom.publish(m)
            if count%10==0 and s["heartbeat"]:
                fsm.publish(String(data="%s|%.9f|%s"%("ALLOW" if s["allowed"] else "BLOCKED",time.time()+s["stamp_offset"],s["reason"])))
            count+=1; stopped.wait(.01)
    thread=threading.Thread(target=broadcast); thread.start()
    def latest():
        with lock: return statuses[-1] if statuses else ""
    def wait(predicate,description,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate(): return
            if process.poll() is not None: raise RuntimeError("node exited; "+logdir)
            time.sleep(.02)
        raise RuntimeError(description+"; latest="+latest()+"; logs="+logdir)
    def send():
        m=Path(); m.header.frame_id="world"; p=PoseStamped(); p.pose.position.x=.5; p.pose.position.z=2; m.poses=[p]; path.publish(m)
    def count():
        with lock:return len(outputs)
    try:
        wait(lambda:path.get_num_connections()>0,"waypoint connection")
        wait(lambda:"禁止" in latest() and "FSM=LAND" in latest(),"proactive controller-blocked state")
        assert count()==0
        time.sleep(1.2) # physical hover alone must not grant controller permission
        assert "禁止" in latest()
        with lock: state.update(allowed=True,reason="FSM=HOVER, COMMAND=0; ready")
        wait(lambda:"[航点接收: 允许]" in latest() and "COMMAND未启用" in latest(),"proactive ready / COMMAND disabled before first waypoint")
        with lock: state.update(reason="FSM=HOVER, COMMAND=1; ready")
        wait(lambda:"COMMAND已启用" in latest(),"COMMAND-enabled display")
        time.sleep(2.2)
        assert "[航点接收: 允许]" in latest(), "ready incorrectly expired without any trajectory"
        assert count()==0
        print("ADMISSION proactive readiness and no fixed receiving deadline PASS")
        send(); wait(lambda:count()==1,"accepted first waypoint produces trajectory")
        print("ADMISSION first waypoint accepted after proactive READY PASS")
        with lock: state.update(stamp_offset=-10)
        wait(lambda:"禁止" in latest() and "新鲜接收权限" in latest(),"stale latched permission rejected")
        previous=count(); send(); time.sleep(.3); assert count()==previous
        with lock: state.update(stamp_offset=0)
        wait(lambda:"[航点接收: 允许]" in latest(),"fresh permission restored")
        with lock: state.update(heartbeat=False)
        wait(lambda:"禁止" in latest() and "新鲜接收权限" in latest(),"dead heartbeat blocks readiness")
        print("ADMISSION stale latch / dead controller heartbeat PASS")
        with lock: state.update(heartbeat=True,speed=.3)
        wait(lambda:"禁止" in latest() and "等待持续稳定悬停" in latest(),"moving-state proactive display")
        previous=count(); send(); time.sleep(.3); assert count()==previous
        with lock: state.update(speed=0,allowed=False,reason="READY trajectory waiting for COMMAND; expiry=2s")
        wait(lambda:"禁止" in latest() and "READY trajectory" in latest(),"READY occupancy reason")
        print("ADMISSION moving gate and READY occupancy reason PASS")
        with open(os.path.join(logdir,"node.log"),encoding="utf-8") as stream: console=stream.read()
        assert "[航点已接收]" in console and "[航点未接收]" in console
        assert "[轨迹已发送]" in console and "求解成功不等于已开始执行" in console
        print("ALL ADMISSION PRIVATE ROS TESTS PASS; logs="+logdir)
    finally:
        stopped.set(); thread.join(timeout=2); process.terminate()
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill();process.wait()
        log.close()

if __name__=="__main__": main()
