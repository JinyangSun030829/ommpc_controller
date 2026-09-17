// Run ONLY with an isolated ROS master. No PX4, arming or command publishers.
#include "trajectory_manager.hpp"
#include <iostream>
#include <functional>
void require(bool x,const char* msg) { if(!x)throw std::runtime_error(msg); }
int main(int argc,char**argv) {
    ros::init(argc,argv,"manager_isolated_test");
    ros::NodeHandle nh;
    TrajectoryManager manager(nh,"/minisnap_test/manager_trajectory");
    auto publisher=nh.advertise<std_msgs::Float64MultiArray>("/minisnap_test/manager_trajectory",1);
    auto pump=[] { for(int i=0;i<15;++i){ros::spinOnce();ros::WallDuration(.01).sleep();} };
    for(int i=0;i<100 && publisher.getNumSubscribers()==0;++i) pump();
    require(publisher.getNumSubscribers()>0,"manager subscriber connection");
    auto message=[] {
        std_msgs::Float64MultiArray m;m.data={1,8,1};m.data.resize(27,0);
        // constant hover polynomial at z=2
        m.data[26]=2;return m;
    };
    auto send=[&](const std_msgs::Float64MultiArray&m){publisher.publish(m);pump();};
    const Eigen::Vector3d p(0,0,2),v=Eigen::Vector3d::Zero();
    auto activate=[&](Eigen::Vector3d position,Eigen::Vector3d velocity,double age,bool stable,bool permitted) {
        return manager.activate(ros::Time::now(),ros::WallTime::now().toSec(),position,velocity,age,stable,permitted);
    };
    send(message());require(!manager.ready(),"permission rejects incoming trajectory");
    manager.setAcceptEnabled(true);send(message());require(manager.ready(),"READY reception");
    require(!activate(p,v,.01,false,true)&&manager.ready(),"wait stable without activating");
    require(!activate(p,v,.01,true,false)&&manager.ready(),"OFFBOARD ARMED permission");
    require(!activate(p,v,.3,true,true)&&manager.ready(),"stale activation state");
    require(!activate(p,Eigen::Vector3d(.2,0,0),.01,true,true)&&manager.ready(),"moving state");
    require(!activate(Eigen::Vector3d(.2,0,2),v,.01,true,true)&&!manager.ready(),"mismatch clears READY");
    send(message());require(manager.ready(),"resend after mismatch");
    manager.expireReady(ros::WallTime::now().toSec()+3);
    require(!manager.ready(),"expiry clears READY");
    auto moving=message();moving.data[9]=.1;send(moving);
    require(!manager.ready(),"nonzero initial velocity rejected");
    auto accelerating=message();accelerating.data[8]=.1;send(accelerating);
    require(!manager.ready(),"nonzero acceleration rejected");
    auto jerk=message();jerk.data[7]=.1;send(jerk);
    require(!manager.ready(),"nonzero jerk rejected");
    send(message());require(activate(p,v,.01,true,true)&&manager.active(),"valid activation");
    Eigen::Vector3d rp,rv,ra;
    require(manager.evaluate(ros::Time::now(),rp,rv,ra),"active reference evaluation");
    require((rp-p).norm()<1e-10&&rv.norm()==0&&ra.norm()==0,"reference preserved");
    manager.clear();require(!manager.active()&&!manager.ready(),"clear state");
    std::cout<<"ALL MANAGER ISOLATED ROS TESTS PASS\n";
}
