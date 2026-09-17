// Only an isolated loopback ROS master. Never run with a flight/simulator master.
#define OMMPC_FSM_NO_MAIN
#include "../src/traj_tracking_controller_fsm.cpp"
#include <cstdlib>
#include <iostream>
void require(bool x,const char* m) { if(!x)throw std::runtime_error(m); }
struct FsmSafetyTestAccess {
    static void sequential(TrajectoryTrackingController& c) {
        VehicleState vehicle; vehicle.position=Eigen::Vector3d(0,0,2); vehicle.velocity.setZero();
        vehicle.attitude=Eigen::Quaterniond::Identity(); vehicle.valid=true;
        c.state_=c.HOVER; c.has_takeoff_=true; c.is_airborne_=true;
        c.takeoff_origin_valid_=true; c.takeoff_origin_.setZero(); c.hover_position_=vehicle.position;
        c.mavros_state_received_=true; c.mavros_state_.mode="OFFBOARD"; c.mavros_state_.armed=true;
        c.reference_valid_=true; c.last_reference_.p=vehicle.position;
        c.last_reference_.v.setZero(); c.last_reference_.a.setZero();
        c.pending_land_=false; c.pending_takeoff_=false; c.trajectory_->clear(); c.hover_gate_->reset();
        ros::NodeHandle nh;
        auto pub=nh.advertise<std_msgs::Float64MultiArray>(c.config_.trajectory_topic,1);
        bool got_point=false; geometry_msgs::PointStamped received;
        require(c.reference_point_pub_.getTopic()=="/uav1/reference_point","shared absolute reference point topic");
        auto sub=nh.subscribe<geometry_msgs::PointStamped>("/uav1/reference_point",1,
            [&](const geometry_msgs::PointStamped::ConstPtr& point){ received=*point; got_point=true; });
        auto pump=[] { ros::spinOnce(); ros::WallDuration(.01).sleep(); };
        for(int i=0;i<200 && (pub.getNumSubscribers()==0 || c.reference_point_pub_.getNumSubscribers()==0);++i) pump();
        require(pub.getNumSubscribers()>0 && c.reference_point_pub_.getNumSubscribers()>0,"trajectory and PointStamped connections");
        const ros::Time point_stamp=ros::Time::now();
        c.publishReferencePoint(point_stamp,Eigen::Vector3d(1,2,3));
        for(int i=0;i<100 && !got_point;++i) pump();
        require(got_point && received.header.stamp==point_stamp && received.header.frame_id==c.config_.reference_frame &&
            received.point.x==1 && received.point.y==2 && received.point.z==3,"PointStamped coordinates/frame/timestamp");
        auto cfg=ommpc_controller::fsm_changeConfig::__getDefault__();
        cfg.command_or_hover=true; cfg.land_enabled=false; cfg.takeoff_enabled=false;
        c.dynamicCallback(cfg,0); // ONE COMMAND for both trajectories.
        for(int segment=0;segment<2;++segment) {
            const double start=ros::WallTime::now().toSec();
            while(ros::WallTime::now().toSec()-start<1.15) {
                vehicle.stamp=ros::Time::now();
                c.hover_gate_->update(vehicle.position,vehicle.velocity,vehicle.stamp.toSec(),ros::WallTime::now().toSec(),true);
                pump();
            }
            require(c.hover_gate_->stable(ros::WallTime::now().toSec()),"stable admission before next trajectory");
            c.updateTrajectoryPermission(); require(c.admission_enabled_,"next waypoints admitted");
            std_msgs::Float64MultiArray m; m.data={1,8,.1}; m.data.resize(27,0); m.data[26]=2;
            pub.publish(m); for(int i=0;i<100 && !c.trajectory_->ready();++i) pump();
            require(c.trajectory_->ready(),"next trajectory READY");
            vehicle.stamp=ros::Time::now(); c.updateHover(vehicle,vehicle.stamp);
            require(c.state_==c.POLY_TRAJ && c.trajectory_->active(),"autoactivation without another COMMAND");
            Eigen::Vector3d p,v,a;
            c.updatePolynomial(vehicle,vehicle.stamp+ros::Duration(.2),p,v,a);
            require(c.state_==c.BRAKE && c.command_mode_,"natural completion retains COMMAND session");
            c.brake_start_=ros::Time::now()-ros::Duration(c.braking_.duration()+1);
            for(int i=0;i<200 && c.state_==c.BRAKE;++i) {
                vehicle.stamp=ros::Time::now(); c.updateBrake(vehicle,vehicle.stamp,p,v,a); pump();
            }
            require(c.state_==c.HOVER && c.command_mode_,"completion returns to HOVER with session enabled");
            require(c.hover_gate_->stable(ros::WallTime::now().toSec()),"reuse confirmed brake hover window");
        }
        cfg.command_or_hover=false; c.dynamicCallback(cfg,0);
        require(!c.command_mode_,"explicit HOVER disables sequential execution");
        std::ostringstream captured; auto* old=std::cout.rdbuf(captured.rdbuf());
        mavros_msgs::AttitudeTarget command; c.config_.status_color="always"; c.last_print_time_=ros::Time(0);
        c.printStatus(ros::Time::now(),vehicle,vehicle.position,command); std::cout.rdbuf(old);
        require(captured.str().find("\033[36m")!=std::string::npos && captured.str().find("\033[1m")==std::string::npos &&
            captured.str().find("\033[5m")==std::string::npos,"controller uses normal cyan without bold/blink");
        std::cout<<"TWO TRAJECTORIES / ONE COMMAND + PointStamped + normal cyan PASS\n";
    }
    static void run(TrajectoryTrackingController& c) {
        using Eigen::Vector3d;
        VehicleState vehicle; vehicle.position=Vector3d(0,0,2); vehicle.velocity=Vector3d(1.7,-.2,0);
        vehicle.attitude=Eigen::Quaterniond::Identity(); vehicle.valid=true; vehicle.stamp=ros::Time::now();
        c.has_takeoff_=true; c.is_airborne_=true; c.takeoff_origin_valid_=true; c.takeoff_origin_=Vector3d::Zero();
        c.mavros_state_received_=true; c.mavros_state_.mode="OFFBOARD"; c.mavros_state_.armed=true;
        c.state_=c.POLY_TRAJ; c.command_mode_=true; c.reference_valid_=true;
        c.last_reference_.p=vehicle.position; c.last_reference_.v=vehicle.velocity; c.last_reference_.a=Vector3d(0,-1.9,0);
        auto cfg=ommpc_controller::fsm_changeConfig::__getDefault__();
        cfg.land_enabled=true; cfg.command_or_hover=false; cfg.takeoff_enabled=false;
        c.dynamicCallback(cfg,0); c.processTriggers(vehicle,vehicle.stamp);
        require(c.state_==c.BRAKE && c.brake_for_land_ && !c.pending_land_,"single LAND update enters BRAKE, not HOVER");
        Vector3d p,v,a; c.updateBrake(vehicle,vehicle.stamp,p,v,a);
        require((v-vehicle.velocity).norm()<1e-12 && (a-c.last_reference_.a).norm()<1e-12,"LAND brake retains incoming reference derivatives");
        cfg.command_or_hover=true; c.dynamicCallback(cfg,0);
        require(!c.command_mode_ && !cfg.command_or_hover,"LAND rejects later COMMAND reenable");
        const auto end=c.braking_.at(c.braking_.duration());
        c.brake_start_=ros::Time::now()-ros::Duration(c.braking_.duration()+1);
        vehicle.position=end.p; vehicle.velocity.setZero();
        const double hover_window_start=ros::WallTime::now().toSec();
        for(int i=0;i<200;++i) {
            vehicle.stamp=ros::Time::now(); c.updateBrake(vehicle,vehicle.stamp,p,v,a);
            if(ros::WallTime::now().toSec()-hover_window_start<.99)
                require(c.state_==c.BRAKE,"LAND waits sustained hover, not just curve completion");
            if(c.state_==c.LAND)break;
            ros::WallDuration(.01).sleep();
        }
        require(c.state_==c.LAND,"stable BRAKE transitions to LAND");
        c.updateLanding(vehicle,c.landing_start_,p,v,a);
        require((p-end.p).norm()<1e-12 && v.norm()==0 && a.norm()==0,"descent starts continuously at stop reference");
        vehicle.position.z()=.09; vehicle.velocity.setZero(); c.ude_->activate(vehicle,ros::Time::now());
        c.updateLanding(vehicle,c.landing_start_+ros::Duration(.5),p,v,a); c.updateLiftoffAndUde(vehicle,ros::Time::now());
        require(!c.ude_->active() && c.ude_ground_inhibit_,"near-ground UDE disabled");
        vehicle.position.z()=.11; c.updateLiftoffAndUde(vehicle,ros::Time::now());
        require(!c.ude_->active(),"UDE does not chatter at .10m");
        vehicle.position.z()=.26; c.updateLiftoffAndUde(vehicle,ros::Time::now());
        require(c.ude_->active(),"UDE can reenable above hysteresis band");
        vehicle.position.z()=.01;
        c.extended_state_received_=false;
        for(int i=0;i<3;++i)c.updateLanding(vehicle,c.landing_start_+ros::Duration(10+i),p,v,a);
        require(!c.touchdown_unload_active_,"low height/speed alone cannot unload thrust");
        c.extended_state_received_=true;
        c.extended_state_.landed_state=mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
        c.extended_state_wall_=ros::WallTime::now().toSec()-3;
        c.updateLanding(vehicle,c.landing_start_+ros::Duration(20),p,v,a);
        require(!c.touchdown_unload_active_,"stale PX4 ground state cannot unload thrust");
        c.touchdown_unload_active_=true;
        c.updateLanding(vehicle,c.landing_start_+ros::Duration(21),p,v,a);
        require(!c.touchdown_unload_active_,"lost ground confirmation cancels unloading");
        // No controlStep, ARM/DISARM calls, or attitude command publication.
    }
};
int main(int argc,char** argv) {
    const char* uri=std::getenv("ROS_MASTER_URI");
    if(!uri || std::string(uri).find("http://127.0.0.1:")!=0 || std::string(uri).find(":11311")!=std::string::npos) {
        std::cerr<<"REFUSED: requires isolated loopback ROS master on non-default port\n"; return 2;
    }
    ros::init(argc,argv,"fsm_safety_isolated_test");
    ros::NodeHandle nh("~"); nh.setParam("bag/enabled",false);
    TrajectoryTrackingController controller; FsmSafetyTestAccess::sequential(controller); FsmSafetyTestAccess::run(controller);
    std::cout<<"ALL FSM SAFETY ISOLATED ROS TESTS PASS\n";
}
