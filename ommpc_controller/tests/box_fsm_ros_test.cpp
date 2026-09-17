// Compiled twice against the actual controller sources, using an isolated master.
#define OMMPC_FSM_NO_MAIN
#include FSM_BOX_SOURCE
#include <cstdlib>
#include <iostream>
void require(bool value,const char *message)
{
    if (!value) throw std::runtime_error(message);
}

flight_safety::BoxOptions options()
{
    flight_safety::BoxOptions box;
    box.enabled=true;
    box.minimum=Eigen::Vector3d(-10,-10,-.5);
    box.maximum=Eigen::Vector3d(10,10,5);
    box.margin.setZero();
    return box;
}

std::vector<Eigen::MatrixXd> circleCoefficients()
{
    // Rest-to-circle ramp followed by a circle, represented by C3 degree-7 pieces.
    auto state=[](double t, flight_safety::Reference &r, Eigen::Vector3d &j) {
        const double w=1.4, ramp=2;
        double angle, rate, accel, jerk;
        if(t<ramp)
        {
            const double s=t/ramp;
            angle=w*ramp*(2.5*std::pow(s,4)-3*std::pow(s,5)+std::pow(s,6));
            rate=w*(10*std::pow(s,3)-15*std::pow(s,4)+6*std::pow(s,5));
            accel=w/ramp*(30*s*s-60*s*s*s+30*std::pow(s,4));
            jerk=w/(ramp*ramp)*(60*s-180*s*s+120*s*s*s);
        }
        else { angle=w*(t-ramp/2); rate=w; accel=jerk=0; }
        const Eigen::Vector3d radial(std::cos(angle),std::sin(angle),0);
        const Eigen::Vector3d tangent(-std::sin(angle),std::cos(angle),0);
        r.p=radial+Eigen::Vector3d(0,0,2);
        r.v=rate*tangent; r.a=-rate*rate*radial+accel*tangent;
        j=(-rate*rate*rate+jerk)*tangent-3*rate*accel*radial;
    };
    auto choose=[](int n,int k) { double r=1; for(int i=1;i<=k;++i) r*=double(n-i+1)/i; return r; };
    std::vector<Eigen::MatrixXd> result;
    const double h=.25;
    for(int piece=0;piece<20;++piece)
    {
        flight_safety::Reference l,r; Eigen::Vector3d jl,jr;
        state(piece*h,l,jl); state((piece+1)*h,r,jr);
        flight_safety::box_detail::Points b{
            l.p,l.p+h*l.v/7,l.p+2*h*l.v/7+h*h*l.a/42,
            l.p+3*h*l.v/7+h*h*l.a/14+h*h*h*jl/210,
            r.p-3*h*r.v/7+h*h*r.a/14-h*h*h*jr/210,
            r.p-2*h*r.v/7+h*h*r.a/42,r.p-h*r.v/7,r.p};
        Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,8);
        for(int k=0;k<=7;++k)
            for(int i=0;i<=k;++i)
                c.col(7-k)+=choose(7,k)*choose(k,i)*((k-i)%2 ? -1 : 1)*b[i]/std::pow(h,k);
        result.push_back(c);
    }
    return result;
}

flight_safety::BoxOptions circleOptions()
{
    auto box=options();
    box.minimum.x()=box.minimum.y()=-1.01;
    box.maximum.x()=box.maximum.y()=1.01;
    return box;
}

#ifdef FSM_BOX_TRACKING
struct FsmSafetyTestAccess
{
    static void run(TrajectoryTrackingController &c)
    {
        c.box_guard_.configure(options()); c.braking_.configure(options());
        VehicleState vehicle;
        vehicle.position=Eigen::Vector3d(0,0,2);
        vehicle.velocity=Eigen::Vector3d(5,0,0);
        vehicle.attitude=Eigen::Quaterniond::Identity();
        vehicle.valid=true; vehicle.stamp=ros::Time::now();
        c.has_takeoff_=c.is_airborne_=c.takeoff_origin_valid_=true;
        c.takeoff_origin_.setZero();
        c.mavros_state_received_=true;
        c.mavros_state_.mode="OFFBOARD"; c.mavros_state_.armed=true;
        c.state_=c.HOVER; c.hover_position_=vehicle.position; vehicle.velocity.setZero();
        c.reference_valid_=true; c.last_reference_.p=vehicle.position;
        c.last_reference_.v.setZero(); c.last_reference_.a.setZero();
        ros::NodeHandle policy_nh("~");
        policy_nh.setParam("landing/allow_during_trajectory",false);
        auto land_request=ommpc_controller::fsm_changeConfig::__getDefault__();
        land_request.land_enabled=true;
        c.dynamicCallback(land_request,0);
        require(c.pending_land_,"tracking switch OFF still accepts HOVER LAND");
        c.processTriggers(vehicle,ros::Time::now());
        require(!c.safety_latched_ && c.state_==c.BRAKE && c.braking_.at(0).v.norm()==0 &&
                c.braking_.at(0).a.norm()==0,"tracking stationary HOVER LAND executable");
        // Use the real TrajectoryManager subscriber and activation API.
        ros::NodeHandle nh;
        c.trajectory_.reset(new TrajectoryManager(nh,"/isolated_curve_test/path"));
        c.trajectory_->setAcceptEnabled(true);
        auto pub=nh.advertise<std_msgs::Float64MultiArray>("/isolated_curve_test/path",1,true);
        std_msgs::Float64MultiArray msg;
        const auto circle=circleCoefficients();
        msg.data={double(circle.size()),8};
        for(std::size_t i=0;i<circle.size();++i) msg.data.push_back(.25);
        for(const auto &coef:circle)
            for(int axis=0;axis<3;++axis)
                for(int k=0;k<8;++k) msg.data.push_back(coef(axis,k));
        pub.publish(msg);
        for(int i=0;i<200 && !c.trajectory_->ready();++i)
        { ros::spinOnce(); ros::WallDuration(.005).sleep(); }
        require(c.trajectory_->ready(),"tracking receives test polynomial");
        const auto curve_now=ros::Time::now();
        require(c.trajectory_->activate(curve_now-ros::Duration(2.5),ros::WallTime::now().toSec(),
                Eigen::Vector3d(1,0,2),Eigen::Vector3d::Zero(),0,true,true),
                "tracking activates rest-start circle polynomial");
        c.box_guard_.configure(circleOptions()); c.braking_.configure(circleOptions());
        require(c.trajectory_->withinBox(c.box_guard_,3),"test circle full path and acceleration admitted");
        c.state_=c.POLY_TRAJ;
        flight_safety::Reference curve_entry;
        require(c.boxReferenceAt(curve_now,curve_entry),"tracking previews actual polynomial");
        vehicle.position=curve_entry.p; vehicle.velocity=curve_entry.v;
        c.checkBoxSafety(vehicle,curve_now);
        require(c.state_==c.POLY_TRAJ && !c.safety_latched_,"tracking tight circle continues without false brake");
        c.command_mode_=true; c.last_land_switch_=false;
        land_request.land_enabled=true; land_request.command_or_hover=false;
        c.dynamicCallback(land_request,0);
        require(!c.pending_land_ && c.command_mode_ && land_request.command_or_hover &&
                !land_request.land_enabled && c.trajectory_->active() && c.state_==c.POLY_TRAJ,
                "tracking OFF rejects compound LAND without cancelling trajectory");
        policy_nh.setParam("landing/allow_during_trajectory",true);
        land_request.land_enabled=true; land_request.command_or_hover=false;
        c.dynamicCallback(land_request,0);
        require(c.pending_land_ && !c.command_mode_,"tracking runtime ON accepts polynomial LAND");
        policy_nh.setParam("landing/allow_during_trajectory",false);
        c.processTriggers(vehicle,curve_now);
        require(c.state_==c.BRAKE && c.brake_for_land_ && c.braking_.followsPath() &&
                !c.safety_latched_,"tracking circle LAND uses certified path-guided brake");
        c.enterBrake(vehicle,curve_now+ros::Duration(.1),true);
        require(c.brake_start_==curve_now,"tracking curved brake duplicate LAND preserves time");
        for(int i=0;i<16;++i)
        {
            const double t=c.braking_.duration()*i/16;
            const auto r=c.braking_.at(t);
            vehicle.position=r.p; vehicle.velocity=r.v;
            c.last_box_check_wall_=-1;
            c.checkBoxSafety(vehicle,curve_now+ros::Duration(t));
            require(c.state_==c.BRAKE && !c.safety_latched_ && c.brake_start_==curve_now,
                    "tracking committed curved brake runs without restart/fault");
        }
        c.box_guard_.configure(options()); c.braking_.configure(options());
        c.last_box_check_wall_=-1;
        vehicle.position=Eigen::Vector3d(0,0,2); vehicle.velocity=Eigen::Vector3d(5,0,0);
        c.state_=c.POLY_TRAJ;
        c.reference_valid_=true;
        c.last_reference_.p=vehicle.position; c.last_reference_.v=vehicle.velocity;
        c.last_reference_.a.setZero();
        const auto start=ros::Time::now();
        c.enterBrake(vehicle,start,true);
        require(c.state_==c.BRAKE && !c.safety_latched_ && c.brake_for_land_,
                "tracking LAND enters certified BOX BRAKE");
        const auto end=c.braking_.at(c.braking_.duration());
        require(c.box_guard_.options().contains(end.p) && end.v.norm()==0,
                "tracking BOX BRAKE stops inside");
        c.enterBrake(vehicle,start+ros::Duration(.1),true);
        require(c.brake_start_==start,"tracking duplicate LAND preserves curve");
        c.state_=c.HOVER;
        vehicle.position=Eigen::Vector3d(7.9,0,2);
        vehicle.velocity=Eigen::Vector3d(2,0,0);
        c.hover_position_=vehicle.position;
        c.last_reference_.p=vehicle.position; c.last_reference_.v=vehicle.velocity;
        c.checkBoxSafety(vehicle,ros::Time::now());
        require(c.state_==c.BRAKE && !c.brake_for_land_ && !c.safety_latched_,
                "tracking reaction reserve triggers preventive brake before current infeasibility");
        c.state_=c.POLY_TRAJ;
        vehicle.position.x()=9.99;
        c.last_reference_.p=vehicle.position;
        c.enterBrake(vehicle,ros::Time::now(),true);
        require(c.safety_latched_ && !c.command_mode_,
                "tracking uncertifiable stop latches instead of descending");
        auto config=ommpc_controller::fsm_changeConfig::__getDefault__();
        config.command_or_hover=true;
        c.dynamicCallback(config,0);
        require(!config.command_or_hover,"tracking box fault cannot be overridden by COMMAND");
    }
};
#else
struct OmmpcFsmTestAccess
{
    static void run(OMMPC_EXAMPLE &c)
    {
        c.transition_options_.braking_max_acceleration=3;
        c.box_guard_.configure(options()); c.braking_.configure(options());
        c.state_.mode="OFFBOARD"; c.state_.armed=true;
        c.has_takeoff_=c.is_airborne_=c.takeoff_ground_z_valid_=true;
        c.takeoff_ground_z_=0;
        c.odom_data_.p=Eigen::Vector3d(0,0,2);
        c.odom_data_.v=Eigen::Vector3d(5,0,0);
        c.odom_data_.q=Eigen::Quaterniond::Identity();
        c.exec_traj_state_=HOVER; c.hover_pose_.head<3>()=c.odom_data_.p;
        c.odom_data_.v.setZero(); c.reference_valid_=true;
        c.last_reference_.p=c.odom_data_.p;
        c.last_reference_.v.setZero(); c.last_reference_.a.setZero();
        ros::NodeHandle policy_nh("~");
        policy_nh.setParam("landing/allow_during_trajectory",false);
        auto land_request=ommpc_controller::fsm_changeConfig::__getDefault__();
        land_request.land_enabled=true;
        c.stateChangeCallback(land_request,0);
        require(c.land_trigger_,"OMMPC switch OFF still accepts HOVER LAND");
        c.processTriggers(ros::Time::now());
        require(!c.box_fault_latched_ && c.exec_traj_state_==BRAKE &&
                c.braking_.at(0).v.norm()==0 && c.braking_.at(0).a.norm()==0,
                "OMMPC stationary HOVER LAND executable");
        c.exec_traj_state_=POINTS; c.is_command_mode_=true; c.land_enabled_=false;
        land_request.land_enabled=true; land_request.command_or_hover=false;
        c.stateChangeCallback(land_request,0);
        require(!c.land_trigger_ && c.is_command_mode_ && !land_request.land_enabled &&
                land_request.command_or_hover && c.exec_traj_state_==POINTS,
                "OMMPC OFF also rejects TXT LAND without cancelling command");
        c.box_guard_.configure(circleOptions()); c.braking_.configure(circleOptions());
        const auto curve_now=ros::Time::now();
        oneTraj_Data_t curve;
        curve.traj_start_time=curve_now-ros::Duration(2.5);
        for(const auto &coef:circleCoefficients()) curve.traj.emplace_back(.25,coef);
        c.trajectory_data_.traj_queue.push_back(curve);
        c.trajectory_data_.exec_traj=1;
        c.exec_traj_state_=POLY_TRAJ;
        flight_safety::Reference curve_entry;
        require(c.boxReferenceAt(curve_now,curve_entry),"OMMPC previews circle polynomial");
        c.odom_data_.p=curve_entry.p; c.odom_data_.v=curve_entry.v;
        c.checkBoxSafety(curve_now);
        require(c.exec_traj_state_==POLY_TRAJ && !c.box_fault_latched_,
                "OMMPC tight circle continues without false brake");
        c.is_command_mode_=true; c.land_enabled_=false;
        land_request.land_enabled=true; land_request.command_or_hover=false;
        c.stateChangeCallback(land_request,0);
        require(!c.land_trigger_ && c.is_command_mode_ && !land_request.land_enabled &&
                land_request.command_or_hover && c.trajectory_data_.traj_queue.size()==1 &&
                c.exec_traj_state_==POLY_TRAJ,
                "OMMPC OFF rejects compound polynomial LAND without clearing trajectory");
        policy_nh.setParam("landing/allow_during_trajectory",true);
        land_request.land_enabled=true; land_request.command_or_hover=false;
        c.stateChangeCallback(land_request,0);
        require(c.land_trigger_ && !c.is_command_mode_,"OMMPC runtime ON accepts polynomial LAND");
        policy_nh.setParam("landing/allow_during_trajectory",false);
        c.processTriggers(curve_now);
        require(c.exec_traj_state_==BRAKE && c.brake_for_land_ && c.braking_.followsPath() &&
                !c.box_fault_latched_,"OMMPC circle LAND uses certified path-guided brake");
        c.enterBrake(curve_now+ros::Duration(.1),true);
        require(c.brake_start_==curve_now,"OMMPC curved brake duplicate LAND preserves time");
        for(int i=0;i<16;++i)
        {
            const double t=c.braking_.duration()*i/16;
            const auto r=c.braking_.at(t);
            c.odom_data_.p=r.p; c.odom_data_.v=r.v;
            c.last_box_check_wall_=-1;
            c.checkBoxSafety(curve_now+ros::Duration(t));
            require(c.exec_traj_state_==BRAKE && !c.box_fault_latched_ && c.brake_start_==curve_now,
                    "OMMPC committed curved brake runs without restart/fault");
        }
        c.box_guard_.configure(options()); c.braking_.configure(options());
        c.last_box_check_wall_=-1;
        c.odom_data_.p=Eigen::Vector3d(0,0,2); c.odom_data_.v=Eigen::Vector3d(5,0,0);
        c.exec_traj_state_=POLY_TRAJ;
        c.reference_valid_=true;
        c.last_reference_.p=c.odom_data_.p;
        c.last_reference_.v=c.odom_data_.v;
        c.last_reference_.a.setZero();
        const auto start=ros::Time::now();
        c.enterBrake(start,true);
        require(c.exec_traj_state_==BRAKE && !c.box_fault_latched_ && c.brake_for_land_,
                "OMMPC LAND enters certified BOX BRAKE");
        const auto end=c.braking_.at(c.braking_.duration());
        require(c.box_guard_.options().contains(end.p) && end.v.norm()==0,
                "OMMPC BOX BRAKE stops inside");
        c.enterBrake(start+ros::Duration(.1),true);
        require(c.brake_start_==start,"OMMPC duplicate LAND preserves curve");
        c.exec_traj_state_=HOVER;
        c.odom_data_.p=Eigen::Vector3d(7.9,0,2);
        c.odom_data_.v=Eigen::Vector3d(2,0,0);
        c.hover_pose_.head<3>()=c.odom_data_.p;
        c.last_reference_.p=c.odom_data_.p; c.last_reference_.v=c.odom_data_.v;
        c.checkBoxSafety(ros::Time::now());
        require(c.exec_traj_state_==BRAKE && !c.brake_for_land_ && !c.box_fault_latched_,
                "OMMPC reaction reserve triggers preventive brake before current infeasibility");
        c.exec_traj_state_=POLY_TRAJ;
        c.odom_data_.p.x()=9.99; c.last_reference_.p=c.odom_data_.p;
        c.enterBrake(ros::Time::now(),true);
        require(c.box_fault_latched_ && !c.is_command_mode_,
                "OMMPC uncertifiable stop latches instead of descending");
        auto config=ommpc_controller::fsm_changeConfig::__getDefault__();
        config.command_or_hover=true;
        c.stateChangeCallback(config,0);
        require(!config.command_or_hover,"OMMPC box fault cannot be overridden by COMMAND");
    }
};
#endif

int main(int argc,char **argv)
{
    const char *uri=std::getenv("ROS_MASTER_URI");
    if (!uri || std::string(uri).find("http://127.0.0.1:")!=0 ||
        std::string(uri).find(":11311")!=std::string::npos) return 2;
    ros::init(argc,argv,"box_fsm_isolated_test");
    ros::NodeHandle nh("~");
    nh.setParam("flight_box/enabled",1);
    std::vector<double> minimum{-10,-10,-.5},maximum{10,10,5};
    nh.setParam("flight_box/minimum",minimum); nh.setParam("flight_box/maximum",maximum);
    nh.setParam("flight_box/margin",.5);
    const auto inset=flight_safety::loadBoxOptions(nh);
    require(inset.margin.isApprox(Eigen::Vector3d(.5,.5,0)),"scalar .5 only contracts XY");
    nh.setParam("flight_box/margin",std::vector<double>{.4,.6});
    require(flight_safety::loadBoxOptions(nh).margin.isApprox(Eigen::Vector3d(.4,.6,0)),
            "two-axis margin array loader");
    nh.setParam("flight_box/margin",std::vector<double>{.4,.6,.8});
    require(flight_safety::loadBoxOptions(nh).margin.isApprox(Eigen::Vector3d(.4,.6,0)),
            "legacy third-axis margin ignored");
    nh.deleteParam("flight_box/margin");
    const auto zero=flight_safety::loadBoxOptions(nh);
    require(zero.margin.norm()==0,"omitted margin defaults to zero");
    nh.setParam("flight_box/enabled",0);
    require(!flight_safety::loadBoxOptions(nh).enabled,"numeric zero disables protection");
    nh.setParam("bag/enabled",false);
    try
    {
#ifdef FSM_BOX_TRACKING
        TrajectoryTrackingController controller;
        FsmSafetyTestAccess::run(controller);
        std::cout<<"TRACKING BOX FSM TEST PASS (private master)\n";
#else
        OMMPC_EXAMPLE controller; // No init(): no MAVROS services or actuator publishers.
        OmmpcFsmTestAccess::run(controller);
        std::cout<<"OMMPC BOX FSM TEST PASS (private master)\n";
#endif
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr<<error.what()<<std::endl; return 1;
    }
}
