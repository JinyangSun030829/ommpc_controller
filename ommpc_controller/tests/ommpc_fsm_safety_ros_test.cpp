// Run only through run_isolated_cpp_tests.py, never against a flight master.
#define OMMPC_FSM_NO_MAIN
#include OMMPC_FSM_SOURCE
#include <iostream>

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

struct OmmpcFsmTestAccess
{
    static void run(OMMPC_EXAMPLE &c)
    {
        using Eigen::Vector3d;
        c.state_.mode = "OFFBOARD";
        c.state_.armed = true;
        c.has_takeoff_ = true;
        c.takeoff_ground_z_valid_ = true;
        c.takeoff_ground_z_ = 1.0;
        c.takeoff_yaw_ = 0.7;
        c.odom_data_.q = Eigen::Quaterniond::Identity();
        c.odom_data_.p = Vector3d(1, 2, 3);
        c.odom_data_.v = Vector3d(1.7, -.2, 0);
        c.param_.landing_speed = .25;
        c.param_.landing_target_offset = -.1;
        c.param_.dob_landing_disable_height = .1;
        c.param_.touchdown_height = .08;
        c.param_.touchdown_max_vz = .1;
        c.param_.touchdown_max_vxy = .1;
        c.param_.touchdown_confirm_time = .5;
        c.param_.thrust_unload_time = .8;
        c.param_.touchdown_thrust = .05;
        c.param_.service_retry_interval = 1;
        c.exec_traj_state_ = POLY_TRAJ;
        c.reference_valid_ = true;
        c.last_reference_.p = c.odom_data_.p;
        c.last_reference_.v = c.odom_data_.v;
        c.last_reference_.a = Vector3d(0, -1.9, 0);
        auto cfg = ommpc_controller::fsm_changeConfig::__getDefault__();
        cfg.command_or_hover = true;
        cfg.land_enabled = true;
        cfg.takeoff_enabled = false;
        c.stateChangeCallback(cfg, 0);
        const ros::Time start = ros::Time::now();
        // A running polynomial is sampled at LAND's timestamp, rather than
        // using the previous MPC cycle's cached reference.
        oneTraj_Data_t active;
        active.traj_start_time = start - ros::Duration(.2);
        Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3,6);
        coefficients.col(5) = c.odom_data_.p;
        coefficients.col(4) = c.odom_data_.v;
        coefficients.col(3) = .5*c.last_reference_.a;
        active.traj.emplace_back(10.0,coefficients);
        const Eigen::MatrixXd evaluated = active.traj.getPVAJSC(.2);
        flight_safety::Reference seed;
        seed.p=evaluated.col(0); seed.v=evaluated.col(1); seed.a=evaluated.col(2);
        c.trajectory_data_.traj_queue.push_back(active);
        c.trajectory_data_.exec_traj=1;
        c.processTriggers(start);
        require(c.exec_traj_state_ == BRAKE && c.brake_for_land_, "LAND must enter BRAKE");
        require(!c.land_trigger_ && !c.is_command_mode_, "LAND consumes trigger and disables COMMAND");
        require(c.trajectory_data_.traj_queue.empty(), "LAND clears polynomial admission/execution");
        const auto entry = c.braking_.at(0);
        require((entry.p-seed.p).norm()<1e-12 && (entry.v-seed.v).norm()<1e-12 &&
                (entry.a-seed.a).norm()<1e-12, "braking preserves incoming p/v/a");
        c.enterBrake(start + ros::Duration(.1), true);
        require(c.brake_start_ == start, "duplicate LAND must not restart braking");
        cfg.command_or_hover = true;
        c.stateChangeCallback(cfg, 0);
        require(!cfg.command_or_hover && !c.is_command_mode_, "LAND owns COMMAND through braking");
        require(!c.beginLanding(start), "direct LAND entry rejected before stable hover");

        const auto stopped = c.braking_.at(c.braking_.duration());
        c.brake_start_ = ros::Time::now() - ros::Duration(c.braking_.duration()+1);
        c.odom_data_.p = stopped.p;
        c.odom_data_.v.setZero();
        // Reusing one odometry sample may not fake a stable window.
        c.odom_data_.msg.header.stamp = ros::Time::now();
        for (int i=0; i<20; ++i) c.updateBrakeState(ros::Time::now());
        require(c.exec_traj_state_ == BRAKE, "duplicate odometry cannot confirm landing");
        c.brake_gate_.reset();
        const double wall_start = ros::WallTime::now().toSec();
        while (c.exec_traj_state_ == BRAKE &&
               ros::WallTime::now().toSec()-wall_start < 2.0)
        {
            c.odom_data_.msg.header.stamp = ros::Time::now();
            c.updateBrakeState(c.odom_data_.msg.header.stamp);
            if (ros::WallTime::now().toSec()-wall_start < .99)
                require(c.exec_traj_state_ == BRAKE, "LAND requires sustained hover");
            ros::WallDuration(.01).sleep();
        }
        require(c.exec_traj_state_ == LAND, "confirmed hover must enter LAND");
        const auto descent_entry = c.descent_.at(0);
        require((descent_entry.p-stopped.p).norm()<1e-12 &&
                descent_entry.v.norm()==0 && descent_entry.a.norm()==0, "continuous stop/descent join");
        require(std::abs(c.landing_target_z_-.9)<1e-12 &&
                std::abs(c.descent_.at(c.descent_.duration()).p.z()-.9)<1e-12,
                "landing target is takeoff initial Z minus 0.1 m");
        require(c.hover_pose_(3)==.7, "landing retains takeoff yaw");
        cfg.command_or_hover = true;
        c.stateChangeCallback(cfg, 0);
        require(!cfg.command_or_hover, "LAND rejects COMMAND while descending");

        Controller_Output_t output;
        output.thrust=.3; output.bodyrates.setZero();
        c.odom_data_.p.z()=1.0;
        c.updateLandingContact(ros::Time::now(), output);
        require(!c.touchdown_unload_active_, "low altitude/velocity alone is not touchdown");
        c.extended_state_ready_=true;
        c.extended_state_.landed_state=mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
        c.extended_state_wall_=ros::WallTime::now().toSec()-3;
        c.updateLandingContact(ros::Time::now(), output);
        require(!c.landing_detect_active_, "stale ON_GROUND rejected");
        c.extended_state_wall_=ros::WallTime::now().toSec();
        c.updateLandingContact(ros::Time::now(), output);
        require(c.landing_detect_active_ && !c.touchdown_unload_active_, "fresh contact begins confirmation");
        c.landing_detect_start_=ros::Time::now()-ros::Duration(.6);
        c.updateLandingContact(ros::Time::now(), output);
        require(c.touchdown_unload_active_, "confirmed contact starts smooth unloading");
        c.extended_state_wall_=ros::WallTime::now().toSec()-3;
        c.updateLandingContact(ros::Time::now(), output);
        require(!c.touchdown_unload_active_, "stale contact cancels unloading before disarm");

        c.exec_traj_state_=BRAKE;
        c.brake_for_land_=false;
        const auto original_start=c.brake_start_;
        c.enterBrake(ros::Time::now(),true);
        require(c.brake_for_land_ && c.brake_start_==original_start, "HOVER brake upgrades to LAND without restart");
        std::cout << "ALL OMMPC BRAKE/LAND STATE TESTS PASS (no actuator publishers/services)\n";
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ommpc_fsm_safety_test");
    try
    {
        OMMPC_EXAMPLE controller; // No init(): no flight subscriptions, publishers, or services.
        OmmpcFsmTestAccess::run(controller);
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
