// Only run on a private loopback ROS master. Never initialize flight interfaces.
#define OMMPC_FSM_NO_MAIN
#include OMMPC_FSM_SOURCE
#include <cstdlib>
#include <iostream>

static void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

struct OmmpcFsmTestAccess
{
    static void run(OMMPC_EXAMPLE &c)
    {
        c.param_.state_timeout = .5;
        c.param_.use_ref_txt = false;
        c.odom_ready_ = c.hover_initialized_ = true;
        c.odom_data_.p = Eigen::Vector3d(1, 2, 3);
        c.odom_data_.v.setZero();
        c.odom_data_.rcv_stamp = ros::Time::now();
        c.has_takeoff_ = true;
        c.state_.armed = true;
        c.state_.mode = "OFFBOARD";
        c.is_command_mode_ = true;
        c.trajectory_data_.exec_traj = 0;
        std::string reason;
        auto allowed = [&] { return c.canAcceptPolynomial(reason); };
        require(allowed() && reason.find(", COMMAND=1") != std::string::npos,
                "ready OMMPC must allow; COMMAND marker matches minisnap protocol");
        c.has_takeoff_ = false;
        require(!allowed() && reason.find("等待起飞完成") != std::string::npos, "takeoff gate");
        c.has_takeoff_ = true;
        for (auto state : {TAKEOFF, POLY_TRAJ, POINTS, BRAKE, LAND})
        {
            c.exec_traj_state_ = state;
            require(!allowed(), "only HOVER permits new polynomial trajectories");
        }
        c.exec_traj_state_ = HOVER;
        c.is_command_mode_ = false;
        require(!allowed() && reason.find("COMMAND未开启") != std::string::npos, "COMMAND gate");
        c.is_command_mode_ = true;
        c.state_.armed = false;
        require(!allowed(), "armed gate");
        c.state_.armed = true;
        c.state_.mode = "POSCTL";
        require(!allowed(), "OFFBOARD gate");
        c.state_.mode = "OFFBOARD";
        c.trajectory_ready_ = true;
        require(!allowed(), "READY occupancy gate");
        c.trajectory_ready_ = false;
        c.trajectory_data_.exec_traj = 1;
        require(!allowed(), "execution occupancy gate");
        c.trajectory_data_.exec_traj = 0;
        c.land_trigger_ = true;
        require(!allowed(), "pending LAND gate");
        c.land_trigger_ = false;
        c.takeoff_trigger_ = true;
        require(!allowed(), "pending TAKEOFF gate");
        c.takeoff_trigger_ = false;
        c.param_.use_ref_txt = c.txt_start_pending_ = true;
        require(!allowed(), "pending TXT must not advertise polynomial permission");
        c.param_.use_ref_txt = c.txt_start_pending_ = false;
        c.consecutive_failures_ = 1;
        require(!allowed(), "solver recovery gate");
        c.consecutive_failures_ = 0;
        c.box_fault_latched_ = true;
        require(!allowed(), "box fault gate");
        c.box_fault_latched_ = false;
        c.odom_data_.rcv_stamp = ros::Time::now() - ros::Duration(1);
        require(!allowed(), "stale odometry gate");
        c.odom_data_.rcv_stamp = ros::Time::now() + ros::Duration(1);
        require(!allowed(), "future odometry gate");
        c.odom_data_.rcv_stamp = ros::Time::now();
        c.odom_data_.v.x() = std::numeric_limits<double>::quiet_NaN();
        require(!allowed(), "nonfinite state gate");
        c.odom_data_.v.setZero();
        c.odom_ready_ = false;
        require(!allowed(), "missing odometry gate");
        c.odom_ready_ = true;

        // Real ROS String publisher on exactly the requested topic; no flight IO.
        const std::string topic = "/traj_tracking_controller/trajectory_admission_status";
        c.admission_pub_ = c.node_.advertise<std_msgs::String>(topic, 1, true);
        c.publishAdmissionStatus(ros::WallTimerEvent());
        std::string received;
        auto subscriber = c.node_.subscribe<std_msgs::String>(
            topic, 1, [&](const std_msgs::String::ConstPtr &msg) { received = msg->data; });
        auto wait = [&](const std::string &prefix) {
            const double deadline = ros::WallTime::now().toSec() + 3;
            while (ros::WallTime::now().toSec() < deadline)
            {
                ros::spinOnce();
                if (received.find(prefix) == 0)
                    return;
                ros::WallDuration(.01).sleep();
            }
            throw std::runtime_error("admission topic message missing: " + prefix);
        };
        wait("ALLOW|"); // Late subscriber must receive latched status.
        const auto split = received.find('|', 6);
        require(split != std::string::npos && received.substr(split + 1).find("OMMPC:") == 0,
                "wire format matches ALLOW|wall_timestamp|reason");
        const double stamp = std::stod(received.substr(6, split - 6));
        require(std::abs(stamp - ros::WallTime::now().toSec()) < 1, "fresh absolute wall stamp");

        // Timer uses wall time: permissions continue refreshing even if ROS time is paused.
        c.admission_timer_ = c.node_.createWallTimer(
            ros::WallDuration(.2), &OMMPC_EXAMPLE::publishAdmissionStatus, &c);
        c.is_command_mode_ = false;
        received.clear();
        wait("BLOCKED|");
        require(received.find("COMMAND未开启") != std::string::npos, "blocked topic explains reason");
        c.is_command_mode_ = true;
        c.odom_data_.rcv_stamp = ros::Time::now();
        received.clear();
        wait("ALLOW|");
        c.admission_timer_.stop();
        std::cout << "ALL OMMPC ADMISSION GATES / SHARED TOPIC / LATCH / HEARTBEAT TESTS PASS\n";
    }
};

int main(int argc, char **argv)
{
    const char *uri = std::getenv("ROS_MASTER_URI");
    if (!uri || std::string(uri).find("http://127.0.0.1:") != 0 ||
        std::string(uri).find(":11311") != std::string::npos)
    {
        std::cerr << "Private random-port loopback master required\n";
        return 2;
    }
    ros::init(argc, argv, "ommpc_admission_test");
    try
    {
        OMMPC_EXAMPLE controller; // No init(): no actuators/subscriptions/services.
        OmmpcFsmTestAccess::run(controller);
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
