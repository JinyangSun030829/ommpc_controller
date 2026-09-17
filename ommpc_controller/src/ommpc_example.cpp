#include "disturbance_observer.hpp"
#include "ommpc_controller.hpp"
#include "ommpc_controller/fsm_changeConfig.h"
#include "stable_hover.h"
#include "traj_tracking_controller/box_flight_safety_ros.hpp"
#include "traj_tracking_controller/flight_safety.hpp"
#include <algorithm>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iomanip>
#include <limits>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <stdexcept>
#include <string>

#include <fstream>
#include <ros/package.h>

enum Exec_Traj_State_t
{
    HOVER = 10,     // execute the hover trajectory
    POLY_TRAJ = 11, // execute the polynomial trajectory
    POINTS = 12,    // execute the point trajectory (txt)
    TAKEOFF = 13,
    LAND = 14,
    BRAKE = 15
};

// FSM-only settings. MPC and DOB gains remain in Parameter_t/config/params.yaml.
struct FsmTransitionOptions
{
    minisnap::HoverOptions hover;
    double braking_max_acceleration = 4.0;
    double braking_max_jerk = 4.0;
    double braking_position_tolerance = 0.15;
    double landing_max_acceleration = 0.50;
    double landing_max_jerk = 1.0;
    double extended_state_timeout = 2.0;
    bool allow_land_during_trajectory = true;

    void load(const ros::NodeHandle &nh)
    {
        nh.param("braking/max_acceleration", braking_max_acceleration, braking_max_acceleration);
        nh.param("braking/max_jerk", braking_max_jerk, braking_max_jerk);
        nh.param("braking/position_tolerance", braking_position_tolerance,
                 braking_position_tolerance);
        nh.param("trajectory/hover/duration", hover.duration, hover.duration);
        nh.param("trajectory/hover/max_speed", hover.max_speed, hover.max_speed);
        nh.param("trajectory/hover/max_position_span", hover.max_position_span,
                 hover.max_position_span);
        nh.param("trajectory/hover/max_state_age", hover.max_state_age, hover.max_state_age);
        nh.param("landing/max_acceleration", landing_max_acceleration, landing_max_acceleration);
        nh.param("landing/max_jerk", landing_max_jerk, landing_max_jerk);
        nh.param("landing/extended_state_timeout", extended_state_timeout, extended_state_timeout);
        nh.param("landing/allow_during_trajectory", allow_land_during_trajectory,
                 allow_land_during_trajectory);
        hover.validate();
        for (double value : {braking_max_acceleration, braking_max_jerk, braking_position_tolerance,
                             landing_max_acceleration, landing_max_jerk, extended_state_timeout})
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument("invalid FSM transition limits");
    }
};

class OMMPC_EXAMPLE
{
  private:
    friend struct OmmpcFsmTestAccess;
    ros::NodeHandle node_;
    ros::NodeHandle private_nh_{"~"};
    ros::Publisher cmd_pub_;
    ros::Publisher reference_position_pub_;
    // Same admission protocol/topic as the tracking FSM; run only one controller.
    ros::Publisher admission_pub_;
    ros::WallTimer admission_timer_;
    bool admission_displayed_ = false;
    bool last_admission_allowed_ = false;
    // DOB debug publishers
    ros::Publisher dob_raw_pub_;
    ros::Publisher dob_filtered_pub_;
    ros::Publisher dob_comp_pub_;
    ros::Publisher dob_dot_pub_;
    ros::Publisher dob_nominal_acc_pub_;
    ros::Publisher dob_velocity_error_pub_;
    ros::Publisher dob_status_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_, mpc_traj_sub_;
    ros::Subscriber extended_state_sub_;
    ros::ServiceClient set_mode_client_, arming_client_srv_;
    ros::Timer exec_timer_;
    mavros_msgs::State state_;
    mavros_msgs::ExtendedState extended_state_;
    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Exec_Traj_State_t exec_traj_state_ = HOVER;
    Parameter_t param_;
    Trajectory_Data_t trajectory_data_;
    MpcController ommpc_controller_;
    Controller_Output_t last_u_;
    bool is_command_mode_ = false;
    bool takeoff_enabled_ = false, last_takeoff_enabled_ = false, takeoff_trigger_ = false;
    bool land_enabled_ = false, last_land_enabled_ = false, land_trigger_ = false;
    bool has_takeoff_ = false, has_land_ = true;
    // 表示物理上是否已经离地
    // 与“起飞流程是否完整完成”不是同一个概念
    bool is_airborne_ = false;
    bool dob_airborne_active_ = false;
    // 只有起飞完成后新收到的轨迹才能置 true
    bool trajectory_ready_ = false;
    bool odom_ready_ = false;
    bool hover_initialized_ = false;
    // 用于拒绝起飞前生成的旧轨迹
    ros::Time takeoff_complete_stamp_;
    // 本次飞行起飞地面高度
    double takeoff_ground_z_ = 0.0;
    bool takeoff_ground_z_valid_ = false;
    Eigen::Vector3d takeoff_origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d landing_start_position_ = Eigen::Vector3d::Zero();
    double takeoff_yaw_ = 0.0;
    double landing_target_z_ = 0.0;
    ros::Time last_arm_attempt_;
    ros::Time landing_detect_start_;
    ros::Time last_disarm_attempt_;
    bool landing_detect_active_ = false;
    bool extended_state_ready_ = false;
    double extended_state_wall_ = 0.0;
    FsmTransitionOptions transition_options_;
    minisnap::StableHover brake_gate_;
    flight_safety::BoxStop braking_;
    flight_safety::BoxGuard box_guard_;
    bool box_fault_latched_ = false;
    double last_box_check_wall_ = -1;
    flight_safety::SmoothDescent descent_;
    flight_safety::Reference last_reference_;
    bool reference_valid_ = false;
    bool brake_for_land_ = false;
    ros::Time brake_start_;
    std::vector<Eigen::Vector3d> reference_accelerations_;
    std::vector<Eigen::Vector3d> reference_jerks_;
    bool touchdown_unload_active_ = false;
    ros::Time touchdown_unload_start_;
    double touchdown_start_thrust_ = 0.0;
    int consecutive_failures_ = 0;
    double start_takeoff_land_time;
    bool enu_frame_, vel_in_body_;
    Eigen::Vector4d hover_pose_;

    int line_cnt_ = 0, number_of_steps_ = 0;
    bool txt_start_pending_ = false;
    bool txt_trajectory_loaded_ = false;
    ros::Time txt_start_stamp_;
    std::vector<std::vector<double>> test_trajectory_;
    std::vector<Eigen::Vector3d> quad_positions_;
    std::vector<Eigen::Vector3d> quad_velocities_;
    std::vector<double> yaws_;

    dynamic_reconfigure::Server<ommpc_controller::fsm_changeConfig> state_change_server_;
    dynamic_reconfigure::Server<ommpc_controller::fsm_changeConfig>::CallbackType
        state_change_cb_type_;

    static const char *stateName(Exec_Traj_State_t state)
    {
        switch (state)
        {
        case HOVER:
            return "HOVER";
        case POLY_TRAJ:
            return "POLY_TRAJ";
        case POINTS:
            return "POINTS";
        case TAKEOFF:
            return "TAKEOFF";
        case LAND:
            return "LAND";
        case BRAKE:
            return "BRAKE";
        default:
            return "UNKNOWN";
        }
    }

    void transitionTo(Exec_Traj_State_t next_state, const std::string &reason)
    {
        if (exec_traj_state_ == next_state)
            return;

        const Exec_Traj_State_t previous_state = exec_traj_state_;
        if (previous_state == POINTS || next_state == POINTS)
        {
            // Every entry reloads the file; every exit discards the old progress.
            test_trajectory_.clear();
            number_of_steps_ = 0;
            line_cnt_ = 0;
            txt_trajectory_loaded_ = false;
            txt_start_stamp_ = ros::Time(0);
        }
        exec_traj_state_ = next_state;
        ROS_WARN("[FSM TRANSITION] %s -> %s | %s", stateName(previous_state), stateName(next_state),
                 reason.c_str());
    }

    geometry_msgs::Vector3Stamped makeVector3Msg(const ros::Time &stamp,
                                                 const Eigen::Vector3d &value,
                                                 const std::string &frame_id = "world") const
    {
        geometry_msgs::Vector3Stamped msg;

        msg.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;

        msg.header.frame_id = frame_id;

        msg.vector.x = value.x();
        msg.vector.y = value.y();
        msg.vector.z = value.z();

        return msg;
    }
    bool canAcceptPolynomial(std::string &reason) const
    {
        const double odom_age = odom_ready_
                                    ? (ros::Time::now() - odom_data_.rcv_stamp).toSec()
                                    : -1.0;
        const bool odom_fresh = odom_ready_ && hover_initialized_ &&
                                std::isfinite(odom_age) && odom_age >= -0.10 &&
                                odom_age <= param_.state_timeout && odom_data_.p.allFinite() &&
                                odom_data_.v.allFinite();
        const bool occupied = trajectory_ready_ || trajectory_data_.exec_traj == 1;
        const bool offboard = state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD;
        const bool txt_pending = param_.use_ref_txt && txt_start_pending_;
        std::ostringstream blocked;
        if (box_fault_latched_)
            blocked << "安全区域故障已锁定; ";
        if (!odom_fresh)
            blocked << "等待有效且新鲜的里程计; ";
        if (!has_takeoff_)
            blocked << "等待起飞完成; ";
        if (exec_traj_state_ != HOVER)
            blocked << "当前不是HOVER状态; ";
        if (!state_.armed)
            blocked << "无人机未解锁; ";
        if (!offboard)
            blocked << "当前不是OFFBOARD模式; ";
        if (!is_command_mode_)
            blocked << "COMMAND未开启; ";
        if (occupied)
            blocked << "已有待执行或正在执行的多项式轨迹; ";
        if (land_trigger_ || takeoff_trigger_)
            blocked << "正在处理起飞或降落请求; ";
        if (txt_pending)
            blocked << "即将进入TXT轨迹执行; ";
        if (consecutive_failures_ != 0)
            blocked << "控制器求解失败恢复中; ";
        const bool allowed = blocked.str().empty();
        std::ostringstream details;
        details << "OMMPC: " << (allowed ? "允许接收多项式轨迹; " : blocked.str())
                << "takeoff_complete=" << has_takeoff_ << ", FSM=" << stateName(exec_traj_state_)
                << ", COMMAND=" << is_command_mode_ << ", mode=" << state_.mode
                << ", armed=" << state_.armed << ", trajectory_pending=" << occupied
                << ", odom_age=" << std::fixed << std::setprecision(3) << odom_age << "s";
        reason = details.str();
        return allowed;
    }

    void publishAdmissionStatus(const ros::WallTimerEvent &)
    {
        std::string reason;
        const bool allowed = canAcceptPolynomial(reason);
        std::ostringstream wire;
        // Wall-clock heartbeat makes latched permissions expire if this node stops.
        wire << (allowed ? "ALLOW|" : "BLOCKED|") << std::setprecision(17)
             << ros::WallTime::now().toSec() << "|" << reason;
        std_msgs::String status;
        status.data = wire.str();
        admission_pub_.publish(status);
        if (!admission_displayed_ || allowed != last_admission_allowed_)
        {
            ROS_WARN("[TRAJ ADMISSION] %s | takeoff_complete=%d, FSM=%s, COMMAND=%d, "
                     "mode=%s, armed=%d, trajectory_pending=%d, box_fault=%d",
                     allowed ? "ALLOW" : "BLOCKED", static_cast<int>(has_takeoff_),
                     stateName(exec_traj_state_), static_cast<int>(is_command_mode_),
                     state_.mode.c_str(), static_cast<int>(state_.armed),
                     static_cast<int>(trajectory_ready_ || trajectory_data_.exec_traj == 1),
                     static_cast<int>(box_fault_latched_));
            admission_displayed_ = true;
            last_admission_allowed_ = allowed;
        }
    }

    void MpcTrajectoryCallback(const traj_utils::PolyTraj::ConstPtr &msg)
    {
        // 接收轨迹的第一层安全门

        std::string admission_reason;
        const bool flight_ready = canAcceptPolynomial(admission_reason);

        /*
         * 非飞行就绪状态下收到的轨迹直接丢弃。
         *
         * 尤其包括：
         *
         * 地面
         * TAKEOFF
         * LAND
         * 未解锁
         * 非OFFBOARD
         * 非command mode
         * 已经有一条轨迹等待执行
         */
        if (!flight_ready)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[TRAJ SAFETY] Trajectory rejected. "
                "has_takeoff=%d, state=%d, armed=%d, "
                "offboard=%d, command_mode=%d, "
                "trajectory_pending=%d",
                static_cast<int>(has_takeoff_), static_cast<int>(exec_traj_state_),
                static_cast<int>(state_.armed),
                static_cast<int>(state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD),
                static_cast<int>(is_command_mode_),
                static_cast<int>(trajectory_ready_ || trajectory_data_.exec_traj == 1));

            return;
        }

        /*
         * 到这里才真正把轨迹交给原来的解析器。
         */
        /*
         * 当前状态机采用单轨迹策略。
         * 正常情况下旧轨迹会在执行结束时被清除；这里额外清理
         * 异常退出或旧版本遗留的非活动轨迹，保证解析器从空队列开始。
         */
        if (!trajectory_data_.traj_queue.empty() || trajectory_data_.exec_traj != 0)
        {
            clearTrajectoryCommand(
                "discarding stale inactive trajectory before accepting a new one");
        }

        if (box_guard_.enabled())
        {
            const std::size_t count = msg->duration.size();
            if (msg->order < 3 || msg->order > 32 || count == 0 ||
                msg->coef_x.size() != count * (msg->order + 1) ||
                msg->coef_y.size() != count * (msg->order + 1) ||
                msg->coef_z.size() != count * (msg->order + 1))
            {
                ROS_ERROR("[BOX] Rejected malformed polynomial coefficient arrays.");
                return;
            }
            for (std::size_t piece = 0; piece < count; ++piece)
            {
                Eigen::MatrixXd coefficients(3, msg->order + 1);
                for (int j = 0; j <= msg->order; ++j)
                {
                    const std::size_t index = piece * (msg->order + 1) + j;
                    coefficients.col(j) =
                        Eigen::Vector3d(msg->coef_x[index], msg->coef_y[index], msg->coef_z[index]);
                }
                if (!box_guard_.polynomialAllowed(coefficients, msg->duration[piece],
                                                  transition_options_.braking_max_acceleration))
                {
                    ROS_ERROR("[BOX] Polynomial rejected: full path or acceleration "
                              "not certified within cuboid/braking limits.");
                    return;
                }
            }
        }
        trajectory_data_.feed_from_traj_utils(msg);

        // 第二层检查：轨迹解析是否正常

        if (trajectory_data_.traj_queue.empty() || trajectory_data_.exec_traj != 1)
        {
            clearTrajectoryCommand("received trajectory is invalid");

            return;
        }

        const ros::Time now = ros::Time::now();

        // 第三层检查：拒绝已经过期的轨迹

        if (trajectory_data_.total_traj_end_time <= now)
        {
            clearTrajectoryCommand("received trajectory already expired");

            return;
        }

        // 第四层检查：
        // 轨迹开始时间不能早于本次起飞完成时间。
        // 这样即使规划器把起飞前的旧轨迹重新发过来，
        // 也不会立即执行。

        if (trajectory_data_.total_traj_start_time < takeoff_complete_stamp_)
        {
            clearTrajectoryCommand("trajectory was generated before takeoff completed");

            return;
        }

        // 通过所有检查后，才允许状态机执行。

        trajectory_ready_ = true;

        ROS_WARN("[TRAJ SAFETY] New trajectory accepted. "
                 "Waiting for execution.");
    }
    void publishDOBDebug()
    {
        DobDebugData data;

        ommpc_controller_.getDobDebugData(data);

        const ros::Time stamp = data.stamp.isZero() ? ros::Time::now() : data.stamp;

        dob_raw_pub_.publish(makeVector3Msg(stamp, data.disturbance_raw));

        dob_filtered_pub_.publish(makeVector3Msg(stamp, data.disturbance_filtered));

        dob_comp_pub_.publish(makeVector3Msg(stamp, data.disturbance_comp));

        dob_dot_pub_.publish(makeVector3Msg(stamp, data.disturbance_dot_filtered));

        dob_nominal_acc_pub_.publish(makeVector3Msg(stamp, data.nominal_acceleration));

        dob_velocity_error_pub_.publish(makeVector3Msg(stamp, data.velocity_error));

        /*
         * status.data 索引定义：
         *
         * [0] enable
         * [1] initialized
         * [2] input_valid
         * [3] delayed_thrustacc [m/s^2]
         * [4] observer_dt [s]
         * [5] ramp
         */
        std_msgs::Float64MultiArray status;

        status.layout.dim.resize(1);
        status.layout.dim[0].label = "enable,initialized,input_valid,"
                                     "delayed_thrustacc,observer_dt,ramp";

        status.layout.dim[0].size = 6;
        status.layout.dim[0].stride = 6;

        status.data.resize(6);

        status.data[0] = data.enabled ? 1.0 : 0.0;

        status.data[1] = data.initialized ? 1.0 : 0.0;

        status.data[2] = data.input_valid ? 1.0 : 0.0;

        status.data[3] = data.delayed_thrustacc;

        status.data[4] = data.observer_dt;

        status.data[5] = data.ramp;

        dob_status_pub_.publish(status);
    }
    void publishReferencePosition()
    {
        Eigen::Vector3d ref_position;

        if (!ommpc_controller_.getCurrentReferencePosition(ref_position))
        {
            return;
        }

        geometry_msgs::PointStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";

        msg.point.x = ref_position.x();
        msg.point.y = ref_position.y();
        msg.point.z = ref_position.z();

        reference_position_pub_.publish(msg);
    }
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        odom_data_.feed(msg, enu_frame_, vel_in_body_);
        odom_ready_ = true;

        /*
         * 第一次收到有效odom时，
         * 自动把当前位置设置为初始悬停点。
         */
        if (!hover_initialized_)
        {
            hover_pose_.head<3>() = odom_data_.p;
            hover_pose_(3) = get_yaw_from_quaternion(odom_data_.q);
            hover_initialized_ = true;
            ROS_INFO("[FSM] Initial hover pose "
                     "initialized from odometry.");
        }
    }
    void ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg)
    {
        extended_state_ = *msg;
        extended_state_ready_ = true;
        extended_state_wall_ = ros::WallTime::now().toSec();
    }

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg) { imu_data_.feed(msg, enu_frame_); }

    void StateCallback(const mavros_msgs::State::ConstPtr &msg) { state_ = *msg; }

    bool trajectoryLandAllowed()
    {
        bool allowed = transition_options_.allow_land_during_trajectory;
        // Read on each request so rosparam changes take effect without restart.
        private_nh_.getParam("landing/allow_during_trajectory", allowed);
        return allowed;
    }

    void stateChangeCallback(ommpc_controller::fsm_changeConfig &config, uint32_t level)
    {
        const bool command_was_enabled = is_command_mode_;
        last_takeoff_enabled_ = takeoff_enabled_;
        last_land_enabled_ = land_enabled_;
        land_enabled_ = config.land_enabled;
        takeoff_enabled_ = config.takeoff_enabled;
        if (last_takeoff_enabled_ == false && takeoff_enabled_ == true)
        {
            takeoff_trigger_ = true;
        }
        if (last_land_enabled_ == false && land_enabled_ == true)
        {
            if ((exec_traj_state_ == POLY_TRAJ || exec_traj_state_ == POINTS) &&
                !trajectoryLandAllowed())
            {
                config.land_enabled = land_enabled_ = false;
                config.command_or_hover = command_was_enabled;
                ROS_WARN("[LAND] Rejected in %s: landing/allow_during_trajectory=false; "
                         "trajectory continues. Select HOVER before LAND, or enable the switch.",
                         stateName(exec_traj_state_));
            }
            else
            {
                land_trigger_ = true;
                takeoff_trigger_ = false;
            }
        }
        // A LAND request owns the reference until touchdown/disarm. Config
        // refreshes cannot re-enable trajectories during BRAKE-for-LAND/LAND.
        if (box_fault_latched_ || land_trigger_ || exec_traj_state_ == LAND ||
            (exec_traj_state_ == BRAKE && brake_for_land_))
            config.command_or_hover = false;
        is_command_mode_ = config.command_or_hover;
        if (!command_was_enabled && is_command_mode_)
        {
            txt_start_pending_ = exec_traj_state_ == HOVER || exec_traj_state_ == TAKEOFF;
            if (has_takeoff_ && exec_traj_state_ == HOVER)
            {
                ROS_WARN("[COMMAND] Enabled in HOVER: trajectory execution "
                         "is allowed and the polynomial receiver is READY. "
                         "If TXT is enabled, the file loads on POINTS entry.");
            }
            else
            {
                ROS_WARN("[COMMAND] Enabled, but trajectory execution is not "
                         "ready (state=%s, takeoff_complete=%d).",
                         stateName(exec_traj_state_), static_cast<int>(has_takeoff_));
            }
        }
        else if (command_was_enabled && !is_command_mode_)
        {
            txt_start_pending_ = false;
            ROS_WARN("[COMMAND] HOVER selected: polynomial/POINTS execution "
                     "is disabled; an active trajectory will be cancelled.");
        }
    }

    void send_cmd(const Controller_Output_t &output)
    {
        mavros_msgs::AttitudeTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.body_rate.x = output.bodyrates(0);
        cmd.body_rate.y = output.bodyrates(1);
        cmd.body_rate.z = output.bodyrates(2);
        if (output.thrust >= 0.9)
            cmd.thrust = 0.9;
        else if (output.thrust <= 0.04)
            cmd.thrust = 0.04;
        else
            cmd.thrust = output.thrust;
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        cmd_pub_.publish(cmd);
        ommpc_controller_.recordAppliedControl(cmd.thrust, output.bodyrates, cmd.header.stamp);
    }
    void clearTrajectoryCommand(const std::string &reason)
    {
        trajectory_data_.traj_queue.clear();

        trajectory_data_.exec_traj = 0;

        trajectory_data_.total_traj_start_time = ros::Time(0);

        trajectory_data_.total_traj_end_time = ros::Time(0);

        trajectory_ready_ = false;

        ROS_WARN("[TRAJ SAFETY] Trajectory cleared: %s", reason.c_str());
    }
    void set_hov_with_odom()
    {
        hover_pose_.head<3>() = odom_data_.p;
        // Once an ARM-success takeoff origin exists, keep the whole flight at
        // the takeoff yaw.  After a completed landing the origin is invalidated
        // and ground standby uses the current yaw again.
        hover_pose_(3) =
            takeoff_ground_z_valid_ ? takeoff_yaw_ : get_yaw_from_quaternion(odom_data_.q);
    }

    void enterBrake(const ros::Time &now, bool for_land,
                    const flight_safety::BoxAssessment *prepared = nullptr)
    {
        if (for_land && ((!has_takeoff_ && !is_airborne_) || !takeoff_ground_z_valid_))
        {
            ROS_WARN("[LAND] Rejected: takeoff is incomplete or its origin is unavailable.");
            return;
        }
        if (exec_traj_state_ == BRAKE)
        {
            // Upgrade an existing stop to LAND without restarting its curve.
            brake_for_land_ = brake_for_land_ || for_land;
            if (brake_for_land_)
                is_command_mode_ = false;
            return;
        }

        flight_safety::Reference seed;
        seed.p = odom_data_.p;
        seed.v = odom_data_.v;
        if (reference_valid_)
            seed = last_reference_;
        if (exec_traj_state_ == POLY_TRAJ && !trajectory_data_.traj_queue.empty())
        {
            const auto &active = trajectory_data_.traj_queue.front();
            const double t = std::max(0.0, std::min((now - active.traj_start_time).toSec(),
                                                    active.traj.getTotalDuration()));
            const Eigen::MatrixXd pvaj = active.traj.getPVAJSC(t);
            seed.p = pvaj.col(0);
            seed.v = pvaj.col(1);
            seed.a = pvaj.col(2);
        }
        flight_safety::Preview preview;
        if (box_guard_.enabled())
        {
            const bool available =
                (exec_traj_state_ != POLY_TRAJ || !trajectory_data_.traj_queue.empty()) &&
                (exec_traj_state_ != POINTS ||
                 (txt_trajectory_loaded_ && !test_trajectory_.empty()));
            const bool completed =
                (exec_traj_state_ == POLY_TRAJ && !trajectory_data_.traj_queue.empty() &&
                 now >= trajectory_data_.traj_queue.front().traj_end_time) ||
                (exec_traj_state_ == POINTS && txt_trajectory_loaded_ &&
                 (now - txt_start_stamp_).toSec() >= (number_of_steps_ - 1) * param_.ref_time_step);
            flight_safety::Reference current;
            if (available && !completed && boxReferenceAt(now, current))
                seed = current;
            if (available)
                preview = [&](double dt, flight_safety::Reference &r) {
                    return boxReferenceAt(now + ros::Duration(dt), r);
                };
            const bool reuse =
                prepared &&
                prepared->reusable(seed, odom_data_.p, odom_data_.v, box_guard_.options(),
                                   transition_options_.braking_max_acceleration,
                                   transition_options_.braking_max_jerk);
            const auto plan =
                reuse ? *prepared
                      : box_guard_.planStop(seed, odom_data_.p, odom_data_.v,
                                            transition_options_.braking_max_acceleration,
                                            transition_options_.braking_max_jerk, preview);
            if (!plan.has_stop || plan.decision == flight_safety::BoxDecision::FAILSAFE)
            {
                latchBoxFault("stop cannot be certified: " + plan.reason);
                return;
            }
            braking_ = plan.stop;
            ROS_WARN("[BRAKE PLAN] %s.",
                     reuse ? "reuse current BOX assessment plan" : "shared planner generated stop");
        }
        else if (!braking_.tryStart(seed, transition_options_.braking_max_acceleration,
                                    transition_options_.braking_max_jerk))
        {
            latchBoxFault("reference stop cannot be certified: " + braking_.reason());
            return;
        }
        brake_start_ = now;
        brake_for_land_ = for_land;
        brake_gate_.reset();
        txt_start_pending_ = false;
        clearTrajectoryCommand("bounded braking started");
        if (for_land)
            is_command_mode_ = false;
        transitionTo(BRAKE, for_land ? "LAND requested: bounded braking first"
                                     : "trajectory cancelled/completed: bounded braking");
        ROS_WARN("[BRAKE] destination=%s, duration=%.3f s, A_bound=%.3f, "
                 "J_bound=%.3f; require %.2f s stable hover after stopping.",
                 for_land ? "LAND" : "HOVER", braking_.duration(), braking_.accelerationBound(),
                 braking_.jerkBound(), transition_options_.hover.duration);
        ROS_WARN("[BRAKE PLAN] %s (continuous box/A/J certification).",
                 braking_.followsPath() ? "follow trajectory with decreasing phase speed"
                                        : "generic bounded stop / legacy when BOX disabled");
    }

    void updateBrakeState(const ros::Time &now)
    {
        if (box_fault_latched_)
            return;
        const double elapsed = std::max(0.0, (now - brake_start_).toSec());
        const auto reference = braking_.at(elapsed);
        const double position_error = (odom_data_.p - reference.p).norm();
        const double wall = ros::WallTime::now().toSec();
        const ros::Time measurement_stamp = odom_data_.msg.header.stamp.isZero()
                                                ? odom_data_.rcv_stamp
                                                : odom_data_.msg.header.stamp;
        const double state_age = (now - measurement_stamp).toSec();
        const bool permitted = elapsed >= braking_.duration() &&
                               state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD &&
                               state_.armed && state_age >= -0.1 &&
                               state_age <= transition_options_.hover.max_state_age &&
                               position_error <= transition_options_.braking_position_tolerance;
        brake_gate_.update(odom_data_.p, odom_data_.v, measurement_stamp.toSec(), wall, permitted);
        if (!brake_gate_.stable(wall))
        {
            if (elapsed >= braking_.duration())
                ROS_WARN_THROTTLE(1.0,
                                  "[BRAKE] Holding stop point: error=%.3f m, "
                                  "speed=%.3f m/s; %s.",
                                  position_error, odom_data_.v.norm(),
                                  brake_gate_.diagnostic(wall).c_str());
            return;
        }

        hover_pose_.head<3>() = reference.p;
        hover_pose_(3) = takeoff_yaw_;
        ROS_WARN("[BRAKE] Stable hover confirmed for %.2f s; speed=%.3f m/s.",
                 transition_options_.hover.duration, odom_data_.v.norm());
        if (brake_for_land_)
            beginLanding(now);
        else
            transitionTo(HOVER, "bounded braking and stable hover completed");
    }

    bool executeSmoothReference(const ros::Time &now, Controller_Output_t &output)
    {
        if (box_fault_latched_)
            return false;
        const bool landing = exec_traj_state_ == LAND;
        const double elapsed = std::max(0.0, landing ? now.toSec() - start_takeoff_land_time
                                                     : (now - brake_start_).toSec());
        for (int i = 0; i <= nstep; ++i)
        {
            const double t = elapsed + i * param_.step_T;
            const auto reference = landing ? descent_.at(t) : braking_.at(t);
            quad_positions_[i] = reference.p;
            quad_velocities_[i] = reference.v;
            reference_accelerations_[i] = reference.a;
            reference_jerks_[i] = landing ? descent_.jerk(t) : braking_.jerk(t);
            yaws_[i] = takeoff_yaw_;
        }
        ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_,
                                           takeoff_yaw_, yaws_, &reference_accelerations_,
                                           &reference_jerks_);
        return execMPCAndPublish(output);
    }

    bool beginLanding(const ros::Time &now)
    {
        if (!has_takeoff_ && !is_airborne_)
        {
            ROS_WARN("[FSM] LAND rejected: takeoff has not completed.");
            land_trigger_ = false;
            return false;
        }
        if (!takeoff_ground_z_valid_)
        {
            ROS_ERROR("[FSM] LAND rejected: takeoff origin is invalid.");
            land_trigger_ = false;
            return false;
        }

        if (exec_traj_state_ != BRAKE || !brake_for_land_ ||
            !brake_gate_.stable(ros::WallTime::now().toSec()))
        {
            ROS_WARN("[LAND] Deferred: bounded braking and stable hover are required.");
            return false;
        }
        start_takeoff_land_time = now.toSec();
        // Continue from the stop reference, not a new instantaneous XY lock.
        landing_start_position_ = hover_pose_.head<3>();
        landing_target_z_ = takeoff_ground_z_ + param_.landing_target_offset;
        Eigen::Vector3d landing_end = landing_start_position_;
        landing_end.z() = landing_target_z_;
        if (!box_guard_.segmentAllowed(landing_start_position_, landing_end))
        {
            brake_for_land_ = false;
            transitionTo(HOVER, "LAND reference outside flight box; remain at stop point");
            ROS_ERROR(
                "[BOX] LAND rejected: vertical descent/relative target outside inset cuboid.");
            return false;
        }
        descent_.start(landing_start_position_, landing_target_z_, param_.landing_speed,
                       transition_options_.landing_max_acceleration,
                       transition_options_.landing_max_jerk);
        hover_pose_.head<3>() = landing_start_position_;
        hover_pose_(3) = takeoff_yaw_;
        is_command_mode_ = false;
        txt_start_pending_ = false;
        clearTrajectoryCommand("landing started");
        landing_detect_active_ = false;
        touchdown_unload_active_ = false;
        touchdown_start_thrust_ = 0.0;
        last_disarm_attempt_ = ros::Time(0);
        transitionTo(LAND, "bounded braking and stable hover confirmed; smooth descent started");
        if (odom_data_.p.z() - takeoff_ground_z_ <= param_.dob_landing_disable_height)
        {
            dob_airborne_active_ = false;
            ommpc_controller_.resetDisturbanceObserver();
            ROS_WARN("[DOB] Disabled immediately on near-ground LAND entry.");
        }
        land_trigger_ = false;

        ROS_INFO("[LAND] Locked xy=[%.3f %.3f], start_z=%.3f, "
                 "target_z=%.3f (takeoff initial z %+.3f).",
                 landing_start_position_(0), landing_start_position_(1), landing_start_position_(2),
                 landing_target_z_, param_.landing_target_offset);
        return true;
    }

    double get_yaw_from_quaternion(const Eigen::Quaterniond &q)
    {
        return atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    }

    void updateLandingContact(const ros::Time &now, Controller_Output_t &output)
    {
        const double wall = ros::WallTime::now().toSec();
        const bool on_ground =
            extended_state_ready_ && wall >= extended_state_wall_ &&
            wall - extended_state_wall_ <= transition_options_.extended_state_timeout &&
            extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
        const double height = odom_data_.p.z() - takeoff_ground_z_;
        const bool candidate = on_ground && height < param_.touchdown_height &&
                               std::abs(odom_data_.v.z()) < param_.touchdown_max_vz &&
                               odom_data_.v.head<2>().norm() < param_.touchdown_max_vxy;

        // Height/low speed alone can describe a hover: require fresh PX4 contact.
        if (touchdown_unload_active_ && !on_ground)
        {
            touchdown_unload_active_ = false;
            landing_detect_active_ = false;
            ROS_WARN("[LAND] ON_GROUND lost/stale; thrust unloading cancelled.");
        }
        if (!touchdown_unload_active_)
        {
            if (!candidate)
                landing_detect_active_ = false;
            else
            {
                if (!landing_detect_active_)
                {
                    landing_detect_active_ = true;
                    landing_detect_start_ = now;
                    ROS_WARN(
                        "[LAND] Fresh ON_GROUND and touchdown thresholds met; confirming contact.");
                }
                if ((now - landing_detect_start_).toSec() >= param_.touchdown_confirm_time)
                {
                    touchdown_unload_active_ = true;
                    touchdown_unload_start_ = now;
                    touchdown_start_thrust_ = output.thrust;
                    ROS_WARN("[LAND] Contact confirmed; unloading thrust from %.3f to %.3f.",
                             touchdown_start_thrust_, param_.touchdown_thrust);
                }
            }
        }
        if (!touchdown_unload_active_)
            return;

        const double elapsed = std::max(0.0, (now - touchdown_unload_start_).toSec());
        const double alpha = param_.thrust_unload_time <= 0.0
                                 ? 1.0
                                 : std::min(1.0, elapsed / param_.thrust_unload_time);
        output.thrust = (1.0 - alpha) * touchdown_start_thrust_ + alpha * param_.touchdown_thrust;
        output.bodyrates *= 1.0 - alpha;
        // Do not disarm before completing the smooth unloading stage.
        if (!on_ground || elapsed < param_.thrust_unload_time ||
            (!last_disarm_attempt_.isZero() &&
             (now - last_disarm_attempt_).toSec() < param_.service_retry_interval))
            return;
        last_disarm_attempt_ = now;
        if (!toggle_arm_disarm(false))
        {
            ROS_WARN("[LAND] DISARM rejected; retrying in %.2f s.", param_.service_retry_interval);
            return;
        }

        has_land_ = true;
        has_takeoff_ = false;
        is_airborne_ = false;
        dob_airborne_active_ = false;
        ommpc_controller_.resetDisturbanceObserver();
        landing_detect_active_ = false;
        touchdown_unload_active_ = false;
        takeoff_ground_z_valid_ = false;
        brake_for_land_ = false;
        reference_valid_ = false;
        last_arm_attempt_ = ros::Time(0);
        last_disarm_attempt_ = ros::Time(0);
        set_hov_with_odom();
        transitionTo(HOVER, "PX4 ON_GROUND confirmed, unloading completed and DISARM succeeded");
    }

    bool toggle_arm_disarm(bool arm)
    {
        mavros_msgs::CommandBool arm_cmd;

        arm_cmd.request.value = arm;

        // 1. ROS service本身是否调用成功
        if (!arming_client_srv_.call(arm_cmd))
        {
            ROS_ERROR("%s service call failed!", arm ? "ARM" : "DISARM");

            return false;
        }

        // 2. PX4是否接受命令
        if (!arm_cmd.response.success)
        {
            ROS_ERROR("%s rejected by PX4! "
                      "MAV_RESULT=%u",
                      arm ? "ARM" : "DISARM", static_cast<unsigned int>(arm_cmd.response.result));

            return false;
        }

        ROS_INFO("%s accepted by PX4. "
                 "MAV_RESULT=%u",
                 arm ? "ARM" : "DISARM", static_cast<unsigned int>(arm_cmd.response.result));

        return true;
    }

    bool execMPCAndPublish(Controller_Output_t &u)
    {
        bool result = ommpc_controller_.execMPC(odom_data_, u);

        if (result)
        {
            reference_valid_ = ommpc_controller_.getCurrentReferenceState(
                last_reference_.p, last_reference_.v, last_reference_.a);
            publishReferencePosition();
        }

        return result;
    }

    // Update compensation before generating this cycle's MPC references.
    void updateDob()
    {
        // Update translational disturbance observer
        // 必须放在 setTrajectoryReference()、
        // setTextReference() 和 setHoverReference() 之前，
        // 因为这些函数需要使用本周期最新的扰动估计。
        // DOB activation logic
        // 目标：
        // 1. 地面等待时不运行 DOB
        // 2. TAKEOFF 真正离地后启动 DOB
        // 3. TAKEOFF / HOVER / trajectory 持续运行 DOB
        // 4. LAND 高空阶段继续运行 DOB
        // 5. LAND 距离地面 <= landing_disable_height 时关闭 DOB

        const bool offboard_and_armed =
            state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed;

        /*
         * 判断这一周期是不是刚刚检测到离地。
         *
         * 用于防止startDisturbanceObserver()之后
         * 同一个周期又立即update一次。
         */
        bool dob_just_started = false;

        // 1. 检测真正离地
        if (offboard_and_armed && (exec_traj_state_ == TAKEOFF || exec_traj_state_ == BRAKE) &&
            takeoff_ground_z_valid_ && !is_airborne_)
        {
            const double delta_z = odom_data_.p(2) - takeoff_ground_z_;

            const double vz = odom_data_.v(2);

            const bool liftoff_detected =
                delta_z > param_.takeoff_liftoff_height && vz > param_.takeoff_liftoff_vz;

            if (liftoff_detected)
            {
                is_airborne_ = true;

                ROS_WARN("[FSM] LIFTOFF detected. "
                         "dz=%.3f, vz=%.3f",
                         delta_z, vz);
            }
        }
        const bool dob_activation_allowed =
            exec_traj_state_ != LAND ||
            (takeoff_ground_z_valid_ &&
             odom_data_.p(2) - takeoff_ground_z_ > param_.dob_landing_disable_height);
        if (param_.dob_enable && is_airborne_ && !dob_airborne_active_ && dob_activation_allowed &&
            offboard_and_armed)
        {
            dob_airborne_active_ = true;
            ommpc_controller_.startDisturbanceObserver(odom_data_);
            dob_just_started = true;
            ROS_WARN("[DOB] Observer activated.");
        }

        // 2. 已经离地以后持续运行
        // 2. 判断当前DOB是否允许运行
        // LAND阶段：
        //   高于配置阈值：DOB继续运行
        //   低于或等于配置阈值：DOB关闭并清零补偿

        double height_above_ground = std::numeric_limits<double>::infinity();

        /*
         * 默认允许DOB继续运行。
         *
         * 只有LAND并且已经接近地面时才禁止。
         */
        bool dob_near_ground_disable = false;
        const bool landing_or_entering_land = exec_traj_state_ == LAND;

        if (landing_or_entering_land)
        {
            /*
             * LAND期间必须有有效地面高度，
             * 否则为了安全直接关闭DOB。
             */
            if (takeoff_ground_z_valid_)
            {
                height_above_ground = odom_data_.p(2) - takeoff_ground_z_;

                if (height_above_ground <= param_.dob_landing_disable_height)
                {
                    dob_near_ground_disable = true;
                }
            }
            else
            {
                /*
                 * 无法确定实际离地高度时，
                 * LAND阶段不继续做DOB补偿。
                 */
                dob_near_ground_disable = true;
            }
        }

        // 3. DOB正常运行条件
        // 注意：
        // 这里已经不再禁止整个LAND状态。
        const bool dob_should_run = param_.dob_enable && offboard_and_armed &&
                                    dob_airborne_active_ && !dob_near_ground_disable;

        if (dob_should_run && !dob_just_started)
        {
            ommpc_controller_.updateDisturbanceObserver(odom_data_);
        }

        // 4. DOB关闭条件
        const bool dob_should_stop =
            dob_airborne_active_ &&
            (!param_.dob_enable || !offboard_and_armed || dob_near_ground_disable);

        if (dob_should_stop)
        {
            /*
             * 一旦LAND过程中低于配置高度阈值，
             * 本次降落不再重新开启DOB。
             */
            dob_airborne_active_ = false;

            ommpc_controller_.resetDisturbanceObserver();

            if (exec_traj_state_ == LAND &&
                height_above_ground < std::numeric_limits<double>::infinity())
            {
                ROS_WARN("[DOB] Disabled near ground. "
                         "height=%.3f m, threshold=%.3f m",
                         height_above_ground, param_.dob_landing_disable_height);
            }
            else
            {
                ROS_WARN("[DOB] DOB deactivated.");
            }
        }
    }

    void latchBoxFault(const std::string &reason)
    {
        box_fault_latched_ = true;
        is_command_mode_ = false;
        takeoff_trigger_ = land_trigger_ = false;
        txt_start_pending_ = false;
        clearTrajectoryCommand("flight-box fault");
        dob_airborne_active_ = false;
        ommpc_controller_.resetDisturbanceObserver();
        ROS_ERROR("[BOX FAILSAFE] %s. Output stopped and latched; no disarm, no automatic LAND. "
                  "Verify configured PX4 OFFBOARD-loss action. Restart only on ground.",
                  reason.c_str());
    }

    bool boxReferenceAt(const ros::Time &stamp, flight_safety::Reference &r) const
    {
        r = flight_safety::Reference();
        if (exec_traj_state_ == POLY_TRAJ && !trajectory_data_.traj_queue.empty())
        {
            const auto &active = trajectory_data_.traj_queue.front();
            const double t = std::max(0.0, std::min((stamp - active.traj_start_time).toSec(),
                                                    active.traj.getTotalDuration()));
            const Eigen::MatrixXd pvaj = active.traj.getPVAJSC(t);
            r.p = pvaj.col(0);
            r.v = pvaj.col(1);
            r.a = pvaj.col(2);
            if ((stamp - active.traj_start_time).toSec() >= active.traj.getTotalDuration())
            {
                r.v.setZero();
                r.a.setZero();
            }
        }
        else if (exec_traj_state_ == BRAKE)
            r = braking_.at(std::max(0.0, (stamp - brake_start_).toSec()));
        else if (exec_traj_state_ == LAND)
            r = descent_.at(std::max(0.0, stamp.toSec() - start_takeoff_land_time));
        else if (exec_traj_state_ == TAKEOFF)
        {
            const double t = std::max(0.0, stamp.toSec() - start_takeoff_land_time);
            r.p = takeoff_origin_;
            r.p.z() =
                std::min(param_.takeoff_altitude, takeoff_origin_.z() + param_.takeoff_speed * t);
            if (r.p.z() < param_.takeoff_altitude)
                r.v.z() = param_.takeoff_speed;
        }
        else if (exec_traj_state_ == POINTS && txt_trajectory_loaded_ && !test_trajectory_.empty())
            return txtReferenceAt(stamp, r);
        else
            r.p = hover_pose_.head<3>();
        return true;
    }

    void checkBoxSafety(const ros::Time &now)
    {
        if (!box_guard_.enabled() || box_fault_latched_ ||
            (!has_takeoff_ && !is_airborne_ && exec_traj_state_ != TAKEOFF))
            return;
        if (!box_guard_.options().contains(odom_data_.p, false))
        {
            latchBoxFault("measured position outside world cuboid");
            return;
        }
        const double wall = ros::WallTime::now().toSec();
        if (last_box_check_wall_ >= 0 && wall - last_box_check_wall_ >= 0 &&
            wall - last_box_check_wall_ < box_guard_.options().check_interval)
            return;
        last_box_check_wall_ = wall;
        if (touchdown_unload_active_)
            return;
        flight_safety::Reference reference;
        boxReferenceAt(now, reference);
        auto preview = [&](double dt, flight_safety::Reference &future) {
            return boxReferenceAt(now + ros::Duration(dt), future);
        };
        const auto assessment = box_guard_.assess(reference, odom_data_.p, odom_data_.v,
                                                  transition_options_.braking_max_acceleration,
                                                  transition_options_.braking_max_jerk, preview,
                                                  exec_traj_state_ == BRAKE ? &braking_ : nullptr,
                                                  exec_traj_state_ == BRAKE ?
                                                      std::max(0.0, (now - brake_start_).toSec()) : 0.0);
        if (assessment.decision == flight_safety::BoxDecision::FAILSAFE)
            latchBoxFault(assessment.reason);
        else if (assessment.decision == flight_safety::BoxDecision::BRAKE &&
                 exec_traj_state_ != BRAKE)
        {
            const bool for_land = exec_traj_state_ == LAND;
            is_command_mode_ = false;
            ROS_WARN("[BOX PREVENTIVE BRAKE] %s; destination=%s.", assessment.reason.c_str(),
                     for_land ? "LAND" : "HOVER");
            enterBrake(now, for_land, &assessment);
        }
    }

    void processTriggers(const ros::Time &now_time)
    {
        // LAND interrupts the trajectory, not the motion reference: stop first.
        if (land_trigger_)
        {
            land_trigger_ = false;
            // The policy was checked on the request edge. An accepted LAND is not cancelled.
            if (exec_traj_state_ == LAND || (exec_traj_state_ == BRAKE && brake_for_land_))
                ROS_WARN(
                    "[LAND] Already braking for LAND or descending; duplicate trigger ignored.");
            else
            {
                takeoff_trigger_ = false;
                enterBrake(now_time, true);
            }
        }
        if (takeoff_trigger_ && exec_traj_state_ != HOVER)
        {
            ROS_WARN("[FSM] TAKEOFF rejected: current state=%s.", stateName(exec_traj_state_));
            takeoff_trigger_ = false;
        }
    }

    void execFSMCallback(const ros::TimerEvent &e)
    {
        if (!ros::ok() || box_fault_latched_)
        {
            return;
        }
        exec_timer_.stop();

        Controller_Output_t u;
        u.bodyrates.setZero();
        u.thrust = 0.0;
        bool ret = false;
        ros::Time now_time = ros::Time::now();
        if (!odom_ready_ || !hover_initialized_)
        {
            ROS_WARN_THROTTLE(1.0, "[FSM] Waiting for valid odometry.");
            if (ros::ok())
            {
                exec_timer_.start();
            }

            return;
        }
        const double odom_age = (now_time - odom_data_.rcv_stamp).toSec();

        if (odom_age > param_.state_timeout || odom_age < -0.10)
        {
            ROS_ERROR_THROTTLE(0.5, "[FSM] Odom timeout %.3f s", odom_age);
            Controller_Output_t safe_u = last_u_;
            send_cmd(safe_u);
            if (ros::ok())
            {
                exec_timer_.start();
            }
            return;
        }
        updateDob();

        publishDOBDebug();

        processTriggers(now_time);
        checkBoxSafety(now_time);
        if (box_fault_latched_)
            return;
        const Exec_Traj_State_t state_before_control = exec_traj_state_;
        switch (exec_traj_state_)
        {
        case HOVER:
        {
            if (takeoff_trigger_ && !has_takeoff_ && !is_airborne_)
            {
                if (state_.mode != mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    ROS_WARN_THROTTLE(1.0, "[FSM] TAKEOFF pending: waiting for OFFBOARD mode.");
                    u.bodyrates.setZero();
                    u.thrust = 0.0;
                    ret = true;
                    break;
                }

                if (!std::isfinite(odom_data_.p(2)) || !std::isfinite(param_.takeoff_altitude) ||
                    param_.takeoff_speed <= 0.0 ||
                    param_.takeoff_altitude <= odom_data_.p(2) + param_.takeoff_tolerance)
                {
                    ROS_ERROR("[FSM] Invalid takeoff setup: initial_z=%.3f, "
                              "absolute_target_z=%.3f, speed=%.3f, tolerance=%.3f.",
                              odom_data_.p(2), param_.takeoff_altitude, param_.takeoff_speed,
                              param_.takeoff_tolerance);
                    takeoff_trigger_ = false;
                    u.bodyrates.setZero();
                    u.thrust = 0.0;
                    ret = true;
                    break;
                }

                Eigen::Vector3d top = odom_data_.p, bottom = odom_data_.p;
                top.z() = param_.takeoff_altitude;
                bottom.z() += param_.landing_target_offset;
                if (!box_guard_.segmentAllowed(odom_data_.p, top) ||
                    !box_guard_.segmentAllowed(odom_data_.p, bottom))
                {
                    takeoff_trigger_ = false;
                    ROS_ERROR("[BOX] TAKEOFF rejected before ARM: origin/targets outside cuboid.");
                    ret = true;
                    break;
                }
                bool arm_succeeded = state_.armed;
                if (!arm_succeeded)
                {
                    const bool can_retry =
                        last_arm_attempt_.isZero() ||
                        (now_time - last_arm_attempt_).toSec() >= param_.service_retry_interval;
                    if (!can_retry)
                    {
                        u.bodyrates.setZero();
                        u.thrust = 0.0;
                        ret = true;
                        break;
                    }
                    last_arm_attempt_ = now_time;
                    arm_succeeded = toggle_arm_disarm(true);
                }

                if (arm_succeeded)
                {
                    /*
                     * 与统一状态机一致：只有ARM成功以后，才锁存本次
                     * 飞行的起飞原点、地面高度和固定偏航角。
                     */
                    const ros::Time takeoff_now = ros::Time::now();
                    start_takeoff_land_time = takeoff_now.toSec();
                    takeoff_origin_ = odom_data_.p;
                    takeoff_yaw_ = get_yaw_from_quaternion(odom_data_.q);
                    hover_pose_.head<3>() = takeoff_origin_;
                    hover_pose_(3) = takeoff_yaw_;
                    takeoff_ground_z_ = takeoff_origin_(2);
                    takeoff_ground_z_valid_ = true;
                    is_airborne_ = false;
                    landing_detect_active_ = false;
                    touchdown_unload_active_ = false;
                    clearTrajectoryCommand("takeoff started");
                    dob_airborne_active_ = false;
                    ommpc_controller_.resetDisturbanceObserver();
                    transitionTo(TAKEOFF, "ARM succeeded; constant-speed takeoff started");
                    takeoff_trigger_ = false;
                    ommpc_controller_.setHoverReference(hover_pose_);
                    ret = execMPCAndPublish(u);

                    ROS_INFO("[TAKEOFF] Locked xyz0=[%.3f %.3f %.3f], "
                             "yaw=%.2f deg, "
                             "absolute target_z=%.3f.",
                             takeoff_origin_(0), takeoff_origin_(1), takeoff_origin_(2),
                             takeoff_yaw_ * 180.0 / M_PI, param_.takeoff_altitude);
                }
                else
                {
                    // Keep the trigger pending and retry ARM at the configured
                    // service interval instead of entering numerical-failure
                    // recovery.
                    u.bodyrates.setZero();
                    u.thrust = 0.0;
                    ret = true;
                }
            }
            else if (takeoff_trigger_)
            {
                ROS_WARN("[FSM] TAKEOFF rejected: takeoff already completed.");
                takeoff_trigger_ = false;
                if (state_.armed && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    ommpc_controller_.setHoverReference(hover_pose_);
                    ret = execMPCAndPublish(u);
                }
                else
                {
                    u.bodyrates.setZero();
                    u.thrust = 0.0;
                    ret = true;
                }
            }
            else if (consecutive_failures_ == 0 &&
                     now_time >= trajectory_data_.total_traj_start_time &&
                     now_time <= trajectory_data_.total_traj_end_time &&
                     trajectory_data_.exec_traj == 1 && (!trajectory_data_.traj_queue.empty()) &&
                     is_command_mode_ && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD &&
                     has_takeoff_ && state_.armed && state_.armed && trajectory_ready_)
            {
                // same as the below
                set_hov_with_odom();
                oneTraj_Data_t *traj_info = &trajectory_data_.traj_queue.front();
                traj_info = &trajectory_data_.traj_queue.front();
                trajectory_data_.total_traj_start_time = traj_info->traj_start_time;

                double traj_time = (now_time - traj_info->traj_start_time).toSec();
                /*
                 * 这条轨迹开始消费。
                 *
                 * trajectory_ready_只用于
                 * HOVER -> POLY_TRAJ 的一次性许可。
                 */
                trajectory_ready_ = false;
                ommpc_controller_.setTrajectoryReference(traj_info->traj, traj_time, hover_pose_(3),
                                                         traj_info->yaw_traj, odom_data_);
                ret = execMPCAndPublish(u);
                if (ret)
                {
                    trajectory_ready_ = false;
                    txt_start_pending_ = false;
                    transitionTo(POLY_TRAJ,
                                 "COMMAND enabled and a READY polynomial trajectory was activated");
                }
            }
            else if (consecutive_failures_ == 0 && has_takeoff_ && state_.armed &&
                     param_.use_ref_txt && txt_start_pending_ && !trajectory_ready_ &&
                     is_command_mode_ && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                txt_start_pending_ = false;
                transitionTo(POINTS,
                             "COMMAND requested TXT execution; file will be loaded in POINTS");
                // Keep the previous hover reference for this transition cycle.
                // The POINTS handler loads the file before issuing any TXT reference.
                ommpc_controller_.setHoverReference(hover_pose_);
                ret = execMPCAndPublish(u);
            }
            else if (state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true)
            {
                ommpc_controller_.setHoverReference(hover_pose_);
                ret = execMPCAndPublish(u);
            }
            else
            {
                // Not in offboard: skip MPC but keep publishing so PX4 can enter offboard
                u.bodyrates.setZero();
                u.thrust = 0.0;
                ret = true;
            }
        }
        break;

        case POLY_TRAJ:
        {
            if (!has_takeoff_ || !state_.armed ||
                state_.mode != mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                clearTrajectoryCommand("OFFBOARD/arming permission lost");
                set_hov_with_odom();
                transitionTo(HOVER, "polynomial execution authority lost");
                u.bodyrates.setZero();
                u.thrust = 0.0;
                ret = true;
                break;
            }
            const bool finished = trajectory_data_.exec_traj != 1 ||
                                  trajectory_data_.traj_queue.size() != 1 ||
                                  now_time < trajectory_data_.total_traj_start_time ||
                                  now_time > trajectory_data_.total_traj_end_time;
            if (!is_command_mode_ || finished)
            {
                ROS_WARN("[POLY] %s; braking before stable HOVER.",
                         !is_command_mode_ ? "HOVER requested" : "execution completed/invalidated");
                enterBrake(now_time, false);
                updateBrakeState(now_time);
                ret = executeSmoothReference(now_time, u);
                break;
            }
            auto &trajectory = trajectory_data_.traj_queue.front();
            const double elapsed = (now_time - trajectory.traj_start_time).toSec();
            ommpc_controller_.setTrajectoryReference(trajectory.traj, elapsed, takeoff_yaw_,
                                                     trajectory.yaw_traj, odom_data_);
            ret = execMPCAndPublish(u);
        }
        break;

        case POINTS:
        {
            const bool points_execution_safe =
                has_takeoff_ && state_.armed &&
                state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && is_command_mode_;
            if (!points_execution_safe)
            {
                if (has_takeoff_ && state_.armed &&
                    state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    enterBrake(now_time, false);
                    updateBrakeState(now_time);
                    ret = executeSmoothReference(now_time, u);
                }
                else
                {
                    set_hov_with_odom();
                    transitionTo(HOVER, "TXT execution authority lost");
                    u.bodyrates.setZero();
                    u.thrust = 0.0;
                    ret = true;
                }
                break;
            }
            if (!txt_trajectory_loaded_)
            {
                ROS_WARN("[TXT] Entered POINTS: loading the configured file.");
                if (!readDataFromFile())
                {
                    set_hov_with_odom();
                    transitionTo(HOVER, "TXT file load/validation failed");
                    ommpc_controller_.setHoverReference(hover_pose_);
                    ret = execMPCAndPublish(u);
                    break;
                }
                // Start the trajectory clock after file I/O, not at node startup.
                bool txt_inside = true;
                for (const auto &row : test_trajectory_)
                    txt_inside =
                        txt_inside &&
                        (!box_guard_.enabled() ||
                         box_guard_.options().contains(Eigen::Vector3d(row[0], row[1], row[2])));
                if (!txt_inside)
                {
                    set_hov_with_odom();
                    transitionTo(HOVER, "TXT reference outside flight box");
                    ROS_ERROR("[BOX] TXT rejected: position sample outside inset cuboid.");
                    ommpc_controller_.setHoverReference(hover_pose_);
                    ret = execMPCAndPublish(u);
                    break;
                }
                txt_start_stamp_ = ros::Time::now();
                txt_trajectory_loaded_ = true;
                ROS_WARN("[TXT] Execution started from sample 0: %d samples, "
                         "sample_dt=%.4f s, duration=%.3f s.",
                         number_of_steps_, param_.ref_time_step,
                         (number_of_steps_ - 1) * param_.ref_time_step);
            }

            const ros::Time txt_now = ros::Time::now();
            const double elapsed = std::max(0.0, (txt_now - txt_start_stamp_).toSec());
            const double duration = (number_of_steps_ - 1) * param_.ref_time_step;
            if (elapsed >= duration)
            {
                const auto &last = test_trajectory_.back();
                last_reference_.p = Eigen::Vector3d(last[0], last[1], last[2]);
                last_reference_.v = Eigen::Vector3d(last[3], last[4], last[5]);
                last_reference_.a.setZero();
                reference_valid_ = true;
                enterBrake(txt_now, false);
                updateBrakeState(txt_now);
                ret = executeSmoothReference(txt_now, u);
                ROS_WARN("[TXT] Complete; braking before HOVER. Select HOVER then "
                         "COMMAND to reload and execute from sample 0 again.");
                break;
            }

            get_txt_des(txt_now);
            const double reference_yaw =
                takeoff_ground_z_valid_ ? takeoff_yaw_ : get_yaw_from_quaternion(odom_data_.q);
            ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_,
                                               reference_yaw, yaws_, &reference_accelerations_,
                                               &reference_jerks_);
            ret = execMPCAndPublish(u);
        }
        break;

        case TAKEOFF:
        {
            takeoff_trigger_ = false;
            const double now_sec = ros::Time::now().toSec();
            for (int i = 0; i < nstep + 1; ++i)
            {
                const double elapsed =
                    std::max(0.0, now_sec - start_takeoff_land_time + i * param_.step_T);
                const double desired_z = std::min(
                    takeoff_origin_(2) + param_.takeoff_speed * elapsed, param_.takeoff_altitude);
                quad_positions_[i] =
                    Eigen::Vector3d(takeoff_origin_(0), takeoff_origin_(1), desired_z);
                quad_velocities_[i] = Eigen::Vector3d(
                    0.0, 0.0, desired_z < param_.takeoff_altitude ? param_.takeoff_speed : 0.0);
                yaws_[i] = takeoff_yaw_;
            }
            ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_,
                                               takeoff_yaw_, yaws_);
            ret = execMPCAndPublish(u);
            if (odom_data_.p(2) >= param_.takeoff_altitude - param_.takeoff_tolerance)
            {
                transitionTo(HOVER, "absolute takeoff altitude reached");
                hover_pose_.head<3>() = odom_data_.p;
                hover_pose_(2) = param_.takeoff_altitude;
                hover_pose_(3) = takeoff_yaw_;
                has_takeoff_ = true;
                has_land_ = false;
                takeoff_complete_stamp_ = ros::Time::now();
                /*
                 * 起飞完成时仍然没有合法轨迹。
                 * 必须重新收到一条新轨迹才会变成true。
                 */
                trajectory_ready_ = false;

                ROS_INFO("[HOVER] Hover xyz=[%.3f %.3f %.3f], "
                         "fixed yaw=%.2f deg. "
                         "Trajectory receiver enabled.",
                         hover_pose_(0), hover_pose_(1), hover_pose_(2),
                         takeoff_yaw_ * 180.0 / M_PI);
                if (is_command_mode_)
                {
                    ROS_WARN("[COMMAND] Already enabled: polynomial "
                             "execution is now allowed and the trajectory "
                             "receiver is READY.");
                }
            }
        }
        break;

        case BRAKE:
        {
            updateBrakeState(now_time);
            ret = executeSmoothReference(now_time, u);
            if (ret && exec_traj_state_ == LAND)
                updateLandingContact(now_time, u);
        }
        break;

        case LAND:
        {
            ret = executeSmoothReference(now_time, u);
            if (ret)
                updateLandingContact(now_time, u);
        }
        break;

        default:
        {
            ret = false;
            transitionTo(HOVER, "unknown state recovery");
            ROS_ERROR("[MPCctrl] Unknown exec_traj_state_; recovered to HOVER.");
        }

        break;
        }

        if (box_fault_latched_)
            return;
        if (ret)
        {
            consecutive_failures_ = 0;
            send_cmd(u);
            last_u_ = u;
        }
        else
        {
            consecutive_failures_++;

            ROS_ERROR("[MPCctrl] Numerical error! "
                      "failed_state=%d",
                      static_cast<int>(state_before_control));

            // 轨迹状态失败：立即取消轨迹
            if (state_before_control == BRAKE || state_before_control == LAND)
            {
                // Never turn a LAND-owned stop/descent into an abrupt HOVER reset.
                ROS_ERROR("[MPCctrl] Solver failed during %s; reference state retained. "
                          "No new actuator command published this cycle.",
                          stateName(state_before_control));
            }
            else if (state_before_control == POLY_TRAJ || state_before_control == POINTS)
            {
                clearTrajectoryCommand("MPC failure");
                set_hov_with_odom();
                transitionTo(HOVER, "MPC numerical failure during trajectory");
            }

            // TAKEOFF失败：
            // 如果已经离地，进入空中HOVER，
            // 但不要谎称takeoff已经成功。
            else if (state_before_control == TAKEOFF)
            {
                set_hov_with_odom();
                transitionTo(HOVER, "MPC numerical failure during TAKEOFF");
                if (is_airborne_)
                {
                    has_land_ = false;
                    ROS_ERROR("[MPCctrl] "
                              "TAKEOFF MPC failed "
                              "after liftoff. "
                              "Switch to AIR HOVER.");
                }
                else
                {
                    ROS_ERROR("[MPCctrl] "
                              "TAKEOFF MPC failed "
                              "before liftoff.");
                }
            }

            else
            {
                set_hov_with_odom();
                transitionTo(HOVER, "MPC numerical failure recovery");
            }
        }

        if (state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true &&
            exec_traj_state_ != TAKEOFF && exec_traj_state_ != LAND && exec_traj_state_ != BRAKE)
        {
            ommpc_controller_.estimateThrustModel(imu_data_.a);
        }
        if (ros::ok())
        {
            exec_timer_.start();
        }
    }

    bool readDataFromFile()
    {
        const std::string traj_path =
            ros::package::getPath("ommpc_controller") + param_.ref_filename;
        std::ifstream file(traj_path.c_str());
        if (!file.is_open())
        {
            ROS_ERROR("[TXT] Cannot open file: %s", traj_path.c_str());
            return false;
        }

        std::vector<std::vector<double>> loaded;
        std::string line;
        size_t file_line = 0;
        while (std::getline(file, line))
        {
            ++file_line;
            const size_t comment = line.find('#');
            if (comment != std::string::npos)
                line.erase(comment);
            std::istringstream linestream(line);
            std::vector<double> row;
            std::string token;
            while (linestream >> token)
            {
                try
                {
                    size_t parsed = 0;
                    const double value = std::stod(token, &parsed);
                    if (parsed != token.size() || !std::isfinite(value))
                    {
                        ROS_ERROR("[TXT] Invalid numeric value at %s:%zu.", traj_path.c_str(),
                                  file_line);
                        return false;
                    }
                    row.push_back(value);
                }
                catch (const std::exception &error)
                {
                    ROS_ERROR("[TXT] Invalid numeric token at %s:%zu: %s.", traj_path.c_str(),
                              file_line, error.what());
                    return false;
                }
            }
            if (!linestream.eof() || (!row.empty() && row.size() < 7))
            {
                ROS_ERROR("[TXT] Invalid row at %s:%zu; expected at least "
                          "7 numeric columns: x y z vx vy vz yaw.",
                          traj_path.c_str(), file_line);
                return false;
            }
            if (!row.empty())
                loaded.push_back(row);
        }
        if (file.bad() || loaded.empty() ||
            loaded.size() > static_cast<size_t>(std::numeric_limits<int>::max()))
        {
            ROS_ERROR("[TXT] Empty, unreadable or oversized file: %s", traj_path.c_str());
            return false;
        }

        test_trajectory_.swap(loaded);
        number_of_steps_ = static_cast<int>(test_trajectory_.size());
        line_cnt_ = 0;
        ROS_INFO("[TXT] Loaded and validated %d samples from %s.", number_of_steps_,
                 traj_path.c_str());
        return true;
    }

    // Shared discrete TXT reference semantics for control and box preview.
    // Derivatives are velocity finite differences, not analytic polynomial derivatives.
    bool txtReferenceAt(const ros::Time &stamp, flight_safety::Reference &r,
                        Eigen::Vector3d *jerk = nullptr) const
    {
        r = flight_safety::Reference();
        if (jerk)
            jerk->setZero();
        if (test_trajectory_.empty() || param_.ref_time_step <= 0)
            return false;
        const double elapsed = std::max(0.0, (stamp - txt_start_stamp_).toSec());
        const double duration = (test_trajectory_.size() - 1) * param_.ref_time_step;
        const std::size_t index = std::min(
            test_trajectory_.size() - 1, static_cast<std::size_t>(elapsed / param_.ref_time_step));
        const auto &row = test_trajectory_[index];
        r.p = Eigen::Vector3d(row[0], row[1], row[2]);
        if (elapsed >= duration)
            return true; // Terminal preview is stationary.
        r.v = Eigen::Vector3d(row[3], row[4], row[5]);
        auto velocity = [&](std::size_t i) {
            const auto &data = test_trajectory_[i];
            return Eigen::Vector3d(data[3], data[4], data[5]);
        };
        if (index + 1 < test_trajectory_.size())
        {
            r.a = (velocity(index + 1) - r.v) / param_.ref_time_step;
            if (jerk && index + 2 < test_trajectory_.size())
                *jerk = ((velocity(index + 2) - velocity(index + 1)) / param_.ref_time_step - r.a) /
                        param_.ref_time_step;
        }
        return true;
    }

    void get_txt_des(const ros::Time &now)
    {
        const double elapsed = std::max(0.0, (now - txt_start_stamp_).toSec());
        line_cnt_ = static_cast<int>(std::min(std::floor(elapsed / param_.ref_time_step),
                                              static_cast<double>(number_of_steps_ - 1)));
        for (int i = 0; i <= nstep; ++i)
        {
            const double sample_time = elapsed + i * param_.step_T;
            const int index =
                static_cast<int>(std::min(std::floor(sample_time / param_.ref_time_step),
                                          static_cast<double>(number_of_steps_ - 1)));
            const auto &row = test_trajectory_[index];
            flight_safety::Reference reference;
            txtReferenceAt(txt_start_stamp_ + ros::Duration(sample_time), reference,
                           &reference_jerks_[i]);
            quad_positions_[i] = reference.p;
            quad_velocities_[i] = reference.v;
            reference_accelerations_[i] = reference.a;
            yaws_[i] = row[6];
        }
    }

    template <typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
    {
        if (nh.getParam(name, val))
        {
            // pass
        }
        else
        {
            ROS_ERROR_STREAM("Read param_: " << name << " failed.");
            ROS_BREAK();
        }
    };

  public:
    OMMPC_EXAMPLE(/* args */){};
    ~OMMPC_EXAMPLE(){};
    void init(ros::NodeHandle &nh)
    {
        const auto box_options = flight_safety::loadBoxOptions(nh);
        box_guard_.configure(box_options);
        braking_.configure(box_options);
        transition_options_.load(nh);
        brake_gate_ = minisnap::StableHover(transition_options_.hover);
        reference_accelerations_.resize(nstep + 1);
        reference_jerks_.resize(nstep + 1);
        enu_frame_ = true;
        // for real world flight, vel_in_body should be set to false!
        vel_in_body_ = true;
        exec_traj_state_ = HOVER;

        cmd_pub_ =
            nh.advertise<mavros_msgs::AttitudeTarget>("/uav1/mavros/setpoint_raw/attitude", 10);
        reference_position_pub_ =
            nh.advertise<geometry_msgs::PointStamped>("/uav1/reference_point", 10);

        dob_raw_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
            "/ommpc_controller/dob/disturbance_raw", 20);
        dob_filtered_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
            "/ommpc_controller/dob/disturbance_filtered", 20);
        dob_comp_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
            "/ommpc_controller/dob/disturbance_comp", 20);
        dob_dot_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
            "/ommpc_controller/dob/disturbance_dot", 20);
        dob_nominal_acc_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
            "/ommpc_controller/dob/nominal_acceleration", 20);
        dob_velocity_error_pub_ =
            nh.advertise<geometry_msgs::Vector3Stamped>("/ommpc_controller/dob/velocity_error", 20);
        dob_status_pub_ =
            nh.advertise<std_msgs::Float64MultiArray>("/ommpc_controller/dob/status", 20);
        set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
        arming_client_srv_ = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/uav1/mavros/local_position/odom", 10,
                                                     &OMMPC_EXAMPLE::OdomCallback, this);
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/uav1/mavros/imu/data", 10,
                                                  &OMMPC_EXAMPLE::IMUCallback, this);
        state_sub_ = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10,
                                                      &OMMPC_EXAMPLE::StateCallback, this);
        mpc_traj_sub_ = nh.subscribe<traj_utils::PolyTraj>(
            "/drone_0_planning/trajectory", 1, &OMMPC_EXAMPLE::MpcTrajectoryCallback, this,
            ros::TransportHints().tcpNoDelay());
        extended_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>(
            "/uav1/mavros/extended_state", 10, &OMMPC_EXAMPLE::ExtendedStateCallback, this);

        int trials = 0;
        while (ros::ok() && !state_.connected)
        {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            if (trials++ > 5)
                ROS_ERROR("Unable to connnect to PX4!!!");
        }

        state_change_cb_type_ = boost::bind(&OMMPC_EXAMPLE::stateChangeCallback, this, _1, _2);
        state_change_server_.setCallback(state_change_cb_type_);

        read_essential_param(nh, "control/state_timeout", param_.state_timeout);
        read_essential_param(nh, "mavros/service_retry_interval", param_.service_retry_interval);
        read_essential_param(nh, "takeoff/altitude", param_.takeoff_altitude);
        read_essential_param(nh, "takeoff/speed", param_.takeoff_speed);
        read_essential_param(nh, "takeoff/tolerance", param_.takeoff_tolerance);
        read_essential_param(nh, "takeoff/liftoff_height", param_.takeoff_liftoff_height);
        read_essential_param(nh, "takeoff/liftoff_vz", param_.takeoff_liftoff_vz);
        read_essential_param(nh, "landing/speed", param_.landing_speed);
        read_essential_param(nh, "landing/target_offset", param_.landing_target_offset);
        read_essential_param(nh, "landing/touchdown_height", param_.touchdown_height);
        read_essential_param(nh, "landing/touchdown_max_vz", param_.touchdown_max_vz);
        read_essential_param(nh, "landing/touchdown_max_vxy", param_.touchdown_max_vxy);
        read_essential_param(nh, "landing/touchdown_confirm_time", param_.touchdown_confirm_time);
        read_essential_param(nh, "landing/thrust_unload_time", param_.thrust_unload_time);
        read_essential_param(nh, "landing/touchdown_thrust", param_.touchdown_thrust);
        read_essential_param(nh, "ref_txt/enable", param_.use_ref_txt);
        read_essential_param(nh, "ref_txt/time_step", param_.ref_time_step);
        read_essential_param(nh, "ref_txt/ref_filename", param_.ref_filename);
        read_essential_param(nh, "hover_percentage", param_.hover_percent);
        read_essential_param(nh, "MPC_params/Q_pos_xy", param_.Q_pos_xy);
        read_essential_param(nh, "MPC_params/Q_pos_z", param_.Q_pos_z);
        read_essential_param(nh, "MPC_params/Q_attitude_rp", param_.Q_attitude_rp);
        read_essential_param(nh, "MPC_params/Q_attitude_yaw", param_.Q_attitude_yaw);
        read_essential_param(nh, "MPC_params/Q_velocity", param_.Q_velocity);
        read_essential_param(nh, "MPC_params/R_thrust", param_.R_thrust);
        read_essential_param(nh, "MPC_params/R_pitchroll", param_.R_pitchroll);
        read_essential_param(nh, "MPC_params/R_yaw", param_.R_yaw);
        read_essential_param(nh, "MPC_params/min_thrust", param_.min_thrust);
        read_essential_param(nh, "MPC_params/max_thrust", param_.max_thrust);
        read_essential_param(nh, "MPC_params/max_bodyrate_xy", param_.max_bodyrate_xy);
        read_essential_param(nh, "MPC_params/max_bodyrate_z", param_.max_bodyrate_z);
        read_essential_param(nh, "MPC_params/state_cost_exponential",
                             param_.state_cost_exponential);
        read_essential_param(nh, "MPC_params/input_cost_exponential",
                             param_.input_cost_exponential);
        read_essential_param(nh, "MPC_params/step_T", param_.step_T);
        read_essential_param(nh, "use_fix_yaw", param_.use_fix_yaw);
        read_essential_param(nh, "use_trajectory_ending_pos", param_.use_trajectory_ending_pos);
        read_essential_param(nh, "DOB/enable", param_.dob_enable);
        read_essential_param(nh, "DOB/l1", param_.dob_l1);
        read_essential_param(nh, "DOB/l2", param_.dob_l2);
        read_essential_param(nh, "DOB/lpf_tau", param_.dob_lpf_tau);
        read_essential_param(nh, "DOB/derivative_lpf_tau", param_.dob_derivative_lpf_tau);
        read_essential_param(nh, "DOB/max_acc", param_.dob_max_acc);
        read_essential_param(nh, "DOB/max_jerk", param_.dob_max_jerk);
        read_essential_param(nh, "DOB/ramp_time", param_.dob_ramp_time);
        read_essential_param(nh, "DOB/input_delay", param_.dob_input_delay);
        read_essential_param(nh, "DOB/landing_disable_height", param_.dob_landing_disable_height);
        const bool invalid_fsm_parameters =
            (param_.use_ref_txt &&
             (!std::isfinite(param_.ref_time_step) || param_.ref_time_step <= 0.0)) ||
            param_.state_timeout <= 0.0 || param_.service_retry_interval < 0.0 ||
            param_.takeoff_speed <= 0.0 || param_.takeoff_tolerance < 0.0 ||
            param_.takeoff_liftoff_height < 0.0 || param_.takeoff_liftoff_vz < 0.0 ||
            param_.landing_speed <= 0.0 || param_.landing_target_offset >= 0.0 ||
            param_.touchdown_height < 0.0 || param_.touchdown_max_vz < 0.0 ||
            param_.touchdown_max_vxy < 0.0 || param_.touchdown_confirm_time < 0.0 ||
            param_.thrust_unload_time < 0.0 || param_.touchdown_thrust < 0.0 ||
            param_.touchdown_thrust > 1.0;
        if (invalid_fsm_parameters)
        {
            ROS_FATAL("[FSM] Invalid takeoff/landing parameters in config/params.yaml.");
            ros::shutdown();
            return;
        }
        if (param_.use_ref_txt)
        {
            ROS_INFO("[TXT] Enabled; file loading is deferred until entering "
                     "POINTS after takeoff and COMMAND selection.");
        }

        quad_positions_.clear();
        quad_positions_.resize(nstep + 1);
        quad_velocities_.clear();
        quad_velocities_.resize(nstep + 1);
        yaws_.clear();
        yaws_.resize(nstep + 1);

        ommpc_controller_.init(param_);

        ROS_INFO("[FSM] OMMPC controller started in HOVER: absolute "
                 "takeoff_target_z=%.3f m, takeoff_speed=%.3f m/s, "
                 "liftoff=(dz>%.3f m AND world_vz>%.3f m/s), "
                 "landing_target=takeoff_initial_z%+.3f m.",
                 param_.takeoff_altitude, param_.takeoff_speed, param_.takeoff_liftoff_height,
                 param_.takeoff_liftoff_vz, param_.landing_target_offset);

        // hover_pose_ << 0.0, 0.0, 0.0, 0.0;

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &OMMPC_EXAMPLE::execFSMCallback, this);
        admission_pub_ = nh.advertise<std_msgs::String>(
            "/traj_tracking_controller/trajectory_admission_status", 1, true);
        admission_timer_ = nh.createWallTimer(
            ros::WallDuration(0.2), &OMMPC_EXAMPLE::publishAdmissionStatus, this);
        ROS_INFO("[TRAJ ADMISSION] Publishing at 5 Hz: "
                 "/traj_tracking_controller/trajectory_admission_status (std_msgs/String).");
    }
};

#ifndef OMMPC_FSM_NO_MAIN
int main(int argc, char **argv)
{

    ros::init(argc, argv, "ommpc_controller_example_node");
    ros::NodeHandle nh("~");

    OMMPC_EXAMPLE ommpc_example;
    try
    {
        ommpc_example.init(nh);
    }
    catch (const std::exception &error)
    {
        ROS_FATAL("[FSM] Initialization failed: %s", error.what());
        return 1;
    }

    ros::spin();

    return 0;
}
#endif
