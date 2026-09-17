#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <ommpc_controller/fsm_changeConfig.h>

#include "box_flight_safety_ros.hpp"
#include "console_colors.h"
#include "controller_config.hpp"
#include "flight_safety.hpp"
#include "state_provider.hpp"
#include "trajectory_manager.hpp"
#include "ude_controller.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
double clamp(double value, double minimum, double maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

double yawFromQuaternion(const Eigen::Quaterniond &q)
{
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

std::string joinTopic(const std::string &prefix, const std::string &suffix)
{
    if (prefix.empty())
        return suffix;
    if (prefix[prefix.size() - 1] == '/')
        return prefix + (suffix[0] == '/' ? suffix.substr(1) : suffix);
    return prefix + (suffix[0] == '/' ? suffix : "/" + suffix);
}
} // namespace

class TrajectoryTrackingController
{
  public:
    enum FlightState
    {
        HOVER = 0,
        TAKEOFF = 1,
        POLY_TRAJ = 2,
        LAND = 3,
        BRAKE = 4
    };

    TrajectoryTrackingController() : nh_(), private_nh_("~")
    {
        config_.load(private_nh_);
        const auto box_options = flight_safety::loadBoxOptions(private_nh_);
        box_guard_.configure(box_options);
        braking_.configure(box_options);
        hover_gate_.reset(new minisnap::StableHover(config_.activation.hover));
        brake_gate_.reset(new minisnap::StableHover(config_.activation.hover));
        state_provider_.reset(new StateProvider(nh_, config_));
        trajectory_.reset(new TrajectoryManager(nh_, config_.trajectory_topic, config_.activation));
        ude_.reset(new UdeController(config_));

        const std::string mavros = config_.mavros_namespace;
        mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>(
            joinTopic(mavros, "state"), 10, &TrajectoryTrackingController::mavrosStateCallback,
            this);
        extended_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>(
            joinTopic(mavros, "extended_state"), 10,
            &TrajectoryTrackingController::extendedStateCallback, this);
        attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
            joinTopic(mavros, "setpoint_raw/attitude"), 10);
        admission_pub_ =
            private_nh_.advertise<std_msgs::String>("trajectory_admission_status", 1, true);
        reference_point_pub_ =
            nh_.advertise<geometry_msgs::PointStamped>("/uav1/reference_point", 1);
        admission_timer_ = nh_.createWallTimer(
            ros::WallDuration(.2), &TrajectoryTrackingController::publishAdmissionStatus, this);
        arming_client_ =
            nh_.serviceClient<mavros_msgs::CommandBool>(joinTopic(mavros, "cmd/arming"));

        std::string vehicle_prefix = mavros;
        const std::string suffix = "/mavros";
        if (vehicle_prefix.size() >= suffix.size() &&
            vehicle_prefix.compare(vehicle_prefix.size() - suffix.size(), suffix.size(), suffix) ==
                0)
            vehicle_prefix.erase(vehicle_prefix.size() - suffix.size());
        ude_state_bag_topic_ = joinTopic(vehicle_prefix, "sunray/debug/ude_and_state");

        dynamic_server_.reset(new DynamicServer(private_nh_));
        DynamicServer::CallbackType callback =
            boost::bind(&TrajectoryTrackingController::dynamicCallback, this, _1, _2);
        dynamic_server_->setCallback(callback);

        kp_ << config_.kp_x, config_.kp_y, config_.kp_z;
        kd_ << config_.kd_x, config_.kd_y, config_.kd_z;
        state_ = HOVER;
        trajectory_->setAcceptEnabled(false);
        openBag();
        if (config_.bag_enabled)
        {
            bag_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
                config_.odom_topic, 100, &TrajectoryTrackingController::bagOdomCallback, this);
            bag_vision_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
                config_.vision_pose_topic, 100,
                &TrajectoryTrackingController::bagVisionPoseCallback, this);
        }

        ROS_INFO("[FSM] Unified controller started in state HOVER. "
                 "state_source=%s, "
                 "takeoff_target_z=%.3f m (absolute world Z), "
                 "landing_target=initial_z%+.3f m.",
                 state_provider_->sourceName().c_str(), config_.takeoff_altitude,
                 config_.landing_target_offset);
    }

    ~TrajectoryTrackingController() { closeBag(); }

    bool valid() const
    {
        return state_provider_ && state_provider_->configured() && config_.control_rate > 0.0 &&
               config_.mass > 0.0 && config_.gravity > 0.0 && config_.hover_thrust > 0.0 &&
               config_.max_tilt_deg > 0.0 && config_.max_tilt_deg < 89.0 &&
               config_.min_vertical_force_ratio > 0.0 &&
               config_.max_vertical_force_ratio >= config_.min_vertical_force_ratio;
    }

    void run()
    {
        ros::Rate rate(config_.control_rate);
        while (ros::ok())
        {
            ros::spinOnce();
            const ros::Time now = ros::Time::now();
            const double nominal_dt = 1.0 / config_.control_rate;
            double dt = last_loop_time_.isZero() ? nominal_dt : (now - last_loop_time_).toSec();
            if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1)
                dt = nominal_dt;
            last_loop_time_ = now;
            controlStep(now, dt);
            rate.sleep();
        }
    }

  private:
    friend struct FsmSafetyTestAccess;
    typedef dynamic_reconfigure::Server<ommpc_controller::fsm_changeConfig> DynamicServer;

    void mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        mavros_state_ = *msg;
        mavros_state_received_ = true;
        if (bag_open_)
            try
            {
                bag_.write(joinTopic(config_.mavros_namespace, "state"), ros::Time::now(), *msg);
            }
            catch (const rosbag::BagException &e)
            {
                ROS_ERROR_THROTTLE(1.0, "[BAG] State write failed: %s", e.what());
            }
    }

    void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg)
    {
        extended_state_ = *msg;
        extended_state_received_ = true;
        extended_state_wall_ = ros::WallTime::now().toSec();
        if (bag_open_)
            try
            {
                bag_.write(joinTopic(config_.mavros_namespace, "extended_state"), ros::Time::now(),
                           *msg);
            }
            catch (const rosbag::BagException &e)
            {
                ROS_ERROR_THROTTLE(1.0, "[BAG] Extended-state write failed: %s", e.what());
            }
    }

    void bagOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (!bag_open_)
            return;
        const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        try
        {
            bag_.write(config_.odom_topic, stamp, *msg);
        }
        catch (const rosbag::BagException &error)
        {
            ROS_ERROR_THROTTLE(1.0, "[BAG] Odom write failed: %s.", error.what());
        }
    }

    void bagVisionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!bag_open_)
            return;
        const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        try
        {
            bag_.write(config_.vision_pose_topic, stamp, *msg);
        }
        catch (const rosbag::BagException &error)
        {
            ROS_ERROR_THROTTLE(1.0, "[BAG] Vision-pose write failed: %s.", error.what());
        }
    }

    bool trajectoryLandAllowed()
    {
        bool allowed = config_.allow_land_during_trajectory;
        private_nh_.getParam("landing/allow_during_trajectory", allowed);
        return allowed;
    }

    void dynamicCallback(ommpc_controller::fsm_changeConfig &configuration, uint32_t)
    {
        const bool command_was_enabled = command_mode_;
        if (!last_takeoff_switch_ && configuration.takeoff_enabled)
            pending_takeoff_ = true;
        if (!last_land_switch_ && configuration.land_enabled)
        {
            if (state_ == POLY_TRAJ && !trajectoryLandAllowed())
            {
                configuration.land_enabled = false;
                // LAND scripts may include command_or_hover=false in the same update.
                // Reject the whole LAND interruption, not just its trigger.
                configuration.command_or_hover = command_was_enabled;
                ROS_WARN("[LAND] Rejected in POLY_TRAJ: landing/allow_during_trajectory=false; "
                         "trajectory continues. Select HOVER before LAND, or enable the switch.");
            }
            else
            {
                pending_land_ = true;
                pending_takeoff_ = false;
            }
        }

        // LAND/BRAKE-for-LAND owns the reference until disarm. Later config
        // updates cannot accidentally re-enable trajectory execution.
        if (safety_latched_ || pending_land_ || state_ == LAND ||
            (state_ == BRAKE && brake_for_land_))
            configuration.command_or_hover = false;
        command_mode_ = configuration.command_or_hover;
        if (!command_was_enabled && command_mode_)
        {
            if (has_takeoff_ && state_ == HOVER)
            {
                if (trajectory_->ready())
                {
                    ROS_WARN("[COMMAND] Enabled in HOVER: polynomial execution is "
                             "allowed and a READY trajectory is waiting. The FSM "
                             "will validate stable hover and start position before activation.");
                }
                else
                {
                    ROS_WARN(
                        "[COMMAND] Enabled in HOVER: polynomial execution is "
                        "allowed, but reception still requires stable hover, OFFBOARD and ARMED. "
                        "Wait for [HOVER READY], then send a polynomial trajectory.");
                }
            }
            else
            {
                ROS_WARN("[COMMAND] Enabled, but polynomial execution is not "
                         "ready yet (state=%s, takeoff_complete=%d). The receiver "
                         "requires TAKEOFF -> HOVER and sustained stable hover.",
                         stateName(), static_cast<int>(has_takeoff_));
            }
        }
        else if (command_was_enabled && !command_mode_)
        {
            ROS_WARN("[COMMAND] HOVER selected: polynomial trajectory execution is "
                     "disabled; an active trajectory will be smoothly braked.");
        }
        last_takeoff_switch_ = configuration.takeoff_enabled;
        last_land_switch_ = configuration.land_enabled;
    }

    void controlStep(const ros::Time &now, double dt)
    {
        const double wall_now = ros::WallTime::now().toSec();
        trajectory_->expireReady(wall_now);
        if (safety_latched_)
        {
            setAdmission(false, "safety stop latched; inspect flight mode/state, then restart "
                                "controller on ground");
            ROS_ERROR_THROTTLE(1.0, "[SAFETY] Attitude output stopped; no automatic OFFBOARD "
                                    "re-entry. Restart on ground after inspection.");
            return;
        }
        if (!state_provider_->ready())
        {
            hover_gate_->reset();
            brake_gate_->reset();
            receiver_was_enabled_ = false;
            setAdmission(false, "state source not ready");
            ROS_WARN_THROTTLE(1.0, "[FSM] Waiting for %s state data.",
                              state_provider_->sourceName().c_str());
            publishStandby(now, 0.0);
            return;
        }

        const VehicleState vehicle = state_provider_->state();
        const double state_age = (now - vehicle.stamp).toSec();
        if (!vehicle.valid || state_age > config_.state_timeout || state_age < -0.1)
        {
            hover_gate_->reset();
            brake_gate_->reset();
            receiver_was_enabled_ = false;
            std::ostringstream reason;
            reason << "invalid/stale state, age=" << state_age
                   << "s, limit=" << config_.state_timeout << "s";
            setAdmission(false, reason.str());
            if (state_failure_wall_ < 0)
                state_failure_wall_ = wall_now;
            if (wall_now - state_failure_wall_ > config_.state_command_hold_time)
            {
                safety_latched_ = true;
                trajectory_->clear();
                ude_->reset();
                ROS_ERROR("[SAFETY] State outage exceeded %.3f s; stopped setpoints to allow "
                          "configured PX4 OFFBOARD-loss failsafe (no disarm).",
                          config_.state_command_hold_time);
                return;
            }
            ROS_ERROR_THROTTLE(1.0,
                               "[FSM] State data stale (age %.3f s); holding "
                               "last attitude command.",
                               state_age);
            republishLastCommand(now);
            return;
        }
        state_failure_wall_ = -1;
        if ((is_airborne_ || has_takeoff_) &&
            (!mavros_state_received_ || mavros_state_.mode != "OFFBOARD" || !mavros_state_.armed))
        {
            safety_latched_ = true;
            trajectory_->clear();
            ude_->reset();
            setAdmission(false, "OFFBOARD/ARMED authority lost; output stopped");
            ROS_ERROR("[SAFETY] OFFBOARD/ARMED lost in flight; stopped output and latched, no "
                      "automatic control reacquisition.");
            return;
        }

        if (!hold_initialized_)
        {
            hover_position_ = vehicle.position;
            fixed_yaw_ = yawFromQuaternion(vehicle.attitude);
            hold_initialized_ = true;
            ROS_INFO("[FSM] Initial standby pose: xyz=[%.3f %.3f %.3f], "
                     "yaw=%.2f deg.",
                     hover_position_.x(), hover_position_.y(), hover_position_.z(),
                     fixed_yaw_ * 180.0 / M_PI);
        }

        updateModeSafety();
        processTriggers(vehicle, now);
        updateLiftoffAndUde(vehicle, now);
        checkBoxSafety(vehicle, now);
        if (safety_latched_)
            return;
        const bool hover_permitted = has_takeoff_ && state_ == HOVER && mavros_state_received_ &&
                                     mavros_state_.mode == "OFFBOARD" && mavros_state_.armed &&
                                     state_age <= config_.activation.hover.max_state_age;
        hover_gate_->update(vehicle.position, vehicle.velocity, vehicle.stamp.toSec(), wall_now,
                            hover_permitted);
        updateTrajectoryPermission();

        Eigen::Vector3d reference_position = hover_position_;
        Eigen::Vector3d reference_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d reference_acceleration = Eigen::Vector3d::Zero();

        switch (state_)
        {
        case TAKEOFF:
            updateTakeoff(vehicle, now, reference_position, reference_velocity);
            break;
        case POLY_TRAJ:
            updatePolynomial(vehicle, now, reference_position, reference_velocity,
                             reference_acceleration);
            break;
        case LAND:
            updateLanding(vehicle, now, reference_position, reference_velocity,
                          reference_acceleration);
            break;
        case BRAKE:
            updateBrake(vehicle, now, reference_position, reference_velocity,
                        reference_acceleration);
            break;
        case HOVER:
        default:
            updateHover(vehicle, now);
            reference_position = hover_position_;
            break;
        }

        publishReferencePoint(now, reference_position);
        if (safety_latched_)
            return;
        if (state_ == HOVER && !has_takeoff_ && !is_airborne_)
        {
            // Never leave a hover-thrust command queued while disarmed.
            publishStandby(now, 0.0);
            reference_valid_ = false;
            return;
        }
        updateLiftoffAndUde(vehicle, now);

        mavros_msgs::AttitudeTarget command = calculateCommand(
            vehicle, reference_position, reference_velocity, reference_acceleration, now, dt);

        if (touchdown_unload_active_)
        {
            const double alpha =
                config_.thrust_unload_time <= 0.0
                    ? 1.0
                    : clamp((now - touchdown_unload_start_).toSec() / config_.thrust_unload_time,
                            0.0, 1.0);
            const Eigen::Quaterniond level(Eigen::AngleAxisd(fixed_yaw_, Eigen::Vector3d::UnitZ()));
            command.orientation.w = level.w();
            command.orientation.x = level.x();
            command.orientation.y = level.y();
            command.orientation.z = level.z();
            command.thrust =
                (1.0 - alpha) * touchdown_start_thrust_ + alpha * config_.touchdown_thrust;
        }

        publishAndRecordAttitude(command, now);
        last_command_ = command;
        last_command_valid_ = true;
        last_reference_.p = reference_position;
        last_reference_.v = reference_velocity;
        last_reference_.a = reference_acceleration;
        reference_valid_ = true;
        recordBag(now, reference_position, reference_velocity, reference_acceleration);
        printStatus(now, vehicle, reference_position, command);
    }

    void latchBoxFault(const std::string &reason)
    {
        safety_latched_ = true;
        trajectory_->clear();
        command_mode_ = false;
        pending_takeoff_ = pending_land_ = false;
        ude_->reset();
        setAdmission(false, "flight-box safety fault: " + reason);
        ROS_ERROR("[BOX FAILSAFE] %s. Output stopped and latched; no disarm, no automatic LAND. "
                  "Configured PX4 OFFBOARD-loss failsafe must be verified. Restart only on ground.",
                  reason.c_str());
    }

    bool boxReferenceAt(const ros::Time &stamp, flight_safety::Reference &r) const
    {
        r = flight_safety::Reference();
        if (state_ == POLY_TRAJ)
        {
            if (!trajectory_->active())
                return false;
            if (!trajectory_->evaluate(stamp, r.p, r.v, r.a))
            {
                // Extend a completed polynomial as a stationary endpoint.
                r.v.setZero();
                r.a.setZero();
            }
            return true;
        }
        if (state_ == BRAKE)
            r = braking_.at(std::max(0.0, (stamp - brake_start_).toSec()));
        else if (state_ == LAND)
            r = descent_.at(std::max(0.0, (stamp - landing_start_).toSec()));
        else if (state_ == TAKEOFF)
        {
            const double elapsed = std::max(0.0, (stamp - takeoff_start_).toSec());
            r.p = takeoff_origin_;
            r.p.z() = std::min(config_.takeoff_altitude,
                               takeoff_origin_.z() + config_.takeoff_speed * elapsed);
            if (r.p.z() < config_.takeoff_altitude)
                r.v.z() = config_.takeoff_speed;
        }
        else
            r.p = hover_position_;
        return true;
    }

    void checkBoxSafety(const VehicleState &vehicle, const ros::Time &now)
    {
        if (!box_guard_.enabled() || safety_latched_ ||
            (!has_takeoff_ && !is_airborne_ && state_ != TAKEOFF))
            return;
        if (!box_guard_.options().contains(vehicle.position, false))
        {
            latchBoxFault("measured position outside configured world cuboid");
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
        boxReferenceAt(now, reference); // evaluate() also supplies the endpoint on completion.
        auto preview = [&](double dt, flight_safety::Reference &future) {
            return boxReferenceAt(now + ros::Duration(dt), future);
        };
        const auto assessment =
            box_guard_.assess(reference, vehicle.position, vehicle.velocity,
                              config_.braking_max_acceleration, config_.braking_max_jerk, preview,
                              state_ == BRAKE ? &braking_ : nullptr,
                              state_ == BRAKE ? std::max(0.0, (now - brake_start_).toSec()) : 0.0);
        if (assessment.decision == flight_safety::BoxDecision::FAILSAFE)
            latchBoxFault(assessment.reason);
        else if (assessment.decision == flight_safety::BoxDecision::BRAKE && state_ != BRAKE)
        {
            const bool for_land = state_ == LAND;
            command_mode_ = false;
            ROS_WARN("[BOX PREVENTIVE BRAKE] %s; destination=%s.", assessment.reason.c_str(),
                     for_land ? "LAND" : "HOVER");
            enterBrake(vehicle, now, for_land, &assessment);
        }
    }

    void processTriggers(const VehicleState &vehicle, const ros::Time &now)
    {
        if (pending_land_)
        {
            pending_land_ = false;
            // An accepted request owns braking/landing even if the switch is changed later.
            if ((!has_takeoff_ && !is_airborne_) || state_ == LAND ||
                (state_ == BRAKE && brake_for_land_))
            {
                ROS_WARN("[FSM] LAND rejected: the vehicle has not completed "
                         "takeoff or is already landing.");
            }
            else
            {
                pending_takeoff_ = false;
                enterBrake(vehicle, now, true);
            }
        }

        if (!pending_takeoff_)
            return;
        if (has_takeoff_ || is_airborne_ || state_ != HOVER)
        {
            pending_takeoff_ = false;
            ROS_WARN("[FSM] TAKEOFF rejected: state=%s, has_takeoff=%d.", stateName(),
                     static_cast<int>(has_takeoff_));
            return;
        }
        if (!mavros_state_received_ || !mavros_state_.connected)
        {
            ROS_WARN_THROTTLE(1.0, "[FSM] TAKEOFF pending: waiting for MAVROS.");
            return;
        }
        if (mavros_state_.mode != "OFFBOARD")
        {
            ROS_WARN_THROTTLE(1.0, "[FSM] TAKEOFF pending: switch PX4 to OFFBOARD "
                                   "with traj_tracking_fsm.sh first.");
            return;
        }

        Eigen::Vector3d top = vehicle.position, bottom = vehicle.position;
        top.z() = config_.takeoff_altitude;
        bottom.z() += config_.landing_target_offset;
        if (!box_guard_.segmentAllowed(vehicle.position, top) ||
            !box_guard_.segmentAllowed(vehicle.position, bottom))
        {
            pending_takeoff_ = false;
            ROS_ERROR("[BOX] TAKEOFF rejected before ARM: origin, absolute target or relative LAND "
                      "target outside cuboid.");
            return;
        }

        if (mavros_state_.armed)
        {
            pending_takeoff_ = false;
            startTakeoff(vehicle, now);
            return;
        }

        if (!last_service_request_.isZero() &&
            (now - last_service_request_).toSec() < config_.service_retry_interval)
            return;

        last_service_request_ = now;
        mavros_msgs::CommandBool arm;
        arm.request.value = true;
        if (arming_client_.call(arm) && arm.response.success)
        {
            // As in OMMPC, the definitive takeoff origin is sampled only after
            // the ARM service succeeds, not when the node starts.
            pending_takeoff_ = false;
            startTakeoff(state_provider_->state(), ros::Time::now());
        }
        else
        {
            ROS_WARN("[FSM] ARM request rejected; retrying in %.1f s.",
                     config_.service_retry_interval);
        }
    }

    void startTakeoff(const VehicleState &vehicle, const ros::Time &now)
    {
        Eigen::Vector3d top = vehicle.position, bottom = vehicle.position;
        top.z() = config_.takeoff_altitude;
        bottom.z() += config_.landing_target_offset;
        if (!box_guard_.segmentAllowed(vehicle.position, top) ||
            !box_guard_.segmentAllowed(vehicle.position, bottom))
        {
            ROS_ERROR("[BOX] TAKEOFF rejected: ARM-time origin/targets outside cuboid.");
            return;
        }
        if (config_.takeoff_speed <= 0.0 ||
            config_.takeoff_altitude <= vehicle.position.z() + config_.takeoff_tolerance)
        {
            ROS_ERROR("[FSM] Invalid takeoff: initial_z=%.3f, absolute "
                      "target_z=%.3f, speed=%.3f.",
                      vehicle.position.z(), config_.takeoff_altitude, config_.takeoff_speed);
            return;
        }

        takeoff_origin_ = vehicle.position;
        takeoff_origin_valid_ = true;
        fixed_yaw_ = yawFromQuaternion(vehicle.attitude);
        takeoff_start_ = now;
        hover_position_ =
            Eigen::Vector3d(takeoff_origin_.x(), takeoff_origin_.y(), config_.takeoff_altitude);
        has_takeoff_ = false;
        is_airborne_ = false;
        ude_ground_inhibit_ = false;
        touchdown_unload_active_ = false;
        landing_detect_active_ = false;
        trajectory_->clear();
        ude_->reset();
        transitionTo(TAKEOFF, "ARM succeeded; constant-speed takeoff started");
        ROS_INFO("[TAKEOFF] Locked xyz0=[%.3f %.3f %.3f], "
                 "locked yaw=%.2f deg, absolute target_z=%.3f.",
                 takeoff_origin_.x(), takeoff_origin_.y(), takeoff_origin_.z(),
                 fixed_yaw_ * 180.0 / M_PI, config_.takeoff_altitude);
    }

    void updateTakeoff(const VehicleState &vehicle, const ros::Time &now, Eigen::Vector3d &position,
                       Eigen::Vector3d &velocity)
    {
        const double elapsed = std::max(0.0, (now - takeoff_start_).toSec());
        position.x() = takeoff_origin_.x();
        position.y() = takeoff_origin_.y();
        position.z() = std::min(takeoff_origin_.z() + config_.takeoff_speed * elapsed,
                                config_.takeoff_altitude);
        velocity.setZero();
        if (position.z() < config_.takeoff_altitude)
            velocity.z() = config_.takeoff_speed;

        if (vehicle.position.z() >= config_.takeoff_altitude - config_.takeoff_tolerance)
        {
            has_takeoff_ = true;
            hover_position_ = vehicle.position;
            hover_position_.z() = config_.takeoff_altitude;
            trajectory_->clear();
            transitionTo(HOVER, "absolute takeoff altitude reached");
            ROS_INFO("[HOVER] Waiting for %.2f s stable hover; hover xyz="
                     "[%.3f %.3f %.3f].",
                     config_.activation.hover.duration, hover_position_.x(), hover_position_.y(),
                     hover_position_.z());
            if (command_mode_)
            {
                ROS_WARN("[COMMAND] Already enabled: polynomial execution is "
                         "now allowed and the trajectory receiver is READY.");
            }
        }
    }

    void updateHover(const VehicleState &vehicle, const ros::Time &now)
    {
        if (has_takeoff_ && command_mode_ && trajectory_->ready())
        {
            if (!trajectory_->withinBox(box_guard_, config_.braking_max_acceleration))
            {
                trajectory_->clear();
                ROS_ERROR(
                    "[BOX] Polynomial rejected: complete path not certified inside inset cuboid.");
                return;
            }
            const bool permitted =
                mavros_state_received_ && mavros_state_.mode == "OFFBOARD" && mavros_state_.armed;
            if (trajectory_->activate(now, ros::WallTime::now().toSec(), vehicle.position,
                                      vehicle.velocity, (now - vehicle.stamp).toSec(),
                                      hover_gate_->stable(ros::WallTime::now().toSec()), permitted))
            {
                transitionTo(POLY_TRAJ,
                             "COMMAND enabled and a READY polynomial trajectory was activated");
            }
        }
    }

    void updatePolynomial(const VehicleState &vehicle, const ros::Time &now,
                          Eigen::Vector3d &position, Eigen::Vector3d &velocity,
                          Eigen::Vector3d &acceleration)
    {
        if (!command_mode_)
        {
            enterBrake(vehicle, now, false);
            updateBrake(vehicle, now, position, velocity, acceleration);
            return;
        }

        if (!trajectory_->evaluate(now, position, velocity, acceleration))
        {
            Eigen::Vector3d end_position, end_velocity, end_acceleration;
            const bool has_end =
                trajectory_->endReference(end_position, end_velocity, end_acceleration);
            // Even nonzero terminal derivatives must be stopped continuously.
            if (has_end)
            {
                last_reference_.p = end_position;
                last_reference_.v = end_velocity;
                last_reference_.a = end_acceleration;
                reference_valid_ = true;
            }
            enterBrake(vehicle, now, false);
            updateBrake(vehicle, now, position, velocity, acceleration);
        }
    }

    void enterBrake(const VehicleState &vehicle, const ros::Time &now, bool for_land,
                    const flight_safety::BoxAssessment *prepared = nullptr)
    {
        if (for_land && !takeoff_origin_valid_)
        {
            ROS_ERROR("[LAND] Rejected: takeoff origin unavailable.");
            return;
        }
        if (state_ == BRAKE)
        {
            brake_for_land_ = brake_for_land_ || for_land;
            if (brake_for_land_)
                command_mode_ = false;
            return; // Upgrade HOVER braking to LAND without restarting its curve.
        }
        flight_safety::Reference seed;
        seed.p = hover_position_;
        if (reference_valid_)
            seed = last_reference_;
        if (state_ == POLY_TRAJ)
        {
            Eigen::Vector3d p, v, a;
            if (trajectory_->evaluate(now, p, v, a))
            {
                seed.p = p;
                seed.v = v;
                seed.a = a;
            }
        }
        if (!reference_valid_ && state_ != POLY_TRAJ)
            seed.p = vehicle.position;
        flight_safety::Preview preview;
        if (box_guard_.enabled())
        {
            flight_safety::Reference current;
            if (boxReferenceAt(now, current) &&
                !(state_ == POLY_TRAJ &&
                  !trajectory_->evaluate(now, current.p, current.v, current.a)))
                seed = current;
            preview = [&](double dt, flight_safety::Reference &r) {
                return boxReferenceAt(now + ros::Duration(dt), r);
            };
            // A missing active polynomial has no usable preview; keep the cached
            // seed for the existing cancellation/completion recovery path.
            if (state_ == POLY_TRAJ && !trajectory_->active())
                preview = flight_safety::Preview();
            const bool reuse =
                prepared &&
                prepared->reusable(seed, vehicle.position, vehicle.velocity, box_guard_.options(),
                                   config_.braking_max_acceleration, config_.braking_max_jerk);
            const auto plan = reuse ? *prepared
                                    : box_guard_.planStop(seed, vehicle.position, vehicle.velocity,
                                                          config_.braking_max_acceleration,
                                                          config_.braking_max_jerk, preview);
            if (!plan.has_stop || plan.decision == flight_safety::BoxDecision::FAILSAFE)
            {
                latchBoxFault("stop cannot be certified: " + plan.reason);
                return;
            }
            braking_ = plan.stop;
            ROS_WARN("[BRAKE PLAN] %s.",
                     reuse ? "reuse current BOX assessment plan" : "shared planner generated stop");
        }
        else if (!braking_.tryStart(seed, config_.braking_max_acceleration,
                                    config_.braking_max_jerk))
        {
            latchBoxFault("reference stop cannot be certified: " + braking_.reason());
            return;
        }
        brake_start_ = now;
        brake_for_land_ = for_land;
        brake_gate_->reset();
        hover_gate_->reset();
        trajectory_->clear();
        // Natural completion preserves the session COMMAND enable. Explicit
        // HOVER already disabled it in dynamicCallback; LAND always disables it.
        if (for_land)
            command_mode_ = false;
        transitionTo(BRAKE, for_land ? "LAND requested: bounded braking first"
                                     : "trajectory cancelled/completed: bounded braking");
        ROS_WARN("[BRAKE] destination=%s duration=%.3f s | bounds A=%.3f J=%.3f | "
                 "A_limit=%.3f (configured %.3f); after curve require %.2f s stable hover.",
                 for_land ? "LAND" : "HOVER", braking_.duration(), braking_.accelerationBound(),
                 braking_.jerkBound(), braking_.accelerationLimit(),
                 config_.braking_max_acceleration, config_.activation.hover.duration);
        ROS_WARN("[BRAKE PLAN] %s (continuous box/A/J certification).",
                 braking_.followsPath() ? "follow trajectory with decreasing phase speed"
                                        : "generic bounded stop / legacy when BOX disabled");
    }

    void updateBrake(const VehicleState &vehicle, const ros::Time &now, Eigen::Vector3d &p,
                     Eigen::Vector3d &v, Eigen::Vector3d &a)
    {
        if (safety_latched_)
            return;
        const double elapsed = std::max(0.0, (now - brake_start_).toSec());
        const auto r = braking_.at(elapsed);
        p = r.p;
        v = r.v;
        a = r.a;
        const bool finished = elapsed >= braking_.duration();
        const double wall = ros::WallTime::now().toSec();
        const bool permitted = finished && mavros_state_received_ &&
                               mavros_state_.mode == "OFFBOARD" && mavros_state_.armed &&
                               (vehicle.position - p).norm() <= config_.braking_position_tolerance;
        brake_gate_->update(vehicle.position, vehicle.velocity, vehicle.stamp.toSec(), wall,
                            permitted);
        if (!brake_gate_->stable(wall))
        {
            if (finished)
                ROS_WARN_THROTTLE(
                    1.0, "[BRAKE] Holding stop point; position_error=%.3f m, speed=%.3f m/s; %s.",
                    (vehicle.position - p).norm(), vehicle.velocity.norm(),
                    brake_gate_->diagnostic(wall).c_str());
            return;
        }
        hover_position_ = p;
        ROS_WARN("[BRAKE] Stable hover confirmed for %.2f s; speed=%.3f m/s.",
                 config_.activation.hover.duration, vehicle.velocity.norm());
        if (brake_for_land_)
            enterLanding(vehicle, now);
        else
        {
            *hover_gate_ =
                *brake_gate_; // Already confirmed: do not wait a second full window again.
            transitionTo(HOVER, "bounded braking and stable hover completed");
            if (command_mode_)
                ROS_INFO("[COMMAND] Session remains enabled; next trajectory will activate "
                         "automatically after admission checks.");
        }
    }

    void enterLanding(const VehicleState &vehicle, const ros::Time &now)
    {
        if (!takeoff_origin_valid_)
        {
            ROS_ERROR("[FSM] LAND rejected: takeoff origin is unavailable.");
            return;
        }
        trajectory_->clear();
        command_mode_ = false;
        // Continue from the braking reference, not a new instantaneous XY lock.
        landing_start_position_ = hover_position_;
        landing_target_z_ = takeoff_origin_.z() + config_.landing_target_offset;
        Eigen::Vector3d landing_end = landing_start_position_;
        landing_end.z() = landing_target_z_;
        if (!box_guard_.segmentAllowed(landing_start_position_, landing_end))
        {
            brake_for_land_ = false;
            transitionTo(HOVER, "LAND reference outside flight box; remain at stop point");
            ROS_ERROR(
                "[BOX] LAND rejected: vertical descent/relative target outside inset cuboid.");
            return;
        }
        landing_start_ = now;
        descent_.start(landing_start_position_, landing_target_z_, config_.landing_speed,
                       config_.landing_max_acceleration, config_.landing_max_jerk);
        landing_detect_active_ = false;
        touchdown_unload_active_ = false;
        last_disarm_attempt_ = ros::Time(0);
        transitionTo(LAND, "LAND trigger accepted");
        ROS_INFO("[LAND] Locked xy=[%.3f %.3f], start_z=%.3f, "
                 "target_z=%.3f (= takeoff initial z %+.3f).",
                 landing_start_position_.x(), landing_start_position_.y(),
                 landing_start_position_.z(), landing_target_z_, config_.landing_target_offset);
    }

    void updateLanding(const VehicleState &vehicle, const ros::Time &now, Eigen::Vector3d &position,
                       Eigen::Vector3d &velocity, Eigen::Vector3d &acceleration)
    {
        const double elapsed = std::max(0.0, (now - landing_start_).toSec());
        const auto r = descent_.at(elapsed);
        position = r.p;
        velocity = r.v;
        acceleration = r.a;

        const double height = vehicle.position.z() - takeoff_origin_.z();
        if (ude_->active() && height <= config_.ude_landing_disable_height)
        {
            ude_->reset();
            ude_ground_inhibit_ = true;
            ROS_WARN("[FSM] UDE disabled near ground at relative height %.3f m.", height);
        }

        const bool on_ground =
            extended_state_received_ &&
            ros::WallTime::now().toSec() - extended_state_wall_ <= config_.extended_state_timeout &&
            extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
        // Do not unload on height/velocity alone: a hovering vehicle can meet both.
        const bool landing_candidate =
            on_ground && height < config_.touchdown_height &&
            std::fabs(vehicle.velocity.z()) < config_.touchdown_max_vz &&
            vehicle.velocity.head<2>().norm() < config_.touchdown_max_vxy;

        if (!touchdown_unload_active_)
        {
            if (landing_candidate)
            {
                if (!landing_detect_active_)
                {
                    landing_detect_active_ = true;
                    landing_detect_start_ = now;
                    ROS_WARN("[LAND] Touchdown candidate; confirmation started.");
                }
                if ((now - landing_detect_start_).toSec() >= config_.touchdown_confirm_time)
                {
                    touchdown_unload_active_ = true;
                    touchdown_unload_start_ = now;
                    touchdown_start_thrust_ =
                        last_command_valid_ ? last_command_.thrust : config_.hover_thrust;
                    ROS_WARN("[LAND] Touchdown confirmed; unloading thrust "
                             "from %.3f to %.3f.",
                             touchdown_start_thrust_, config_.touchdown_thrust);
                }
            }
            else
            {
                landing_detect_active_ = false;
            }
        }

        if (touchdown_unload_active_ && on_ground &&
            (now - touchdown_unload_start_).toSec() >= config_.thrust_unload_time &&
            (last_disarm_attempt_.isZero() ||
             (now - last_disarm_attempt_).toSec() >= config_.service_retry_interval))
        {
            last_disarm_attempt_ = now;
            mavros_msgs::CommandBool disarm;
            disarm.request.value = false;
            if (arming_client_.call(disarm) && disarm.response.success)
            {
                has_takeoff_ = false;
                is_airborne_ = false;
                takeoff_origin_valid_ = false;
                landing_detect_active_ = false;
                touchdown_unload_active_ = false;
                trajectory_->clear();
                ude_->reset();
                hover_position_ = vehicle.position;
                transitionTo(HOVER, "PX4 ON_GROUND confirmed and DISARM succeeded");
            }
            else
            {
                ROS_WARN("[LAND] DISARM rejected; retrying in %.1f s.",
                         config_.service_retry_interval);
            }
        }
        else if (touchdown_unload_active_ && !on_ground)
        {
            touchdown_unload_active_ = false;
            landing_detect_active_ = false;
            ROS_WARN_THROTTLE(
                0.5, "[LAND] ON_GROUND lost/stale; cancelled thrust unloading (height %.3f m).",
                height);
        }
    }

    void updateLiftoffAndUde(const VehicleState &vehicle, const ros::Time &now)
    {
        const bool offboard_armed =
            mavros_state_received_ && mavros_state_.mode == "OFFBOARD" && mavros_state_.armed;
        if ((state_ == TAKEOFF || state_ == BRAKE) && takeoff_origin_valid_ && offboard_armed &&
            !is_airborne_)
        {
            const double delta_z = vehicle.position.z() - takeoff_origin_.z();
            if (delta_z > config_.ude_liftoff_height &&
                vehicle.velocity.z() > config_.ude_liftoff_vz)
            {
                is_airborne_ = true;
                ROS_WARN("[FSM] Liftoff detected: dz=%.3f m, world_vz=%.3f "
                         "m/s.",
                         delta_z, vehicle.velocity.z());
            }
        }

        if (state_ == LAND && takeoff_origin_valid_)
        {
            const double height = vehicle.position.z() - takeoff_origin_.z();
            if (height <= config_.ude_landing_disable_height)
                ude_ground_inhibit_ = true;
            else if (height >= config_.ude_landing_reenable_height && !touchdown_unload_active_)
                ude_ground_inhibit_ = false;
        }
        if (config_.ude_enabled && is_airborne_ && offboard_armed && !ude_->active() &&
            !touchdown_unload_active_ && !(state_ == LAND && ude_ground_inhibit_))
        {
            ude_->activate(vehicle, now);
            ROS_WARN("[FSM] UDE activated after liftoff; compensation ramps "
                     "over %.2f s.",
                     config_.ude_ramp_time);
        }
        if (ude_->active() && (!config_.ude_enabled || !offboard_armed || touchdown_unload_active_))
        {
            ude_->reset();
            ROS_WARN("[FSM] UDE reset because OFFBOARD/ARMED was lost.");
        }
    }

    void updateModeSafety()
    {
        const bool offboard_armed =
            mavros_state_received_ && mavros_state_.mode == "OFFBOARD" && mavros_state_.armed;
        if ((state_ == TAKEOFF || state_ == POLY_TRAJ || state_ == LAND || state_ == BRAKE) &&
            !offboard_armed)
            ROS_ERROR_THROTTLE(1.0,
                               "[FSM] Flight state %s but MAVROS is not both "
                               "OFFBOARD and ARMED.",
                               stateName());
    }

    void setAdmission(bool enabled, const std::string &reason)
    {
        admission_enabled_ = enabled;
        admission_reason_ = reason;
        trajectory_->setAcceptEnabled(enabled, reason);
    }
    void publishAdmissionStatus(const ros::WallTimerEvent &)
    {
        const bool occupied = trajectory_->ready() || trajectory_->active();
        const bool allowed = admission_enabled_ && !occupied && !safety_latched_;
        std::ostringstream wire;
        // Absolute wall timestamp prevents a latched message from a stopped/
        // hung controller being mistaken for a fresh permission on reconnect.
        wire << (allowed ? "ALLOW|" : "BLOCKED|") << std::setprecision(17)
             << ros::WallTime::now().toSec() << "|";
        if (occupied)
            wire << (trajectory_->active() ? "POLY_TRAJ executing"
                                           : "READY trajectory waiting for COMMAND; expiry=" +
                                                 std::to_string(config_.activation.max_ready_age) +
                                                 "s; expired trajectory must be resent")
                 << "; ";
        wire << admission_reason_;
        std_msgs::String status;
        status.data = wire.str();
        admission_pub_.publish(status);
    }
    void updateTrajectoryPermission()
    {
        const double wall = ros::WallTime::now().toSec();
        std::ostringstream reason;
        reason << "takeoff_complete=" << has_takeoff_ << ", FSM=" << stateName()
               << ", COMMAND=" << command_mode_ << ", MAVROS_received=" << mavros_state_received_
               << ", mode=" << mavros_state_.mode
               << ", armed=" << static_cast<int>(mavros_state_.armed) << ", "
               << hover_gate_->diagnostic(wall);
        const bool enabled = has_takeoff_ && state_ == HOVER && mavros_state_received_ &&
                             mavros_state_.mode == "OFFBOARD" && mavros_state_.armed &&
                             hover_gate_->stable(wall);
        setAdmission(enabled, reason.str());
        if (enabled && !receiver_was_enabled_)
            ROS_WARN("[HOVER READY] %.2f s stable hover confirmed; trajectory receiver enabled.",
                     config_.activation.hover.duration);
        receiver_was_enabled_ = enabled;
    }

    mavros_msgs::AttitudeTarget calculateCommand(const VehicleState &vehicle,
                                                 const Eigen::Vector3d &reference_position,
                                                 const Eigen::Vector3d &reference_velocity,
                                                 const Eigen::Vector3d &reference_acceleration,
                                                 const ros::Time &now, double dt)
    {
        nominal_acceleration_ = kp_.cwiseProduct(reference_position - vehicle.position) +
                                kd_.cwiseProduct(reference_velocity - vehicle.velocity) +
                                reference_acceleration;
        for (int i = 0; i < 3; ++i)
            nominal_acceleration_(i) = clamp(nominal_acceleration_(i), -config_.max_acceleration,
                                             config_.max_acceleration);

        // Measured attitude accounts for attitude-loop lag. Previous thrust
        // includes force/tilt/thrust saturation and touchdown override.
        const double previous_thrust =
            last_command_valid_ ? last_command_.thrust : config_.hover_thrust;
        const Eigen::Vector3d modeled_applied_acceleration =
            vehicle.attitude * Eigen::Vector3d::UnitZ() *
                (previous_thrust * config_.gravity / config_.hover_thrust) -
            Eigen::Vector3d(0, 0, config_.gravity);
        const Eigen::Vector3d ude_compensation =
            ude_->update(modeled_applied_acceleration, vehicle, dt, now);
        Eigen::Vector3d desired_acceleration = nominal_acceleration_ - ude_compensation;
        Eigen::Vector3d desired_force =
            config_.mass * (desired_acceleration + Eigen::Vector3d(0.0, 0.0, config_.gravity));

        const double minimum_vertical_force =
            config_.min_vertical_force_ratio * config_.mass * config_.gravity;
        const double maximum_vertical_force =
            config_.max_vertical_force_ratio * config_.mass * config_.gravity;
        desired_force.z() =
            clamp(desired_force.z(), minimum_vertical_force, maximum_vertical_force);

        const double maximum_horizontal =
            desired_force.z() * std::tan(config_.max_tilt_deg * M_PI / 180.0);
        const double horizontal_norm = desired_force.head<2>().norm();
        if (horizontal_norm > maximum_horizontal)
            desired_force.head<2>() *= maximum_horizontal / horizontal_norm;

        Eigen::Quaterniond desired_attitude = flight_safety::attitude(desired_force, fixed_yaw_);
        desired_attitude.normalize();

        const double full_thrust =
            config_.mass * config_.gravity / std::max(config_.hover_thrust, 1.0e-6);
        const double thrust = clamp(desired_force.norm() / full_thrust, config_.min_thrust_command,
                                    config_.max_thrust_command);

        mavros_msgs::AttitudeTarget command;
        command.header.stamp = now;
        command.orientation.w = desired_attitude.w();
        command.orientation.x = desired_attitude.x();
        command.orientation.y = desired_attitude.y();
        command.orientation.z = desired_attitude.z();
        command.body_rate.x = 0.0;
        command.body_rate.y = 0.0;
        command.body_rate.z = 0.0;
        command.thrust = thrust;
        command.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        return command;
    }

    void publishStandby(const ros::Time &now, double thrust)
    {
        const Eigen::Quaterniond attitude(Eigen::AngleAxisd(fixed_yaw_, Eigen::Vector3d::UnitZ()));
        mavros_msgs::AttitudeTarget command;
        command.header.stamp = now;
        command.orientation.w = attitude.w();
        command.orientation.x = attitude.x();
        command.orientation.y = attitude.y();
        command.orientation.z = attitude.z();
        command.thrust = thrust;
        command.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        publishAndRecordAttitude(command, now);
        last_command_ = command;
        last_command_valid_ = true;
    }

    void republishLastCommand(const ros::Time &now)
    {
        if (!last_command_valid_)
        {
            publishStandby(now, 0.0);
            return;
        }
        last_command_.header.stamp = now;
        publishAndRecordAttitude(last_command_, now);
    }

    void publishAndRecordAttitude(mavros_msgs::AttitudeTarget &command, const ros::Time &now)
    {
        command.header.stamp = now;
        attitude_pub_.publish(command);
        if (!bag_open_)
            return;
        try
        {
            bag_.write(joinTopic(config_.mavros_namespace, "setpoint_raw/attitude"), now, command);
        }
        catch (const rosbag::BagException &error)
        {
            ROS_ERROR_THROTTLE(1.0, "[BAG] Attitude write failed: %s.", error.what());
        }
    }

    void publishReferencePoint(const ros::Time &now, const Eigen::Vector3d &reference)
    {
        geometry_msgs::PointStamped point;
        point.header.stamp = now;
        point.header.frame_id = config_.reference_frame;
        point.point.x = reference.x();
        point.point.y = reference.y();
        point.point.z = reference.z();
        reference_point_pub_.publish(point);
        if (bag_open_)
            try
            {
                bag_.write(reference_point_pub_.getTopic(), now, point);
            }
            catch (const rosbag::BagException &e)
            {
                ROS_ERROR_THROTTLE(1.0, "[BAG] Reference point write failed: %s", e.what());
            }
    }
    geometry_msgs::Vector3Stamped vectorMessage(const ros::Time &stamp,
                                                const Eigen::Vector3d &value) const
    {
        geometry_msgs::Vector3Stamped message;
        message.header.stamp = stamp;
        message.header.frame_id = "world";
        message.vector.x = value.x();
        message.vector.y = value.y();
        message.vector.z = value.z();
        return message;
    }

    void printStatus(const ros::Time &now, const VehicleState &vehicle,
                     const Eigen::Vector3d &reference_position,
                     const mavros_msgs::AttitudeTarget &command)
    {
        if (!last_print_time_.isZero() && (now - last_print_time_).toSec() < config_.print_interval)
            return;
        last_print_time_ = now;
        const Eigen::Vector3d error = reference_position - vehicle.position;
        std::ostringstream text;
        text << "[CTRL] " << stateName() << std::fixed << std::setprecision(2) << " ref=["
             << reference_position.transpose() << "] pos=[" << vehicle.position.transpose()
             << "] err=[" << error.transpose() << "] thrust=" << std::setprecision(3)
             << command.thrust << " UDE=" << (ude_->active() ? "ON" : "OFF");
        const bool color =
            config_.status_color == "always" ||
            (config_.status_color == "auto" && ::isatty(STDOUT_FILENO) && !std::getenv("NO_COLOR"));
        std::cout << (color ? minisnap_console::reset : "") << (color ? minisnap_console::cyan : "")
                  << text.str() << (color ? minisnap_console::reset : "") << std::endl;
        // Keep rosout/bag diagnostics without a second differently styled INFO line.
        ROS_DEBUG_STREAM(text.str());
    }

    static const char *stateName(FlightState state)
    {
        switch (state)
        {
        case TAKEOFF:
            return "TAKEOFF";
        case POLY_TRAJ:
            return "POLY_TRAJ";
        case LAND:
            return "LAND";
        case BRAKE:
            return "BRAKE";
        case HOVER:
        default:
            return "HOVER";
        }
    }

    const char *stateName() const { return stateName(state_); }

    void transitionTo(FlightState next_state, const std::string &reason)
    {
        if (state_ == next_state)
            return;

        const FlightState previous_state = state_;
        state_ = next_state;
        ROS_WARN("[FSM TRANSITION] %s -> %s | %s", stateName(previous_state), stateName(next_state),
                 reason.c_str());
    }

    void openBag()
    {
        if (!config_.bag_enabled)
            return;
        if (::mkdir(config_.bag_directory.c_str(), 0755) != 0 && errno != EEXIST)
        {
            ROS_ERROR("[BAG] Cannot create directory %s.", config_.bag_directory.c_str());
            return;
        }
        std::time_t timestamp = std::time(NULL);
        std::tm local_time;
        localtime_r(&timestamp, &local_time);
        std::ostringstream name;
        name << config_.bag_directory << "/traj_tracking_fsm_"
             << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".bag";
        try
        {
            bag_.open(name.str(), rosbag::bagmode::Write);
            bag_open_ = true;
            ROS_INFO("[BAG] Recording to %s.", name.str().c_str());
        }
        catch (const rosbag::BagException &error)
        {
            ROS_ERROR("[BAG] Open failed: %s.", error.what());
        }
    }

    void closeBag()
    {
        if (!bag_open_)
            return;
        bag_.close();
        bag_open_ = false;
        ROS_INFO("[BAG] Recording closed.");
    }

    void recordBag(const ros::Time &now, const Eigen::Vector3d &position,
                   const Eigen::Vector3d &velocity, const Eigen::Vector3d &acceleration)
    {
        if (!bag_open_)
            return;
        try
        {
            bag_.write("/traj_tracking_controller/ref_pos", now, vectorMessage(now, position));
            bag_.write("/traj_tracking_controller/ref_vel", now, vectorMessage(now, velocity));
            bag_.write("/traj_tracking_controller/ref_acc", now, vectorMessage(now, acceleration));
            std_msgs::Float64MultiArray status;
            status.data.resize(6);
            status.data[0] = ude_->estimate().x();
            status.data[1] = ude_->estimate().y();
            status.data[2] = ude_->estimate().z();
            status.data[3] = static_cast<double>(state_);
            status.data[4] = ude_->active() ? 1.0 : 0.0;
            status.data[5] = trajectory_->active() ? 1.0 : 0.0;
            bag_.write(ude_state_bag_topic_, now, status);
        }
        catch (const rosbag::BagException &error)
        {
            ROS_ERROR_THROTTLE(1.0, "[BAG] Write failed: %s.", error.what());
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ControllerConfig config_;
    std::unique_ptr<StateProvider> state_provider_;
    std::unique_ptr<TrajectoryManager> trajectory_;
    std::unique_ptr<minisnap::StableHover> hover_gate_;
    std::unique_ptr<minisnap::StableHover> brake_gate_;
    std::unique_ptr<UdeController> ude_;
    std::unique_ptr<DynamicServer> dynamic_server_;

    ros::Subscriber mavros_state_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Subscriber bag_odom_sub_;
    ros::Subscriber bag_vision_pose_sub_;
    ros::Publisher attitude_pub_;
    ros::Publisher admission_pub_;
    ros::Publisher reference_point_pub_;
    ros::WallTimer admission_timer_;
    bool admission_enabled_ = false;
    std::string admission_reason_ = "controller initializing; waiting for state";
    ros::ServiceClient arming_client_;
    std::string ude_state_bag_topic_;

    mavros_msgs::State mavros_state_;
    mavros_msgs::ExtendedState extended_state_;
    bool mavros_state_received_ = false;
    bool extended_state_received_ = false;
    double extended_state_wall_ = 0;
    bool receiver_was_enabled_ = false;
    bool ude_ground_inhibit_ = false;
    bool brake_for_land_ = false;
    bool reference_valid_ = false;
    bool safety_latched_ = false;
    double state_failure_wall_ = -1;
    flight_safety::Reference last_reference_;
    flight_safety::BoxStop braking_;
    flight_safety::BoxGuard box_guard_;
    double last_box_check_wall_ = -1;
    flight_safety::SmoothDescent descent_;
    ros::Time brake_start_;

    FlightState state_ = HOVER;
    bool hold_initialized_ = false;
    bool command_mode_ = false;
    bool last_takeoff_switch_ = false;
    bool last_land_switch_ = false;
    bool pending_takeoff_ = false;
    bool pending_land_ = false;
    bool has_takeoff_ = false;
    bool is_airborne_ = false;
    bool takeoff_origin_valid_ = false;

    Eigen::Vector3d kp_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d kd_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d nominal_acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d hover_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d takeoff_origin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d landing_start_position_ = Eigen::Vector3d::Zero();
    double fixed_yaw_ = 0.0;
    double landing_target_z_ = 0.0;

    ros::Time last_loop_time_;
    ros::Time last_print_time_;
    ros::Time last_service_request_;
    ros::Time takeoff_start_;
    ros::Time landing_start_;
    ros::Time landing_detect_start_;
    ros::Time touchdown_unload_start_;
    ros::Time last_disarm_attempt_;
    bool landing_detect_active_ = false;
    bool touchdown_unload_active_ = false;
    double touchdown_start_thrust_ = 0.0;

    mavros_msgs::AttitudeTarget last_command_;
    bool last_command_valid_ = false;
    rosbag::Bag bag_;
    bool bag_open_ = false;
};

#ifndef OMMPC_FSM_NO_MAIN
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_tracking_controller");
    try
    {
        TrajectoryTrackingController controller;
        if (!controller.valid())
        {
            ROS_FATAL("Invalid unified trajectory-controller configuration.");
            return 1;
        }
        controller.run();
        return 0;
    }
    catch (const std::exception &error)
    {
        ROS_FATAL("Controller startup failed: %s", error.what());
        return 1;
    }
}
#endif
