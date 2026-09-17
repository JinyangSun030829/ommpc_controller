#ifndef OMMPC_CONTROLLER_TRAJECTORY_MANAGER_HPP
#define OMMPC_CONTROLLER_TRAJECTORY_MANAGER_HPP

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include "stable_hover.h"
#include "box_flight_safety.hpp"

#include <cmath>
#include <mutex>
#include <string>

// Owns exactly one Minimum-Snap trajectory.  A received trajectory is held in
// READY state and its clock does not start until activate() is called by the
// flight state machine.
class TrajectoryManager
{
public:
    TrajectoryManager(ros::NodeHandle &nh, const std::string &topic,
                      const minisnap::ActivationOptions &options = minisnap::ActivationOptions()) : activation_(options)
    {
        activation_.validate();
        subscriber_ = nh.subscribe<std_msgs::Float64MultiArray>(
            topic, 1, &TrajectoryManager::callback, this);
    }

    void setAcceptEnabled(bool enabled, const std::string& reason = "permission disabled")
    {
        std::lock_guard<std::mutex> lock(mutex_);
        accept_enabled_ = enabled;
        rejection_reason_ = enabled ? "" : reason;
    }

    bool ready() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return ready_ && !active_;
    }

    bool active() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_;
    }

    bool activate(const ros::Time &now, double wall_now,
                  const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
                  double state_age, bool stable, bool permitted)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ready_ || active_)
            return false;
        const std::string rejection = minisnap::activationRejection(activation_, position, velocity,
            start_position_, start_velocity_, start_acceleration_, start_jerk_,
            state_age, wall_now - ready_received_, stable, permitted);
        if (!rejection.empty())
        {
            ROS_WARN_THROTTLE(1.0, "[TRAJ] Activation rejected: %s.", rejection.c_str());
            if (wall_now - ready_received_ > activation_.max_ready_age ||
                wall_now < ready_received_ ||
                (position - start_position_).norm() > activation_.max_position_error ||
                start_velocity_.norm() > activation_.boundary_tolerance ||
                start_acceleration_.norm() > activation_.boundary_tolerance ||
                start_jerk_.norm() > activation_.boundary_tolerance)
                clearUnlocked();
            return false;
        }
        start_time_ = now;
        ready_ = false;
        active_ = true;
        ROS_INFO("[TRAJ] Polynomial trajectory activated (duration %.3f s).",
                 total_duration_);
        return true;
    }

    // Returns false once the trajectory has reached its final time.
    bool evaluate(const ros::Time &now, Eigen::Vector3d &position,
                  Eigen::Vector3d &velocity,
                  Eigen::Vector3d &acceleration) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || segment_times_.size() == 0)
            return false;

        const double elapsed = std::max(0.0, (now - start_time_).toSec());
        if (elapsed >= total_duration_)
        {
            evaluateSegment(segment_times_.size() - 1,
                            segment_times_(segment_times_.size() - 1),
                            position, velocity, acceleration);
            return false;
        }

        double local_time = elapsed;
        Eigen::Index segment = 0;
        while (segment + 1 < segment_times_.size() &&
               local_time >= segment_times_(segment))
        {
            local_time -= segment_times_(segment);
            ++segment;
        }
        evaluateSegment(segment, local_time, position, velocity, acceleration);
        return true;
    }

    bool endReference(Eigen::Vector3d &position, Eigen::Vector3d &velocity,
                      Eigen::Vector3d &acceleration) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (segment_times_.size() == 0)
            return false;
        const Eigen::Index last = segment_times_.size() - 1;
        evaluateSegment(last, segment_times_(last), position, velocity,
                        acceleration);
        return true;
    }

    bool withinBox(const flight_safety::BoxGuard &guard,double braking_limit) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!guard.enabled()) return true;
        if (segment_times_.size()==0) return false;
        for (Eigen::Index segment=0;segment<segment_times_.size();++segment)
        {
            Eigen::MatrixXd coefficients(3,coefficient_count_);
            for (int axis=0;axis<3;++axis)
                for (int j=0;j<coefficient_count_;++j)
                    coefficients(axis,j)=coefficients_(segment,axis*coefficient_count_+j);
            if (!guard.polynomialAllowed(coefficients,segment_times_(segment),braking_limit)) return false;
        }
        return true;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        clearUnlocked();
    }

    void expireReady(double wall_now)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (ready_ && (wall_now < ready_received_ || wall_now - ready_received_ > activation_.max_ready_age))
        {
            clearUnlocked();
            ROS_WARN("[TRAJ] READY expired; resend waypoints from stable HOVER.");
        }
    }

private:
    void clearUnlocked()
    {
        ready_ = false;
        active_ = false;
        start_time_ = ros::Time(0);
        total_duration_ = 0.0;
        coefficient_count_ = 0;
        segment_times_.resize(0);
        coefficients_.resize(0, 0);
    }

    static bool positiveInteger(double value, int &result)
    {
        if (!std::isfinite(value) || value < 1.0)
            return false;
        const double rounded = std::round(value);
        if (std::fabs(value - rounded) > 1.0e-9 ||
            rounded > static_cast<double>(1000000))
            return false;
        result = static_cast<int>(rounded);
        return true;
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!accept_enabled_)
        {
            ROS_WARN_THROTTLE(1.0, "[TRAJ] Rejected: %s.", rejection_reason_.c_str());
            return;
        }
        if (ready_ || active_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[TRAJ] Rejected: one trajectory is already "
                              "ready or executing.");
            return;
        }
        if (msg->data.size() < 2)
        {
            ROS_ERROR("[TRAJ] Invalid message: fewer than two header values.");
            return;
        }

        int segment_count = 0;
        int coefficient_count = 0;
        if (!positiveInteger(msg->data[0], segment_count) ||
            !positiveInteger(msg->data[1], coefficient_count) ||
            segment_count > 10000 || coefficient_count != 8)
        {
            ROS_ERROR("[TRAJ] Invalid segment/coefficient count.");
            return;
        }

        const size_t expected =
            2u + static_cast<size_t>(segment_count) +
            static_cast<size_t>(segment_count) * 3u *
                static_cast<size_t>(coefficient_count);
        if (msg->data.size() != expected)
        {
            ROS_ERROR("[TRAJ] Invalid data length: expected %zu, got %zu.",
                      expected, msg->data.size());
            return;
        }

        Eigen::VectorXd times(segment_count);
        Eigen::MatrixXd coefficients(segment_count, 3 * coefficient_count);
        double total = 0.0;
        for (int i = 0; i < segment_count; ++i)
        {
            const double duration = msg->data[2 + i];
            if (!std::isfinite(duration) || duration <= 0.0)
            {
                ROS_ERROR("[TRAJ] Segment %d has invalid duration %.6f.", i,
                          duration);
                return;
            }
            times(i) = duration;
            total += duration;
        }
        if (!std::isfinite(total) || total > 10000.0)
        {
            ROS_ERROR("[TRAJ] Invalid total duration."); return;
        }

        size_t index = 2u + static_cast<size_t>(segment_count);
        for (int row = 0; row < segment_count; ++row)
        {
            for (int col = 0; col < 3 * coefficient_count; ++col)
            {
                const double value = msg->data[index++];
                if (!std::isfinite(value))
                {
                    ROS_ERROR("[TRAJ] Non-finite polynomial coefficient.");
                    return;
                }
                coefficients(row, col) = value;
            }
        }

        segment_times_ = times;
        coefficients_ = coefficients;
        coefficient_count_ = coefficient_count;
        total_duration_ = total;
        evaluateSegment(0, 0.0, start_position_, start_velocity_, start_acceleration_);
        start_jerk_.setZero();
        for (int axis = 0; axis < 3; ++axis)
            start_jerk_(axis) = 6.0 * coefficients_(0, axis * coefficient_count_ + coefficient_count_ - 4);
        if (!start_position_.allFinite() || !start_velocity_.allFinite() ||
            !start_acceleration_.allFinite() || !start_jerk_.allFinite() ||
            start_velocity_.norm() > activation_.boundary_tolerance ||
            start_acceleration_.norm() > activation_.boundary_tolerance ||
            start_jerk_.norm() > activation_.boundary_tolerance)
        {
            clearUnlocked();
            ROS_ERROR("[TRAJ] Rejected: requires finite zero-v/a/j snap hover start."); return;
        }
        ready_received_ = ros::WallTime::now().toSec();
        ready_ = true;
        active_ = false;
        ROS_INFO("[TRAJ] Accepted %d segments and marked READY "
                 "(duration %.3f s). The FSM will start it when COMMAND "
                 "execution is enabled.", segment_count, total_duration_);
    }

    void evaluateSegment(Eigen::Index segment, double time,
                         Eigen::Vector3d &position,
                         Eigen::Vector3d &velocity,
                         Eigen::Vector3d &acceleration) const
    {
        position.setZero();
        velocity.setZero();
        acceleration.setZero();
        for (int axis = 0; axis < 3; ++axis)
        {
            const int first = axis * coefficient_count_;
            for (int j = 0; j < coefficient_count_; ++j)
            {
                const int power = coefficient_count_ - 1 - j;
                const double coefficient = coefficients_(segment, first + j);
                position(axis) += coefficient * std::pow(time, power);
                if (power >= 1)
                    velocity(axis) += power * coefficient *
                                      std::pow(time, power - 1);
                if (power >= 2)
                    acceleration(axis) += power * (power - 1) * coefficient *
                                          std::pow(time, power - 2);
            }
        }
    }

    ros::Subscriber subscriber_;
    mutable std::mutex mutex_;
    bool accept_enabled_ = false;
    std::string rejection_reason_ = "controller not initialized";
    minisnap::ActivationOptions activation_;
    double ready_received_ = 0.0;
    Eigen::Vector3d start_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d start_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d start_acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d start_jerk_ = Eigen::Vector3d::Zero();
    bool ready_ = false;
    bool active_ = false;
    ros::Time start_time_;
    int coefficient_count_ = 0;
    double total_duration_ = 0.0;
    Eigen::VectorXd segment_times_;
    Eigen::MatrixXd coefficients_;
};

#endif
