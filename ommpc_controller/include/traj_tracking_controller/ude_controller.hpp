#ifndef OMMPC_CONTROLLER_UDE_CONTROLLER_HPP
#define OMMPC_CONTROLLER_UDE_CONTROLLER_HPP

#include "controller_config.hpp"
#include "state_provider.hpp"
#include "flight_safety.hpp"

#include <Eigen/Dense>
#include <algorithm>

class UdeController
{
public:
    explicit UdeController(const ControllerConfig &config)
        : config_(config)
    {
        tau_ << std::max(config_.ude_tau_x, 1.0e-6),
                std::max(config_.ude_tau_y, 1.0e-6),
                std::max(config_.ude_tau_z, 1.0e-6);
    }

    void activate(const VehicleState &state, const ros::Time &now)
    {
        observer_.reset(state.velocity,state.stamp.toSec());
        estimate_.setZero();
        active_ = true;
        skip_first_update_ = true;
        start_time_ = now;
    }

    void reset()
    {
        observer_.reset(Eigen::Vector3d::Zero(),0);
        estimate_.setZero();
        active_ = false;
        skip_first_update_ = false;
        start_time_ = ros::Time(0);
    }

    Eigen::Vector3d update(const Eigen::Vector3d &modeled_applied_acceleration,
                           const VehicleState &state, double /*dt*/,
                           const ros::Time &now)
    {
        if (!active_)
        {
            estimate_.setZero();
            return Eigen::Vector3d::Zero();
        }

        if (skip_first_update_)
        {
            observer_.reset(state.velocity,state.stamp.toSec());
            estimate_.setZero();
            skip_first_update_ = false;
        }
        else
        {
            estimate_=observer_.update(state.velocity,state.stamp.toSec(),modeled_applied_acceleration,
                                      tau_,config_.ude_max_estimate,config_.state_timeout);
        }

        const double ramp = config_.ude_ramp_time <= 0.0
                                ? 1.0
                                : clamp((now - start_time_).toSec() /
                                            config_.ude_ramp_time,
                                        0.0, 1.0);
        return ramp * estimate_;
    }

    bool active() const { return active_; }
    const Eigen::Vector3d &estimate() const { return estimate_; }

private:
    static double clamp(double value, double minimum, double maximum)
    {
        return std::max(minimum, std::min(maximum, value));
    }

    ControllerConfig config_;
    Eigen::Vector3d tau_ = Eigen::Vector3d::Ones();
    flight_safety::DisturbanceObserver observer_;
    Eigen::Vector3d estimate_ = Eigen::Vector3d::Zero();
    ros::Time start_time_;
    bool active_ = false;
    bool skip_first_update_ = false;
};

#endif
