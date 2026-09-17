// Offline circle rehearsal using the SAME core as trajectory_generator_node.
// No ROS master, subscribers, publishers or flight commands.
#include "trajectory_planner.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char **argv)
{
    try
    {
        if (argc != 19)
            throw std::invalid_argument("expected radius, V/A/J and 14 planner/template options");
        auto number = [&](int i) {
            std::size_t used = 0;
            const double value = std::stod(argv[i], &used);
            if (used != std::string(argv[i]).size() || !std::isfinite(value))
                throw std::invalid_argument("invalid numeric argument");
            return value;
        };
        auto integer = [&](int i) {
            const double v = number(i);
            if (std::floor(v) != v || std::abs(v) > 100000)
                throw std::invalid_argument("invalid integer argument");
            return static_cast<int>(v);
        };
        const double radius = number(1);
        minisnap::PlannerOptions o;
        o.max_velocity = number(2); o.max_acceleration = number(3); o.max_jerk = number(4);
        o.min_segment_time = number(5); o.limit_margin = number(6);
        o.seed_safety_ratio = number(7); o.max_total_time = number(8);
        o.max_iterations = integer(9); o.bound_tolerance = number(10);
        o.bound_max_depth = integer(11); o.duplicate_distance = number(12);
        o.max_segments = integer(13); o.compare_legacy_seed = integer(14) != 0;
        o.compression_budget_ms = number(15); o.compression_max_trials = integer(16);
        const int per_turn = integer(17), targets = integer(18);
        o.order = 4;
        // Deterministic calibration: the Python output explicitly disables compression.
        o.compress_time = false;
        o.validate();
        if (radius <= 0 || per_turn < 4 || targets <= 3 * per_turn || targets > 1998)
            throw std::invalid_argument("invalid circle template");
        const double pi = std::acos(-1.0);
        Eigen::MatrixXd path(targets + 2, 3);
        path.row(0) << 0, radius, 0;
        for (int i = 1; i <= targets; ++i)
        {
            const double angle = 2 * pi * i / per_turn;
            path.row(i) << radius * std::sin(angle), radius * std::cos(angle), 0;
        }
        path.row(targets + 1) << 0, 0, 0;
        const auto result = minisnap::TrajectoryPlanner(o).plan(path, Eigen::Vector3d::Zero());
        if (result.trajectory.times.size() != targets + 1)
            throw std::runtime_error("waypoints were merged; template no longer matches");
        // Middle two circles, excluding the first ramp and final circle/return.
        const int begin = per_turn, end = 3 * per_turn, samples = 128;
        double length = 0, cruise_time = 0, minimum_speed = 1e300, maximum_speed = 0;
        double radial_error = 0;
        for (int segment = begin; segment < end; ++segment)
        {
            const double duration = result.trajectory.times(segment), h = duration / samples;
            cruise_time += duration;
            for (int k = 0; k <= samples; ++k)
            {
                const double t = h * k;
                const auto p = result.trajectory.evaluate(segment, t);
                const double speed = result.trajectory.evaluate(segment, t, 1).norm();
                minimum_speed = std::min(minimum_speed, speed);
                maximum_speed = std::max(maximum_speed, speed);
                radial_error = std::max(radial_error, std::abs(p.head<2>().norm() - radius));
                // Composite Simpson integration of actual polynomial arc length.
                length += h / 3 * (k == 0 || k == samples ? 1 : (k % 2 ? 4 : 2)) * speed;
            }
        }
        std::cout << std::setprecision(17)
                  << "{\"mean_speed\":" << length / cruise_time
                  << ",\"cruise_length\":" << length << ",\"cruise_time\":" << cruise_time
                  << ",\"minimum_speed\":" << minimum_speed
                  << ",\"maximum_speed\":" << maximum_speed
                  << ",\"radial_error\":" << radial_error
                  << ",\"peak_velocity\":" << result.peak_bounds(0)
                  << ",\"peak_acceleration\":" << result.peak_bounds(1)
                  << ",\"peak_jerk\":" << result.peak_bounds(2)
                  << ",\"total_time\":" << result.trajectory.times.sum()
                  << ",\"planning_ms\":" << result.planning_ms
                  << ",\"segments\":" << result.trajectory.times.size() << "}\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "offline circle probe: " << e.what() << '\n';
        return 1;
    }
}
