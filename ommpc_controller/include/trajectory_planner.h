#ifndef OMMPC_TRAJECTORY_PLANNER_H
#define OMMPC_TRAJECTORY_PLANNER_H

#include "trajectory_generator.h"
#include <string>

namespace minisnap {
struct PlannerOptions {
    int order = 4;
    double max_velocity = 3.0;
    double max_acceleration = 2.0;
    double max_jerk = 4.0;
    double seed_safety_ratio = 0.9;
    double limit_margin = 0.98;
    double min_segment_time = 0.001; // numerical floor, not a per-waypoint dwell
    double duplicate_distance = 1e-4;
    double max_total_time = 3600.0;
    int max_segments = 2000;
    int max_iterations = 30;
    double bound_tolerance = 0.002;
    int bound_max_depth = 12;
    bool compress_time = true;
    int compression_max_trials = 32;
    double compression_budget_ms = 50.0;
    double compression_step = 0.05;
    bool use_s_curve_seed = true;
    bool compare_legacy_seed = true;
    bool stationary_uniform_retime = true;
    void validate() const;
};

struct PlanResult {
    PolynomialTrajectory trajectory;
    Eigen::MatrixXd path;
    Eigen::Vector3d peak_bounds = Eigen::Vector3d::Zero(); // speed, acceleration, jerk
    int iterations = 0;
    int removed_duplicates = 0;
    double planning_ms = 0.0;
    double feasible_duration = 0.0;
    double uniform_time_factor = 1.0;
    int compression_trials = 0;
    int local_compressions = 0;
    int seed_comparison_solves = 0;
    bool used_legacy_seed = false;
};

// ROS-independent: sanitization, time seed, polynomial optimization and
// conservative derivative validation. Throws on invalid/infeasible inputs;
// never returns an unvalidated trajectory.
class TrajectoryPlanner {
public:
    explicit TrajectoryPlanner(const PlannerOptions& options = PlannerOptions());
    PlanResult plan(const Eigen::MatrixXd& path,
                    const Eigen::Vector3d& initial_velocity,
                    const Eigen::Vector3d& initial_acceleration = Eigen::Vector3d::Zero(),
                    const CancelCheck& cancelled = CancelCheck()) const;
    Eigen::VectorXd allocateTimes(const Eigen::MatrixXd& path,
                                 const Eigen::Vector3d& initial_velocity) const;
private:
    PlannerOptions options_;
};
}  // namespace minisnap
#endif
