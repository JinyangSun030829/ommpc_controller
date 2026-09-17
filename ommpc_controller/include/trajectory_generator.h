#ifndef OMMPC_TRAJECTORY_GENERATOR_H
#define OMMPC_TRAJECTORY_GENERATOR_H

#include <Eigen/Core>
#include <functional>
#include <vector>

namespace minisnap {
using CancelCheck = std::function<bool()>;

// Both layouts are descending powers, with x/y/z blocks in each segment row.
// Normalized coefficients use tau=t/T; coefficients use physical seconds and
// retain the legacy Float64MultiArray wire format.
struct PolynomialTrajectory {
    int order = 0;
    Eigen::VectorXd times;
    Eigen::MatrixXd normalized_coefficients;
    Eigen::MatrixXd coefficients;
    double relative_residual = 0.0;
    int system_nonzeros = 0;

    Eigen::Vector3d evaluate(int segment, double time, int derivative = 0) const;
    std::vector<double> serialize() const;
    // Conservative vector-norm upper bound over the ENTIRE segment, using
    // Bernstein convex hulls with adaptive subdivision (not point sampling).
    double derivativeBound(int segment, int derivative,
                           double relative_tolerance = 0.002,
                           int max_depth = 12) const;
};
}  // namespace minisnap

class TrajectoryGeneratorTool {
public:
    minisnap::PolynomialTrajectory Solve(
        int order, const Eigen::MatrixXd& path, const Eigen::MatrixXd& velocity,
        const Eigen::MatrixXd& acceleration, const Eigen::VectorXd& times,
        const minisnap::CancelCheck& cancelled = minisnap::CancelCheck()) const;

    // Compatibility entry point. Supported orders are 3 (jerk) and 4 (snap).
    Eigen::MatrixXd SolveQPClosedForm(
        int order, const Eigen::MatrixXd& path, const Eigen::MatrixXd& velocity,
        const Eigen::MatrixXd& acceleration, const Eigen::VectorXd& times) const;
};

#endif
