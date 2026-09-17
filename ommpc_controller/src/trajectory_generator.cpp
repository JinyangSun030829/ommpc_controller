#include "trajectory_generator.h"

#include <Eigen/LU>
#include <Eigen/Sparse>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
double falling(int n, int d) {
    double x = 1.0;
    for (int j = 0; j < d; ++j) x *= n - j;
    return x;
}
double choose(int n, int k) {
    double x = 1.0;
    for (int j = 1; j <= k; ++j) x *= double(n - k + j) / j;
    return x;
}
void checkCancel(const minisnap::CancelCheck& check) {
    if (check && check()) throw std::runtime_error("planning cancelled");
}
using Controls = Eigen::Matrix<double, Eigen::Dynamic, 3>;

// de Casteljau subdivision gives another exact convex-hull upper bound.
double boundBezier(const Controls& c, double tolerance, int depth) {
    double upper = 0.0;
    for (int i = 0; i < c.rows(); ++i) upper = std::max(upper, c.row(i).norm());
    Controls work = c, left(c.rows(), 3), right(c.rows(), 3);
    const int degree = static_cast<int>(c.rows()) - 1;
    left.row(0) = work.row(0);
    right.row(degree) = work.row(degree);
    for (int level = 1; level <= degree; ++level) {
        for (int j = 0; j <= degree - level; ++j)
            work.row(j) = (0.5 * (work.row(j) + work.row(j + 1))).eval();
        left.row(level) = work.row(0);
        right.row(degree - level) = work.row(degree - level);
    }
    const double lower = std::max(c.row(0).norm(),
        std::max(c.row(degree).norm(), work.row(0).norm()));
    if (depth == 0 || upper <= lower * (1.0 + tolerance) + 1e-10)
        return upper;
    return std::max(boundBezier(left, tolerance, depth - 1),
                    boundBezier(right, tolerance, depth - 1));
}
}  // namespace

namespace minisnap {
Eigen::Vector3d PolynomialTrajectory::evaluate(int segment, double time,
                                              int derivative) const {
    if (segment < 0 || segment >= times.size() || derivative < 0 ||
        derivative >= 2 * order || !std::isfinite(time))
        throw std::invalid_argument("invalid polynomial evaluation");
    const int n = 2 * order;
    const double tau = std::max(0.0, std::min(1.0, time / times(segment)));
    Eigen::Vector3d value = Eigen::Vector3d::Zero();
    for (int axis = 0; axis < 3; ++axis) {
        double x = 0.0;
        for (int power = n - 1; power >= derivative; --power)
            x = x * tau + normalized_coefficients(segment, axis * n + n - 1 - power)
                          * falling(power, derivative);
        value(axis) = x / std::pow(times(segment), derivative);
    }
    return value;
}

double PolynomialTrajectory::derivativeBound(int segment, int derivative,
                                            double tolerance, int depth) const {
    if (segment < 0 || segment >= times.size() || derivative < 1 ||
        derivative >= 2 * order || !std::isfinite(tolerance) || tolerance <= 0.0 ||
        depth < 0 || depth > 20)
        throw std::invalid_argument("invalid derivative-bound request");
    const int n = 2 * order, degree = n - 1 - derivative;
    Controls power = Controls::Zero(degree + 1, 3);
    for (int k = 0; k <= degree; ++k)
        for (int axis = 0; axis < 3; ++axis)
            power(k, axis) = normalized_coefficients(
                segment, axis * n + n - 1 - k - derivative)
                * falling(k + derivative, derivative)
                / std::pow(times(segment), derivative);
    Controls bernstein = Controls::Zero(degree + 1, 3);
    for (int i = 0; i <= degree; ++i)
        for (int k = 0; k <= i; ++k)
            bernstein.row(i) += power.row(k) * choose(i, k) / choose(degree, k);
    if (!bernstein.allFinite()) throw std::runtime_error("non-finite derivative bound");
    // Small round-off allowance; this is a numerical certificate, not formal
    // interval arithmetic. Validation also checks boundary/continuity residuals.
    const double roundoff = 128.0 * std::numeric_limits<double>::epsilon()
                            * (1.0 + power.cwiseAbs().sum());
    return boundBezier(bernstein, tolerance, depth) + roundoff;
}

std::vector<double> PolynomialTrajectory::serialize() const {
    if ((order != 3 && order != 4) || times.size() < 1 ||
        coefficients.rows() != times.size() || coefficients.cols() != 6 * order ||
        !times.allFinite() || (times.array() <= 0.0).any() ||
        !std::isfinite(times.sum()) || !coefficients.allFinite())
        throw std::runtime_error("invalid trajectory serialization");
    std::vector<double> values;
    values.reserve(2 + times.size() + coefficients.size());
    values.push_back(times.size());
    values.push_back(2 * order);
    for (int i = 0; i < times.size(); ++i) values.push_back(times(i));
    for (int i = 0; i < coefficients.rows(); ++i)
        for (int j = 0; j < coefficients.cols(); ++j)
            values.push_back(coefficients(i, j));
    return values;
}
}  // namespace minisnap

minisnap::PolynomialTrajectory TrajectoryGeneratorTool::Solve(
    int order, const Eigen::MatrixXd& path, const Eigen::MatrixXd& velocity,
    const Eigen::MatrixXd& acceleration, const Eigen::VectorXd& times,
    const minisnap::CancelCheck& cancelled) const {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;
    if ((order != 3 && order != 4) || times.size() < 1 || times.size() > 10000 ||
        path.rows() != times.size() + 1 || path.cols() != 3 ||
        velocity.rows() != 2 || velocity.cols() != 3 ||
        acceleration.rows() != 2 || acceleration.cols() != 3 ||
        !path.allFinite() || !velocity.allFinite() || !acceleration.allFinite() ||
        !times.allFinite() || (times.array() < 1e-3).any() ||
        (times.array() > 1e4).any() || !std::isfinite(times.sum()))
        throw std::invalid_argument("invalid order, boundaries, path, or segment times");
    checkCancel(cancelled);
    const int segments = static_cast<int>(times.size()), n = 2 * order;
    const int free_count = (segments - 1) * (order - 1);

    // Constant normalized Hermite map; never invert a global matrix.
    MatrixXd mapping = MatrixXd::Zero(n, n), q = MatrixXd::Zero(n, n);
    for (int d = 0; d < order; ++d) {
        mapping(d, n - 1 - d) = falling(d, d);
        for (int power = d; power < n; ++power)
            mapping(order + d, n - 1 - power) = falling(power, d);
    }
    for (int a = order; a < n; ++a)
        for (int b = order; b < n; ++b)
            q(n - 1 - a, n - 1 - b) =
                falling(a, order) * falling(b, order) / double(a + b - 2 * order + 1);
    const Eigen::FullPivLU<MatrixXd> hermite(mapping);
    if (!hermite.isInvertible()) throw std::runtime_error("Hermite map is singular");
    const MatrixXd basis = hermite.solve(MatrixXd::Identity(n, n));
    const Eigen::RowVector3d origin = path.row(0);
    const MatrixXd relative_path = path.rowwise() - origin;
    std::vector<MatrixXd> maps;
    maps.reserve(segments);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(segments * 4 * (order - 1) * (order - 1));
    MatrixXd rhs = MatrixXd::Zero(free_count, 3);

    auto index = [segments, order](int knot, int d) {
        return (d == 0 || knot == 0 || knot == segments)
            ? -1 : (knot - 1) * (order - 1) + d - 1;
    };
    auto fixed = [&](int segment) {
        MatrixXd values = MatrixXd::Zero(n, 3);
        values.row(0) = relative_path.row(segment);
        values.row(order) = relative_path.row(segment + 1);
        if (segment == 0) {
            values.row(1) = velocity.row(0);
            values.row(2) = acceleration.row(0);
        }
        if (segment + 1 == segments) {
            values.row(order + 1) = velocity.row(1);
            values.row(order + 2) = acceleration.row(1);
        }
        // In order 4, boundary jerk is fixed to zero.
        return values;
    };

    for (int s = 0; s < segments; ++s) {
        if (s % 32 == 0) checkCancel(cancelled);
        VectorXd scales(n);
        for (int d = 0; d < order; ++d)
            scales(d) = scales(order + d) = std::pow(times(s), d);
        maps.push_back(basis * scales.asDiagonal());
        MatrixXd h = maps.back().transpose() * q * maps.back();
        h *= std::pow(times(s), 1 - 2 * order);
        h = (0.5 * (h + h.transpose())).eval();
        if (!h.allFinite()) throw std::runtime_error("non-finite local Hessian");
        const MatrixXd values = fixed(s);
        const MatrixXd load = -h * values;
        for (int a = 0; a < n; ++a) {
            const int ia = index(s + a / order, a % order);
            if (ia < 0) continue;
            rhs.row(ia) += load.row(a);
            for (int b = 0; b < n; ++b) {
                const int ib = index(s + b / order, b % order);
                if (ib >= 0) triplets.emplace_back(ia, ib, h(a, b));
            }
        }
    }

    minisnap::PolynomialTrajectory trajectory;
    trajectory.order = order;
    trajectory.times = times;
    MatrixXd solution = MatrixXd::Zero(free_count, 3);
    if (free_count > 0) {
        Eigen::SparseMatrix<double> h(free_count, free_count);
        h.setFromTriplets(triplets.begin(), triplets.end());
        VectorXd scale(free_count);
        for (int i = 0; i < free_count; ++i) {
            const double diagonal = h.coeff(i, i);
            if (!std::isfinite(diagonal) || diagonal <= 0.0)
                throw std::runtime_error("invalid free-system diagonal");
            scale(i) = 1.0 / std::sqrt(diagonal);
        }
        for (int col = 0; col < h.outerSize(); ++col)
            for (Eigen::SparseMatrix<double>::InnerIterator entry(h, col); entry; ++entry)
                entry.valueRef() *= scale(entry.row()) * scale(entry.col());
        h.makeCompressed();
        const MatrixXd scaled_rhs = scale.asDiagonal() * rhs;
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(h);
        if (solver.info() != Eigen::Success || !solver.vectorD().allFinite() ||
            (solver.vectorD().array() <= 0.0).any())
            throw std::runtime_error("free-system LDLT failed or is not positive definite");
        const MatrixXd x = solver.solve(scaled_rhs);  // all three axes, one factorization
        if (solver.info() != Eigen::Success || !x.allFinite())
            throw std::runtime_error("free-system solve failed");
        trajectory.relative_residual = (h * x - scaled_rhs).norm()
                                       / std::max(1.0, scaled_rhs.norm());
        if (!std::isfinite(trajectory.relative_residual) || trajectory.relative_residual > 1e-7)
            throw std::runtime_error("free-system residual exceeds tolerance");
        trajectory.system_nonzeros = static_cast<int>(h.nonZeros());
        solution = scale.asDiagonal() * x;
    }

    trajectory.normalized_coefficients.resize(segments, 3 * n);
    trajectory.coefficients.resize(segments, 3 * n);
    for (int s = 0; s < segments; ++s) {
        if (s % 32 == 0) checkCancel(cancelled);
        MatrixXd values = fixed(s);
        for (int d = 0; d < n; ++d) {
            const int i = index(s + d / order, d % order);
            if (i >= 0) values.row(d) = solution.row(i);
        }
        MatrixXd coefficients = maps[s] * values;
        coefficients.row(n - 1) += origin;
        for (int axis = 0; axis < 3; ++axis)
            for (int k = 0; k < n; ++k) {
                trajectory.normalized_coefficients(s, axis * n + k) = coefficients(k, axis);
                trajectory.coefficients(s, axis * n + k) =
                    coefficients(k, axis) / std::pow(times(s), n - 1 - k);
            }
    }
    if (!trajectory.normalized_coefficients.allFinite() || !trajectory.coefficients.allFinite())
        throw std::runtime_error("non-finite polynomial coefficients");
    return trajectory;
}

Eigen::MatrixXd TrajectoryGeneratorTool::SolveQPClosedForm(
    int order, const Eigen::MatrixXd& path, const Eigen::MatrixXd& velocity,
    const Eigen::MatrixXd& acceleration, const Eigen::VectorXd& times) const {
    return Solve(order, path, velocity, acceleration, times).coefficients;
}
