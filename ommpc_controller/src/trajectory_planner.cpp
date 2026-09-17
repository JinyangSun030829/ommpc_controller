#include "trajectory_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace minisnap {
void PlannerOptions::validate() const {
    const double positive[] = {max_velocity, max_acceleration, max_jerk,
        min_segment_time, duplicate_distance, max_total_time, bound_tolerance,
        compression_budget_ms, compression_step};
    for (double x : positive)
        if (!std::isfinite(x) || x <= 0.0)
            throw std::invalid_argument("planner limits/tolerances must be finite and positive");
    if ((order != 3 && order != 4) || !std::isfinite(seed_safety_ratio) ||
        seed_safety_ratio <= 0.0 || seed_safety_ratio >= 1.0 ||
        !std::isfinite(limit_margin) || limit_margin <= 0.0 || limit_margin > 1.0 ||
        min_segment_time < 1e-3 || max_total_time > 1e4 ||
        max_segments < 1 || max_segments > 10000 || max_iterations < 1 ||
        max_iterations > 100 || bound_max_depth < 0 || bound_max_depth > 20 ||
        compression_max_trials < 0 || compression_max_trials > 200 || compression_step >= 0.25)
        throw std::invalid_argument("invalid planner configuration");
}

TrajectoryPlanner::TrajectoryPlanner(const PlannerOptions& options) : options_(options) {
    options_.validate();
}

Eigen::VectorXd TrajectoryPlanner::allocateTimes(
    const Eigen::MatrixXd& path, const Eigen::Vector3d& initial_velocity) const {
    if (path.cols() != 3 || path.rows() < 2 || !path.allFinite() ||
        !initial_velocity.allFinite() || path.rows() - 1 > options_.max_segments)
        throw std::invalid_argument("invalid time-allocation input");
    const int segments = static_cast<int>(path.rows()) - 1;
    const double vmax = options_.max_velocity * options_.seed_safety_ratio;
    // Tangential/normal acceleration budgets; final VECTOR norm is checked below.
    const double acceleration = options_.max_acceleration * 0.65;
    const double jerk = options_.max_jerk * options_.seed_safety_ratio;
    Eigen::VectorXd distances(segments), speed = Eigen::VectorXd::Constant(segments + 1, vmax);
    for (int s = 0; s < segments; ++s) {
        distances(s) = (path.row(s + 1) - path.row(s)).norm();
        if (distances(s) <= 0.0 || !std::isfinite(distances(s)))
            throw std::invalid_argument("time allocation requires distinct consecutive points");
    }
    for (int i = 1; i < segments; ++i) {
        const Eigen::Vector3d before = (path.row(i) - path.row(i - 1)).transpose() / distances(i - 1);
        const Eigen::Vector3d after = (path.row(i + 1) - path.row(i)).transpose() / distances(i);
        const double cosine = std::max(-1.0, std::min(1.0, before.dot(after)));
        const double angle = std::acos(cosine);
        const double curvature = 2.0 * std::sin(angle * 0.5)
                                 / (0.5 * (distances(i - 1) + distances(i)));
        if (curvature > 1e-8) speed(i) = std::min(vmax, std::sqrt(acceleration / curvature));
        if (angle >= 2.6179938779914944) speed(i) = 0.0; // conservative seed, NOT a stop constraint
    }
    speed(0) = initial_velocity.norm();
    speed(segments) = 0.0;
    for (int i = 0; i < segments; ++i)
        speed(i + 1) = std::min(speed(i + 1),
            std::sqrt(speed(i) * speed(i) + 2.0 * acceleration * distances(i)));
    // Keep the actual start velocity fixed; do not silently replace it with a
    // backwards-pass speed. The polynomial validation handles that transition.
    for (int i = segments - 1; i > 0; --i)
        speed(i) = std::min(speed(i),
            std::sqrt(speed(i + 1) * speed(i + 1) + 2.0 * acceleration * distances(i)));

    Eigen::VectorXd times(segments);
    for (int i = 0; i < segments; ++i) {
        const double va = speed(i), vb = speed(i + 1), d = distances(i);
        const double cap = std::max(vmax, std::max(va, vb));
        const double peak = std::min(cap,
            std::max(std::max(va, vb), std::sqrt(acceleration * d + 0.5 * (va * va + vb * vb))));
        const double accelerating_distance = std::max(0.0, (peak * peak - va * va) / (2 * acceleration));
        const double braking_distance = std::max(0.0, (peak * peak - vb * vb) / (2 * acceleration));
        double duration = (2.0 * peak - va - vb) / acceleration
            + std::max(0.0, d - accelerating_distance - braking_distance) / std::max(peak, 1e-6);
        // For an initial speed too high for the seed acceleration budget, give
        // the first transition time to decelerate rather than a zero duration.
        duration = std::max(duration, std::fabs(vb - va) / acceleration);
        duration = std::max(duration, std::sqrt(2.0 * std::fabs(vb - va) / jerk));
        if (i == 0) {
            const Eigen::Vector3d direction = (path.row(1) - path.row(0)).transpose() / d;
            const double change = (direction * vb - initial_velocity).norm();
            duration = std::max(duration, change / acceleration);
            duration = std::max(duration, std::sqrt(2.0 * change / jerk));
        }
        times(i) = std::max(options_.min_segment_time, duration);
    }
    if (segments == 1 && initial_velocity.norm() < 1e-8) {
        // Only a single rest-to-rest segment has these fixed shape constants.
        // Never apply rest-to-rest seeds independently to a continuous path.
        const double d = distances(0);
        const double cv = options_.order == 4 ? 2.1875 : 1.875;
        const double ca = options_.order == 4 ? 7.513188404399293 : 5.773502691896258;
        const double cj = options_.order == 4 ? 52.5 : 60.0;
        times(0) = std::max(times(0), std::max(cv * d / vmax,
            std::max(std::sqrt(ca * d / options_.max_acceleration), std::cbrt(cj * d / jerk))));
    }
    if (options_.use_s_curve_seed && segments > 1 && initial_velocity.norm() < 1e-8) {
        // One continuous arc-length S-curve, NOT independent stops at every
        // waypoint. Smoothly resolve the start/end ramp even with dense points.
        // It is a timing seed only: discrete route curvature is approximate and
        // the optimized 3D polynomial must pass the full vector checks.
        std::vector<double> curvature;
        Eigen::VectorXd corner_cap=Eigen::VectorXd::Constant(segments+1,vmax);
        for(int i=1;i<segments;++i) {
            const Eigen::Vector3d before=(path.row(i)-path.row(i-1)).transpose()/distances(i-1);
            const Eigen::Vector3d after=(path.row(i+1)-path.row(i)).transpose()/distances(i);
            const double angle=std::acos(std::max(-1.0,std::min(1.0,before.dot(after))));
            if(angle>=2.6179938779914944)return times; // keep conservative reversal seed
            const double k=2.0*std::sin(angle*.5)/(.5*(distances(i-1)+distances(i)));
            curvature.push_back(k);
            if(k>1e-8)corner_cap(i)=std::min(vmax,std::sqrt(acceleration/k));
        }
        std::sort(curvature.begin(),curvature.end());
        // An isolated entry/exit corner must not slow every cruise segment.
        // Local geometric caps below retain the sharper-corner allowance.
        const double typical=curvature[static_cast<size_t>(.8*(curvature.size()-1))];
        double cap=typical>1e-8?std::min(vmax,std::sqrt(acceleration/typical)):vmax;
        const double length=distances.sum(), j=options_.max_jerk*.65, a=acceleration;
        double tj=std::min(a/j,std::sqrt(cap/j));
        double ta=std::max(0.0,cap/a-tj);
        double ramp=2*tj+ta;
        if(length<cap*ramp) {
            tj=std::cbrt(length/(2*j));
            if(j*tj<=a) { cap=j*tj*tj;ta=0; }
            else { cap=.5*(std::sqrt(std::pow(a*a/j,2)+4*a*length)-a*a/j);tj=a/j;ta=std::max(0.0,cap/a-tj); }
            ramp=2*tj+ta;
        }
        const double cruise=std::max(0.0,(length-cap*ramp)/cap);
        const double phase_time[]={tj,ta,tj,cruise,tj,ta,tj};
        const double phase_jerk[]={j,0,-j,0,-j,0,j};
        const double total=2*ramp+cruise;
        auto distanceAt=[&](double t) {
            double s=0,v=0,acc=0;
            for(int phase=0;phase<7;++phase) {
                const double h=std::max(0.0,std::min(t,phase_time[phase]));
                s+=v*h+.5*acc*h*h+phase_jerk[phase]*h*h*h/6;
                v+=acc*h+.5*phase_jerk[phase]*h*h;
                acc+=phase_jerk[phase]*h;
                t-=h;
                if(t<=0)break;
            }
            return s;
        };
        double cumulative=0,previous=0;
        for(int i=0;i<segments;++i) {
            cumulative+=distances(i);
            double low=previous,high=total;
            if(i+1<segments)for(int iteration=0;iteration<55;++iteration) {
                const double middle=.5*(low+high);
                if(distanceAt(middle)<cumulative)low=middle;else high=middle;
            }
            const double end=i+1==segments?total:.5*(low+high);
            // Do not inherit per-segment sqrt(delta-v/J) budgets here: those
            // wrongly reset the acceleration ramp at every dense waypoint.
            const double corner_time=distances(i)/std::min(corner_cap(i),corner_cap(i+1));
            times(i)=std::max(options_.min_segment_time,std::max(end-previous,corner_time));
            previous=end;
        }
    }
    return times;
}

PlanResult TrajectoryPlanner::plan(const Eigen::MatrixXd& input,
    const Eigen::Vector3d& initial_velocity, const Eigen::Vector3d& initial_acceleration,
    const CancelCheck& cancelled) const {
    const auto start = std::chrono::steady_clock::now();
    if (input.cols() != 3 || input.rows() < 2 || !input.allFinite() ||
        !initial_velocity.allFinite() || !initial_acceleration.allFinite() ||
        input.rows() > options_.max_segments + 1)
        throw std::invalid_argument("empty, oversized, or non-finite planning input");
    if (initial_velocity.norm() > options_.max_velocity ||
        initial_acceleration.norm() > options_.max_acceleration)
        throw std::invalid_argument("initial state already exceeds configured limits");
    if (cancelled && cancelled()) throw std::runtime_error("planning cancelled");
    PlanResult result;
    std::vector<Eigen::Vector3d> points;
    points.push_back(input.row(0).transpose());
    for (int i = 1; i < input.rows(); ++i) {
        const Eigen::Vector3d p = input.row(i).transpose();
        if ((p - points.back()).norm() <= options_.duplicate_distance) ++result.removed_duplicates;
        else points.push_back(p);
    }
    if (points.size() == 1) {
        if (initial_velocity.norm() > 1e-6 || initial_acceleration.norm() > 1e-6)
            throw std::invalid_argument("stationary path cannot absorb a moving initial state");
        points.push_back(points.front()); // valid constant hover polynomial
    }
    result.path.resize(points.size(), 3);
    double total_distance = 0.0;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        result.path.row(i) = points[i].transpose();
        if (i > 0) total_distance += (points[i] - points[i - 1]).norm();
    }
    // Conservative no-extra-space policy: we do not silently invent a braking
    // waypoint or assume that overshoot outside the requested route is allowed.
    if (initial_velocity.squaredNorm() > 2.0 * options_.max_acceleration * total_distance + 1e-9)
        throw std::invalid_argument("requested path is shorter than the conservative stopping distance; add a braking route");
    Eigen::VectorXd times = total_distance == 0.0
        ? Eigen::VectorXd::Constant(1, std::max(0.2, options_.min_segment_time))
        : allocateTimes(result.path, initial_velocity);
    Eigen::MatrixXd velocity = Eigen::MatrixXd::Zero(2, 3), acceleration = velocity;
    velocity.row(0) = initial_velocity.transpose();
    acceleration.row(0) = initial_acceleration.transpose();
    const Eigen::Vector3d limits(options_.max_velocity, options_.max_acceleration, options_.max_jerk);
    const Eigen::Vector3d acceptance = limits.cwiseMin(
        (limits * options_.limit_margin).cwiseMax(
            Eigen::Vector3d(initial_velocity.norm(), initial_acceleration.norm(), 0.0)));
    TrajectoryGeneratorTool generator;
    // Shared by feasibility and EVERY compression candidate. A local time
    // change couples the whole optimum, so no segment is exempt from checking.
    auto validateTrajectory = [&](const PolynomialTrajectory& trajectory) {
        const auto& duration = trajectory.times;
        Eigen::MatrixXd bounds(duration.size(), 3);
        double error = 0.0;
        for (int s = 0; s < duration.size(); ++s) {
            if (cancelled && cancelled()) throw std::runtime_error("planning cancelled");
            for (int d = 1; d <= 3; ++d)
                bounds(s, d - 1) = trajectory.derivativeBound(s, d, options_.bound_tolerance, options_.bound_max_depth);
            error = std::max(error, (trajectory.evaluate(s, 0.0) - result.path.row(s).transpose()).norm());
            error = std::max(error, (trajectory.evaluate(s, duration(s)) - result.path.row(s + 1).transpose()).norm());
            if (s + 1 < duration.size())
                for (int d = 1; d < options_.order; ++d)
                    error = std::max(error, (trajectory.evaluate(s, duration(s), d) - trajectory.evaluate(s + 1, 0.0, d)).norm());
        }
        for (int d = 1; d < options_.order; ++d) {
            const Eigen::Vector3d expected = d == 1 ? initial_velocity : (d == 2 ? initial_acceleration : Eigen::Vector3d::Zero());
            error = std::max(error, (trajectory.evaluate(0, 0.0, d) - expected).norm());
            error = std::max(error, trajectory.evaluate(duration.size() - 1, duration(duration.size() - 1), d).norm());
        }
        if (!std::isfinite(error) || error > 1e-5 || !bounds.allFinite())
            throw std::runtime_error("waypoint, continuity, boundary or derivative validation failed");
        trajectory.serialize();
        return bounds;
    };
    for (int iteration = 1; iteration <= options_.max_iterations; ++iteration) {
        if (cancelled && cancelled()) throw std::runtime_error("planning cancelled");
        if (!times.allFinite() || !std::isfinite(times.sum()) ||
            times.sum() > options_.max_total_time)
            throw std::runtime_error("retiming exceeds the configured maximum trajectory duration");
        result.trajectory = generator.Solve(options_.order, result.path, velocity,
                                             acceleration, times, cancelled);
        Eigen::MatrixXd segment_bounds = validateTrajectory(result.trajectory);
        result.peak_bounds = segment_bounds.colwise().maxCoeff().transpose();
        result.iterations = iteration;
        // Tiny tolerance only accommodates floating-point equality at a fixed
        // boundary state lying exactly on its configured limit.
        if ((result.peak_bounds.array() <= acceptance.array() + 1e-9).all()) {
            // The continuous seed fixes dense-route timing, but an isolated
            // entry/exit corner can make it conservative on a long sparse
            // route. Keep the shorter of TWO fully validated feasible seeds.
            // Disable comparison in the alternative to prevent recursion.
            if (options_.compare_legacy_seed && total_distance > 0.0 && times.size() > 1 &&
                initial_velocity.norm() < 1e-8 && initial_acceleration.norm() < 1e-8) {
                PlannerOptions alternative=options_;
                alternative.use_s_curve_seed=false;
                alternative.compare_legacy_seed=false;
                alternative.stationary_uniform_retime=false;
                alternative.compress_time=false;
                alternative.min_segment_time=std::max(.1,options_.min_segment_time);
                try {
                    auto legacy=TrajectoryPlanner(alternative).plan(result.path,initial_velocity,initial_acceleration,cancelled);
                    result.seed_comparison_solves=legacy.iterations;
                    if(legacy.trajectory.times.sum()<times.sum()) {
                        Eigen::MatrixXd candidate_bounds=validateTrajectory(legacy.trajectory);
                        result.trajectory=std::move(legacy.trajectory);
                        times=result.trajectory.times;
                        segment_bounds=std::move(candidate_bounds);
                        result.peak_bounds=segment_bounds.colwise().maxCoeff().transpose();
                        result.used_legacy_seed=true;
                    }
                } catch(const std::exception&) {
                    if(cancelled&&cancelled())throw std::runtime_error("planning cancelled");
                    // Alternative failure never invalidates the feasible seed.
                }
            }
            result.feasible_duration = times.sum();
            // Exact uniform scaling applies only to ZERO derivative boundaries.
            // Moving-state core API keeps its original feasibility policy.
            if (options_.compress_time && total_distance > 0.0 && initial_velocity.norm() < 1e-8 && initial_acceleration.norm() < 1e-8) {
                const auto compression_start = std::chrono::steady_clock::now();
                auto budgetAvailable = [&] {
                    if (cancelled && cancelled()) throw std::runtime_error("planning cancelled");
                    return result.compression_trials < options_.compression_max_trials &&
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - compression_start).count() < options_.compression_budget_ms;
                };
                auto requiredFactor = [&](const Eigen::Vector3d& b) {
                    return std::max(b(0) / acceptance(0), std::max(std::sqrt(b(1) / acceptance(1)), std::cbrt(b(2) / acceptance(2))));
                };
                auto tryTimes = [&](const Eigen::VectorXd& candidate_times) {
                    ++result.compression_trials;
                    if ((candidate_times.array() < options_.min_segment_time).any() ||
                        candidate_times.sum() >= times.sum() - 1e-8) return false;
                    try {
                        auto candidate = generator.Solve(options_.order, result.path, velocity, acceleration, candidate_times, cancelled);
                        Eigen::MatrixXd bounds = validateTrajectory(candidate);
                        const Eigen::Vector3d peaks = bounds.colwise().maxCoeff().transpose();
                        if ((peaks.array() > acceptance.array() + 1e-9).any()) return false;
                        // Commit only the completely checked candidate.
                        times = candidate_times;
                        result.trajectory = std::move(candidate);
                        result.peak_bounds = peaks;
                        segment_bounds = std::move(bounds);
                        return true;
                    } catch (const std::exception&) {
                        if (cancelled && cancelled()) throw std::runtime_error("planning cancelled");
                        return false; // retain the last feasible solution
                    }
                };
                const double uniform = std::max(options_.min_segment_time / times.minCoeff(), 1.005 * requiredFactor(result.peak_bounds));
                if (uniform < 1.0 - 1e-6 && budgetAvailable()) {
                    const Eigen::VectorXd candidate = times * uniform;
                    if (tryTimes(candidate)) result.uniform_time_factor = uniform;
                }
                double step = options_.compression_step;
                while (step >= 0.002 && budgetAvailable()) {
                    bool accepted = false;
                    // Rebuild slack groups after each pass. A threshold crossing
                    // or boundary-limited segment separates groups.
                    for (int begin = 0; begin < times.size() && budgetAvailable();) {
                        if (requiredFactor(segment_bounds.row(begin).transpose()) >= 1.0 - step ||
                            times(begin) * (1.0 - step) < options_.min_segment_time) { ++begin; continue; }
                        int end = begin + 1;
                        while (end < times.size() && requiredFactor(segment_bounds.row(end).transpose()) < 1.0 - step &&
                               times(end) * (1.0 - step) >= options_.min_segment_time) ++end;
                        Eigen::VectorXd candidate = times;
                        candidate.segment(begin, end - begin) *= 1.0 - step;
                        if (tryTimes(candidate)) { accepted = true; ++result.local_compressions; }
                        begin = end;
                    }
                    if (!accepted) step *= 0.5;
                }
                // Full final validation; budget is soft (one candidate can finish
                // after it), never a reason to skip checking.
                segment_bounds = validateTrajectory(result.trajectory);
                result.peak_bounds = segment_bounds.colwise().maxCoeff().transpose();
            }
            result.planning_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            result.trajectory.serialize(); // validate legacy representation before returning
            return result;
        }
        auto timeFactor = [&](const Eigen::Vector3d& bounds) {
            return std::min(3.0, std::max(1.02, 1.03 * std::max(
                bounds(0) / acceptance(0), std::max(
                    std::sqrt(bounds(1) / acceptance(1)), std::cbrt(bounds(2) / acceptance(2))))));
        };
        // Extend only violating segments first. In particular, extending an
        // already-feasible initial segment with fixed nonzero velocity can
        // create rebound/overshoot instead of reducing it.
        if ((options_.stationary_uniform_retime || iteration >= 8) && initial_velocity.norm() < 1e-8 && initial_acceleration.norm() < 1e-8) {
            // For stationary boundaries use exact scaling immediately. Local
            // extension of one dense ramp segment can create a new neighboring
            // jerk spike, propagating and greatly lengthening the route. Safe
            // local shortening is done later with commit-or-revert validation.
            times *= timeFactor(result.peak_bounds);
        } else {
            for (int s = 0; s < times.size(); ++s)
                if ((segment_bounds.row(s).transpose().array() > acceptance.array() + 1e-9).any())
                    times(s) *= timeFactor(segment_bounds.row(s).transpose());
        }
        // Every changed time set is re-solved and conservatively rechecked;
        // local retiming is a feasibility heuristic, not a convergence proof.
    }
    std::ostringstream error;
    error << "retiming did not converge after " << result.iterations
          << " iterations; derivative upper bounds=" << result.peak_bounds.transpose();
    throw std::runtime_error(error.str());
}
}  // namespace minisnap
