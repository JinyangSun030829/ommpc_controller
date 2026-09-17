#ifndef OMMPC_BOX_FLIGHT_SAFETY_HPP
#define OMMPC_BOX_FLIGHT_SAFETY_HPP

#include "flight_safety.hpp"
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace flight_safety
{
// World-frame metres. No implicit TF, obstacle avoidance, or hardware certification.
struct BoxOptions
{
    bool enabled = false;
    Eigen::Vector3d minimum = Eigen::Vector3d(-8, -8, -.5);
    Eigen::Vector3d maximum = Eigen::Vector3d(8, 8, 4);
    Eigen::Vector3d margin = Eigen::Vector3d::Zero();
    double check_interval = .05;
    double lookahead = .5;
    double reaction_time = .15;
    double max_stop_duration = 12;
    int lookahead_samples = 2;
    int duration_trials = 80;
    int subdivision_depth = 6;
    int path_stop_segments = 16;

    // Contract only horizontal faces; Z bounds must still contain the LAND target.
    Eigen::Vector3d effectiveMargin() const
    {
        return Eigen::Vector3d(margin.x(), margin.y(), 0.0);
    }

    void validate() const
    {
        if (!minimum.allFinite() || !maximum.allFinite() || !margin.allFinite() ||
            (margin.array() < 0).any() ||
            (maximum - minimum - 2 * effectiveMargin()).minCoeff() <= 0)
            throw std::invalid_argument("invalid flight_box bounds/margin");
        for (double value : {check_interval, lookahead, reaction_time, max_stop_duration})
            if (!std::isfinite(value) || value <= 0)
                throw std::invalid_argument("invalid flight_box timing");
        if (lookahead < check_interval + reaction_time || lookahead_samples < 1 ||
            lookahead_samples > 8 || duration_trials < 8 || duration_trials > 120 ||
            subdivision_depth < 1 || subdivision_depth > 10 || path_stop_segments < 4 ||
            path_stop_segments > 64)
            throw std::invalid_argument("invalid flight_box search/preview settings");
    }

    bool contains(const Eigen::Vector3d &p, bool inset = true) const
    {
        if (!p.allFinite())
            return false;
        const Eigen::Vector3d reserve = inset ? effectiveMargin() : Eigen::Vector3d::Zero();
        return (p.array() >= (minimum + reserve).array() - 1e-9).all() &&
               (p.array() <= (maximum - reserve).array() + 1e-9).all();
    }

    Eigen::Vector3d clampInside(const Eigen::Vector3d &p) const
    {
        const Eigen::Vector3d reserve = effectiveMargin();
        return p.cwiseMax(minimum + reserve).cwiseMin(maximum - reserve);
    }
};

namespace box_detail
{
using Points = std::vector<Eigen::Vector3d>;

inline Eigen::Vector3d evaluate(Points points, double s)
{
    for (std::size_t count = points.size(); count > 1; --count)
        for (std::size_t i = 0; i + 1 < count; ++i)
            points[i] = (1 - s) * points[i] + s * points[i + 1];
    return points.front();
}

inline void split(const Points &points, Points &left, Points &right)
{
    Points work = points;
    left.resize(points.size());
    right.resize(points.size());
    for (std::size_t level = 0; level < points.size(); ++level)
    {
        left[level] = work.front();
        right[points.size() - 1 - level] = work[points.size() - 1 - level];
        for (std::size_t i = 0; i + 1 < points.size() - level; ++i)
            work[i] = .5 * (work[i] + work[i + 1]);
    }
}

inline Points derivative(const Points &points, double duration)
{
    Points result(points.size() - 1);
    for (std::size_t i = 0; i < result.size(); ++i)
        result[i] = (points.size() - 1) * (points[i + 1] - points[i]) / duration;
    return result;
}

// A convex hull certificate, not a discrete collision sample. Subdivision
// tightens conservative control-point bounds without losing continuous-time coverage.
inline bool inside(const Points &points, const BoxOptions &box, int depth)
{
    if (!box.contains(points.front()) || !box.contains(points.back()))
        return false;
    bool all = true;
    for (const auto &p : points)
        all = all && box.contains(p);
    if (all)
        return true;
    if (depth == 0)
        return false;
    Points left, right;
    split(points, left, right);
    return inside(left, box, depth - 1) && inside(right, box, depth - 1);
}

inline bool bounded(const Points &points, double limit, int depth, double &bound)
{
    double hull = 0;
    for (const auto &point : points)
    {
        if (!point.allFinite())
            return false;
        hull = std::max(hull, point.norm());
    }
    if (hull <= limit + 1e-9)
    {
        bound = std::max(bound, hull);
        return true;
    }
    if (depth == 0 || points.front().norm() > limit + 1e-9 || points.back().norm() > limit + 1e-9)
        return false;
    Points left, right;
    split(points, left, right);
    return bounded(left, limit, depth - 1, bound) && bounded(right, limit, depth - 1, bound);
}
} // namespace box_detail

using Preview = std::function<bool(double, Reference &)>;

// Follow the nominal curve, with a constant-velocity tracking-error forecast.
// This is a model, not a measured acceleration or a disturbance guarantee.
inline Preview alignedPreview(const Reference &seed, const Reference &nominal,
                              const Preview &preview)
{
    if (!preview)
        return Preview();
    const Eigen::Vector3d dp = seed.p - nominal.p;
    const Eigen::Vector3d dv = seed.v - nominal.v;
    const Eigen::Vector3d da = seed.a - nominal.a;
    return [=](double t, Reference &r) {
        if (!preview(t, r))
            return false;
        r.p += dp + t * dv + .5 * t * t * da;
        r.v += dv + t * da;
        r.a += da;
        return r.p.allFinite() && r.v.allFinite() && r.a.allFinite();
    };
}

// Same interface as SmoothStop. When enabled, finite candidate search supplies
// only continuously certified curves; failure is NOT proof of global infeasibility.
class BoxStop
{
  public:
    void configure(const BoxOptions &options)
    {
        options.validate();
        box_ = options;
    }
    bool tryStart(const Reference &seed, double max_a, double max_j,
                  const Preview &preview = Preview())
    {
        valid_ = false;
        path_segments_.clear();
        path_mode_ = false;
        reason_.clear();
        if (!seed.p.allFinite() || !seed.v.allFinite() || !seed.a.allFinite() ||
            !std::isfinite(max_a) || !std::isfinite(max_j) || max_a <= 0 || max_j <= 0)
        {
            reason_ = "invalid stopping state/limits";
            return false;
        }
        if (!box_.enabled)
        {
            legacy_.start(seed, max_a, max_j);
            legacy_mode_ = true;
            valid_ = true;
            return true;
        }
        legacy_mode_ = false;
        if (!box_.contains(seed.p))
        {
            reason_ = "stop entry outside inset box";
            return false;
        }
        if (seed.a.norm() > max_a + 1e-9)
        {
            reason_ = "entry acceleration exceeds configured braking limit";
            return false;
        }
        if (seed.v.norm() < 1e-9 && seed.a.norm() < 1e-9)
        {
            points_.assign(6, seed.p);
            velocity_ = box_detail::derivative(points_, .05);
            acceleration_ = box_detail::derivative(velocity_, .05);
            jerk_ = box_detail::derivative(acceleration_, .05);
            duration_ = .05;
            acceleration_bound_ = jerk_bound_ = 0;
            acceleration_limit_ = max_a;
            valid_ = true;
            return true;
        }
        // A curved stop is attempted before the generic endpoint search.
        // Only the reconstructed, continuously certified reference is executed.
        if (preview && tryPathStop(seed, max_a, max_j, preview))
            return true;
        const Eigen::Vector3d center = .5 * (box_.minimum + box_.maximum);
        double duration = .05;
        for (int trial = 0; trial < box_.duration_trials && duration <= box_.max_stop_duration;
             ++trial, duration *= 1.075)
        {
            const Eigen::Vector3d natural =
                seed.p + .4 * duration * seed.v + .05 * duration * duration * seed.a;
            // Degree 5 reproduces the existing stop, but uses tight subdivided
            // derivative bounds instead of unnecessarily long coarse hull bounds.
            box_detail::Points free_stop(6, natural);
            free_stop[0] = seed.p;
            free_stop[1] = seed.p + duration * seed.v / 5;
            if (accept(free_stop, duration, max_a, max_j))
                return true;
            // Degree 7 allows a different endpoint and a curved stop, while
            // preserving entry p/v/a, with terminal v/a/j all exactly zero.
            const Eigen::Vector3d displacement = natural - seed.p;
            std::vector<Eigen::Vector3d> goals{box_.clampInside(natural),
                                               box_.clampInside(seed.p + .75 * displacement),
                                               box_.clampInside(seed.p + .5 * displacement),
                                               box_.clampInside(seed.p),
                                               box_.clampInside(seed.p + .25 * (center - seed.p)),
                                               box_.clampInside(seed.p + .5 * (center - seed.p)),
                                               center};
            for (const auto &goal : goals)
            {
                box_detail::Points candidate(8, goal);
                candidate[0] = seed.p;
                candidate[1] = seed.p + duration * seed.v / 7;
                candidate[2] =
                    seed.p + 2 * duration * seed.v / 7 + duration * duration * seed.a / 42;
                candidate[3] =
                    seed.p + 3 * duration * seed.v / 7 + duration * duration * seed.a / 14;
                if (accept(candidate, duration, max_a, max_j))
                    return true;
            }
        }
        reason_ = "no certified box stop found within finite search limits";
        return false;
    }

    void start(const Reference &seed, double max_a, double max_j)
    {
        if (!tryStart(seed, max_a, max_j))
            throw std::runtime_error(reason_);
    }
    Reference at(double time) const
    {
        if (!valid_)
            throw std::logic_error("uninitialized box stop");
        if (legacy_mode_)
            return legacy_.at(time);
        if (path_mode_)
        {
            const auto &part = pathPart(time);
            const double s = std::max(0.0, std::min(1.0, (time - part.start) / part.duration));
            Reference r;
            r.p = box_detail::evaluate(part.p, s);
            if (time < duration_)
            {
                r.v = box_detail::evaluate(part.v, s);
                r.a = box_detail::evaluate(part.a, s);
            }
            return r;
        }
        const double s = std::max(0.0, std::min(1.0, time / duration_));
        Reference r;
        r.p = box_detail::evaluate(points_, s);
        if (time < duration_)
        {
            r.v = box_detail::evaluate(velocity_, s);
            r.a = box_detail::evaluate(acceleration_, s);
        }
        return r;
    }
    Eigen::Vector3d jerk(double time) const
    {
        if (!valid_)
            throw std::logic_error("uninitialized box stop");
        if (legacy_mode_)
            return legacy_.jerk(time);
        if (time >= duration_)
            return Eigen::Vector3d::Zero();
        if (path_mode_)
        {
            const auto &part = pathPart(time);
            return box_detail::evaluate(
                part.j, std::max(0.0, std::min(1.0, (time - part.start) / part.duration)));
        }
        return box_detail::evaluate(jerk_, std::max(0.0, time / duration_));
    }
    double duration() const
    {
        return legacy_mode_ ? legacy_.duration() : duration_;
    }
    double accelerationBound() const
    {
        return legacy_mode_ ? legacy_.accelerationBound() : acceleration_bound_;
    }
    double jerkBound() const
    {
        return legacy_mode_ ? legacy_.jerkBound() : jerk_bound_;
    }
    double accelerationLimit() const
    {
        return legacy_mode_ ? legacy_.accelerationLimit() : acceleration_limit_;
    }
    const std::string &reason() const
    {
        return reason_;
    }
    bool followsPath() const
    {
        return path_mode_;
    }
    bool certifiesRemaining(const Reference &r, double elapsed, const BoxOptions &box, double max_a,
                            double max_j) const
    {
        if (!valid_ || legacy_mode_ || elapsed < 0 || !std::isfinite(elapsed) ||
            !box_.minimum.isApprox(box.minimum, 1e-12) ||
            !box_.maximum.isApprox(box.maximum, 1e-12) ||
            !box_.effectiveMargin().isApprox(box.effectiveMargin(), 1e-12) ||
            acceleration_bound_ > max_a + 1e-9 || jerk_bound_ > max_j + 1e-9)
            return false;
        const auto nominal = at(elapsed);
        // This shortcut applies only to the already certified nominal state.
        // Actual deviations must still pass a separate planning certificate.
        return (r.p - nominal.p).norm() <= 1e-9 && (r.v - nominal.v).norm() <= 1e-9 &&
               (r.a - nominal.a).norm() <= 1e-9;
    }

  private:
    struct PathPart
    {
        box_detail::Points p, v, a, j;
        double start = 0, duration = 0;
    };

    const PathPart &pathPart(double time) const
    {
        for (const auto &part : path_segments_)
            if (time < part.start + part.duration)
                return part;
        return path_segments_.back();
    }

    bool tryPathStop(const Reference &seed, double max_a, double max_j, const Preview &preview)
    {
        Reference origin;
        if (!preview(0, origin) || !origin.p.allFinite() || !origin.v.allFinite() ||
            !origin.a.allFinite())
            return false;
        double duration = .05;
        for (int trial = 0; trial < box_.duration_trials && duration <= box_.max_stop_duration;
             ++trial, duration *= 1.075)
        {
            for (double blend : {1.0, 0.0})
            {
                // q'(s)=1-3s^2+2s^3; q'(0)=1, q'(1)=0, q'' endpoints=0.
                // Thus the path keeps turning while progressively slowing to rest.
                box_detail::Points correction(6, Eigen::Vector3d::Zero());
                correction[0] = seed.p - origin.p;
                correction[1] = correction[0] + duration * (seed.v - origin.v) / 5;
                correction[2] = correction[0] + 2 * duration * (seed.v - origin.v) / 5 +
                                duration * duration * (seed.a - origin.a) / 20;
                const auto cv = box_detail::derivative(correction, duration);
                const auto ca = box_detail::derivative(cv, duration);
                auto sample = [&](int i, Reference &r) {
                    const double s = double(i) / box_.path_stop_segments;
                    // blend=0 also tries the unwarped remaining path: useful when
                    // the preview itself is already a certified stopping manoeuvre.
                    const double phase = duration * (s + blend * (-s * s * s + .5 * s * s * s * s));
                    const double rate = 1 + blend * (-3 * s * s + 2 * s * s * s);
                    const double rate_dot = blend * (-6 * s + 6 * s * s) / duration;
                    Reference original;
                    if (!preview(phase, original))
                        return false;
                    r.p = original.p + box_detail::evaluate(correction, s);
                    r.v = original.v * rate + box_detail::evaluate(cv, s);
                    r.a = original.a * rate * rate + original.v * rate_dot +
                          box_detail::evaluate(ca, s);
                    if (i == 0)
                        r = seed;
                    if (i == box_.path_stop_segments)
                    {
                        r.v.setZero();
                        r.a.setZero();
                    }
                    return r.p.allFinite() && r.v.allFinite() && r.a.allFinite();
                };
                Reference left;
                if (!sample(0, left))
                    continue;
                std::vector<PathPart> parts;
                double a_bound = 0, j_bound = 0;
                const double h = duration / box_.path_stop_segments;
                bool certified = true;
                for (int i = 0; i < box_.path_stop_segments; ++i)
                {
                    Reference right;
                    if (!sample(i + 1, right))
                    {
                        certified = false;
                        break;
                    }
                    PathPart part;
                    part.start = i * h;
                    part.duration = h;
                    // Quintic Hermite pieces preserve p/v/a at every join.
                    part.p = {left.p,
                              left.p + h * left.v / 5,
                              left.p + 2 * h * left.v / 5 + h * h * left.a / 20,
                              right.p - 2 * h * right.v / 5 + h * h * right.a / 20,
                              right.p - h * right.v / 5,
                              right.p};
                    part.v = box_detail::derivative(part.p, h);
                    part.a = box_detail::derivative(part.v, h);
                    part.j = box_detail::derivative(part.a, h);
                    if (i + 1 == box_.path_stop_segments)
                    {
                        // Last piece also matches its initial jerk and reaches jerk=0.
                        const Eigen::Vector3d j0 = part.j.front();
                        part.p = {left.p,
                                  left.p + h * left.v / 7,
                                  left.p + 2 * h * left.v / 7 + h * h * left.a / 42,
                                  left.p + 3 * h * left.v / 7 + h * h * left.a / 14 +
                                      h * h * h * j0 / 210,
                                  right.p,
                                  right.p,
                                  right.p,
                                  right.p};
                        part.v = box_detail::derivative(part.p, h);
                        part.a = box_detail::derivative(part.v, h);
                        part.j = box_detail::derivative(part.a, h);
                    }
                    if (!box_detail::inside(part.p, box_, box_.subdivision_depth) ||
                        !box_detail::bounded(part.a, max_a, box_.subdivision_depth, a_bound) ||
                        !box_detail::bounded(part.j, max_j, box_.subdivision_depth, j_bound))
                    {
                        certified = false;
                        break;
                    }
                    parts.push_back(std::move(part));
                    left = right;
                }
                if (certified)
                {
                    path_segments_ = std::move(parts);
                    path_mode_ = valid_ = true;
                    duration_ = duration;
                    acceleration_bound_ = a_bound;
                    jerk_bound_ = j_bound;
                    acceleration_limit_ = max_a;
                    return true;
                }
            }
        }
        return false;
    }

    bool accept(const box_detail::Points &points, double duration, double max_a, double max_j)
    {
        if (!box_detail::inside(points, box_, box_.subdivision_depth))
            return false;
        const auto velocity = box_detail::derivative(points, duration);
        const auto acceleration = box_detail::derivative(velocity, duration);
        const auto jerk = box_detail::derivative(acceleration, duration);
        double a_bound = 0, j_bound = 0;
        if (!box_detail::bounded(acceleration, max_a, box_.subdivision_depth, a_bound) ||
            !box_detail::bounded(jerk, max_j, box_.subdivision_depth, j_bound))
            return false;
        points_ = points;
        velocity_ = velocity;
        acceleration_ = acceleration;
        jerk_ = jerk;
        duration_ = duration;
        acceleration_bound_ = a_bound;
        jerk_bound_ = j_bound;
        acceleration_limit_ = max_a;
        valid_ = true;
        return true;
    }
    BoxOptions box_;
    SmoothStop legacy_;
    bool legacy_mode_ = true, valid_ = false;
    bool path_mode_ = false;
    std::vector<PathPart> path_segments_;
    box_detail::Points points_, velocity_, acceleration_, jerk_;
    double duration_ = 0, acceleration_bound_ = 0, jerk_bound_ = 0, acceleration_limit_ = 0;
    std::string reason_;
};

enum class BoxDecision
{
    CLEAR,
    BRAKE,
    FAILSAFE
};
struct BoxAssessment
{
    BoxDecision decision = BoxDecision::CLEAR;
    std::string reason;
    bool has_stop = false;
    BoxStop stop;
    Eigen::Vector3d measured_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d measured_v = Eigen::Vector3d::Zero();

    bool reusable(const Reference &seed, const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                  const BoxOptions &box, double max_a, double max_j) const
    {
        return decision != BoxDecision::FAILSAFE && has_stop && (p - measured_p).norm() <= 1e-9 &&
               (v - measured_v).norm() <= 1e-9 &&
               stop.certifiesRemaining(seed, 0, box, max_a, max_j);
    }
};

class BoxGuard
{
  public:
    void configure(const BoxOptions &options)
    {
        options.validate();
        box_ = options;
    }
    const BoxOptions &options() const
    {
        return box_;
    }
    bool enabled() const
    {
        return box_.enabled;
    }
    bool segmentAllowed(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
    {
        return !box_.enabled || (box_.contains(from) && box_.contains(to));
    }
    bool stoppable(const Reference &state, double max_a, double max_j,
                   const Preview &preview = Preview()) const
    {
        BoxStop stop;
        stop.configure(box_);
        return stop.tryStart(state, max_a, max_j, preview);
    }
    // One planner for prediction and execution. The nominal reference plan is
    // retained; the measured-state certificate is a separate feasibility check.
    BoxAssessment planStop(const Reference &reference, const Eigen::Vector3d &p,
                           const Eigen::Vector3d &v, double max_a, double max_j,
                           const Preview &preview) const
    {
        BoxAssessment result;
        result.measured_p = p;
        result.measured_v = v;
        result.stop.configure(box_);
        Reference measured = reference;
        measured.p = p;
        measured.v = v;
        if (!box_.contains(p, false) || !result.stop.tryStart(reference, max_a, max_j, preview) ||
            (!result.stop.certifiesRemaining(measured, 0, box_, max_a, max_j) &&
             !stoppable(measured, max_a, max_j, alignedPreview(measured, reference, preview))))
        {
            result.decision = BoxDecision::FAILSAFE;
            result.reason = "current reference/measured-state stop cannot be certified inside box";
            return result;
        }
        result.has_stop = true;
        return result;
    }
    BoxAssessment assess(const Reference &reference, const Eigen::Vector3d &actual_position,
                         const Eigen::Vector3d &actual_velocity, double max_a, double max_j,
                         const Preview &preview, const BoxStop *committed_stop = nullptr,
                         double committed_elapsed = 0) const
    {
        BoxAssessment result;
        if (!box_.enabled)
            return result;
        Reference actual = reference;
        actual.p = actual_position;
        actual.v = actual_velocity;
        const Preview measured_preview = alignedPreview(actual, reference, preview);
        auto certified = [&](const Reference &r, double dt, const Preview &guide) {
            return (committed_stop && committed_stop->certifiesRemaining(r, committed_elapsed + dt,
                                                                         box_, max_a, max_j)) ||
                   stoppable(r, max_a, max_j, guide);
        };
        if (committed_stop &&
            committed_stop->certifiesRemaining(reference, committed_elapsed, box_, max_a, max_j))
        {
            if (!box_.contains(actual.p, false) || !certified(actual, 0, measured_preview))
            {
                result.decision = BoxDecision::FAILSAFE;
                result.reason = "measured-state stop cannot be certified inside box";
                return result;
            }
        }
        else
            result = planStop(reference, actual.p, actual.v, max_a, max_j, preview);
        if (result.decision == BoxDecision::FAILSAFE)
            return result;
        // Include sensor/actuator delay plus the longest interval between checks.
        const double delay = box_.reaction_time + box_.check_interval;
        // Advance along the real curve, not a frozen tangential/acceleration model.
        Reference delayed;
        const Preview delayed_preview = [=](double dt, Reference &r) {
            return measured_preview && measured_preview(delay + dt, r);
        };
        if (!delayed_preview(0, delayed) || !certified(delayed, delay, delayed_preview))
        {
            result.decision = BoxDecision::BRAKE;
            result.reason = "measured-state reaction reserve exhausted; brake now";
            return result;
        }
        for (int i = 1; i <= box_.lookahead_samples; ++i)
        {
            const double dt = box_.lookahead * i / box_.lookahead_samples;
            const Preview future_preview = [=](double t, Reference &r) {
                return preview && preview(dt + t, r);
            };
            const Preview future_measured = [=](double t, Reference &r) {
                return measured_preview && measured_preview(dt + t, r);
            };
            Reference future, shadow;
            if (!future_preview(0, future) || !future_measured(0, shadow) ||
                !certified(future, dt, future_preview) || !certified(shadow, dt, future_measured))
            {
                result.decision = BoxDecision::BRAKE;
                result.reason = "lookahead leaves certified stopping envelope; brake now";
                return result;
            }
        }
        return result;
    }

    // Polynomial coefficients in descending powers, xyz by rows.
    bool polynomialAllowed(
        const Eigen::MatrixXd &coefficients, double duration,
        double max_braking_acceleration = std::numeric_limits<double>::infinity()) const
    {
        if (!box_.enabled)
            return true;
        if (coefficients.rows() != 3 || coefficients.cols() < 1 || coefficients.cols() > 33 ||
            !coefficients.allFinite() || !std::isfinite(duration) || duration <= 0)
            return false;
        const int degree = coefficients.cols() - 1;
        auto choose = [](int n, int k) {
            double value = 1;
            for (int j = 1; j <= k; ++j)
                value *= double(n - j + 1) / j;
            return value;
        };
        box_detail::Points points(degree + 1, Eigen::Vector3d::Zero());
        for (int i = 0; i <= degree; ++i)
            for (int power = 0; power <= i; ++power)
                points[i] += coefficients.col(degree - power) * std::pow(duration, power) *
                             choose(i, power) / choose(degree, power);
        if (!box_detail::inside(points, box_, box_.subdivision_depth))
            return false;
        if (degree >= 2 && std::isfinite(max_braking_acceleration))
        {
            double bound = 0;
            const auto acceleration =
                box_detail::derivative(box_detail::derivative(points, duration), duration);
            return box_detail::bounded(acceleration, max_braking_acceleration,
                                       box_.subdivision_depth, bound);
        }
        return true;
    }

  private:
    BoxOptions box_;
};
} // namespace flight_safety
#endif
