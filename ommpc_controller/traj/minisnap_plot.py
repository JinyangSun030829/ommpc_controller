#!/usr/bin/env python3
"""Plot the legacy polynomial message: position, velocity, acceleration, jerk
and spatial curvature. No yaw, attitude, gravity compensation or body rates.

Math helpers are ROS-independent. V/A/J maxima use numerical polynomial
stationary points; curvature uses stationary-point candidates plus bracketed
refinement on moving intervals AFTER low-speed cleaning. These are numerical
extrema, not certified Bernstein bounds. Curvature is undefined at zero speed.
"""
import json
import math
import os
import tempfile
import threading

import numpy as np
from numpy.polynomial import polynomial as poly
import matplotlib
matplotlib.use("Agg")  # One worker owns all plotting; no GUI/event-loop dependency.
import matplotlib.pyplot as plt

# 曲率清洗设置：只影响最后一幅图及曲率统计，不改变轨迹或 P/V/A/J。
DEFAULT_CURVATURE_MIN_SPEED = 0.3  # m/s：低于此速度的区间不计算曲率。
DEFAULT_CURVATURE_GUARD_TIME = 0.15  # s：低速区间前后额外排除的时间。


def parse_trajectory(data, max_segments=10000):
    values = np.asarray(data, dtype=float)
    if values.ndim != 1 or len(values) < 2 or not np.all(np.isfinite(values)):
        raise ValueError("trajectory is empty, non-finite, or not a vector")
    segments, count = values[:2]
    if (segments != round(segments) or not 1 <= segments <= max_segments or
            count != round(count) or count not in (6, 8)):
        raise ValueError("expected positive segment count and 6/8 coefficients per axis")
    segments, count = int(segments), int(count)
    expected = 2 + segments + segments * 3 * count
    if len(values) != expected:
        raise ValueError("trajectory length mismatch: expected %d, received %d" % (expected, len(values)))
    times = values[2:2 + segments].copy()
    if np.any(times <= 0) or not math.isfinite(float(np.sum(times))):
        raise ValueError("segment times must be positive with a finite sum")
    coefficients = values[2 + segments:].reshape(segments, 3, count).copy()
    # Ascending powers in tau=t/T, then physical derivatives are obtained by
    # dividing each tau derivative by T. This also improves root conditioning.
    with np.errstate(over="ignore", invalid="ignore"):
        normalized = coefficients[:, :, ::-1] * times[:, None, None] ** np.arange(count)
    if not np.all(np.isfinite(normalized)):
        raise ValueError("normalized coefficients overflow")
    return times, normalized


def derivative_coefficients(normalized, duration, derivative):
    return np.stack([poly.polyder(axis, m=derivative) / duration ** derivative
                     for axis in normalized])


def evaluate(coefficients, tau):
    return np.stack([poly.polyval(tau, axis) for axis in coefficients], axis=-1)


def _roots(coefficients):
    """Numerical roots on [0,1], with coefficient scaling and conservative
    imaginary-part filtering. Curvature also has a separate grid/refinement
    fallback because its high-degree root equation can be ill-conditioned.
    """
    coefficients = np.asarray(coefficients, dtype=float)
    magnitude = float(np.max(np.abs(coefficients)))
    if not math.isfinite(magnitude):
        raise ValueError("non-finite extremum polynomial")
    if magnitude == 0:
        return np.empty(0)
    coefficients = poly.polytrim(coefficients / magnitude, tol=1e-13)
    if len(coefficients) < 2:
        return np.empty(0)
    roots = poly.polyroots(coefficients)
    real = roots.real[(np.abs(roots.imag) <= 1e-7) &
                      (roots.real >= -1e-9) & (roots.real <= 1 + 1e-9)]
    return np.unique(np.clip(real, 0., 1.))


def _squared_norm(coefficients):
    value = np.zeros(1)
    for axis in coefficients:
        value = poly.polyadd(value, poly.polymul(axis, axis))
    return value


def norm_maximum(coefficients):
    candidates = np.unique(np.r_[0., 1., _roots(poly.polyder(_squared_norm(coefficients)))])
    norms = np.linalg.norm(evaluate(coefficients, candidates), axis=1)
    index = int(np.argmax(norms))
    return float(norms[index]), float(candidates[index])


def component_maximum(coefficients):
    result = []
    for axis in coefficients:
        candidates = np.r_[0., 1., _roots(poly.polyder(axis))]
        result.append(float(np.max(np.abs(poly.polyval(candidates, axis)))))
    return np.array(result)


def spatial_curvature(velocity, acceleration, min_speed=DEFAULT_CURVATURE_MIN_SPEED):
    velocity, acceleration = np.asarray(velocity), np.asarray(acceleration)
    speed = np.linalg.norm(velocity, axis=-1)
    valid = speed >= min_speed
    result = np.full(np.shape(speed), np.nan)
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        np.divide(np.linalg.norm(np.cross(velocity, acceleration), axis=-1),
                  speed ** 3, out=result, where=valid)
    return result


def _golden_maximum(function, left, right):
    ratio = (math.sqrt(5.) - 1.) / 2.
    x1, x2 = right - ratio * (right - left), left + ratio * (right - left)
    f1, f2 = function(x1), function(x2)
    for _ in range(55):
        if right - left < 1e-12:
            break
        if f1 < f2:
            left, x1, f1 = x1, x2, f2
            x2 = left + ratio * (right - left)
            f2 = function(x2)
        else:
            right, x2, f2 = x2, x1, f1
            x1 = right - ratio * (right - left)
            f1 = function(x1)
    return (f1, x1) if f1 >= f2 else (f2, x2)


def _speed_intervals(velocity_coefficients, min_speed, low_speed):
    """Find continuous speed-cutoff intervals, including dips between samples."""
    threshold = _squared_norm(velocity_coefficients)
    threshold[0] -= min_speed ** 2
    edges = np.unique(np.r_[0., _roots(threshold), 1.])
    intervals = []
    for left, right in zip(edges[:-1], edges[1:]):
        if right - left > 1e-12:
            speed = np.linalg.norm(evaluate(velocity_coefficients, (left + right) / 2))
            if (speed < min_speed) == low_speed:
                intervals.append((float(left), float(right)))
    return intervals


def curvature_exclusion_intervals(times, derivatives, min_speed, guard_time):
    """Expand low-speed intervals in global physical time, across segment knots."""
    excluded, start = [], 0.
    total = float(np.sum(times))
    for duration, piece in zip(times, derivatives):
        for left, right in _speed_intervals(piece[1], min_speed, True):
            interval = (max(0., start + left * duration - guard_time),
                        min(total, start + right * duration + guard_time))
            if excluded and interval[0] <= excluded[-1][1]:
                excluded[-1] = (excluded[-1][0], max(excluded[-1][1], interval[1]))
            else:
                excluded.append(interval)
        start += duration
    return excluded


def _retained_piece_intervals(start, duration, excluded):
    end, cursor, retained = start + duration, start, []
    for left, right in excluded:
        if right <= start or left >= end:
            continue
        if left > cursor:
            retained.append(((cursor - start) / duration, (left - start) / duration))
        cursor = max(cursor, right)
    if cursor < end:
        retained.append(((cursor - start) / duration, 1.))
    return retained


def curvature_maximum(velocity_coefficients, acceleration_coefficients, min_speed,
                      retained_intervals=None):
    # kappa² = N/S³; stationary equation N'S - 3NS'=0. Evaluate kappa
    # DIRECTLY from vectors at candidates to avoid cancellation in N evaluation.
    speed_squared = _squared_norm(velocity_coefficients)
    cross = []
    for i, j in ((1, 2), (2, 0), (0, 1)):
        cross.append(poly.polysub(poly.polymul(velocity_coefficients[i], acceleration_coefficients[j]),
                                  poly.polymul(velocity_coefficients[j], acceleration_coefficients[i])))
    numerator = _squared_norm(cross)
    stationary = poly.polysub(poly.polymul(poly.polyder(numerator), speed_squared),
        3 * poly.polymul(numerator, poly.polyder(speed_squared)))
    moving = _speed_intervals(velocity_coefficients, min_speed, False)
    if retained_intervals is not None:
        moving = [(max(left, keep_left), min(right, keep_right))
                  for left, right in moving for keep_left, keep_right in retained_intervals
                  if min(right, keep_right) - max(left, keep_left) > 1e-12]
    critical = _roots(stationary)
    best = (None, None)

    def value(tau):
        v = evaluate(velocity_coefficients, tau)
        a = evaluate(acceleration_coefficients, tau)
        speed = float(np.linalg.norm(v))
        # A threshold crossing located by roots may round just below the cutoff.
        if speed < min_speed * (1 - 1e-7):
            return -math.inf
        return float(np.linalg.norm(np.cross(v, a)) / speed ** 3)

    def consider(candidate):
        nonlocal best
        maximum, location = candidate
        if math.isfinite(maximum) and (best[0] is None or maximum > best[0]):
            best = float(maximum), float(location)

    for left, right in moving:
        if right - left < 1e-12 or np.linalg.norm(evaluate(
                velocity_coefficients, (left + right) / 2)) < min_speed:
            continue
        # Degree-bounded polynomial candidates plus a grid and local refinement.
        grid = np.linspace(left, right, 65)
        values = np.array([value(tau) for tau in grid])
        for tau in np.r_[left, right, critical[(critical > left) & (critical < right)]]:
            consider((value(tau), tau))
        peak = int(np.argmax(values))
        consider((values[peak], grid[peak]))
        for i in range(1, len(grid) - 1):
            if values[i] >= values[i - 1] and values[i] >= values[i + 1]:
                # Do not waste iterations refining an exactly straight segment.
                if values[i] > 1e-12:
                    consider(_golden_maximum(value, grid[i - 1], grid[i + 1]))
    return best


def analyze_trajectory(data, sample_dt=0.01, max_samples=200000,
                       curvature_min_speed=DEFAULT_CURVATURE_MIN_SPEED, cancelled=None,
                       curvature_guard_time=DEFAULT_CURVATURE_GUARD_TIME):
    if not math.isfinite(sample_dt) or sample_dt <= 0:
        raise ValueError("sample_dt must be finite and positive")
    if not math.isfinite(curvature_min_speed) or curvature_min_speed <= 0:
        raise ValueError("curvature_min_speed must be finite and positive")
    if not math.isfinite(curvature_guard_time) or curvature_guard_time < 0:
        raise ValueError("curvature_guard_time must be finite and nonnegative")
    times, coefficients = parse_trajectory(data)
    segments = len(times)
    if max_samples < 2 * segments:
        raise ValueError("max_samples must accommodate both endpoints of every segment")
    # Sum(ceil(T/dt)+1) <= total/dt + 2*segments. Keep BOTH sides of every
    # junction, so even a derivative jump is visible and cannot be skipped.
    total = float(np.sum(times))
    dt = max(sample_dt, total / max(1, max_samples - 2 * segments))
    maxima = {name: {"value": None, "time_s": None, "segment": None}
              for name in ("speed", "acceleration", "jerk", "curvature")}
    components = np.zeros((3, 3))
    time_parts, state_parts = [], [[] for _ in range(4)]
    piece_derivatives = []
    start = 0.

    def record(name, maximum, tau, segment):
        if maximum is not None and (maxima[name]["value"] is None or maximum > maxima[name]["value"]):
            maxima[name] = {"value": float(maximum), "time_s": float(start + tau * times[segment]),
                            "segment": segment + 1}

    for segment, duration in enumerate(times):
        if cancelled and cancelled():
            raise RuntimeError("plot request superseded")
        tau = np.linspace(0., 1., int(math.ceil(duration / dt)) + 1)
        time_parts.append(start + tau * duration)
        derivatives = [derivative_coefficients(coefficients[segment], duration, d) for d in range(4)]
        piece_derivatives.append(derivatives)
        for d in range(4):
            state_parts[d].append(evaluate(derivatives[d], tau))
        for d, name in enumerate(("speed", "acceleration", "jerk"), start=1):
            maximum, location = norm_maximum(derivatives[d])
            record(name, maximum, location, segment)
            components[d - 1] = np.maximum(components[d - 1], component_maximum(derivatives[d]))
        start += duration
    states = [np.concatenate(parts) for parts in state_parts]
    # Use the same continuous exclusion intervals for plotted samples AND refined
    # extrema. Otherwise a removed spike can still stretch the axis via its marker.
    excluded = curvature_exclusion_intervals(
        times, piece_derivatives, curvature_min_speed, curvature_guard_time)
    start = 0.
    for segment, (duration, derivatives) in enumerate(zip(times, piece_derivatives)):
        if cancelled and cancelled():
            raise RuntimeError("plot request superseded")
        retained = _retained_piece_intervals(start, duration, excluded)
        maximum, location = curvature_maximum(
            derivatives[1], derivatives[2], curvature_min_speed, retained)
        record("curvature", maximum, location, segment)
        start += duration
    global_times = np.concatenate(time_parts)
    curvature = spatial_curvature(states[1], states[2], curvature_min_speed)
    low_speed_count = int(np.sum(~np.isfinite(curvature)))
    for left, right in excluded:
        mask = (global_times > left) & (global_times < right)
        if left == 0.:
            mask |= global_times == 0.
        if right == total:
            mask |= global_times == total
        curvature[mask] = np.nan
    if not all(np.all(np.isfinite(values)) for values in states):
        raise ValueError("non-finite polynomial evaluation")
    if np.any(np.isinf(curvature)):
        raise ValueError("curvature overflow")
    # Cross-check against displayed samples; a poorly conditioned high-degree
    # curvature root must not make the reported maximum smaller than the plot.
    finite = np.flatnonzero(np.isfinite(curvature))
    if len(finite):
        i = finite[np.argmax(curvature[finite])]
        if maxima["curvature"]["value"] is None or curvature[i] > maxima["curvature"]["value"]:
            maxima["curvature"] = {"value": float(curvature[i]), "time_s": float(global_times[i]),
                                    "segment": int(np.searchsorted(np.cumsum(times), global_times[i], side="left")) + 1}
    report = {
        "segment_count": segments, "coefficients_per_axis": coefficients.shape[2],
        "duration_s": total, "sample_count": len(global_times), "requested_sample_dt_s": sample_dt,
        "effective_sample_dt_upper_bound_s": dt, "curvature_min_speed_m_s": curvature_min_speed,
        "curvature_guard_time_s": curvature_guard_time,
        "curvature_excluded_intervals_s": [list(interval) for interval in excluded],
        "curvature_low_speed_sample_count": low_speed_count,
        "curvature_guard_excluded_sample_count": int(np.sum(~np.isfinite(curvature))) - low_speed_count,
        "curvature_excluded_sample_count": int(np.sum(~np.isfinite(curvature))),
        "maximum_method": "V/A/J: numerical polynomial extrema; curvature: numerical extrema/refinement on cleaned intervals (speed cutoff plus temporal guard)",
        "maxima": maxima,
        "max_abs_components": {name: dict(zip(("x", "y", "z"), values.tolist()))
                               for name, values in zip(("velocity", "acceleration", "jerk"), components)}
    }
    return {"time": global_times, "position": states[0], "velocity": states[1],
            "acceleration": states[2], "jerk": states[3], "curvature": curvature, "report": report}


def build_figure(analysis):
    report = analysis["report"]
    fig, axes = plt.subplots(5, 1, figsize=(12, 15), sharex=True)
    colors = ("tab:red", "tab:green", "tab:blue")
    for row, (name, unit, metric) in enumerate((
            ("position", "m", None), ("velocity", "m/s", "speed"),
            ("acceleration", "m/s²", "acceleration"), ("jerk", "m/s³", "jerk"))):
        ax = axes[row]
        for axis, color in enumerate(colors):
            ax.plot(analysis["time"], analysis[name][:, axis], color=color,
                    linewidth=1., label=("X", "Y", "Z")[axis])
        if metric:
            norm = np.linalg.norm(analysis[name], axis=1)
            ax.plot(analysis["time"], norm, color="black", linewidth=1.4, label="Magnitude")
            peak = report["maxima"][metric]
            ax.scatter([peak["time_s"]], [peak["value"]], color="black", s=26, zorder=5)
            ax.set_title("Max magnitude = %.6g %s at t = %.6g s" %
                         (peak["value"], unit, peak["time_s"]), fontsize=10, loc="left")
        ax.set_ylabel("%s [%s]" % (name.capitalize(), unit))
        ax.legend(loc="upper right", ncol=4)
        ax.grid(True, alpha=.3)
    peak = report["maxima"]["curvature"]
    # Explicit NaN separators prevent a coarse sampling grid from drawing a
    # connecting line through an excluded interval containing no sampled point.
    gaps = [(left + right) / 2 for left, right in report["curvature_excluded_intervals_s"]]
    plot_time = np.r_[analysis["time"], gaps]
    plot_curvature = np.r_[analysis["curvature"], np.full(len(gaps), np.nan)]
    order = np.argsort(plot_time, kind="stable")
    axes[4].plot(plot_time[order], plot_curvature[order], color="tab:purple",
                 label="Cleaned spatial curvature")
    cutoff = report["curvature_min_speed_m_s"]
    guard = report["curvature_guard_time_s"]
    if peak["value"] is None:
        title = "Curvature N/A: no retained moving interval after cleaning"
    else:
        title = "Max cleaned curvature (numerical) = %.6g 1/m at t = %.6g s; speed >= %.6g m/s" % (
            peak["value"], peak["time_s"], cutoff)
        axes[4].scatter([peak["time_s"]], [peak["value"]], color="tab:purple", s=26, zorder=5)
    title += "\nLow-speed intervals + %.3g s guard excluded; %d samples removed" % (
        guard, report["curvature_excluded_sample_count"])
    axes[4].set_title(title, fontsize=10, loc="left")
    axes[4].set_ylabel("Curvature [1/m]")
    axes[4].set_xlabel("Time [s]")
    axes[4].legend(loc="upper right")
    axes[4].grid(True, alpha=.3)
    axes[4].set_ylim(bottom=0.)
    axes[4].set_xlim(0., report["duration_s"])
    title = "Minimum-%s Reference | %d segments | %.3f s" % (
        "snap" if report["coefficients_per_axis"] == 8 else "jerk", report["segment_count"], report["duration_s"])
    fig.suptitle(title, fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, .975))
    return fig


def summary_lines(report):
    lines = []
    for name, label, unit in (("speed", "速度模长", "m/s"),
            ("acceleration", "加速度模长", "m/s²"), ("jerk", "jerk模长", "m/s³"),
            ("curvature", "清洗后曲率（数值估计）", "1/m")):
        maximum = report["maxima"][name]
        if maximum["value"] is None:
            lines.append("%s最大值: N/A（清洗后无有效运动区间）" % label)
        else:
            lines.append("%s最大值: %.9g %s，t=%.9g s，第%d段" %
                         (label, maximum["value"], unit, maximum["time_s"], maximum["segment"]))
    lines.append("曲率清洗: 速度阈值=%.6g m/s，低速区间前后各排除%.6g s；排除%d个样本。" %
                 (report["curvature_min_speed_m_s"], report["curvature_guard_time_s"],
                  report["curvature_excluded_sample_count"]))
    return lines


class TrajectoryPlotter:
    def __init__(self, ros):
        self.ros = ros
        self.dt = float(ros.get_param("~sample_dt", .01))
        self.max_samples = int(ros.get_param("~max_samples", 200000))
        self.cutoff = float(ros.get_param("~curvature_min_speed", DEFAULT_CURVATURE_MIN_SPEED))
        self.guard = float(ros.get_param("~curvature_guard_time", DEFAULT_CURVATURE_GUARD_TIME))
        self.dpi = int(ros.get_param("~dpi", 200))
        self.directory = os.path.abspath(os.path.expanduser(ros.get_param("~output_dir", "~/uav_logs")))
        if (not math.isfinite(self.dt) or self.dt <= 0 or not math.isfinite(self.cutoff) or
                self.cutoff <= 0 or not math.isfinite(self.guard) or self.guard < 0 or
                not 2 <= self.max_samples <= 2000000 or not 50 <= self.dpi <= 600):
            raise ValueError("invalid plot sampling, curvature cutoff, or DPI configuration")
        self.condition = threading.Condition()
        self.pending = None
        self.generation = 0
        self.stopped = False
        self.worker = threading.Thread(target=self._work, name="minisnap-plot", daemon=True)
        self.worker.start()

    def callback(self, msg):
        with self.condition:
            self.generation += 1
            self.pending = (self.generation, tuple(msg.data))
            self.condition.notify()

    def _cancelled(self, generation):
        with self.condition:
            return self.stopped or generation != self.generation

    def _work(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopped or self.pending is not None)
                if self.stopped:
                    return
                generation, data = self.pending
                self.pending = None
            fig, temporary = None, []
            try:
                analysis = analyze_trajectory(data, self.dt, self.max_samples, self.cutoff,
                                               lambda: self._cancelled(generation), self.guard)
                if self._cancelled(generation):
                    continue
                fig = build_figure(analysis)
                os.makedirs(self.directory, exist_ok=True)
                for suffix in (".png", ".json"):
                    descriptor, path = tempfile.mkstemp(prefix=".minisnap_", suffix=suffix, dir=self.directory)
                    os.close(descriptor)
                    temporary.append(path)
                fig.savefig(temporary[0], dpi=self.dpi)
                with open(temporary[1], "w", encoding="utf-8") as stream:
                    json.dump(analysis["report"], stream, indent=2, ensure_ascii=False, allow_nan=False)
                with self.condition:
                    if self.stopped or generation != self.generation:
                        continue
                    image = os.path.join(self.directory, "trajectory_pvaj_curvature.png")
                    metrics = os.path.join(self.directory, "trajectory_metrics.json")
                    os.replace(temporary[0], image)
                    os.replace(temporary[1], metrics)
                for line in summary_lines(analysis["report"]):
                    self.ros.loginfo(line)
                self.ros.loginfo("轨迹图: %s；最大值报告: %s", image, metrics)
            except Exception as error:
                if not self._cancelled(generation):
                    self.ros.logerr("轨迹绘图失败: %s", error)
            finally:
                if fig is not None:
                    plt.close(fig)
                for path in temporary:
                    if os.path.exists(path):
                        os.unlink(path)  # Only this request's mkstemp files, never user output.

    def close(self):
        with self.condition:
            self.stopped = True
            self.condition.notify_all()
        self.worker.join(timeout=5)


def main():
    import rospy
    from std_msgs.msg import Float64MultiArray
    rospy.init_node("trajectory_plotter_node", anonymous=True)
    try:
        plotter = TrajectoryPlotter(rospy)
    except ValueError as error:
        rospy.logfatal("轨迹绘图参数错误: %s", error)
        return
    rospy.on_shutdown(plotter.close)
    rospy.Subscriber(rospy.get_param("~trajectory_topic", "/planning/poly_trajectory"),
                     Float64MultiArray, plotter.callback, queue_size=1)
    rospy.loginfo("轨迹绘图已启动：P/V/A/J/清洗后三维曲率；曲率速度阈值=%.6g m/s，时间保护=%.6g s。",
                  plotter.cutoff, plotter.guard)
    rospy.spin()


if __name__ == "__main__":
    main()
