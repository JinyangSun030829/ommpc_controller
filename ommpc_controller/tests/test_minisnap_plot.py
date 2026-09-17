#!/usr/bin/env python3
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / "traj" / "minisnap_plot.py"
SPEC = importlib.util.spec_from_file_location("minisnap_plot", SOURCE)
plotter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(plotter)


def message(polynomials, times=None, count=8):
    # Test input polynomials are ascending powers in PHYSICAL time, not tau.
    times = [1.] * len(polynomials) if times is None else times
    result = [len(polynomials), count] + list(times)
    for segment in polynomials:
        for coefficients in segment:
            result.extend(list(coefficients) + [0.] * (count - len(coefficients)))
            result[-count:] = result[-count:][::-1]
    return result


class PlotTests(unittest.TestCase):
    def test_vector_norms_and_components(self):
        data = message([[[0, 1], [0, 2], [0, 2]]], times=[2.])
        analysis = plotter.analyze_trajectory(data)
        maximum = analysis["report"]["maxima"]
        self.assertAlmostEqual(maximum["speed"]["value"], 3.)
        self.assertEqual(maximum["acceleration"]["value"], 0.)
        self.assertEqual(maximum["jerk"]["value"], 0.)
        self.assertEqual(maximum["curvature"]["value"], 0.)
        np.testing.assert_allclose(analysis["position"][-1], [2, 4, 4])
        self.assertEqual(analysis["report"]["max_abs_components"]["velocity"], {"x": 1., "y": 2., "z": 2.})

    def test_off_grid_extrema(self):
        cases = [
            ("speed", [0, 0, .5, -1./3]),
            ("acceleration", [0, 0, 0, 1./6, -1./12]),
            ("jerk", [0, 0, 0, 0, 1./24, -1./60]),
        ]
        for metric, coefficients in cases:
            analysis = plotter.analyze_trajectory(message([[coefficients, [0], [0]]], count=6), sample_dt=.4)
            peak = analysis["report"]["maxima"][metric]
            self.assertAlmostEqual(peak["value"], .25, places=10)
            self.assertAlmostEqual(peak["time_s"], .5, places=8)
            self.assertTrue(np.all(np.abs(analysis["time"] - .5) > .1))

    def test_curvature_off_grid_maximum(self):
        # p(t)=(t,(t-.37)^2,0), kappa max=2 at .37 seconds.
        data = message([[[0, 1], [.37**2, -.74, 1], [0]]])
        analysis = plotter.analyze_trajectory(data, sample_dt=.2)
        peak = analysis["report"]["maxima"]["curvature"]
        self.assertAlmostEqual(peak["value"], 2., places=8)
        self.assertAlmostEqual(peak["time_s"], .37, places=6)

    def test_spatial_curvature_helix(self):
        t = np.linspace(0, 2*math.pi, 101)
        velocity = np.column_stack((-np.sin(t), np.cos(t), np.ones(len(t))))
        acceleration = np.column_stack((-np.cos(t), -np.sin(t), np.zeros(len(t))))
        np.testing.assert_allclose(plotter.spatial_curvature(velocity, acceleration), .5, atol=1e-12)

    def test_zero_speed_is_undefined(self):
        analysis = plotter.analyze_trajectory(message([[[2], [3], [4]]]))
        self.assertTrue(np.all(np.isnan(analysis["curvature"])))
        self.assertIsNone(analysis["report"]["maxima"]["curvature"]["value"])
        json.dumps(analysis["report"], allow_nan=False)
        self.assertIn("N/A", plotter.summary_lines(analysis["report"])[3])

    def test_curvature_cutoff_at_rest(self):
        # p=(t²/2,t³/3,0); curvature diverges at rest. Report the max on
        # speed>=.1, NOT infinity, zero, or a purported full-curve maximum.
        cutoff = .1
        data = message([[[0, 0, .5], [0, 0, 0, 1./3], [0]]])
        analysis = plotter.analyze_trajectory(data, curvature_min_speed=cutoff,
                                             curvature_guard_time=0)
        tau = math.sqrt((math.sqrt(1 + 4*cutoff**2) - 1) / 2)
        expected = 1 / (tau * (1 + tau*tau)**1.5)
        peak = analysis["report"]["maxima"]["curvature"]
        self.assertAlmostEqual(peak["time_s"], tau, places=7)
        self.assertAlmostEqual(peak["value"], expected, places=6)
        self.assertTrue(np.isnan(analysis["curvature"][0]))

    def test_cleaning_catches_unsampled_low_speed_dip(self):
        # v=(t-.37,.001,0): nearly stops between the .25/.5 plotting samples.
        # Analytic cutoff intervals, not sampled speeds, must seed the guard.
        data = message([[[0, -.37, .5], [0, .001], [0]]])
        raw = plotter.analyze_trajectory(data, sample_dt=.25,
                                         curvature_min_speed=.0001, curvature_guard_time=0)
        clean = plotter.analyze_trajectory(data, sample_dt=.25,
                                           curvature_min_speed=.05, curvature_guard_time=.1)
        self.assertGreater(raw["report"]["maxima"]["curvature"]["value"], 900000)
        self.assertLess(clean["report"]["maxima"]["curvature"]["value"], 1)
        self.assertEqual(clean["report"]["curvature_low_speed_sample_count"], 0)
        self.assertEqual(clean["report"]["curvature_guard_excluded_sample_count"], 2)
        intervals = clean["report"]["curvature_excluded_intervals_s"]
        self.assertEqual(len(intervals), 1)
        self.assertAlmostEqual(intervals[0][0], .37 - math.sqrt(.05**2 - .001**2) - .1)
        self.assertTrue(np.all(np.isnan(clean["curvature"][[1, 2]])))
        for name in ("time", "position", "velocity", "acceleration", "jerk"):
            np.testing.assert_array_equal(raw[name], clean[name])
        for name in ("speed", "acceleration", "jerk"):
            self.assertEqual(raw["report"]["maxima"][name], clean["report"]["maxima"][name])

    def test_guard_crosses_segment_boundary(self):
        # Slow segment ends at t=1; following moving segment must be masked to 1.15.
        data = message([[[0], [0], [0]], [[0, 1], [0], [0]]])
        analysis = plotter.analyze_trajectory(data, sample_dt=.05)
        self.assertEqual(analysis["report"]["curvature_excluded_intervals_s"], [[0., 1.15]])
        index = np.flatnonzero(np.isclose(analysis["time"], 1.1))
        self.assertTrue(np.all(np.isnan(analysis["curvature"][index])))
        self.assertEqual(analysis["report"]["maxima"]["curvature"]["value"], 0.)

    def test_entire_trajectory_excluded_by_guard(self):
        data = message([[[0, -.37, .5], [0, .001], [0]]])
        analysis = plotter.analyze_trajectory(data, curvature_min_speed=.05,
                                             curvature_guard_time=2)
        self.assertTrue(np.all(np.isnan(analysis["curvature"])))
        self.assertIsNone(analysis["report"]["maxima"]["curvature"]["value"])
        fig = plotter.build_figure(analysis)
        try:
            self.assertIn("N/A", fig.axes[4].get_title(loc="left"))
        finally:
            plotter.plt.close(fig)

    def test_render_does_not_bridge_unsampled_excluded_interval(self):
        data = message([[[0, -.37, .5], [0, .001], [0]]])
        analysis = plotter.analyze_trajectory(data, sample_dt=.5,
                                             curvature_min_speed=.01, curvature_guard_time=0)
        self.assertTrue(np.all(np.isfinite(analysis["curvature"])))
        fig = plotter.build_figure(analysis)
        try:
            self.assertTrue(np.any(np.isnan(fig.axes[4].lines[0].get_ydata())))
        finally:
            plotter.plt.close(fig)

    def test_both_sides_of_knots(self):
        data = message([[[0, 0, 0, 1./6], [0], [2]],
                        [[1./6, .5, .5, .5], [0], [2]]], count=6)
        analysis = plotter.analyze_trajectory(data, sample_dt=.3)
        indices = np.flatnonzero(analysis["time"] == 1.)
        self.assertEqual(len(indices), 2)
        np.testing.assert_allclose(analysis["jerk"][indices, 0], [1, 3])
        self.assertEqual(analysis["report"]["maxima"]["jerk"]["segment"], 2)
        self.assertEqual(analysis["report"]["maxima"]["jerk"]["value"], 3.)

    def test_sampling_budget_and_final_endpoint(self):
        data = message([[[0, 1], [0], [2]]] * 245, [1.] * 245)
        analysis = plotter.analyze_trajectory(data, sample_dt=.001, max_samples=2000)
        self.assertLessEqual(analysis["report"]["sample_count"], 2000)
        self.assertEqual(analysis["time"][-1], 245.)

    def test_invalid_inputs(self):
        good = message([[[0, 1], [0], [0]]])
        for bad in ([], [0, 8], [1.5, 8], good[:-1], [1, 8, 0]+good[3:],
                    [1, 8, -1]+good[3:], [1, 8, math.inf]+good[3:],
                    [1, 7]+good[2:], good[:3]+[math.nan]+good[4:]):
            with self.assertRaises(ValueError):
                plotter.analyze_trajectory(bad)
        for kwargs in ({"sample_dt": 0}, {"curvature_min_speed": 0}, {"max_samples": 1},
                       {"curvature_guard_time": -1}, {"curvature_guard_time": math.nan}):
            with self.assertRaises(ValueError):
                plotter.analyze_trajectory(good, **kwargs)
        with self.assertRaises(RuntimeError):
            plotter.analyze_trajectory(good, cancelled=lambda: True)

    def test_render_five_plots(self):
        analysis = plotter.analyze_trajectory(message([[[0, 1], [.37**2, -.74, 1], [2]]]))
        fig = plotter.build_figure(analysis)
        try:
            self.assertEqual(len(fig.axes), 5)
            self.assertTrue("Jerk" in fig.axes[3].get_ylabel())
            self.assertTrue("Curvature" in fig.axes[4].get_ylabel())
            with tempfile.TemporaryDirectory(prefix="minisnap_plot_test_") as directory:
                destination = Path(directory) / "test.png"
                fig.savefig(str(destination), dpi=80)
                self.assertGreater(destination.stat().st_size, 20000)
        finally:
            plotter.plt.close(fig)

    def test_latest_request_worker_and_saved_report(self):
        class FakeROS:
            def __init__(self, directory):
                self.parameters = {"~output_dir": directory, "~dpi": 80}
                self.errors = []
            def get_param(self, name, default):
                return self.parameters.get(name, default)
            def loginfo(self, *args):
                pass
            def logerr(self, *args):
                self.errors.append(args)
        with tempfile.TemporaryDirectory(prefix="minisnap_worker_test_") as directory:
            ros = FakeROS(directory)
            worker = plotter.TrajectoryPlotter(ros)
            try:
                worker.callback(SimpleNamespace(data=message([[[0,1],[0],[2]]])))
                worker.callback(SimpleNamespace(data=message([[[0,2],[0],[2]]])))
                destination = Path(directory) / "trajectory_metrics.json"
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    if destination.exists():
                        with destination.open(encoding="utf-8") as stream:
                            report = json.load(stream)
                        if report["maxima"]["speed"]["value"] == 2.:
                            break
                    time.sleep(.03)
                else:
                    self.fail("latest plot result was not saved")
                self.assertFalse(ros.errors)
                self.assertGreater((Path(directory) / "trajectory_pvaj_curvature.png").stat().st_size, 20000)
            finally:
                worker.close()
            self.assertFalse(list(Path(directory).glob(".minisnap_*")))


if __name__ == "__main__":
    unittest.main(verbosity=2)
