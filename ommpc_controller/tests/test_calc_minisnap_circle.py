"""Offline unit tests. The optional probe test uses the real C++ planner."""
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock
import contextlib
import io

SCRIPT = Path(__file__).resolve().parent.parent / "minisnap_refactor/traj/calc_minisnap_circle.py"
if not SCRIPT.is_file():
    SCRIPT = Path(__file__).resolve().parent.parent / "traj/calc_minisnap_circle.py"
spec = importlib.util.spec_from_file_location("calculator", SCRIPT)
calculator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calculator)


class CalculatorTests(unittest.TestCase):
    def setUp(self):
        self.limits = {"gravity": 9.81, "tilt_tangent": math.tan(math.radians(65)),
                       "acceleration": 12, "thrust_min": .04, "thrust_max": .9,
                       "force_min": 1, "force_max": 30, "rate_xy": 6, "rate_z": 4}

    def test_formulas(self):
        r = calculator.circle_requirements(2, 4, 21, 9.81)
        self.assertEqual(r["omega"], 2)
        self.assertEqual(r["acceleration"], 8)
        self.assertEqual(r["jerk"], 16)
        self.assertAlmostEqual(r["period"], math.pi)
        self.assertAlmostEqual(r["thrust"], .21 * math.hypot(9.81, 8) / 9.81)
        self.assertFalse(calculator.violations(r, self.limits))

    def test_invalid_inputs(self):
        for radius, speed, hover in ((0, 1, 21), (-1, 1, 21), (1, 0, 21), (1, -1, 21),
                                     (1, 1, 0), (1, 1, 101), (math.nan, 1, 21),
                                     (1, math.inf, 21), (1e-300, 1e300, 21)):
            with self.subTest((radius, speed, hover)):
                with self.assertRaises(calculator.ConfigurationError):
                    calculator.circle_requirements(radius, speed, hover, 9.81)

    def test_percent_has_no_ambiguous_fraction_autodetection(self):
        self.assertEqual(calculator.circle_requirements(1, 1, .21, 9.81)["hover"], .0021)

    def test_infeasible_and_reserve_policy(self):
        self.assertTrue(calculator.violations(
            calculator.circle_requirements(.5, 8, 21, 9.81), self.limits, 0))
        r = calculator.circle_requirements(1, .1, 85, 9.81)
        self.assertFalse(calculator.violations(r, self.limits, 0))
        self.assertTrue(calculator.violations(r, self.limits))

    def test_independent_controller_verdicts(self):
        r = calculator.circle_requirements(2, 5, 21, 9.81)
        checks = calculator.controller_checks(r, self.limits)
        self.assertFalse(all(c["passed"] for c in checks["tracking"]))
        self.assertTrue(all(c["passed"] for c in checks["OMMPC"]))
        # A tracking thrust limit must not be silently imposed on OMMPC.
        limits = dict(self.limits, thrust_max=.20)
        checks = calculator.controller_checks(
            calculator.circle_requirements(1.5, 1, 21, 9.81), limits)
        self.assertFalse(all(c["passed"] for c in checks["tracking"]))
        self.assertTrue(all(c["passed"] for c in checks["OMMPC"]))

    def test_reports_both_controllers_with_numeric_limits(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            results = calculator.display_controller_results("test",
                calculator.circle_requirements(2, 5, 21, 9.81), self.limits, .21)
        text = output.getvalue()
        self.assertIn("[tracking] 原始限幅：未通过", text)
        self.assertIn("[OMMPC] 原始限幅：通过", text)
        self.assertIn("所需 12.5", text)
        self.assertIn("裕度范围[0, 9.6]", text)
        self.assertFalse(results["tracking"]["reserve"])
        self.assertTrue(results["OMMPC"]["reserve"])

    def test_tracking_failure_does_not_skip_ommpc_rehearsal(self):
        output = io.StringIO()
        probe = {"mean_speed": 5, "minimum_speed": 5, "maximum_speed": 5,
                 "radial_error": 0, "peak_velocity": 5, "peak_acceleration": 13,
                 "peak_jerk": 20, "total_time": 20, "planning_ms": 1}
        with mock.patch.object(calculator, "CIRCLE_RADIUS_M", 2), \
             mock.patch.object(calculator, "TARGET_CRUISE_SPEED_MPS", 5), \
             mock.patch.object(calculator, "HOVER_THRUST_PERCENT", 21), \
             mock.patch.object(calculator, "find_package", return_value=Path("unused")), \
             mock.patch.object(calculator, "load_configuration", return_value=(self.limits, calculator.PLANNER_DEFAULTS)), \
             mock.patch.object(calculator, "find_probe", return_value=Path("unused")), \
             mock.patch.object(calculator, "calibrate", return_value=([6, 14, 25], probe)) as calibration, \
             mock.patch.object(calculator, "run_probe", return_value=probe) as rehearsal, \
             contextlib.redirect_stdout(output):
            result = calculator.main()
        calibration.assert_called_once()
        rehearsal.assert_called_once()
        self.assertEqual(result, 2)
        self.assertIn("[OMMPC] 通过（标准模板完整参考轨迹", output.getvalue())
        self.assertIn("[tracking] 不可实现", output.getvalue())
        self.assertNotIn('<arg name="max_vel"', output.getvalue())

    def test_upper_bounds(self):
        r = calculator.probe_requirements({"peak_acceleration": 4, "peak_jerk": 8}, .21, 9.81)
        self.assertAlmostEqual(r["rate_xy"], 8 / 9.81)
        self.assertAlmostEqual(r["rate_z"], 8 / 9.81 * 4 / math.hypot(9.81, 4))

    def test_calibration_time_scaling(self):
        calls = []
        def runner(settings):
            calls.append(settings)
            return {"mean_speed": settings[0] * .75}
        circle = calculator.circle_requirements(1.5, 2, 21, 9.81)
        settings, result = calculator.calibrate(circle, calculator.PLANNER_DEFAULTS, runner)
        self.assertAlmostEqual(result["mean_speed"], 2)
        self.assertEqual(len(calls), 2)
        scale = calls[1][0] / calls[0][0]
        self.assertAlmostEqual(calls[1][1] / calls[0][1], scale ** 2)
        self.assertAlmostEqual(calls[1][2] / calls[0][2], scale ** 3)
        self.assertEqual(len(settings), 3)

    def test_nonconvergence_does_not_emit_settings(self):
        with self.assertRaises(calculator.ConfigurationError):
            calculator.calibrate(calculator.circle_requirements(1, 1, 21, 9.81),
                                 calculator.PLANNER_DEFAULTS, lambda _: {"mean_speed": .5})

    def test_file_setting_validation(self):
        output = io.StringIO()
        with mock.patch.object(calculator, "CIRCLE_RADIUS_M", math.nan), \
             contextlib.redirect_stderr(output), contextlib.redirect_stdout(output):
            result = calculator.main()
        self.assertEqual(result, 3)
        self.assertIn("输入无效", output.getvalue())
        self.assertNotIn('<arg name="max_vel"', output.getvalue())

    def test_cli_override_is_rejected(self):
        process = subprocess.run([sys.executable, str(SCRIPT), "nan", "1", "21"],
                                 capture_output=True, text=True)
        self.assertEqual(process.returncode, 3)
        self.assertIn("不支持命令行参数", process.stderr)
        self.assertNotIn('<arg name="max_vel"', process.stdout)


if __name__ == "__main__":
    unittest.main()
