#!/usr/bin/env python3
"""Three-input, offline circle parameter calculator; never sends flight commands.

Edit the three user settings below, then run: python3 calc_minisnap_circle.py
Hover setting is ALWAYS percent (21 means 21%; 0.21 means 0.21%).
"""

import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import json
import xml.etree.ElementTree as ET

# ===== 用户设置：只需修改下面三个值，然后直接运行本文件 =====
CIRCLE_RADIUS_M = 1.5            # 圆半径，单位 m。
TARGET_CRUISE_SPEED_MPS = 4    # 圆周巡航段平均速度，单位 m/s。
HOVER_THRUST_PERCENT = 21      # 悬停油门百分比：21表示21%，不要填写0.21。

# ===== 其他假设与调整量（通常无需修改） =====
GRAVITY_DEFAULT = 9.81
RESERVE_RATIO = 0.20  # 模型上限预留20%，不是已测得的扰动裕度。
OMMPC_COMMAND_MIN = 0.04  # ommpc_example.cpp::send_cmd 当前限幅。
OMMPC_COMMAND_MAX = 0.90
WAYPOINTS_PER_TURN = 8  # 当前 fly_cir.py: theta=i/16*4*pi。
CIRCLE_TARGETS = 34  # 当前 fly_cir.py: range(1,35)，之后回圆心。
SPEED_RELATIVE_TOLERANCE = 0.005
RADIUS_RELATIVE_TOLERANCE = 0.05  # 巡航段采样半径误差，非连续几何证明。
CALIBRATION_ATTEMPTS = 4
PROBE_TIMEOUT_SECONDS = 60
INITIAL_ACCELERATION_FACTOR = 1.5
INITIAL_JERK_FACTOR = 2.0
PLANNER_DEFAULTS = {
    "min_segment_time": 0.001, "limit_margin": 0.98,
    "seed_safety_ratio": 0.9, "max_total_time": 3600.0,
    "max_iterations": 30, "bound_tolerance": 0.002,
    "bound_max_depth": 12, "duplicate_distance": 1e-4,
    "max_segments": 2000, "compare_legacy_seed": True,
    "compression_budget_ms": 50.0, "compression_max_trials": 32,
}


class ConfigurationError(ValueError):
    pass


def positive(value, name):
    if isinstance(value, bool):
        raise ConfigurationError(name + " 必须为有效正数")
    try:
        value = float(value)
    except (ValueError, TypeError):
        raise ConfigurationError(name + " 必须为有效正数")
    if not math.isfinite(value) or value <= 0:
        raise ConfigurationError(name + " 必须为有效正数")
    return value


def circle_requirements(radius, speed, hover_percent, gravity):
    radius = positive(radius, "半径")
    speed = positive(speed, "速度")
    hover_percent = positive(hover_percent, "悬停油门百分比")
    if hover_percent > 100:
        raise ConfigurationError("悬停油门必须在 (0,100]%，例如21表示21%")
    try:
        omega = speed / radius
        acceleration = speed * omega
        jerk = acceleration * omega
        force = math.hypot(gravity, acceleration)
        result = {
            "radius": radius, "speed": speed, "hover": hover_percent / 100,
            "omega": omega, "acceleration": acceleration, "jerk": jerk,
            "period": 2 * math.pi / omega,
            "tilt_deg": math.degrees(math.atan2(acceleration, gravity)),
            "thrust": hover_percent / 100 * force / gravity,
            "specific_force": force,
            # 固定Euler偏航、水平理想圆的期望机体系角速度上界。
            "rate_xy": jerk / force,
            "rate_z": acceleration * jerk / (gravity * force),
        }
    except (OverflowError, ZeroDivisionError):
        raise ConfigurationError("输入量级导致数值溢出，无法计算")
    if not all(math.isfinite(v) and v > 0 for v in result.values()):
        raise ConfigurationError("输入量级导致数值溢出或下溢，无法计算")
    return result


def load_configuration(package):
    try:
        import yaml
    except ImportError:
        raise ConfigurationError("缺少 PyYAML；请使用装有 python3-yaml 的项目Python环境")
    try:
        tracking = yaml.safe_load((package / "config/traj_tracking_controller.yaml").read_text())
        ommpc = yaml.safe_load((package / "config/params.yaml").read_text())
        if not isinstance(tracking, dict) or not isinstance(ommpc, dict):
            raise ConfigurationError("YAML顶层必须是参数映射")
        launch = ET.parse(package / "launch/minisnap_3D.launch").getroot()
        control = tracking["control"]
        mpc = ommpc["MPC_params"]
        if not isinstance(control, dict) or not isinstance(mpc, dict):
            raise ConfigurationError("control和MPC_params必须是参数映射")
        gravity = positive(control.get("gravity", GRAVITY_DEFAULT), "control/gravity")
        tilt = positive(control["max_tilt_deg"], "control/max_tilt_deg")
        if tilt >= 89:
            raise ConfigurationError("控制器倾角必须小于89度")
        lower = positive(control["min_thrust_command"], "min_thrust_command")
        upper = positive(control["max_thrust_command"], "max_thrust_command")
        if not lower < upper <= 1:
            raise ConfigurationError("控制器推力上下限无效")
        vertical_low = positive(control["min_vertical_force_ratio"], "min_vertical_force_ratio")
        vertical_high = positive(control["max_vertical_force_ratio"], "max_vertical_force_ratio")
        if not vertical_low <= 1 <= vertical_high:
            raise ConfigurationError("竖直力限幅不包含重力，当前水平定高模型不适用")
        limits = {
            "gravity": gravity, "tilt_tangent": math.tan(math.radians(tilt)),
            "acceleration": positive(control["max_acceleration"], "control/max_acceleration"),
            "thrust_min": lower, "thrust_max": upper,
            "vertical_force_min": vertical_low, "vertical_force_max": vertical_high,
            "force_min": positive(mpc["min_thrust"], "MPC_params/min_thrust"),
            "force_max": positive(mpc["max_thrust"], "MPC_params/max_thrust"),
            "rate_xy": positive(mpc["max_bodyrate_xy"], "max_bodyrate_xy"),
            "rate_z": positive(mpc["max_bodyrate_z"], "max_bodyrate_z"),
        }
        if limits["force_min"] >= limits["force_max"] or not limits["force_min"] <= gravity:
            raise ConfigurationError("OMMPC推力加速度限幅不适用于当前悬停模型")
        arguments = {n.attrib["name"]: n.attrib.get("default", "") for n in launch.findall("arg")}
        node = next(n for n in launch.findall("node") if n.attrib.get("type") == "trajectory_generator_node")
        parameters = {}
        for param in node.findall("param"):
            raw = param.attrib.get("value", "")
            raw = re.sub(r"\$\(arg ([^)]+)\)", lambda m: arguments[m.group(1)], raw)
            parameters[param.attrib["name"]] = raw
        if int(parameters.get("planning/dev_order", "4")) != 4:
            raise ConfigurationError("当前规划节点只支持 dev_order=4")
        options = dict(PLANNER_DEFAULTS)
        for key, default in options.items():
            raw = parameters.get("planning/" + key)
            if raw is not None:
                if isinstance(default, bool):
                    if raw.lower() not in ("true", "false", "1", "0"):
                        raise ConfigurationError("无效布尔规划参数: " + key)
                    options[key] = raw.lower() in ("true", "1")
                elif isinstance(default, int):
                    options[key] = int(raw)
                else:
                    options[key] = float(raw)
        if not 0 < options["limit_margin"] <= 1 or not 0 < options["seed_safety_ratio"] < 1:
            raise ConfigurationError("规划裕度参数无效")
        for key, value in options.items():
            if not math.isfinite(float(value)):
                raise ConfigurationError("非有限规划参数: " + key)
        return limits, options
    except (OSError, ET.ParseError, KeyError, TypeError, StopIteration, ValueError, yaml.YAMLError) as e:
        raise ConfigurationError("配置读取失败: " + str(e))


def controller_checks(requirements, limits, reserve=RESERVE_RATIO, hover=None):
    """Independent controllers: never apply tracking limits to the OMMPC result."""
    use = 1 - reserve
    r, g = requirements, limits["gravity"]
    def item(label, value, minimum, maximum, unit=""):
        return {"label": label, "value": value, "min": minimum, "max": maximum,
                "unit": unit, "passed": minimum <= value <= maximum}
    tracking = [
        item("参考加速度（模长保守检查）", r["acceleration"], 0,
             limits["acceleration"] * use, "m/s²"),
        item("所需倾角", math.degrees(math.atan2(r["acceleration"], g)), 0,
             math.degrees(math.atan(limits["tilt_tangent"] * use)), "°"),
        item("归一化推力", r["thrust"], limits["thrust_min"], limits["thrust_max"] * use),
        item("竖直力/重力（水平定高）", 1, limits.get("vertical_force_min", 1),
             limits.get("vertical_force_max", 1)),
    ]
    ommpc = [
        item("推力加速度", r["specific_force"], limits["force_min"],
             limits["force_max"] * use, "m/s²"),
        item("归一化推力（send_cmd）", r["thrust"], OMMPC_COMMAND_MIN, OMMPC_COMMAND_MAX * use),
        item("期望水平机体角速度（模长保守检查）", r["rate_xy"], 0,
             limits["rate_xy"] * use, "rad/s"),
        item("期望Z机体角速度", r["rate_z"], 0, limits["rate_z"] * use, "rad/s"),
    ]
    if hover is None:
        hover = r.get("hover")
    if hover is not None:
        tracking.append(item("悬停归一化推力", hover, limits["thrust_min"], limits["thrust_max"] * use))
        ommpc.append(item("悬停归一化推力", hover, OMMPC_COMMAND_MIN, OMMPC_COMMAND_MAX * use))
        ommpc.append(item("悬停推力加速度", g, limits["force_min"], limits["force_max"] * use, "m/s²"))
    return {"tracking": tracking, "OMMPC": ommpc}


def violations(requirements, limits, reserve=RESERVE_RATIO):
    # Compatibility for existing offline tests; the UI uses the separated reports.
    return [name + ": " + check["label"] + "未通过"
            for name, checks in controller_checks(requirements, limits, reserve).items()
            for check in checks if not check["passed"]]


def display_controller_results(stage, requirements, limits, hover):
    raw = controller_checks(requirements, limits, 0, hover)
    reserved = controller_checks(requirements, limits, RESERVE_RATIO, hover)
    print("\n=== %s：分别检查两套控制器 ===" % stage)
    results = {}
    for name in ("tracking", "OMMPC"):
        raw_ok = all(c["passed"] for c in raw[name])
        reserve_ok = all(c["passed"] for c in reserved[name])
        results[name] = {"raw": raw_ok, "reserve": reserve_ok}
        print("[%s] 原始限幅：%s；%g%%裕度：%s" %
              (name, "通过" if raw_ok else "未通过", 100 * RESERVE_RATIO,
               "通过" if reserve_ok else "未通过"))
        for original, check in zip(raw[name], reserved[name]):
            print("  %s: 所需 %.5g %s；原始范围[%.5g, %.5g]，裕度范围[%.5g, %.5g] -> %s" %
                  (check["label"], check["value"], check["unit"], original["min"], original["max"],
                   check["min"], check["max"], "通过" if check["passed"] else "未通过"))
    return results


def display_final_results(initial, full=None, geometry_ok=True):
    print("\n=== 最终结果 ===")
    passed = {}
    for name in ("tracking", "OMMPC"):
        passed[name] = initial[name]["reserve"] and full is not None and full[name]["reserve"] and geometry_ok
        if passed[name]:
            reason = "通过（标准模板完整参考轨迹的模型/裕度检查；非真机保证）"
        elif not initial[name]["reserve"]:
            reason = "不可实现于当前模型/裕度策略：理想圆初筛未通过，具体项见上方"
        elif full is None:
            reason = "完整轨迹未预演，不能将初筛通过当作最终可行"
        elif not full[name]["reserve"]:
            reason = "当前候选方案不可实现：完整轨迹保守检查未通过，具体项见上方"
        else:
            reason = "当前候选方案不可实现：未通过共同的半径精度检查"
        print("[%s] %s" % (name, reason))
    return all(passed.values())


def probe_requirements(probe, hover, gravity):
    a, j = probe["peak_acceleration"], probe["peak_jerk"]
    force = math.hypot(gravity, a)
    return {
        "acceleration": a, "jerk": j, "specific_force": force,
        "thrust": hover * force / gravity,
        # Conservative continuous bounds for fixed-Euler-yaw horizontal motion.
        "rate_xy": j / gravity,
        "rate_z": j / gravity * a / force,
    }


def find_probe(package):
    # In a sourced catkin environment this is not necessarily on PATH.
    candidates = [package / "lib/minisnap_circle_probe"]
    for ancestor in package.parents:
        candidates.extend((ancestor / "devel/lib/ommpc_controller/minisnap_circle_probe",
                           ancestor / "devel/MPC_basic/lib/ommpc_controller/minisnap_circle_probe"))
    path = shutil.which("minisnap_circle_probe")
    if path:
        candidates.insert(0, Path(path))
    for candidate in candidates:
        if candidate.is_file() and os.access(str(candidate), os.X_OK):
            return candidate
    raise ConfigurationError("找不到离线求解器；先运行 /home/yundrone/Sunray/build_OMMPC.sh")


def find_package():
    package = Path(__file__).resolve().parent.parent
    if (package / "config/traj_tracking_controller.yaml").is_file():
        return package
    try:
        result = subprocess.run(["rospack", "find", "ommpc_controller"],
                                capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            return Path(result.stdout.strip())
    except (OSError, subprocess.TimeoutExpired):
        pass
    raise ConfigurationError("找不到 ommpc_controller 包；请直接运行包内 traj 下的脚本")


def run_probe(executable, radius, settings, options):
    keys = ("min_segment_time", "limit_margin", "seed_safety_ratio", "max_total_time",
            "max_iterations", "bound_tolerance", "bound_max_depth", "duplicate_distance",
            "max_segments", "compare_legacy_seed", "compression_budget_ms", "compression_max_trials")
    values = [radius] + list(settings) + [int(options[k]) if isinstance(options[k], bool) else options[k]
                                       for k in keys] + [WAYPOINTS_PER_TURN, CIRCLE_TARGETS]
    try:
        process = subprocess.run([str(executable)] + [format(float(v), ".17g") for v in values],
                                 capture_output=True, text=True, timeout=PROBE_TIMEOUT_SECONDS)
        if process.returncode:
            raise ConfigurationError(process.stderr.strip() or "离线规划失败")
        probe = json.loads(process.stdout)
        needed = ("mean_speed", "minimum_speed", "maximum_speed", "radial_error",
                  "peak_velocity", "peak_acceleration", "peak_jerk", "total_time", "planning_ms")
        for key in needed:
            value = probe[key]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ConfigurationError("离线求解器返回无效数据: " + key)
        if probe["mean_speed"] <= 0:
            raise ConfigurationError("巡航段平均速度为零，无法校准")
        return probe
    except (OSError, subprocess.TimeoutExpired, json.JSONDecodeError, KeyError, TypeError) as e:
        raise ConfigurationError("无法完成离线预演: " + str(e))


def calibrate(circle, options, runner):
    margin = options["limit_margin"]
    settings = [circle["speed"] / margin,
                INITIAL_ACCELERATION_FACTOR * circle["acceleration"] / margin,
                INITIAL_JERK_FACTOR * circle["jerk"] / margin]
    for attempt in range(CALIBRATION_ATTEMPTS):
        if not all(math.isfinite(x) and x > 0 for x in settings):
            raise ConfigurationError("校准参数数值溢出")
        probe = runner(settings)
        ratio = circle["speed"] / probe["mean_speed"]
        if abs(ratio - 1) <= SPEED_RELATIVE_TOLERANCE:
            return settings, probe
        settings = [settings[0] * ratio, settings[1] * ratio ** 2, settings[2] * ratio ** 3]
    raise ConfigurationError("校准未收敛，不能给出有效的最终参数")


def format_settings(settings):
    # Preserve precision so a printed recommendation reproduces the rehearsal.
    return [format(x, ".12g") for x in settings]


def main():
    try:
        if not 0 <= RESERVE_RATIO < 1:
            raise ConfigurationError("RESERVE_RATIO必须在[0,1)之间")
        # Validate inputs before attempting to locate ROS package/configuration.
        radius = positive(CIRCLE_RADIUS_M, "半径")
        speed = positive(TARGET_CRUISE_SPEED_MPS, "速度")
        hover_percent = positive(HOVER_THRUST_PERCENT, "悬停油门百分比")
        circle_requirements(radius, speed, hover_percent, GRAVITY_DEFAULT)
        package = find_package()
        limits, options = load_configuration(package)
        circle = circle_requirements(radius, speed, hover_percent, limits["gravity"])
        print("目标: 半径 %.4g m，巡航平均速度 %.4g m/s，悬停油门 %.4g%%" %
              (radius, speed, hover_percent))
        print("理想圆: 向心加速度 %.4f m/s²，jerk %.4f m/s³，倾角 %.2f°，推力 %.4f" %
              (circle["acceleration"], circle["jerk"], circle["tilt_deg"], circle["thrust"]))
        initial = display_controller_results("理想圆初筛", circle, limits, circle["hover"])
        # Only stop if NEITHER controller can pass. A tracking failure must not
        # prevent rehearsal/complete diagnostics for a potentially valid OMMPC.
        if not any(result["reserve"] for result in initial.values()):
            display_final_results(initial)
            print("两套控制器初筛均未通过；不预演、不输出共同适用的最终参数。")
            return 2
        executable = find_probe(package)
        settings, probe = calibrate(circle, options,
                                    lambda s: run_probe(executable, radius, s, options))
        # Re-run EXACTLY the values to be printed, not hidden higher precision values.
        settings = [float(x) for x in format_settings(settings)]
        probe = run_probe(executable, radius, settings, options)
        if abs(probe["mean_speed"] / speed - 1) > SPEED_RELATIVE_TOLERANCE:
            raise ConfigurationError("打印精度复核失败，不能输出参数")
        demands = probe_requirements(probe, circle["hover"], limits["gravity"])
        full = display_controller_results("完整多项式预演（保守上界）", demands, limits, circle["hover"])
        geometry_ok = probe["radial_error"] <= radius * RADIUS_RELATIVE_TOLERANCE
        print("[共同半径精度] 巡航采样误差 %.5g m；允许 %.5g m -> %s" %
              (probe["radial_error"], radius * RADIUS_RELATIVE_TOLERANCE,
               "通过" if geometry_ok else "未通过"))
        if not display_final_results(initial, full, geometry_ok):
            print("未满足两套控制器共同适用条件；不输出共同适用的最终参数。")
            print("这是对本方案的拒绝，不证明所有其他轨迹设计都不可行。")
            return 2
        print("\n可实现（标准航点模板的离线参考轨迹满足模型限幅及%g%%裕度；非真机安全认证）" %
              (100 * RESERVE_RATIO))
        print("巡航平均速度 %.6f m/s；巡航瞬时速度范围 %.4f～%.4f m/s" %
              (probe["mean_speed"], probe["minimum_speed"], probe["maximum_speed"]))
        print("巡航采样半径最大误差 %.4f m；全轨迹 V/A/J 保守上界 %.4f / %.4f / %.4f" %
              (probe["radial_error"], probe["peak_velocity"], probe["peak_acceleration"], probe["peak_jerk"]))
        print("全轨迹时间 %.3f s；本机离线计算耗时 %.1f ms" % (probe["total_time"], probe["planning_ms"]))
        print("\nminisnap_3D.launch 参数（不是下发命令）：")
        v, a, j = format_settings(settings)
        for name, value in (("dev_order", "4"), ("max_vel", v), ("max_acc", a), ("max_jerk", j)):
            print('    <arg name="%s" default="%s"/>' % (name, value))
        print('    <!-- 保持预演确定性：替换规划节点内现有的同名参数 -->')
        print('    <param name="planning/compress_time" value="false"/>')
        print("\n适用条件: fly_cir.py radius=%.12g，setup=0，8点/圈，34圆周目标点后回圆心；" % radius)
        print("          起点必须已悬停在圆周首点 (圆心x, 圆心y+radius, 固定高度)。")
        print("          航点半径不由launch控制；从圆心/其他位置起飞后直接发送，须重新预演。")
        print("悬停油门输入仅用于检查；控制器hover_thrust/hover_percentage应与实测值一致，本脚本不修改。")
        print("未检查: 实际推力非线性、电池、载荷、扰动、闭环稳定性、场地边界、障碍物及定位质量。")
        print("该结果不是全程恒速/精确圆；须在仿真及实际生成的参考轨迹中复核。")
        return 0
    except (ConfigurationError, OverflowError) as e:
        print("无法判定/输入无效：" + str(e), file=sys.stderr)
        print("[tracking] 完整轨迹无法判定；[OMMPC] 完整轨迹无法判定。", file=sys.stderr)
        print("未输出最终参数；不能把配置、求解或工具故障当作物理不可实现。", file=sys.stderr)
        return 3


if __name__ == "__main__":
    if len(sys.argv) != 1:
        print("不支持命令行参数；请修改文件开头的三个用户设置，然后直接运行本文件。", file=sys.stderr)
        sys.exit(3)
    sys.exit(main())
