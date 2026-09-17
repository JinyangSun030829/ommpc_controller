# Tracking cancellation and LAND safety fix

LAND is a single dynamic-reconfigure update, not a separate HOVER command
followed seconds later by LAND. State numbers 0..3 remain unchanged; BRAKE=4.

POLY cancellation/completion and LAND first construct a bounded stop reference
from the current polynomial p/v/a (or the last emitted reference). The velocity
curve has Bernstein controls `[v0, v0+a0*T/4, 0, 0, 0]`. Derivative control-point
norms bound acceleration and jerk over the entire interval. Duration is increased
until those bounds pass. End velocity, acceleration and jerk are zero. Entry
p/v/a are continuous, but initial jerk need not equal the incoming jerk.
If initial acceleration already exceeds the configured braking limit, the limit
is explicitly raised to 1.05 times initial acceleration and printed; it cannot
be instantaneously removed. This is not obstacle avoidance: reserve stopping
space, especially for long/high-speed trajectories.

After the curve, actual speed/position span must pass the same sustained hover
gate for a full second, and position error must be <= braking/position_tolerance.
Only then may descent begin. LAND cannot be interrupted by COMMAND. A LAND
request during HOVER braking upgrades its destination without restarting.

Descent uses quintic speed ramps with continuous p/v/a/jerk at joins. Default
braking limits are 3 m/s², 4 m/s³; descent limits .5 m/s², 1 m/s³, speed .25 m/s.
The duration and the sustained-hover requirement are printed separately.

The force-to-ZYX conversion uses `roll=atan2(-Fy,hypot(Fx,Fz))`; the resulting
body Z axis is exactly parallel to the limited desired force, so the final
quaternion respects the configured tilt limit.

UDE now filters velocity increments minus modeled applied acceleration. The
model uses the previous limited thrust and measured attitude; it is still an
approximation requiring a calibrated hover-thrust model. tau and max_estimate
retain their meanings. max_integral is accepted for old YAML compatibility but
is no longer used: clipping an absolute velocity integral created a bias.
Distinct sensor timestamps drive the observer; duplicate samples do not update,
and clock reversal or a large gap resets it. Near-ground disable/reenable heights
default to .10/.25 m; compensation retains the activation ramp.

Touchdown requires fresh PX4 ON_GROUND AND low height/velocity for .5 s. Unloading
lasts .8 s before disarm is attempted. Lost/stale ON_GROUND cancels unloading.

State age > control/state_timeout (default .10 s) starts a limited last-command
hold (control/state_command_hold_time, default .20 s). A longer outage stops
setpoints and latches safety; no autonomous disarm/mode switch is performed.
In-flight OFFBOARD/ARMED loss also stops output and latches. Inspect/restart on
ground; old flight references never automatically resume. The resulting PX4
action depends on its OFFBOARD-loss configuration, which MUST be verified in SITL
before real flight. Do not treat this implementation as a guaranteed safe landing
when state estimation is lost.

Hover trimming now preserves both scheduling-time and measurement-time anchors,
preventing jitter from discarding a full one-second window. Requirements are not
relaxed. Rejection logs print takeoff/FSM/MAVROS/mode/armed and window coverage;
HOVER READY explicitly prints the confirmed duration. MAVROS state and extended
state are added to the bag for future diagnosis.

Tests: flight_safety_offline_test checks force alignment, final tilt, 100 random
braking boundaries, analytical bounds, derivative consistency, short/long descent,
and observer convergence/duplicates/gaps. hover_gate_offline_test adds clock
jitter. fsm_safety_ros_test exercises LAND/BRAKE/hover wait/COMMAND rejection/UDE
hysteresis/ground confirmation on a private loopback ROS master. These are not
PX4/Gazebo dynamics or real-flight qualification.
