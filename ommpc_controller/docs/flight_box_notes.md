# Shared cuboid protection

Remote package: /home/yundrone/Sunray/External_Module/MPC_basic/src/ommpc_controller/ommpc_controller

Both launch files load config/flight_box.yaml into the controller private namespace.
Both state machines use the same BoxStop/BoxGuard implementation in
include/traj_tracking_controller/box_flight_safety.hpp.

## Configuration

- minimum/maximum: world ENU xyz metres; examples are not calibrated site boundaries.
- margin: optional, defaults to 0. A scalar 0.5 moves the four X/Y faces
  inward by 0.5 m. Z limits remain unchanged. An xy array sets the two
  horizontal axes separately; legacy xyz arrays ignore Z with a warning.
- enabled: 0/false preserves the old stop, 1/true enables cuboid protection.
  margin=0 does NOT disable protection.
- reaction_time: sensor/actuator delay reserve; check_interval is added to it.
  Prediction advances on the actual reference curve plus p/v tracking-error
  extrapolation, rather than freezing the current acceleration direction.
- lookahead/lookahead_samples: preventive trajectory preview; not continuous
  verification of the complete stopping envelope along every future time.
- duration_trials/max_stop_duration/subdivision_depth: finite search/certificate
  limits. Failure means no certified candidate found, not proven global infeasibility.
- path_stop_segments: 4..64 (default 16) Hermite pieces for a trajectory-guided
  stop. This controls reconstruction density, not collision-sampling safety.
- braking/max_acceleration and braking/max_jerk stay in each existing controller YAML.
  With protection enabled the limits are strict. Polynomial admission verifies
  its full acceleration is within the configured braking limit, so a trajectory
  exceeding that limit is rejected instead of silently increasing the limit.

The inset box must contain the ARM-time origin, the absolute takeoff altitude,
and the relative LAND target (origin Z - 0.1 m). Otherwise takeoff is rejected
before ARM, and the ARM-time position is checked again.
No obstacle map, safe corridor, or path-around-obstacle logic is implemented.

## Runtime

1. Polynomial paths are certified over complete segments using subdivided
   Bernstein control-point hulls; safe endpoints alone are insufficient.
2. BOX BRAKE first tries a trajectory-guided stop when a preview is available:
   phase speed decreases smoothly from 1 to 0 while the original path keeps
   turning. Quintic Hermite pieces reconstruct this guide, preserving p/v/a
   at entry and joins; the final piece additionally reaches zero terminal jerk.
   The executed curve is this reconstruction, not an uncertified sampled path.
   Every piece has continuous-time position/A/J certification. If the guide
   fails, the existing duration/alternative-endpoint search is still attempted.
   Certified position,
   vector acceleration and vector jerk bounds cover the entire stop curve.
   Entry p/v/a are retained; terminal v/a/j are zero.
3. Actual position is checked against the outer box each control cycle. Every
   check_interval the current reference stop, a measured-p/v shadow stop, delay
   reserve, and sampled future reference AND measured-error shadow stops are
   assessed using the same trajectory-guided/generic planner used by BRAKE.
   Completed polynomials are extended as stationary endpoints for preview.
   During BRAKE, the already certified remainder is reused for matching nominal
   states instead of falsely declaring failure when a second stop search fails.
   Measured deviations still require a separate certificate; outer-box checks
   never use this shortcut. A second guide profile also attempts an unwarped
   stopping preview followed by zero terminal derivatives.
4. If only the future/delay reserve fails, stop now and hold HOVER, or continue
   the LAND-owned stop/stable-hover/descent sequence.
5. If the current stop cannot be certified, latch the fault, reject COMMAND/
   trajectories and stop actuator setpoints. No automatic disarm or normal LAND
   is requested. The independently configured PX4 OFFBOARD-loss action then
   determines the vehicle response: it must be verified before enabling this.
   This fallback is not guaranteed to remain inside the box.
6. A requested LAND whose vertical path/target is outside the inset box remains
   at the confirmed stop point rather than clamping the landing target.
7. A preventive stop during takeoff does not claim takeoff completion or publish
   ground standby thrust after confirmed liftoff; trajectories remain disabled,
   but a subsequent LAND request is allowed using the recorded origin.

## Limitations / commissioning

This certifies nominal references, not physical flight in the presence of
unbounded disturbances, saturation, estimation errors, or delayed controller
execution. Measured acceleration is not available from these state interfaces;
the shadow-stop acceleration uses reference acceleration as an approximation.
Zero margin leaves no reserved position-error buffer. Initial acceleration or
speed may already make certification impossible; no instant stop or hard
position clipping can fix such a state.
The p/v-error forecast assumes constant error velocity; it is not a robust
uncertainty tube. No limits are silently increased for circular flight:
centripetal acceleration v^2/R must fit the configured acceleration capability,
and a certified stopping manoeuvre must fit the acceleration/jerk bounds.
Geometric path containment alone is not a promise of an emergency stop.
Hermite joins are C2 (jerk may have finite jumps), not globally C3.
This change does not rescale the running normal trajectory or change hardware
failsafe policy. Search failure still latches; OFFBOARD-loss handling must be
validated before enablement.

Do not use example limits on hardware. Measure the actual bounds and frame,
verify braking/tilt/thrust capability and PX4 failsafe, then test at low speed in
simulation before hardware commissioning. No running flight node is restarted
by deployment or tests.

## 2026-09-17 consistency fixes

- Stationary stops initialize all derivative control points, including reuse.
- BoxAssessment retains the current nominal stop and measured p/v snapshot.
  Preventive BRAKE consumes that exact plan when seed, measurement, bounds and
  limits still match. A changed context forces new certification.
- Explicit stops in both FSMs use BoxGuard::planStop and state-specific previews
  (TAKEOFF/HOVER/LAND included), not a different restricted candidate family.
- OMMPC TXT control and preview share txtReferenceAt, including finite-difference
  acceleration/jerk and a stationary terminal preview. The file remains a
  discrete sampled reference; this does not certify it as a continuous polynomial.
  On natural completion the original terminal seed velocity is retained for the
  bounded BRAKE rather than silently deleting nonzero incoming velocity.
- No velocity-frame switches, state-outage/failsafe policy or polynomial
  admission gates are changed by this patch. flight_box/enabled stays unchanged.
