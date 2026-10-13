# TITATIT USD workspace audit

The WBC configuration uses the joint chain from the local
`assets/titatit_piper/titatit_piper.usd`, including joint frames, axes and limits.
At nominal chassis height 0.30 m the authored mount chain places link0 at
world height 0.44252 m. This calculation assumes a level chassis.

Run with a Python environment providing USD (`pxr`), NumPy and SciPy:

```bash
python scripts/tools/check_titatit_workspace.py --full-limits --samples 200
```

The seeded check samples X=[0.25,0.55], Y=[-0.30,0.30], world Z=[0.50,0.82]
in meters. It applies the same 0.60 m radial projection, clamped look-at Euler
angles and Piper tool-frame rotation as the command generator. Joint targets
use 90% of the USD angular endpoints, retaining zero offsets and one-sided
joint2/joint3 ranges. This is not the centered Isaac Lab soft-limit formula.

Observed result: 180/200 targets solved within 2 cm and 10 degrees;
95th-percentile position residual 2.90 cm, maximum 4.24 cm;
maximum orientation residual 0.2845 rad. The remaining targets are not
certified reachable at this tolerance. Numerical IK failures do not prove
geometric impossibility. These figures are offline IK residuals, not policy
tracking performance. This box expands lateral/forward reach but deliberately
removes near-base X targets, so it is not a strict superset of the old box.

No collision, torque, dynamic stability, or tilted-base reachability guarantee
is provided by this audit. The current asset disables self-collisions; replay
must therefore also inspect arm/chassis interference visually.

Base commands start at +/-0.2 m/s and +/-0.2 rad/s, and curricula can reach
+/-1 m/s and +/-1 rad/s. Lateral commands remain zero for the fixed wheels.
Wheel targets now reach the configured actuator velocity ceiling of 25 rad/s;
that does not guarantee the requested base speed under load or while turning.

Arm and wheel action scaling changed: use a new training run, not unchanged
replay of an old checkpoint. PPO settings are unchanged. Play deliberately
retains a stationary base and initial target ranges for initial diagnostics.
