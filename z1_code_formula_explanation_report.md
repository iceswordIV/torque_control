---
title: "Unitree Z1 Torque-Control Project: Code and Formula Explanation"
author: "Prepared with ChatGPT for Chunsheng Zeng"
date: "2026-06-04"
geometry: margin=0.75in
fontsize: 10pt
papersize: a4
---

# Unitree Z1 Torque-Control Project: Code and Formula Explanation

## Executive summary

This report explains the main formulas and code structure used in the Unitree Z1 torque-control project. The project compares four custom torque-control variants against a Unitree SDK low-command baseline in Gazebo:

1. Augmented PD control without friction/damping compensation.
2. Augmented PD control with friction/damping compensation.
3. Computed torque control without friction/damping compensation.
4. Computed torque control with friction/damping compensation.
5. Unitree SDK lowcmd baseline.

The most important conclusion from the experiments is that the best pure-torque controller is the augmented PD controller with friction/damping compensation. Computed torque is theoretically more complete, but in this project it is limited by model mismatch, external-loop delay, and lower effective feedback bandwidth. The SDK baseline uses a different control architecture: it sends desired joint position, desired velocity, feedforward torque, and low-level gains into the Unitree/Gazebo control path. That internal feedback can generate much larger effective joint torque than the explicit feedforward torque value alone.

The gravity term `N(q)` is also explained carefully. In this project `N(q)` is the gravity torque vector, not a matrix. It can be computed from the derivative of gravitational potential energy or by the virtual-work/Jacobian-transpose method. The uploaded gravitational-forces note uses the relation:

```text
partial V_i / partial q = - J_i(q)^T F^b_wi
```

The project can validate the implemented `gravity_vector(q)` by comparing three independent calculations: the current code, finite-difference derivative of total potential energy, and the body-Jacobian virtual-work formula.

# 1. Runtime architecture

The main runtime script is:

```text
z1_project/torque_main.py
```

It does not integrate the robot state itself. Instead, it receives measured joint position and velocity from Gazebo or the robot bridge, computes a torque command, sends the torque command out, and logs the result.

The runtime data flow is:

```text
Gazebo / robot feedback
    -> q_actual, dq_actual
    -> trajectory.py builds q_des, dq_des, ddq_des
    -> controller.py or test_controller.py computes tau
    -> robot_io.py writes tau to file IPC
    -> ros_torque_bridge.py publishes torque command to Gazebo
    -> Gazebo physics updates q_actual, dq_actual
```

This means the custom torque controller is an external loop around Gazebo. It is not the same as Unitree SDK lowcmd control, where the low-level feedback may be applied inside the Unitree/Gazebo controller.

## 1.1 Main files

| File | Purpose |
|---|---|
| `torque_main.py` | Main online runtime loop. Reads state, builds trajectory, selects controller, writes CSV logs. |
| `trajectory.py` | Builds one-joint, scaled-pose, full-pose, quintic, and S-curve trajectories. |
| `controller.py` | Main computed-torque controller. |
| `test_controller.py` | Diagnostic and comparison controllers: augmented PD, friction-model variants, gravity-only, etc. |
| `z1_analytic_dynamics.py` | Analytic dynamics model: mass matrix, `dM/dq`, Coriolis matrix, gravity vector. |
| `ros_torque_bridge.py` | ROS/Gazebo file bridge. Reads torque file and publishes `MotorCmd` messages. |
| `robot_io.py` | File-based communication interface between Python controller and bridge. |
| `compare_torque.py` | Offline torque preview and feedforward/computed-torque comparison. |
| `simulate_offline.py` | Closed-loop offline simulation using the project dynamics model. |
| `record_sdk_forward_torque.py` / SDK scripts | SDK baseline and replay/record experiments. |

# 2. Joint-space trajectory generation

The controller operates in joint space. For every sample time it needs:

```text
q_des(t), dq_des(t), ddq_des(t)
```

The project supports one-joint relative motion, absolute pose motion, and scaled pose motion. The forward pose target used in the experiments is:

```text
q_target = [0, 1.5, -1.0, -0.54, 0, 0] rad
```

For a scaled-pose test, the goal is:

```text
q_goal = q_start + scale * (q_target - q_start)
```

where `scale` is 0.25, 0.50, 0.75, or 1.00.

## 2.1 Quintic trajectory

The original trajectory profile is quintic:

```text
s = t / T
b(s) = 10 s^3 - 15 s^4 + 6 s^5
q_des = q_start + b(s) * (q_goal - q_start)
```

The velocity and acceleration are analytic:

```text
db/dt  = (30 s^2 - 60 s^3 + 30 s^4) / T
 d2b/dt2 = (60 s - 180 s^2 + 120 s^3) / T^2
```

Then:

```text
dq_des  = db/dt  * (q_goal - q_start)
ddq_des = d2b/dt2 * (q_goal - q_start)
```

## 2.2 Seventh-order S-curve trajectory

Most final Gazebo tests use the smoother S-curve profile:

```text
b(s) = 35 s^4 - 84 s^5 + 70 s^6 - 20 s^7
```

with:

```text
db/dt = (140 s^3 - 420 s^4 + 420 s^5 - 140 s^6) / T

d2b/dt2 = (420 s^2 - 1680 s^3 + 2100 s^4 - 840 s^5) / T^2
```

This matters because torque control depends directly on `ddq_des`. A smoother acceleration profile reduces torque discontinuities and improves stability.

# 3. Manipulator dynamics model

The project uses the standard joint-space rigid-body dynamics form:

```text
M(q) qdd + C(q, dq) dq + N(q) = tau
```

where:

| Symbol | Meaning |
|---|---|
| `q` | 6x1 joint angle vector. |
| `dq` | 6x1 joint velocity vector. |
| `qdd` | 6x1 joint acceleration vector. |
| `M(q)` | 6x6 joint-space mass/inertia matrix. |
| `C(q,dq)` | 6x6 Coriolis/centrifugal matrix. |
| `N(q)` | 6x1 gravity torque vector. |
| `tau` | 6x1 commanded joint torque vector. |

The main dynamics implementation is in:

```text
z1_analytic_dynamics.py
```

The project uses product-of-exponentials style kinematics, spatial/body Jacobians, inertial parameters, and center-of-mass transforms. The code follows the downloaded Unitree xacro/URDF-style geometry rather than the slightly different pasted table of absolute Z coordinates.

## 3.1 Mass matrix

The mass matrix is computed by summing link kinetic-energy contributions. For link `l`, the project builds the body Jacobian columns `J_lj`. Then the mass matrix is assembled as:

```text
M_ij(q) = sum over links l >= max(i,j) of J_li(q)^T G_l J_lj(q)
```

where `G_l` is the spatial inertia of link `l` transformed to the link COM frame. The code symmetrizes `M`:

```text
M = 0.5 * (M + M.T)
```

This helps remove tiny numerical asymmetries.

## 3.2 Derivative of mass matrix

Computed torque requires the Coriolis matrix. The project has two ways to compute `dM/dq`:

1. Analytic Lie-bracket derivative of the Jacobian columns.
2. Finite-difference derivative of `M(q)`.

The analytic path is faster and cleaner when the compiled fast backend is available. The finite-difference path is useful for verification.

The finite-difference central method is:

```text
dM/dq_k approximately [M(q + h e_k) - M(q - h e_k)] / (2h)
```

The forward method is:

```text
dM/dq_k approximately [M(q + h e_k) - M(q)] / h
```

## 3.3 Coriolis matrix from dM/dq

The Coriolis matrix is built from Christoffel-style terms:

```text
C_ij(q,dq) = 0.5 * sum_k [dM_ij/dq_k + dM_ik/dq_j - dM_kj/dq_i] dq_k
```

This is implemented in `coriolis_from_dM()`.

# 4. Gravity vector N(q)

In this project, `N(q)` means gravity only. Damping and friction are handled separately in the friction-model controllers.

The total potential energy is:

```text
V(q) = sum_i m_i g h_i(q)
```

where `h_i(q)` is the vertical position of the center of mass of link `i`. The gravity torque vector is:

```text
N(q) = partial V / partial q
```

This means each joint component is:

```text
N_j(q) = partial V / partial q_j
```

`N_j` is not only the mass of link `j`. It includes all downstream masses whose COM height changes when joint `j` moves.

## 4.1 Virtual-work form used to validate N(q)

The uploaded note explains the equivalent virtual-work expression:

```text
partial V_i / partial q = - J_i(q)^T F^b_wi
```

where `J_i(q)` is the body Jacobian of link `i`, and `F^b_wi` is the gravity wrench of link `i` expressed in the body frame. The gravity wrench starts in the space frame as:

```text
F^s_wi = [0, 0, -m_i g, 0, 0, 0]^T
```

and is transformed into the body frame by:

```text
F^b_wi = Ad_gsli^T F^s_wi
```

The full gravity vector can therefore be checked by:

```text
N(q) = sum_i partial V_i / partial q
     = - sum_i J_i(q)^T F^b_wi
```

depending on the sign convention used in the equation of motion. If the result has the opposite sign, the sign convention is reversed, not necessarily physically wrong.

## 4.2 Recommended N(q) validation script

A useful validation script should compare:

```text
1. gravity_vector(q) from the project code
2. finite-difference derivative of V(q) = sum_i m_i g z_i(q)
3. body-Jacobian virtual-work result: -sum_i J_i^T F_i^b
```

For correct implementation, the maximum absolute difference should be very small, typically around `1e-6` to `1e-4` depending on the finite-difference step.

This test directly answers whether the implemented `N(q)` is consistent with both the potential-energy and Jacobian-transpose formulations.

# 5. Controller formulas

The project compares four custom controllers. All controllers operate in joint space and use measured `q_actual, dq_actual` from Gazebo or robot feedback.

Define:

```text
e  = q_des  - q
de = dq_des - dq
```

## 5.1 Computed torque control

The computed-torque controller uses:

```text
ddq_cmd = ddq_des + Kd de + Kp e

tau = M(q) ddq_cmd + C(q,dq) dq + N(q)
```

This attempts to cancel nonlinear dynamics and impose a second-order tracking-error behavior. In theory, if the model is exact and the loop rate is high, computed torque should track well.

In practice, in this project computed torque is sensitive to:

- errors in `M`, `C`, and `N`,
- inaccurate or delayed `q,dq` feedback,
- low external-loop bandwidth,
- friction/deadband effects in Gazebo,
- torque saturation and solver behavior.

## 5.2 Computed torque with friction/damping compensation

The friction/damping version adds a Gazebo-style model:

```text
tau = M(q) ddq_cmd + C(q,dq) dq + N(q)
    + tau_damping + tau_friction
```

where:

```text
tau_damping = D * dq_des
```

and:

```text
tau_friction = F * direction
```

The `direction` is taken from `sign(dq_des)` when the desired velocity is nonzero; near zero velocity it uses the sign of the position error if the error is outside a small deadband.

The official URDF-like constants used initially are:

```text
D = [1, 2, 1, 1, 1, 1]
F = [1, 2, 1, 1, 1, 1]
```

The final tuned experiments used an empirical J4 friction compensation:

```text
F = [1, 2, 1, 1.5, 1, 1]
```

This does not mean the official URDF J4 friction is 1.5. It means the closed-loop Gazebo torque-control path behaved as if additional J4 compensation was useful.

## 5.3 Augmented PD control

Augmented PD separates feedforward model torque and direct joint-torque feedback:

```text
tau_ff = M(q) ddq_des + C(q,dq) dq_des + N(q)

tau_fb = Kp e + Kd de

tau = tau_ff + tau_fb
```

This is different from computed torque. In computed torque, feedback acceleration is multiplied by `M(q)`. In augmented PD, feedback is applied directly as joint torque.

This difference was important experimentally. Augmented PD with friction/damping was more robust in Gazebo than computed torque.

## 5.4 Augmented PD with friction/damping compensation

The best pure-torque controller in the experiments was:

```text
tau = tau_ff + tau_fb + tau_damping + tau_friction
```

with:

```text
tau_ff = M(q) ddq_des + C(q,dq) dq_des + N(q)
tau_fb = Kp(q_des - q) + Kd(dq_des - dq)
```

The tuned gain/friction values used in the final tests were approximately:

```text
Kp = [20, 20, 40, 15, 5, 5]
Kd = [3, 3, 6, 2.5, 0.6, 0.4]
D  = [1, 2, 1, 1, 1, 1]
F  = [1, 2, 1, 1.5, 1, 1]
```

## 5.5 Unitree SDK lowcmd baseline

The SDK baseline is not the same as pure torque control. The SDK path sends:

```text
q_cmd, dq_cmd, tau_ff, Kp, Kd
```

and the Unitree/Gazebo controller behaves approximately like:

```text
tau_applied = tau_ff + Kp(q_cmd - q_actual) + Kd(dq_cmd - dq_actual)
```

This feedback is applied closer to the plant and can generate larger internal effective torque than the feedforward command itself. This explains why the SDK baseline can track well even when its explicit feedforward torque `tau_cmd` is not larger than the pure-torque command.

# 6. Friction, damping, deadband, and J4 interpretation

The J4 experiments were important because J4 initially showed an almost constant tracking offset. With `Kp4 = 8`, a 4.5 degree error corresponds to about:

```text
4.5 deg = 0.0785 rad
Kp4 * error = 8 * 0.0785 approximately 0.63 Nm
```

This suggested a missing effective torque of roughly 0.6 Nm. Increasing only J4 friction compensation to 2.0 almost removed the final error but created a large velocity spike. A better compromise was:

```text
Kp4 = 15
Kd4 = 2.5
friction4 = 1.5
```

This reduced J4 error without the large transient velocity caused by friction4 = 2.0.

The best interpretation is:

```text
The official URDF passive friction/damping values are not necessarily wrong. However, the full closed-loop Gazebo control path behaves as if additional J4 compensation is useful.
```

The cause may include:

- simplified joint friction model,
- static friction or breakaway behavior,
- Gazebo controller deadband,
- solver effects,
- ROS/file update delay,
- external-loop lower bandwidth,
- coupling between J2/J3/J4 in multi-joint motion.

# 7. Logging and evaluation metrics

The runtime logs contain columns such as:

```text
t
phase
q_actual_1 ... q_actual_6
dq_actual_1 ... dq_actual_6
q_des_1 ... q_des_6
dq_des_1 ... dq_des_6
ddq_des_1 ... ddq_des_6
tau_1 ... tau_6
tau_ff_1 ... tau_ff_6
tau_fb_1 ... tau_fb_6
tau_total_1 ... tau_total_6
controller_type
```

The `phase` column separates outbound and return motion. Most performance analysis should use the outbound phase, because return-to-start can hide target errors.

The main metrics are:

```text
tracking error: e(t) = q_des(t) - q_actual(t)
max error:      max_t |e(t)|
final error:    e(t_goal)
RMS error:      sqrt(mean(e(t)^2))
achieved %:     actual_motion / commanded_motion * 100%
max torque:     max_t |tau(t)|
```

For forward-pose tests, the report also uses the six-joint error norm:

```text
||e(t)||_2 = sqrt(e1^2 + e2^2 + ... + e6^2)
```

When this value is converted from radians to degrees, it is a whole-arm error norm, not a single-joint error.

# 8. Experimental conclusions supported by the code and data

## 8.1 Single-joint workspace

The expanded single-joint workspace test showed that the tuned augmented PD with friction/damping was the best pure-torque controller over most joints and angles. It was especially strong for J1, J5, and J6. The Unitree SDK baseline was generally excellent, but positive J4 motion still showed limitations and torque saturation in the SDK/Gazebo path.

The main J4 result was:

```text
Original J4 pure-torque behavior: large nearly constant offset.
Tuned J4 pure-torque behavior: close to SDK for negative J4 motion.
```

For J4 -30 deg, tuned augmented PD with friction approached SDK performance, but SDK still had lower maximum error because its lowcmd feedback is applied internally.

## 8.2 Forward-pose coupled motion

The forward-pose test used:

```text
q_target = [0, 1.5, -1.0, -0.54, 0, 0]
```

at 25%, 50%, 75%, and 100% scale.

The best pure-torque controller was again augmented PD with friction/damping. At 100% forward pose, the SDK baseline had lower whole-arm error norm than the best pure-torque controller, but the best pure-torque controller remained close enough to show a successful practical result.

Representative whole-arm norm results at 100% scale were:

```text
SDK:        max/final norm approximately 0.0254 / 0.0049 rad
AugPD fric: max/final norm approximately 0.0503 / 0.0382 rad
```

Converted to degrees:

```text
SDK:        1.46 / 0.28 deg-equivalent
AugPD fric: 2.88 / 2.19 deg-equivalent
```

These are six-joint error norms, not per-joint errors.

## 8.3 Why computed torque did not win

Computed torque is theoretically attractive, but it did not perform best in these Gazebo experiments. The likely reasons are:

1. It depends strongly on accurate `M`, `C`, and `N`.
2. It multiplies feedback acceleration by `M(q)`, so modeling errors can directly affect feedback torque.
3. It is sensitive to delayed or noisy feedback.
4. The current torque path is an external Python/file/ROS loop with limited bandwidth.
5. Gazebo friction/deadband and SDK controller behavior are not perfectly represented by the analytic model.

In contrast, augmented PD with friction/damping uses model feedforward but keeps feedback as direct joint torque. This made it more robust in the Gazebo environment.

# 9. Recommendations for the next development stage

## 9.1 Validate gravity N(q)

Run the three-method validation:

```text
gravity_vector(q)
finite-difference dV/dq
body-Jacobian virtual-work -sum_i J_i^T F_i^b
```

This will provide a strong defense that the gravity term is mathematically consistent.

## 9.2 Add timing instrumentation

To understand remaining errors, log:

```text
actual loop dt
Python controller computation time
file IPC command age
feedback timestamp age
ROS/Gazebo update delay
```

This will quantify the delay and jitter that limit high-gain torque control.

## 9.3 Compare SDK tau_cmd and analytic tau_ff on the same trajectory

For an exact feedforward comparison, compute:

```text
tau_model_ff = M(q_cmd) ddq_cmd + C(q_cmd,dq_cmd) dq_cmd + N(q_cmd)
```

on the SDK trajectory, then compare it with SDK `tau_cmd` and SDK `tau_state`. This separates:

```text
model feedforward difference
from
internal SDK feedback torque
```

## 9.4 Consider an SDK-like hybrid lowcmd controller

A future controller could send:

```text
q_des, dq_des, tau_ff, Kp, Kd
```

through the Unitree/Gazebo lowcmd path. This would test whether the project trajectory and feedforward model can benefit from the same internal feedback layer that makes SDK tracking strong.

# 10. Summary

This project implemented and compared model-based torque controllers for the Unitree Z1 arm in Gazebo. The code uses smooth joint-space trajectories, an analytic dynamics model, gravity compensation, optional friction/damping compensation, and both pure-torque and SDK-style baselines.

The strongest custom controller is augmented PD with friction/damping compensation:

```text
tau = M(q) ddq_des + C(q,dq) dq_des + N(q)
    + Kp(q_des - q) + Kd(dq_des - dq)
    + D dq_des + F direction
```

The project also shows why SDK tracking can be stronger: the SDK lowcmd path applies internal feedback close to the plant, while the custom pure-torque loop applies feedback externally through Python, file IPC, ROS, and Gazebo updates.

The final conclusion is not that computed torque or the gravity model is useless. Rather, the experiments show that in Gazebo, a practical controller must consider not only analytical dynamics, but also friction, deadband, feedback bandwidth, command path, and the distinction between pure external torque and SDK low-level feedback.

# Appendix A. Key command examples

Run the best forward-pose pure-torque controller:

```bash
python3 torque_main.py \
  --mode scaled_pose \
  --target "0 1.5 -1 -0.54 0 0" \
  --scale 1.00 \
  --trajectory-profile scurve \
  --move-time 15 \
  --return-to-start \
  --return-time 15 \
  --duration 32 \
  --test-controller augmented_pd_friction_model \
  --kp "20 20 40 15 5 5" \
  --kd "3 3 6 2.5 0.6 0.4" \
  --model-damping "1 2 1 1 1 1" \
  --model-friction "1 2 1 1.5 1 1" \
  --tau-limit "20 40 25 12 10 10" \
  --dynamics-mode analytic \
  --csv-log logs/forward_100pct_augpd_fric.csv
```

Run computed torque with friction/damping:

```bash
python3 torque_main.py \
  --mode scaled_pose \
  --target "0 1.5 -1 -0.54 0 0" \
  --scale 1.00 \
  --trajectory-profile scurve \
  --move-time 15 \
  --return-to-start \
  --return-time 15 \
  --duration 32 \
  --test-controller gazebo_friction_model \
  --model-damping "1 2 1 1 1 1" \
  --model-friction "1 2 1 1.5 1 1" \
  --tau-limit "20 40 25 12 10 10" \
  --dynamics-mode analytic \
  --csv-log logs/forward_100pct_computed_fric.csv
```

# Appendix B. Recommended gravity validation output

A successful validation should show:

```text
max |N_code - N_mgh|  small
max |N_code - N_pdf|  small
max |N_mgh  - N_pdf|  small
```

If only `max |N_code + N_pdf|` is small, then the implementation and PDF method use opposite sign conventions. This should be described as a convention difference rather than immediately treated as a modeling error.

