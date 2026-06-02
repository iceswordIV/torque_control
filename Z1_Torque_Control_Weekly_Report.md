# Weekly Report: Unitree Z1 Pure Torque Control, Friction Diagnosis, and S-Curve Trajectory Tests

**Project:** Unitree Z1 analytic dynamics and pure torque control  
**Data source:** `z1_project/logs/week_scurve/` in the uploaded project ZIP  
**Code source:** `z1_project/controller.py`, `trajectory.py`, `test_controller.py`, and `torque_main.py`  
**Report focus:** this week's progress on friction/dead-zone diagnosis, augmented PD comparison, and high-acceleration S-curve trajectory tests.

## 1. Executive summary

This week the project moved from a simple question - *why does torque get sent but some joints do not move?* - to a clearer diagnosis. The computed-torque equation and sign convention are correct, but the practical Gazebo plant contains friction, damping, possible dead-zone behavior, and a low effective control update rate. The original frictionless computed-torque controller often produces too little additional motion torque for small slow commands, especially on wrist joints.

The main progress was:

1. **Added and tested augmented PD** as a diagnostic pure-torque controller. It sends direct feedback torque, rather than inertia-scaled feedback torque. It moved several joints much better than the original computed-torque controller, but aggressive gains produced instability or excessive derivative torque on some tests, especially J4 negative motion.
2. **Read the Gazebo model damping/friction values** and implemented a Gazebo friction/damping compensation test controller. The simple model alone did not fully solve J4, which suggests the effective plant behavior is more complex than the URDF friction constants alone.
3. **Added a smooth 7th-order S-curve trajectory** that returns analytic `q_des`, `dq_des`, and `ddq_des`. This is important because the torque model depends directly on desired velocity and acceleration.
4. **Tested high-acceleration S-curve trajectories** for J4. Increasing desired acceleration from T=8 s to T=2 s increased `ddq_des` by 16 times, but the original computed-torque controller still did not move J4. This suggests that in the current model and gain setting, trajectory acceleration alone is not enough for the J4 friction/dead-zone issue.

The best next algorithmic direction is not a fixed friction table. A stronger candidate is **model feedforward + direct PD + bounded lag/integral disturbance torque**, because this can adapt to unknown friction, payload, and model error while remaining pure torque control.

## 2. Robot and simulation parameters used in this report

### 2.1 Joint-space workspace from SDK/project notes

| Joint   |   Min rad |   Max rad |   Min deg |   Max deg |
|:--------|----------:|----------:|----------:|----------:|
| J1      |  -2.61799 |   2.61799 |   -150    |       150 |
| J2      |   0       |   2.87979 |      0    |       165 |
| J3      |  -2.88559 |   0       |   -165.33 |         0 |
| J4      |  -1.51844 |   1.51844 |    -87    |        87 |
| J5      |  -1.3439  |   1.3439  |    -77    |        77 |
| J6      |  -2.79253 |   2.79253 |   -160    |       160 |

### 2.2 Speed and torque limits used for context

| Parameter | Value used in report | Meaning |
|---|---:|---|
| Max joint speed | 180 deg/s = pi rad/s | Physical joint speed limit for J1-J6 from Z1 notes/SDK check |
| Max joint torque | 33 N m | Published Z1 joint torque value used as a hardware context limit |
| Runtime target `dt` | 0.002 s | Intended 500 Hz controller period |
| Observed runtime rate | about 33-52 Hz | Actual Python/Gazebo loop rate in this week's logs |

### 2.3 Gazebo joint damping/friction from `/robot_description`

| Joint   |   damping |   friction |
|:--------|----------:|-----------:|
| J1      |         1 |          1 |
| J2      |         2 |          2 |
| J3      |         1 |          1 |
| J4      |         1 |          1 |
| J5      |         1 |          1 |
| J6      |         1 |          1 |
| gripper |         1 |          1 |

The Gazebo values are important because the original computed-torque controller does not include these terms. They are simulation parameters, not guaranteed real-robot friction values.

### 2.4 Default computed-torque controller gains

The code uses `wn` as a controller tuning value, not a robot hardware frequency limit. In `controller.py`, the default values are converted as `Kp = wn^2` and `Kd = 2*zeta*wn`.

| Joint   |   wn rad/s |   zeta |   Kp = wn^2 |   Kd = 2*zeta*wn |
|:--------|-----------:|-------:|------------:|-----------------:|
| J1      |        2   |      1 |        4    |              4   |
| J2      |        2   |      1 |        4    |              4   |
| J3      |        1.5 |      1 |        2.25 |              3   |
| J4      |        1.5 |      1 |        2.25 |              3   |
| J5      |        1.2 |      1 |        1.44 |              2.4 |
| J6      |        1.2 |      1 |        1.44 |              2.4 |

## 3. Control formulas and corresponding code

### 3.1 Plant model used by analytic computed torque

The ideal model used by the controller is:

$$
M(q)\ddot q + C(q,\dot q)\dot q + N(q) = \tau
$$

The real/Gazebo plant is closer to:

$$
M(q)\ddot q + C(q,\dot q)\dot q + N(q) + D\dot q + F + d_{load} = \tau
$$

where `D dq`, friction, payload, and model errors are disturbances from the controller's point of view.

### 3.2 Computed torque controller

The project code uses the error convention:

$$
e = q_d - q, \quad \dot e = \dot q_d - \dot q
$$

`controller.py` computes:

$$
\ddot q_{cmd} = \ddot q_d + K_d\dot e + K_p e
$$

$$
\tau = M(q)\ddot q_{cmd} + C(q,\dot q)\dot q + N(q)
$$

Equivalent code:

```python
e = q_des - q
de = dq_des - dq
ddq_cmd = ddq_des + Kd @ de + Kp @ e
M, C, N, _ = dynamics_analytic(q, dq)
tau = M @ ddq_cmd + C @ dq + N
```

This is algebraically consistent with the textbook form that defines error as `q - q_des` and uses a negative feedback sign.

### 3.3 Augmented PD diagnostic controller

The diagnostic augmented-PD controller separates model feedforward and feedback torque:

$$
\tau_{ff} = M(q)\ddot q_d + C(q,\dot q)\dot q_d + N(q)
$$

$$
\tau_{fb} = K_p(q_d-q) + K_d(\dot q_d-\dot q)
$$

$$
\tau = \tau_{ff} + \tau_{fb}
$$

The important difference is that the PD feedback is **direct joint torque**. It is not multiplied by `M(q)`. This is why augmented PD can overcome friction/dead-zone better, but it can also become unstable if the direct gains are too aggressive.

The best practical gain sets found during the tuning process were:

| Gain set | Kp | Kd | Observation |
|---|---|---|---|
| KP2/KD2 | `[30,45,60,35,15,15]` | `[8,12,18,18,3,3]` | safe improvement; J5/J6 began moving but still weak |
| KP3/KD3 | `[45,70,75,55,45,45]` | `[12,18,22,28,8,8]` | stronger; many joints improved, but J4 negative became unstable |
| adjusted week set | `[55,90,75,30,55,55]` | `[15,24,24,18,10,10]` | used for S-curve augmented-PD report tests; strong but still shows instability in some directions |

### 3.4 Gazebo friction/damping model controller

A simulation-only test controller was added:

$$
\tau = M(q)\ddot q_{cmd} + C(q,\dot q)\dot q + N(q) + D\dot q_d + F\,\text{direction}
$$

where `D` and `F` are the Gazebo URDF/Xacro values. The direction was based on desired velocity, or position error near zero velocity. This is not a final real-robot solution; it was used only to check whether the Gazebo friction model explains the start-motion failure.

### 3.5 S-curve trajectory profile

The new S-curve trajectory is a 7th-order polynomial. It returns analytic position, velocity, and acceleration; this is required because our dynamics and torque calculation use `q`, `dq`, and `ddq` directly.

Let:

$$
s = t/T
$$

$$
b(s) = 35s^4 - 84s^5 + 70s^6 - 20s^7
$$

$$
\dot b(t) = \frac{140s^3 - 420s^4 + 420s^5 - 140s^6}{T}
$$

$$
\ddot b(t) = \frac{420s^2 - 1680s^3 + 2100s^4 - 840s^5}{T^2}
$$

Then:

$$
q_d = q_0 + b(s)(q_g-q_0)
$$

$$
\dot q_d = \dot b(t)(q_g-q_0), \quad \ddot q_d = \ddot b(t)(q_g-q_0)
$$

Corresponding code in `trajectory.py`:

```python
def scurve_trajectory(q_start, q_goal, t, T):
    s = t / T
    b   = 35*s**4 - 84*s**5 + 70*s**6 - 20*s**7
    bd  = (140*s**3 - 420*s**4 + 420*s**5 - 140*s**6) / T
    bdd = (420*s**2 - 1680*s**3 + 2100*s**4 - 840*s**5) / (T*T)
    delta = q_goal - q_start
    q_des   = q_start + b * delta
    dq_des  = bd * delta
    ddq_des = bdd * delta
    return q_des, dq_des, ddq_des
```

## 4. Experiment results from this week

### 4.1 Single-joint S-curve tests: computed torque

These tests used the original computed-torque controller with S-curve trajectory, 5 deg commands, `move_time=8 s`, `hold_time=2 s`, and default computed-torque gains. From the zero/near-zero pose, `J2 -5 deg` and `J3 +5 deg` were not tested because they violate the known joint limits.

| test      |   actual motion deg |   final error deg |   max tau Nm |   max ddq_des deg/s2 |   rate Hz |
|:----------|--------------------:|------------------:|---------------:|-----------------------:|----------:|
| J1 -5 deg |              -0.026 |            -4.974 |          0.039 |                  0.587 |    37.777 |
| J1 +5 deg |              -0.015 |             5.015 |          0.04  |                  0.587 |    40.419 |
| J2 +5 deg |              -0     |             5     |          3.357 |                  0.587 |    32.709 |
| J3 -5 deg |              -0.033 |            -4.967 |          7.93  |                  0.587 |    39.784 |
| J4 -5 deg |               0.002 |            -5.002 |          2.662 |                  0.587 |    40.032 |
| J4 +5 deg |               0.002 |             4.998 |          2.653 |                  0.587 |    40.693 |
| J5 -5 deg |              -0.011 |            -4.989 |          0.004 |                  0.587 |    46.678 |
| J5 +5 deg |              -0.011 |             5.011 |          0.001 |                  0.587 |    44.07  |
| J6 -5 deg |              -0     |            -5     |          0.01  |                  0.587 |    36.751 |
| J6 +5 deg |              -0     |             5     |          0.01  |                  0.587 |    40.518 |

**Observation.** The computed-torque controller produced almost no motion in the S-curve small-motion tests. This is consistent with the diagnosis that the combination of low effective feedback torque, friction/dead-zone, and time-based reference progression prevents breakaway.

### 4.2 Single-joint S-curve tests: augmented PD

These tests used:

```bash
Kp = "55 90 75 30 55 55"
Kd = "15 24 24 18 10 10"
```

| test      |   actual motion deg |   final error deg |   max tau Nm |   max ddq_des deg/s2 |   rate Hz |
|:----------|--------------------:|------------------:|---------------:|-----------------------:|----------:|
| J1 -5 deg |              -0.391 |            -4.609 |         55.66  |                  0.587 |    35.17  |
| J1 +5 deg |               3.984 |             1.016 |          2.561 |                  0.587 |    39.344 |
| J2 +5 deg |               4.744 |             0.256 |         95.97  |                  0.587 |    41.756 |
| J3 -5 deg |              -4.192 |            -0.808 |          9.907 |                  0.587 |    41.678 |
| J4 -5 deg |               5.819 |           -10.818 |        170.105 |                  0.587 |    37.038 |
| J4 +5 deg |               1.628 |             3.372 |          2.645 |                  0.587 |    43.182 |
| J5 -5 deg |              -3.979 |            -1.021 |          1.452 |                  0.587 |    39.301 |
| J5 +5 deg |               3.972 |             1.028 |          1.366 |                  0.587 |    40.521 |
| J6 -5 deg |              -3.959 |            -1.041 |          1.212 |                  0.587 |    40.495 |
| J6 +5 deg |               3.966 |             1.034 |          1.2   |                  0.587 |    44.267 |

**Observation.** Augmented PD moved J2, J3, J5, and J6 much better than computed torque. J1 positive also tracked reasonably. However, J1 negative and J4 negative were unstable or wrong-direction/overshoot cases, with very high commanded torque. This confirms that direct PD torque can overcome dead-zone but also creates a damping and safety problem if gains are not limited.

![Single joint comparison](single_joint_actual_motion.png)

### 4.3 High-acceleration S-curve test: J4 computed torque

This directly tested the professor's idea: increasing trajectory acceleration should increase the feedforward torque term `M(q)ddq_des` and may help overcome static friction.

|   move time s |   desired deg |   actual motion deg |   final error deg |   max tau4 Nm |   max ddq_des4 deg/s2 |   rate Hz |
|--------------:|--------------:|--------------------:|------------------:|----------------:|------------------------:|----------:|
|             8 |            -5 |              -0.01  |            -4.99  |           2.628 |                   0.587 |    36.561 |
|             4 |            -5 |              -0.007 |            -4.993 |           2.63  |                   2.348 |    35.414 |
|             2 |            -5 |              -0.006 |            -4.993 |           2.638 |                   9.391 |    33.087 |

![High acceleration J4 result](high_accel_j4_result.png)

**Observation.** Reducing move time from 8 s to 2 s increased the maximum desired acceleration from about 0.587 deg/s^2 to about 9.391 deg/s^2, a 16x increase. However, J4 still did not move under the original computed-torque controller. The max J4 torque changed only slightly, from about 2.628 N m to 2.638 N m, suggesting that for this joint and pose the desired-acceleration torque contribution is too small relative to the holding/model torque and friction/dead-zone effect.

This does not invalidate the high-acceleration strategy. It means that **with the current computed-torque gains and analytic model**, S-curve acceleration alone is not sufficient for J4. A trapezoidal or stronger acceleration trajectory may still be worth testing, but it should be combined with torque limits and better feedback.

### 4.4 Full target pose test

Target pose:

```text
[0, 1.5, -1, -0.54, 0, 0]
```

This is a large multi-joint movement, so coupling and gain sensitivity become important.

| ctrl   | joint   |   des deg |   act deg |   err deg |   max tau |
|:-------|:--------|----------:|----------:|----------:|----------:|
| CT     | J1      |    -9.026 |     0.04  |    -9.065 |     0.06  |
| CT     | J2      |    85.944 |    -0     |    85.944 |     4.138 |
| CT     | J3      |   -53.36  |    -0.076 |   -53.285 |     8.462 |
| CT     | J4      |   -37.707 |    -0.013 |   -37.693 |     2.848 |
| CT     | J5      |    13.016 |    -0.041 |    13.057 |     0.015 |
| CT     | J6      |    -5.756 |     0.064 |    -5.819 |     0.012 |
| APD    | J1      |    -9.596 |    -7.886 |    -1.71  |    31.219 |
| APD    | J2      |    85.944 |    87.84  |    -1.896 |    31.028 |
| APD    | J3      |   -53.619 |   -55.263 |     1.644 |    80.115 |
| APD    | J4      |   -37.059 |    40.941 |   -78     |   129.926 |
| APD    | J5      |    13.669 |     7.199 |     6.47  |    24.323 |
| APD    | J6      |    -7.112 |   -11.149 |     4.037 |    42.634 |

![Full pose computed torque](full_pose_computed_torque.png)

![Full pose augmented PD](full_pose_augmented_pd.png)

**Observation.** Computed torque essentially did not move toward the full target pose. Augmented PD moved the main joints much more: J2 and J3 were close to their desired large motions, while J1, J5, and J6 partially tracked. However, J4 moved in the wrong direction by a large amount, again showing that the J4 control/plant behavior is the main instability risk.

## 5. Main technical interpretation

### 5.1 Torque sign is not the problem

The code uses `q_des - q` as the error. The textbook screenshot uses `q - q_des`, so the sign of the feedback term appears different. Algebraically they are equivalent. The current computed-torque sign convention is therefore correct.

### 5.2 The problem is a combination of friction, direct torque magnitude, and trajectory timing

The Gazebo model contains damping and friction. For slow small trajectories, the desired acceleration is small, and the computed-torque feedback is inertia-scaled. If the sensor shows no motion, the time-based reference still continues. The reference may enter its deceleration phase even though the real joint has not started moving. In that case the trajectory strategy can partially cancel the torque needed for breakaway.

This means the issue is not only a controller equation problem. It is also a **trajectory strategy problem**.

### 5.3 Augmented PD is real progress, but not final

Augmented PD is still pure torque control because the command sent to the robot is torque. It proved that direct feedback torque can push through part of the dead-zone. However, increasing Kp/Kd too much leads to high torque and instability. The J4 negative direction is the clearest example.

### 5.4 Fixed friction compensation is not a robust final answer

The Gazebo friction constants are useful for diagnosis, but fixed friction values should not be the final real-robot strategy. A real robot may carry unknown load, and friction depends on direction, temperature, speed, and configuration. The next robust algorithm should estimate unknown disturbance torque online rather than hard-code friction values.

## 6. Recommended next algorithm

The recommended next controller is:

$$
\tau = M(q)\ddot q_d + C(q,\dot q)\dot q_d + N(q) + K_p e + K_d\dot e + \tau_{bias}
$$

where:

$$
\tau_{bias}[k+1] = \lambda \tau_{bias}[k] + K_i e[k]\Delta t
$$

and:

$$
\tau_{bias} = \text{clip}(\tau_{bias}, -\tau_{limit}, +\tau_{limit})
$$

This is a bounded leaky integral or lag disturbance torque term. It remains pure torque control, but it can adapt to unknown friction, payload, and model error. It should also include:

- per-joint feedback torque limits;
- total torque limits;
- velocity filtering for the derivative term;
- joint-limit protection;
- data logging for `tau_ff`, `tau_fb`, `tau_bias`, and `tau_total`.

## 7. Recommended next experiments

1. Discuss the trajectory strategy issue with the professor: a time-based reference can continue into deceleration while the joint is stuck.
2. Add and test a trapezoidal velocity profile or stronger S-curve parameterization to create a larger early acceleration pulse.
3. Add bounded leaky-integral torque bias as a diagnostic controller.
4. Continue single-joint testing before full-pose testing, especially for J4.
5. Use summary metrics: final error, max torque, max velocity, first motion time, overshoot, and mechanical work `W = integral tau dq`.

## 8. Week summary statement for report/presentation

This week, we verified that the computed-torque sign is correct and that the failure is not a simple algebraic error. The main limitation is the mismatch between the ideal frictionless model and the Gazebo plant with friction/dead-zone, together with a time-based trajectory that continues even when actual motion is stuck. Augmented PD showed clear progress because direct torque feedback can overcome some breakaway behavior, but it also introduced damping and stability problems under aggressive gains. The new S-curve trajectory implementation provides analytic `q`, `dq`, and `ddq` for higher-acceleration tests. The next step is to combine model feedforward, direct PD, bounded disturbance adaptation, and improved trajectory strategy.

## Appendix A. Important file locations

| File | Role |
|---|---|
| `z1_project/controller.py` | Original computed-torque controller and default `wn/zeta` gains |
| `z1_project/test_controller.py` | Diagnostic controllers: `pd_only`, `pd_gravity`, `augmented_pd`, `gazebo_friction_model` |
| `z1_project/trajectory.py` | Quintic and new 7th-order S-curve trajectory generation |
| `z1_project/torque_main.py` | Runtime loop, CLI options, CSV logging, trajectory selection |
| `z1_project/logs/week_scurve/` | This week's test data |
| `z1_project/analyze_gain_logs.py` | Log-analysis utility added during gain tuning |

## Appendix B. Generated summary CSVs

The generated artifact `week_scurve_all_summary.csv` contains one row per weekly log file with controller type, target joint, desired motion, actual motion, final error, maximum torque, maximum desired acceleration, and observed loop rate.
