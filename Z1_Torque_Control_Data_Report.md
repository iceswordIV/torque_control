# Unitree Z1 Analytic Computed-Torque Control - Data Report

**Status:** preliminary project report with collected Gazebo/ROS data.

## Executive summary

- The analytic dynamics and computed-torque controller were implemented and connected to a ROS/Gazebo pure-torque bridge.

- A major hidden issue was found and fixed: a second publisher (`/z1_controller`, from `sim_ctrl`) was publishing to the same joint command topics and fighting our torque bridge.

- After the bridge fix, joint 3 computed-torque tracking improved substantially and the final report data show useful proof-of-concept behavior on joints 1 and 3, partial behavior on joint 2, and unresolved behavior on wrist joints 4-6.

- The system is sufficient to report the torque-control method and the current milestone, but performance tuning and plant-model matching remain future work.


## Single-joint summary

| test               |   joint |   target_deg |   actual_move_deg |   achieved_pct |   final_error_deg |   max_abs_error_deg |   max_abs_tau_Nm |   rate_Hz |   max_coupled_error_deg |
|:-------------------|--------:|-------------:|------------------:|---------------:|------------------:|--------------------:|-----------------:|----------:|------------------------:|
| report_j1_10deg    |       1 |           10 |             6.886 |         68.861 |             3.114 |               3.792 |            1.081 |    54.072 |                   0.117 |
| report_j1_30deg    |       1 |           30 |            26.913 |         89.712 |             3.087 |               4.713 |            1.181 |    51.103 |                   0.168 |
| report_j1_5deg     |       1 |            5 |             1.896 |         37.924 |             3.104 |               3.452 |            1.046 |    52.427 |                   0.07  |
| report_j2_10deg    |       2 |           10 |             0.743 |          7.426 |             9.257 |               9.503 |            6.517 |    44.956 |                   0.123 |
| report_j2_30deg    |       2 |           30 |            20.674 |         68.914 |             9.326 |              12.242 |            6.598 |    48.173 |                   0.232 |
| report_j2_5deg     |       2 |            5 |             0     |          0.002 |             5     |               5     |            5.649 |    49.606 |                   0.113 |
| report_j3_neg10deg |       3 |          -10 |            -8.001 |         80.006 |            -1.999 |               2.722 |            7.855 |    48.047 |                   0.184 |
| report_j3_neg30deg |       3 |          -30 |           -28.052 |         93.507 |            -1.948 |               3.713 |            8.04  |    40.609 |                   0.311 |
| report_j3_neg5deg  |       3 |           -5 |            -2.98  |         59.59  |            -2.02  |               2.315 |            8     |    40.668 |                   0.145 |
| report_j4_neg10deg |       4 |          -10 |            -0.098 |          0.984 |            -9.902 |               9.931 |            2.221 |    50.851 |                   0.217 |
| report_j4_neg30deg |       4 |          -30 |            -0.153 |          0.51  |           -29.847 |              29.901 |            2.548 |    47.168 |                  33.078 |
| report_j4_neg5deg  |       4 |           -5 |            -0.067 |          1.338 |            -4.933 |               4.959 |            2.006 |    46.385 |                   0.137 |
| report_j5_10deg    |       5 |           10 |             0.097 |          0.974 |             9.903 |               9.938 |            0.489 |    49.879 |                   0.153 |
| report_j5_30deg    |       5 |           30 |             0.184 |          0.615 |            29.815 |              29.886 |            0.903 |    45.971 |                   0.192 |
| report_j5_5deg     |       5 |            5 |             0.064 |          1.283 |             4.936 |               4.958 |            0.388 |    49.298 |                   0.119 |
| report_j6_10deg    |       6 |           10 |            -0.117 |         -1.167 |            10.117 |              10.117 |            0.007 |    51.981 |                   0.186 |
| report_j6_30deg    |       6 |           30 |            -0.183 |         -0.61  |            30.183 |              30.183 |            0.024 |    46.688 |                   0.289 |
| report_j6_5deg     |       6 |            5 |            -0.083 |         -1.651 |             5.083 |               5.083 |            0.003 |    50.27  |                   0.133 |

## Forward-pose summary

| test                  | target_scale   |   rate_Hz |   desired_j1_deg |   desired_j2_deg |   desired_j3_deg |   desired_j4_deg |   desired_j5_deg |   desired_j6_deg |   actual_j1_deg |   actual_j2_deg |   actual_j3_deg |   actual_j4_deg |   actual_j5_deg |   actual_j6_deg |   final_err_j1_deg |   final_err_j2_deg |   final_err_j3_deg |   final_err_j4_deg |   final_err_j5_deg |   final_err_j6_deg |   max_err_j1_deg |   max_err_j2_deg |   max_err_j3_deg |   max_err_j4_deg |   max_err_j5_deg |   max_err_j6_deg |   max_tau_j1_Nm |   max_tau_j2_Nm |   max_tau_j3_Nm |   max_tau_j4_Nm |   max_tau_j5_Nm |   max_tau_j6_Nm |
|:----------------------|:---------------|----------:|-----------------:|-----------------:|-----------------:|-----------------:|-----------------:|-----------------:|----------------:|----------------:|----------------:|----------------:|----------------:|----------------:|-------------------:|-------------------:|-------------------:|-------------------:|-------------------:|-------------------:|-----------------:|-----------------:|-----------------:|-----------------:|-----------------:|-----------------:|----------------:|----------------:|----------------:|----------------:|----------------:|----------------:|
| report_forward_100pct | 100%           |    45.365 |           93.504 |           85.944 |          -51.882 |          -48.748 |          -60.211 |           78.004 |          55.915 |          87.171 |         -57.281 |          -0.116 |           0.02  |          -0.171 |             37.59  |             -1.227 |              5.4   |            -48.633 |            -60.231 |             78.175 |           81.328 |           15.356 |            5.4   |           48.633 |           60.324 |           78.204 |           1.271 |           6.597 |           8.363 |           2.499 |           0.761 |           0.017 |
| report_forward_25pct  | 25%            |    54.049 |           23.598 |           21.486 |          -12.907 |          -12.367 |          -16.191 |           19.433 |          -0.082 |          10.63  |         -12.476 |          -0.16  |           0.214 |          -0.153 |             23.68  |             10.856 |             -0.43  |            -12.207 |            -16.406 |             19.586 |           23.68  |           13.659 |            2.212 |           12.257 |           16.406 |           19.586 |           0.513 |           6.591 |           7.999 |           2.262 |           0.316 |           0.003 |
| report_forward_50pct  | 50%            |    44.504 |           47.388 |           42.972 |          -25.864 |          -24.593 |          -31.949 |           38.967 |          -0.059 |          35.328 |         -26.395 |          -0.188 |           0.232 |          -0.218 |             47.447 |              7.644 |              0.531 |            -24.405 |            -32.181 |             39.184 |           47.449 |           14.204 |            2.249 |           24.448 |           32.181 |           39.184 |           0.722 |           6.629 |           8.132 |           2.347 |           0.31  |           0.009 |

## Hold summary

| test                   |   rate_Hz |   max_drift_deg |   final_drift_j1_deg |   final_drift_j3_deg |   final_drift_j4_deg |   max_abs_tau_Nm |
|:-----------------------|----------:|----------------:|---------------------:|---------------------:|---------------------:|-----------------:|
| gazebo_gravity_hold    |    50.336 |           0.25  |               -0.098 |                0.003 |               -0.007 |            7.721 |
| test_gravity_only_hold |   161.113 |           0.261 |               -0.115 |               -0.101 |                0.261 |            7.662 |
| test_pd_only_hold      |   162.475 |           0.092 |               -0.092 |                0.019 |               -0.049 |            0.062 |
| test_pd_gravity_hold   |   163.838 |           0.219 |               -0.11  |                0.083 |               -0.216 |            8.933 |

## Diagnosis from the report logs

The current data point to a plant/model problem in Gazebo, not a remaining bridge connection problem.

The command path is now a real torque path when only `ros_torque_bridge.py` is publishing. The Gazebo `UnitreeJointController` computes:

```text
calcTorque = Kp * (q_cmd - q_actual) + Kd * (dq_cmd - dq_actual) + tau_cmd
```

The bridge sends measured `q_actual` and `dq_actual` while setting `Kp = 0` and `Kd = 0`, so the Gazebo controller reduces to:

```text
calcTorque = tau_cmd
```

The old hidden fighting source was the separate `/z1_controller` node from `sim_ctrl`. It must not be running during these tests. Before every report run, verify:

```bash
rostopic info /z1_gazebo/Joint03_controller/command
```

The only publisher should be:

```text
/z1_ros_torque_bridge
```

### Main evidence

1. Joints 1-3 respond to torque, so the bridge path is working.
   - Joint 1 reaches 89.7% of a 30 deg target.
   - Joint 3 reaches 93.5% of a -30 deg target.
   - Joint 3 max error is only 3.713 deg after the bridge fix.

2. Small motions show a deadband-like final error.
   - Joint 1 final error is about 3.1 deg for 5, 10, and 30 deg commands.
   - Joint 3 final error is about 2.0 deg for -5, -10, and -30 deg commands.
   - Joint 2 barely moves for 5 and 10 deg, then moves for 30 deg but remains about 9.3 deg short.

3. Wrist joints 4-6 are effectively not torque-controlled by the current computed-torque command.
   - Joint 4 target -30 deg: actual joint 4 moves only -0.153 deg.
   - Joint 5 target 30 deg: actual joint 5 moves only 0.184 deg.
   - Joint 6 target 30 deg: actual joint 6 moves -0.183 deg, opposite the desired direction.

4. The joint 4 test exposes a serious coupling/model mismatch.
   - In `report_j4_neg30deg`, desired joint 4 motion is -30 deg.
   - Actual joint 4 moves only -0.153 deg.
   - Actual joint 3 moves -33.078 deg even though joint 3 was not the target.
   - The computed-torque model commands substantial upstream torque on joint 3 while joint 4 itself does not follow.

5. Forward-pose tests confirm the same pattern.
   - Joint 2 and joint 3 can move substantially.
   - Joints 4, 5, and 6 remain near their starting positions even for 100% target pose.
   - Joint 1 moves in the 100% target, but not in the 25% and 50% targets, which is consistent with a breakaway/deadband threshold.

### Likely root cause

The strongest explanation is that the analytic computed-torque model does not include the Gazebo plant's joint friction and damping, and the distal wrist joints need explicit friction/deadband compensation or different gains.

The Gazebo Z1 URDF defines nonzero joint damping/friction:

```text
jointDamping = 1.0
jointFriction = 1.0
joint2 uses 2x jointDamping and 2x jointFriction
```

This is large compared with the commanded torque on the distal joints:

```text
joint 4 max commanded torque: about 2.0-2.55 Nm
joint 5 max commanded torque: about 0.39-0.90 Nm
joint 6 max commanded torque: about 0.003-0.024 Nm
```

So joints 5 and 6 are commanded below the Gazebo friction level in almost all tests. Joint 4 receives more torque, but its failed movement plus the large unintended joint 3 motion suggest distal-chain model mismatch and/or incorrect coupling compensation in addition to friction.

The low Python rate remains a limitation, but it is not the main explanation for the report data. Joint 3 tracks reasonably well at 40-48 Hz after the hidden publisher was removed. The larger failure is joint-dependent: joints 4-6 do not respond even when the trajectory is slow.

### Current conclusion

The project has demonstrated computed-torque control through the ROS/Gazebo torque path for the main arm joints, especially joint 3. The unresolved problem is not "no torque command"; it is that the Gazebo plant seen by torque control is not the same plant modeled by `z1_analytic_dynamics.py`.

For a thesis/report wording:

```text
The torque-control bridge was validated after removing a competing command publisher. Single-joint tests show that the torque path can drive the arm, especially joints 1-3. Remaining tracking errors are dominated by Gazebo plant effects not represented in the analytic controller, including joint damping/friction and distal-joint model mismatch. The wrist joints 4-6 require additional actuator identification, friction compensation, or a revised plant model before full-pose computed-torque tracking can be considered solved.
```

### Recommended next tests

1. Run raw torque step tests on each joint, especially joints 4-6, with all other joints held at current state. The goal is to estimate breakaway torque.

2. Add an optional friction compensation term for Gazebo tests:

```text
tau = tau_computed + tau_static * sign(q_des - q_actual) + b * dq_des
```

Start with small values and tune per joint.

3. Test PD+gravity on joints 4-6 with higher wrist gains and compare against computed torque. If PD+gravity moves the wrist but computed torque does not, the analytic coupling/feedforward model is the issue.

4. Compare analytic `N(q)` and commanded `tau` against the Gazebo controller state topic:

```bash
rostopic echo /z1_gazebo/Joint04_controller/state
```

Use `tauEst` to see whether Gazebo is applying the requested torque or clipping it.

5. Keep verifying that `/z1_controller` is not running. If it appears as a publisher on any `/z1_gazebo/JointXX_controller/command` topic, the test is invalid.
