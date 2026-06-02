# Unitree Z1 Torque-Control Evaluation Report

**Project:** Unitree Z1 torque-control evaluation  
**Scope of this version:** single-joint workspace tests plus forward-pose coupled motion evaluation, with torque-command comparison and complete appendix tables.  
**Data sources:** `eval_joint_workspace.zip`, `eval_joint_workspace_sdk.zip`, `joint_workspace_sdk_compare.csv`, `eval_forward_pose_4controllers.zip`, `eval_forward_pose_sdk.zip`, and `forward_pose_sdk_compare.csv`.  
**Total comparison rows:** 150 single-joint rows = 120 pure-torque runs plus 30 SDK lowcmd reference runs; 20 forward-pose rows = 16 pure-torque runs plus 4 SDK lowcmd reference runs.  

## Executive summary

This experiment evaluates four custom torque-control variants and compares their best behavior with the Unitree SDK lowcmd baseline over a single-joint workspace test set. The test set covers 30 commanded relative motions across J1-J6, including both signs for J1, J4, J5, and J6, and safety-direction tests for J2 and J3.

The strongest result is that **augmented PD with friction/damping compensation is the best pure-torque controller in all 30 single-joint tests**. It is the best pure-torque label by maximum tracking error for every joint-angle command. Across all tests, it has a mean maximum error of **1.30 deg** and mean RMS error of **0.86 deg**. The computed-torque controllers do not win any single-joint test in this dataset.

The SDK lowcmd baseline remains an important reference. It has a slightly lower overall mean max error than the best pure-torque controller, **1.11 deg vs 1.30 deg**, and it tracks J2, J3, and positive J4 better. However, the tuned pure-torque controller outperforms SDK lowcmd in **19/30** individual tests by maximum error, especially J5 and J6 and much of J1. This means the pure-torque controller is not merely functional; after tuning, it is competitive with the SDK baseline in several single-joint motions.

The main remaining limitation is **J4 positive direction**. Both SDK and pure torque struggle there, but pure torque is worse. For J4 +30 deg, the best pure-torque result reaches only **50.0%** of the target with **16.16 deg** max error, while SDK reaches **74.2%** with **7.74 deg** max error. This supports the earlier observation that J4 has strong direction-dependent and coupling-sensitive behavior.

The coupled forward-pose evaluation was then added at 25%, 50%, 75%, and 100% of the target pose. The tuned augmented-PD/friction controller remains the best pure-torque option in the coupled tests. At 100% scale, SDK lowcmd still has lower tracking error, but tuned pure torque successfully completes the forward pose with a much smaller error than the computed-torque variants. This confirms that the J4 tuning improvement survives coupled motion reasonably well, although SDK still has the advantage of internal low-level feedback bandwidth.


## Controllers tested

The pure-torque tests used four controller variants:

1. **Augmented PD, no friction/damping:** joint-space PD plus model-based gravity/support terms, without empirical friction compensation.
2. **Augmented PD with friction/damping:** augmented PD with model damping and friction compensation. In this final single-joint workspace run, J4 used empirical friction compensation of 1.5. This is not the official URDF friction value; it is a tuned compensation value for the observed Gazebo/control behavior.
3. **Computed torque, no friction/damping:** model-based torque using the full dynamics structure, without empirical friction/damping compensation.
4. **Computed torque with friction/damping:** computed torque plus the same damping/friction model.

The SDK baseline used the Unitree SDK lowcmd path. This is not exactly the same as pure external torque control. SDK lowcmd sends reference joint position, reference velocity, feedforward torque, and low-level gains into the Unitree/Gazebo controller path. Therefore, SDK can generate larger reported/applied joint torques than the feedforward torque alone.

## Test design

The final single-joint workspace test includes these relative commands:

- J1: -30, -10, -5, +5, +10, +30 deg
- J2: +5, +10, +30 deg
- J3: -5, -10, -30 deg
- J4: -5, -10, -30, +5, +10, +30 deg
- J5: -5, -10, -30, +5, +10, +30 deg
- J6: -5, -10, -30, +5, +10, +30 deg

Each test used a smooth S-curve style motion, moved outbound to the target, and then returned to the start. The main metrics are computed on the outbound phase so the return-to-start phase does not hide target tracking error.

Metrics used:

- **Achieved percent:** actual outbound joint motion divided by desired outbound motion.
- **Final error:** desired minus actual joint position at the outbound endpoint.
- **Max absolute error:** worst absolute tracking error during the outbound phase.
- **RMS error:** root-mean-square tracking error during the outbound phase.
- **Max torque:** maximum absolute commanded or reported torque, depending on controller path.

## Overall controller comparison

| Controller                        |   Runs |   Mean max |   Mean RMS |   Mean final |   Achieved % |   Worst max |   Rate Hz |
|:----------------------------------|-------:|-----------:|-----------:|-------------:|-------------:|------------:|----------:|
| Augmented PD, friction/damping    |     30 |       1.3  |       0.86 |         1.03 |         95.6 |       16.16 |     377.2 |
| Augmented PD, no friction         |     30 |       6.66 |       4.65 |         6.21 |         42   |       17.18 |     381.8 |
| Computed torque, friction/damping |     30 |       5.61 |       3.93 |         5.36 |         87.7 |       29.46 |     383.3 |
| Computed torque, no friction      |     30 |      14.15 |       9.08 |        14.02 |          3.5 |       30.13 |     383.9 |
| Unitree SDK lowcmd                |     30 |       1.11 |       0.67 |         0.47 |         96.7 |        7.74 |     491.2 |

![Controller-level tracking error](assets/fig_controller_error_summary.png)

### Interpretation

Augmented PD with friction/damping is the clear best pure-torque controller. It reduces mean max error from **6.66 deg** in augmented PD without friction to **1.30 deg**. It also reduces mean RMS error from **4.65 deg** to **0.86 deg**. This proves that friction/damping compensation is essential for the current Gazebo torque-control setup.

Computed torque without friction/damping has the worst overall performance, with mean max error above **14 deg** and very low mean achieved motion. Computed torque with friction/damping improves substantially, but its mean max error is still much higher than the tuned augmented PD controller. This suggests that, under the current Python + file IPC + ROS + Gazebo loop, the simpler augmented PD controller is more robust than the full computed-torque path.

## SDK baseline comparison

The Unitree SDK lowcmd baseline provides the reference behavior of the manufacturer-supported control path. Its mean max error is **1.11 deg**, slightly better than the best pure-torque controller's **1.30 deg**. However, this average hides strong joint dependence.

|   Joint |   Tests |   Pure mean max err |   SDK mean max err |   Pure mean RMS |   SDK mean RMS |   Pure achieved |   SDK achieved |   Pure wins (max err) |
|--------:|--------:|--------------------:|-------------------:|----------------:|---------------:|----------------:|---------------:|----------------------:|
|       1 |       6 |                0.26 |               0.3  |            0.11 |           0.15 |           100   |          100   |                     4 |
|       2 |       3 |                1.7  |               0.64 |            1.52 |           0.37 |           117.4 |          100.2 |                     0 |
|       3 |       3 |                1.01 |               0.78 |            0.78 |           0.44 |            91.2 |           99.6 |                     1 |
|       4 |       6 |                4.2  |               2.38 |            2.68 |           1.46 |            74.4 |           87.7 |                     2 |
|       5 |       6 |                0.38 |               1.06 |            0.22 |           0.65 |            99.7 |           98.4 |                     6 |
|       6 |       6 |                0.33 |               1.12 |            0.15 |           0.7  |            99.5 |           97.5 |                     6 |

![Best pure torque vs SDK by joint](assets/fig_per_joint_max_error.png)

### Key SDK-vs-pure findings

- **J1:** both controllers work very well. Pure torque is better in the positive direction and small negative direction; SDK is better for larger negative motions.
- **J2:** SDK is clearly better. The pure-torque controller overshoots J2, especially at +5 and +10 deg.
- **J3:** SDK is generally better except for the -30 deg case, where pure torque has slightly lower max error.
- **J4:** negative J4 is now competitive after tuning, but positive J4 remains difficult. SDK is better for positive J4, but SDK also does not perfectly solve it.
- **J5 and J6:** tuned pure torque is consistently better than SDK in this dataset.

![Per-test max tracking error heatmap](assets/fig_test_max_error_heatmap.png)

## J4 focused result

J4 was the original problem joint, so it deserves a separate interpretation. Earlier tests showed a nearly constant error offset for J4, indicating missing effective compensation. After J4-specific tuning, the negative J4 behavior improved dramatically. The final tuned workspace dataset shows that J4 -30 deg is now close to SDK performance, but positive J4 remains weak.

| test       |   pure_max_err |   sdk_max_err |   pure_ach |   sdk_ach |   pure_tau |   sdk_tau_cmd |   sdk_tau_state |
|:-----------|---------------:|--------------:|-----------:|----------:|-----------:|--------------:|----------------:|
| J4 -30 deg |           0.75 |          1.76 |       99.7 |      99.8 |       3.79 |          2.04 |            4.83 |
| J4 -10 deg |           0.44 |          0.58 |       96.9 |      99.4 |       3.73 |          2.04 |            4.46 |
| J4 -5 deg  |           0.41 |          0.3  |       93.4 |      98.9 |       3.72 |          2.04 |            4.36 |
| J4 +5 deg  |           2.53 |          1.04 |       49.5 |      81   |       2.09 |          2.04 |            7.07 |
| J4 +10 deg |           4.92 |          2.85 |       56.7 |      72.8 |       2.09 |          2.04 |           22.64 |
| J4 +30 deg |          16.16 |          7.74 |       50   |      74.2 |       3.77 |          2.04 |           30    |

![J4 direction-dependent max error](assets/fig_j4_directional_error.png)

![J4 achieved motion](assets/fig_j4_achieved.png)

### J4 interpretation

The final data shows two different J4 behaviors:

1. **Negative J4:** Pure torque is now competitive. For J4 -30 deg, the tuned pure-torque controller has **0.75 deg** max error and **99.7%** achieved motion, while SDK has **1.76 deg** max error and **99.8%** achieved motion. This is an important achievement because the original J4 -30 pure-torque test had about **4-5 deg** error.
2. **Positive J4:** Both controllers degrade, but pure torque degrades more. The best pure-torque J4 +30 run achieves only **50.0%**, while SDK achieves **74.2%**. This indicates direction-dependent plant/controller behavior and likely J3-J4 coupling, saturation, internal controller effects, or gravity/friction asymmetry.

This means J4 is not simply a workspace-limit problem. The SDK baseline proves that the plant can move J4 through the negative side cleanly. The tuned pure-torque controller also proves that external torque control can approach the SDK result on the negative side. The remaining positive-side issue should be treated as a coupled whole-system behavior rather than a single scalar friction problem.

## Torque interpretation

SDK lowcmd has two useful torque-related outputs: the feedforward torque command and the reported/applied joint torque state. Their difference is important.

![SDK feedforward torque vs torque state](assets/fig_sdk_tau_cmd_vs_state.png)

In the SDK baseline, the feedforward torque command can be modest while the reported/applied torque state becomes larger. This supports the conclusion that SDK lowcmd is not equivalent to our pure external torque path. SDK can use internal low-level feedback inside the Unitree/Gazebo control path, while our pure-torque controller computes feedback externally in Python and sends only the final torque.

This explains why copying the SDK feedforward torque alone is not enough. To reproduce SDK behavior, the control architecture also matters: where Kp/Kd are applied, how fast feedback is updated, and how saturation/filtering are handled.


## Torque feedforward and computed-torque comparison method

A direct comparison between SDK feedforward torque and our model-based torque is possible, but the quantities must be named carefully.

For the SDK lowcmd logs, the feedforward torque is stored as `tau_sdk_cmd_i`. This is the explicit torque term sent together with SDK position and velocity references. The SDK log also includes `tau_state_i`, which is the reported/applied joint torque state from the Gazebo/Unitree side. These two values are not the same: `tau_sdk_cmd_i` is the feedforward term, while `tau_state_i` includes the result of the low-level controller and saturation.

For our pure-torque logs, the available comparable value is the final external torque command `tau_i` or `tau_total_i`. For the computed-torque controller, this is the torque sent by the Python controller after model and feedback terms. Therefore, the plots below compare:

- SDK feedforward torque: `tau_sdk_cmd_4`
- SDK reported/applied torque state: `tau_state_4`
- our computed-torque command without friction/damping
- our computed-torque command with friction/damping
- the best tuned augmented-PD/friction command, as a practical reference

The most exact feedforward-to-feedforward comparison would be to recompute our model feedforward term on the SDK trajectory using:

```text
 tau_model_ff = M(q_cmd) ddq_cmd + C(q_cmd, dq_cmd) dq_cmd + N(q_cmd)
```

This should be added as a future analysis script. The current plots are still valuable because they show what each controller path actually commanded or reported during the same workspace motion.

![J4 -30 deg torque comparison](assets/fig_j4_neg30deg_torque_comparison.png)

![J4 +30 deg torque comparison](assets/fig_j4_pos30deg_torque_comparison.png)

### Interpretation of torque comparison

For J4 negative motion, SDK feedforward torque and our model-based torque are of similar order, but SDK reported torque state can be larger. This supports the conclusion that SDK lowcmd is not only a feedforward torque command; it also contains strong low-level feedback inside the Unitree/Gazebo controller path.

For J4 positive motion, the SDK torque state can approach the Gazebo clamp, while tracking is still incomplete. This confirms that positive J4 is not merely a pure-torque tuning issue. It is a coupled plant/control behavior involving direction dependence, internal controller behavior, and possible J3-J4 interaction.

![Mean maximum command torque by controller](assets/fig_mean_max_command_torque_by_controller.png)

## Main achievements

This single-joint workspace experiment demonstrates several concrete achievements:

1. **A full 30-test single-joint workspace benchmark was completed.** The dataset covers both signs where safe and includes all six joints.
2. **Four pure-torque controller variants were evaluated.** This gives a controlled comparison between augmented PD, computed torque, and friction/damping compensation.
3. **A Unitree SDK lowcmd baseline was added.** This provides a manufacturer-style reference instead of judging pure torque in isolation.
4. **The best pure-torque controller was identified.** Augmented PD with tuned friction/damping wins every pure-torque single-joint test.
5. **The original J4 problem was substantially improved.** Negative J4 tracking changed from a large offset to near-SDK-level performance after J4-specific tuning.
6. **The remaining limitation was localized.** Positive J4 and coupled whole-arm motion remain the main risk areas for the next phase.

## Limitations

This report should not overclaim. The current dataset is still single-joint motion. Whole-arm forward-pose motion will create coupling between joints, especially J2, J3, and J4. A controller that works well for isolated J4 motion may still show worse performance during coupled forward-pose motion.

The pure-torque loop also has lower effective bandwidth than SDK lowcmd because it includes Python computation, file IPC, ROS bridge communication, Gazebo topic updates, feedback delay, and loop jitter. High Kp/Kd in this outer loop can magnify delayed or noisy feedback. Therefore, pure-torque gains cannot be copied directly from the SDK low-level controller.

Finally, the empirical J4 friction compensation of 1.5 should not be described as the official URDF friction value. The downloaded URDF/xacro uses J4 friction 1.0. The value 1.5 is an empirical compensation that improves the closed-loop behavior in this particular Gazebo torque-control setup.


## Forward-pose coupled motion evaluation

After the single-joint workspace study, the same control framework was tested on the coupled forward-pose target:

```text
[0, 1.5, -1.0, -0.54, 0, 0]
```

The target was evaluated at 25%, 50%, 75%, and 100% scale. Each test used an outbound S-curve trajectory followed by a return-to-start phase. As in the single-joint analysis, the main metrics below are computed on the outbound phase. The purpose of this section is to check whether the single-joint improvements, especially the tuned J4 compensation, survive coupled J2-J3-J4 whole-arm motion.

### Forward-pose summary

The forward-pose comparison confirms that **augmented PD with friction/damping compensation remains the best pure-torque controller**. It is the best pure-torque controller at all four pose scales. The SDK lowcmd baseline still has the smallest tracking-error norm, but the tuned pure-torque controller is close and uses a comparable overall command torque magnitude.

| scale   | sdk max/final deg   |   SDK tau_state Nm | best pure   | pure max/final deg   |   pure tau_cmd Nm |
|:--------|:--------------------|-------------------:|:------------|:---------------------|------------------:|
| 25pct   | 0.154 / 0.093       |              13.75 | augpd_fric  | 1.343 / 1.195        |             11.37 |
| 50pct   | 0.558 / 0.092       |              13.66 | augpd_fric  | 1.624 / 1.594        |             11.82 |
| 75pct   | 0.915 / 0.269       |              11.79 | augpd_fric  | 1.943 / 1.855        |             12.17 |
| 100pct  | 1.457 / 0.283       |              14.18 | augpd_fric  | 2.879 / 2.188        |             14.56 |

![Forward-pose tracking error by controller](assets/fig_forward_max_error_by_scale.png)

![Forward-pose final error by controller](assets/fig_forward_final_error_by_scale.png)

### Torque interpretation in forward-pose tests

The torque comparison again shows the architectural difference between SDK lowcmd and the pure-torque path. The SDK table reports `tau_state`, which includes the Unitree/Gazebo low-level control result, while the pure-torque logs report the external torque command produced in Python. In the 100% forward-pose test, the SDK maximum torque-state norm is **14.18 Nm**, while the best pure-torque controller reaches **14.56 Nm** command norm. However, the distribution of torque across joints is different.

![Forward-pose torque magnitude by controller](assets/fig_forward_torque_norm_by_scale.png)

![100% forward pose per-joint error](assets/fig_forward_100pct_per_joint_error.png)

![100% forward pose per-joint torque](assets/fig_forward_100pct_per_joint_torque.png)

### Forward-pose interpretation

The result is better than expected. The previous J4 single-joint problem did not destroy the whole-arm forward-pose test after tuning. At 100% scale, the tuned augmented-PD/friction controller has a maximum tracking-error norm of **0.0503 rad** (2.88 deg-equivalent) and final error norm of **0.0382 rad** (2.19 deg-equivalent). The SDK result is still better, with maximum tracking-error norm **0.0254 rad** (1.46 deg-equivalent) and final error norm **0.0049 rad** (0.28 deg-equivalent).

The important achievement is that the tuned pure-torque controller can now complete the full forward pose without the severe J4 failure observed in the early single-joint tests. The remaining difference from SDK is likely due to feedback bandwidth and control architecture rather than workspace limits. SDK lowcmd computes strong low-level feedback closer to the Gazebo/Unitree controller, while the pure-torque controller computes feedback externally in Python and sends final torque through file IPC and ROS.

The computed-torque variants are still not competitive in this forward-pose dataset. Computed torque without friction/damping has very large errors, and computed torque with friction/damping still shows poor coupled behavior. This supports the same conclusion from the single-joint dataset: under the current Python + file IPC + ROS + Gazebo implementation, the simpler augmented PD controller with empirical friction/damping compensation is more robust than the full computed-torque path.

## Appendix A: Best pure-torque vs SDK per test

In the table below, `Pure` and `SDK` are shown as `max error deg / achieved percent`.

| test       | pure          | sdk           | winner   |
|:-----------|:--------------|:--------------|:---------|
| J1 -30 deg | 0.73 / 100.0% | 0.43 / 100.0% | SDK      |
| J1 -10 deg | 0.25 / 100.1% | 0.09 / 100.0% | SDK      |
| J1 -5 deg  | 0.06 / 100.3% | 0.08 / 100.0% | Pure     |
| J1 +5 deg  | 0.05 / 99.9%  | 0.10 / 100.0% | Pure     |
| J1 +10 deg | 0.11 / 100.0% | 0.22 / 100.0% | Pure     |
| J1 +30 deg | 0.33 / 100.0% | 0.88 / 100.0% | Pure     |
| J2 +5 deg  | 1.71 / 131.7% | 0.26 / 100.2% | SDK      |
| J2 +10 deg | 1.69 / 115.8% | 0.46 / 100.2% | SDK      |
| J2 +30 deg | 1.69 / 104.6% | 1.21 / 100.1% | SDK      |
| J3 -30 deg | 1.18 / 97.7%  | 1.56 / 99.9%  | Pure     |
| J3 -10 deg | 0.95 / 92.1%  | 0.55 / 99.6%  | SDK      |
| J3 -5 deg  | 0.89 / 84.0%  | 0.23 / 99.2%  | SDK      |
| J4 -30 deg | 0.75 / 99.7%  | 1.76 / 99.8%  | Pure     |
| J4 -10 deg | 0.44 / 96.9%  | 0.58 / 99.4%  | Pure     |
| J4 -5 deg  | 0.41 / 93.4%  | 0.30 / 98.9%  | SDK      |
| J4 +5 deg  | 2.53 / 49.5%  | 1.04 / 81.0%  | SDK      |
| J4 +10 deg | 4.92 / 56.7%  | 2.85 / 72.8%  | SDK      |
| J4 +30 deg | 16.16 / 50.0% | 7.74 / 74.2%  | SDK      |
| J5 -30 deg | 0.50 / 99.7%  | 1.99 / 99.5%  | Pure     |
| J5 -10 deg | 0.18 / 101.1% | 0.74 / 98.5%  | Pure     |
| J5 -5 deg  | 0.23 / 103.5% | 0.45 / 97.0%  | Pure     |
| J5 +5 deg  | 0.26 / 96.5%  | 0.45 / 97.1%  | Pure     |
| J5 +10 deg | 0.34 / 98.2%  | 0.75 / 98.5%  | Pure     |
| J5 +30 deg | 0.78 / 99.2%  | 1.96 / 99.5%  | Pure     |
| J6 -30 deg | 0.72 / 99.5%  | 2.06 / 99.3%  | Pure     |
| J6 -10 deg | 0.19 / 99.4%  | 0.82 / 97.8%  | Pure     |
| J6 -5 deg  | 0.13 / 99.4%  | 0.52 / 95.7%  | Pure     |
| J6 +5 deg  | 0.11 / 99.6%  | 0.52 / 95.5%  | Pure     |
| J6 +10 deg | 0.21 / 99.6%  | 0.81 / 97.8%  | Pure     |
| J6 +30 deg | 0.58 / 99.6%  | 2.01 / 99.2%  | Pure     |


## Appendix C: All single-joint controller results

Each cell is formatted as:

```text
achieved percent / max error deg / max torque
```

For SDK lowcmd, the cell is:

```text
achieved percent / max error deg / feedforward torque / reported torque state
```

This table includes all 30 joint-angle commands and all five controller paths. The full numeric CSV with separate columns is also included as `appendix_all_controller_results.csv` in the report package.

| test       | AugPD no-fric               | AugPD fric                  | CT no-fric                  | CT fric                      | SDK lowcmd                                     |
|:-----------|:----------------------------|:----------------------------|:----------------------------|:-----------------------------|:-----------------------------------------------|
| J1 -30 deg | 90.5% / 3.61 deg / 1.17 Nm  | 100.0% / 0.73 deg / 1.23 Nm | 12.1% / 27.44 deg / 1.04 Nm | 99.8% / 1.28 deg / 1.15 Nm   | 100.0% / 0.43 deg / ff 0.00 Nm / state 2.24 Nm |
| J1 -10 deg | 71.4% / 3.15 deg / 1.06 Nm  | 100.1% / 0.25 deg / 1.07 Nm | 1.6% / 9.85 deg / 0.38 Nm   | 99.9% / 0.37 deg / 1.05 Nm   | 100.0% / 0.09 deg / ff 0.00 Nm / state 0.77 Nm |
| J1 -5 deg  | 43.1% / 2.96 deg / 1.02 Nm  | 100.3% / 0.06 deg / 1.03 Nm | 2.9% / 4.86 deg / 0.19 Nm   | 99.9% / 0.14 deg / 1.02 Nm   | 100.0% / 0.08 deg / ff 0.00 Nm / state 0.72 Nm |
| J1 +5 deg  | 42.8% / 2.95 deg / 1.02 Nm  | 99.9% / 0.05 deg / 1.02 Nm  | -1.8% / 5.09 deg / 0.19 Nm  | 99.7% / 0.15 deg / 1.02 Nm   | 100.0% / 0.10 deg / ff 0.00 Nm / state 0.73 Nm |
| J1 +10 deg | 71.4% / 3.10 deg / 1.05 Nm  | 100.0% / 0.11 deg / 1.05 Nm | -0.6% / 10.06 deg / 0.38 Nm | 99.9% / 0.29 deg / 1.05 Nm   | 100.0% / 0.22 deg / ff 0.00 Nm / state 0.76 Nm |
| J1 +30 deg | 90.5% / 3.53 deg / 1.15 Nm  | 100.0% / 0.33 deg / 1.15 Nm | 12.0% / 27.58 deg / 1.04 Nm | 99.9% / 0.84 deg / 1.14 Nm   | 100.0% / 0.88 deg / ff 0.00 Nm / state 1.60 Nm |
| J2 +5 deg  | 19.8% / 4.09 deg / 5.32 Nm  | 131.7% / 1.71 deg / 5.90 Nm | -0.0% / 5.00 deg / 4.11 Nm  | 367.8% / 13.50 deg / 5.90 Nm | 100.2% / 0.26 deg / ff 3.95 Nm / state 5.55 Nm |
| J2 +10 deg | 59.7% / 4.34 deg / 5.36 Nm  | 115.8% / 1.69 deg / 5.89 Nm | 0.0% / 10.00 deg / 4.31 Nm  | 229.7% / 13.22 deg / 5.89 Nm | 100.2% / 0.46 deg / ff 3.95 Nm / state 5.56 Nm |
| J2 +30 deg | 85.9% / 5.15 deg / 5.46 Nm  | 104.6% / 1.69 deg / 5.89 Nm | 0.0% / 30.00 deg / 5.14 Nm  | 135.3% / 12.43 deg / 5.89 Nm | 100.1% / 1.21 deg / ff 3.95 Nm / state 5.63 Nm |
| J3 -30 deg | 92.8% / 2.87 deg / 9.00 Nm  | 97.7% / 1.18 deg / 8.95 Nm  | 70.1% / 10.40 deg / 9.06 Nm | 89.9% / 3.91 deg / 9.00 Nm   | 99.9% / 1.56 deg / ff 7.23 Nm / state 9.42 Nm  |
| J3 -10 deg | 77.7% / 2.45 deg / 8.95 Nm  | 92.1% / 0.95 deg / 8.94 Nm  | 8.4% / 9.30 deg / 8.91 Nm   | 66.6% / 3.56 deg / 8.94 Nm   | 99.6% / 0.55 deg / ff 7.23 Nm / state 9.26 Nm  |
| J3 -5 deg  | 55.2% / 2.35 deg / 8.94 Nm  | 84.0% / 0.89 deg / 8.93 Nm  | 1.8% / 4.92 deg / 8.15 Nm   | 32.8% / 3.46 deg / 8.92 Nm   | 99.2% / 0.23 deg / ff 7.23 Nm / state 8.33 Nm  |
| J4 -30 deg | 80.3% / 6.91 deg / 3.82 Nm  | 99.7% / 0.75 deg / 3.79 Nm  | 0.2% / 29.94 deg / 2.33 Nm  | 67.7% / 10.89 deg / 3.77 Nm  | 99.8% / 1.76 deg / ff 2.04 Nm / state 4.83 Nm  |
| J4 -10 deg | 39.4% / 6.27 deg / 3.73 Nm  | 96.9% / 0.44 deg / 3.73 Nm  | 0.6% / 9.95 deg / 2.17 Nm   | 2.9% / 9.71 deg / 3.69 Nm    | 99.4% / 0.58 deg / ff 2.04 Nm / state 4.46 Nm  |
| J4 -5 deg  | 0.7% / 4.97 deg / 3.40 Nm   | 93.4% / 0.41 deg / 3.72 Nm  | 1.2% / 4.94 deg / 2.14 Nm   | 0.6% / 4.97 deg / 3.65 Nm    | 98.9% / 0.30 deg / ff 2.04 Nm / state 4.36 Nm  |
| J4 +5 deg  | 10.2% / 4.60 deg / 2.09 Nm  | 49.5% / 2.53 deg / 2.09 Nm  | -1.3% / 5.07 deg / 2.09 Nm  | 9.2% / 4.55 deg / 2.10 Nm    | 81.0% / 1.04 deg / ff 2.04 Nm / state 7.07 Nm  |
| J4 +10 deg | 35.4% / 6.85 deg / 2.14 Nm  | 56.7% / 4.92 deg / 2.09 Nm  | -0.7% / 10.07 deg / 2.09 Nm | 4.9% / 9.51 deg / 2.09 Nm    | 72.8% / 2.85 deg / ff 2.04 Nm / state 22.64 Nm |
| J4 +30 deg | 45.9% / 17.18 deg / 2.50 Nm | 50.0% / 16.16 deg / 3.77 Nm | -0.3% / 30.08 deg / 2.09 Nm | 1.8% / 29.46 deg / 2.08 Nm   | 74.2% / 7.74 deg / ff 2.04 Nm / state 30.00 Nm |
| J5 -30 deg | 61.8% / 12.98 deg / 1.10 Nm | 99.7% / 0.50 deg / 1.13 Nm  | -0.1% / 30.03 deg / 0.02 Nm | 88.7% / 3.39 deg / 1.11 Nm   | 99.5% / 1.99 deg / ff 0.00 Nm / state 1.27 Nm  |
| J5 -10 deg | 0.0% / 10.00 deg / 0.84 Nm  | 101.1% / 0.18 deg / 1.01 Nm | -0.3% / 10.03 deg / 0.03 Nm | 123.4% / 2.34 deg / 1.02 Nm  | 98.5% / 0.74 deg / ff 0.00 Nm / state 1.08 Nm  |
| J5 -5 deg  | -0.3% / 5.01 deg / 0.41 Nm  | 103.5% / 0.23 deg / 1.04 Nm | -0.6% / 5.03 deg / 0.03 Nm  | 190.5% / 4.53 deg / 1.02 Nm  | 97.0% / 0.45 deg / ff 0.01 Nm / state 1.04 Nm  |
| J5 +5 deg  | 1.0% / 4.95 deg / 0.45 Nm   | 96.5% / 0.26 deg / 1.06 Nm  | 0.7% / 4.97 deg / 0.02 Nm   | 21.9% / 3.91 deg / 1.04 Nm   | 97.1% / 0.45 deg / ff 0.00 Nm / state 1.05 Nm  |
| J5 +10 deg | 0.6% / 9.94 deg / 0.88 Nm   | 98.2% / 0.34 deg / 1.08 Nm  | 0.3% / 9.97 deg / 0.02 Nm   | 48.1% / 5.19 deg / 1.06 Nm   | 98.5% / 0.75 deg / ff 0.00 Nm / state 1.10 Nm  |
| J5 +30 deg | 61.0% / 13.26 deg / 1.16 Nm | 99.2% / 0.78 deg / 1.19 Nm  | 0.1% / 29.96 deg / 0.04 Nm  | 70.7% / 8.78 deg / 1.14 Nm   | 99.5% / 1.96 deg / ff 0.00 Nm / state 1.25 Nm  |
| J6 -30 deg | 61.4% / 13.26 deg / 1.13 Nm | 99.5% / 0.72 deg / 1.15 Nm  | 0.4% / 29.88 deg / 0.01 Nm  | 78.2% / 6.55 deg / 1.10 Nm   | 99.3% / 2.06 deg / ff 0.02 Nm / state 3.00 Nm  |
| J6 -10 deg | 1.2% / 9.88 deg / 0.85 Nm   | 99.4% / 0.19 deg / 1.04 Nm  | 1.2% / 9.88 deg / 0.01 Nm   | 75.6% / 2.44 deg / 1.03 Nm   | 97.8% / 0.82 deg / ff 0.02 Nm / state 1.57 Nm  |
| J6 -5 deg  | 2.4% / 4.89 deg / 0.42 Nm   | 99.4% / 0.13 deg / 1.01 Nm  | 2.3% / 4.89 deg / 0.01 Nm   | 71.5% / 1.43 deg / 1.01 Nm   | 95.7% / 0.52 deg / ff 0.02 Nm / state 1.29 Nm  |
| J6 +5 deg  | -2.5% / 5.13 deg / 0.46 Nm  | 99.6% / 0.11 deg / 1.04 Nm  | -2.5% / 5.13 deg / 0.01 Nm  | 88.7% / 0.56 deg / 1.03 Nm   | 95.5% / 0.52 deg / ff 0.02 Nm / state 1.30 Nm  |
| J6 +10 deg | -1.3% / 10.13 deg / 0.90 Nm | 99.6% / 0.21 deg / 1.06 Nm  | -1.3% / 10.13 deg / 0.01 Nm | 84.5% / 1.55 deg / 1.05 Nm   | 97.8% / 0.81 deg / ff 0.02 Nm / state 1.51 Nm  |
| J6 +30 deg | 61.5% / 13.19 deg / 1.15 Nm | 99.6% / 0.58 deg / 1.15 Nm  | -0.4% / 30.13 deg / 0.02 Nm | 82.2% / 5.33 deg / 1.12 Nm   | 99.2% / 2.01 deg / ff 0.02 Nm / state 2.84 Nm  |

## Appendix B: Reproducibility notes

The comparison file was generated with:

```bash
python3 compare_joint_workspace_sdk.py     --pure-dir logs/eval_joint_workspace     --sdk-dir logs/eval_joint_workspace_sdk     --out logs/joint_workspace_sdk_compare.csv
```

This report was generated from `joint_workspace_sdk_compare.csv` plus the uploaded pure-torque and SDK log archives. The forward-pose results should be appended after the next testing phase rather than rewriting this single-joint analysis from scratch.


## Appendix D: Forward-pose controller results

Each row reports outbound metrics for one forward-pose scale and controller. Error norms are reported in degree-equivalent form for readability; torque is SDK `tau_state` for SDK and external command torque for pure-torque controllers.

| scale   | controller   |   max err deg |   final err deg |   RMS norm rad |   max tau cmd/state Nm |   rate Hz |
|:--------|:-------------|--------------:|----------------:|---------------:|-----------------------:|----------:|
| 100pct  | SDK          |          1.46 |            0.28 |         0.0143 |                  14.18 |     481.5 |
| 100pct  | AugPD+fric   |          2.88 |            2.19 |         0.03   |                  14.56 |     278.4 |
| 100pct  | AugPD        |         23.3  |           18.11 |         0.3118 |                  12.33 |     275.2 |
| 100pct  | CT+fric      |         82.87 |           76.71 |         1.0798 |                  13.15 |     265.2 |
| 100pct  | CT           |        141.4  |          139.41 |         1.6342 |                  11.25 |     278.5 |
| 25pct   | SDK          |          0.15 |            0.09 |         0.0017 |                  13.75 |     481.7 |
| 25pct   | AugPD+fric   |          1.34 |            1.2  |         0.0208 |                  11.37 |     267.9 |
| 25pct   | AugPD        |         18.94 |           17.83 |         0.2447 |                  10.99 |     260.3 |
| 25pct   | CT+fric      |         85.19 |           67.44 |         1.0525 |                  10.92 |     276.9 |
| 25pct   | CT           |         44.05 |           44.05 |         0.4862 |                  10.19 |     255.3 |
| 50pct   | SDK          |          0.56 |            0.09 |         0.0047 |                  13.66 |     481   |
| 50pct   | AugPD+fric   |          1.62 |            1.59 |         0.0236 |                  11.82 |     255.9 |
| 50pct   | AugPD        |         20.36 |           17.93 |         0.2787 |                  11.49 |     257.2 |
| 50pct   | CT+fric      |         82.49 |           66.67 |         1.0625 |                  11.15 |     279.9 |
| 50pct   | CT           |         75.51 |           75.37 |         0.9    |                  10.96 |     270   |
| 75pct   | SDK          |          0.91 |            0.27 |         0.0093 |                  11.79 |     482.1 |
| 75pct   | AugPD+fric   |          1.94 |            1.85 |         0.0265 |                  12.17 |     266.5 |
| 75pct   | AugPD        |         22.14 |           18.05 |         0.2981 |                  11.93 |     270.9 |
| 75pct   | CT+fric      |         81.4  |           70.19 |         1.1078 |                  11.26 |     280.7 |
| 75pct   | CT           |        107.97 |          107.94 |         1.2378 |                  11.11 |     272.8 |
