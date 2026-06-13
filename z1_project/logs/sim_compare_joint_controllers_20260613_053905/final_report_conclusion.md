# Final Controller Comparison Report Notes

## Dataset overview
- Output folder: `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905`
- Raw controller CSV logs analyzed: 120
- Tracking metrics use the outbound/hold segment (`outbound` plus `outbound_hold` when present). Return motion is evaluated separately.
- Return success threshold: commanded-joint final return error <= 2 deg and all-joint final return error <= 5 deg.
- AugPD no friction: 24 CSV logs, 24 unique joint/angle cases, J1:3;J2:3;J3:3;J4:3;J5:6;J6:6.
- AugPD friction: 24 CSV logs, 24 unique joint/angle cases, J1:3;J2:3;J3:3;J4:3;J5:6;J6:6.
- Computed torque baseline: 24 CSV logs, 24 unique joint/angle cases, J1:3;J2:3;J3:3;J4:3;J5:6;J6:6.
- CPID friction: 24 CSV logs, 24 unique joint/angle cases, J1:3;J2:3;J3:3;J4:3;J5:6;J6:6.
- Unitree SDK LOWCMD: 24 CSV logs, 24 unique joint/angle cases, J1:3;J2:3;J3:3;J4:3;J5:6;J6:6.

## Controller comparison result
- Best overall by mean commanded-joint RMS error: AugPD friction (0.346 deg).
- Lowest mean commanded-joint max error: AugPD friction (0.683 deg).
- Largest mean commanded-joint max torque: Unitree SDK LOWCMD (9.310 Nm).
- Adding the friction model to AugPD reduced mean RMS error by 93.7% (5.529 deg to 0.346 deg).
- CPID friction increased mean RMS error by 106.9% relative to AugPD friction (0.346 deg to 0.717 deg).
- Worst tracking cases by commanded-joint max error:
  - Computed torque baseline J5 +30 deg: max error 30.014 deg, RMS 23.956 deg.
  - Computed torque baseline J6 -30 deg: max error 30.009 deg, RMS 23.911 deg.
  - Computed torque baseline J2 +30 deg: max error 30.000 deg, RMS 23.980 deg.
  - Computed torque baseline J6 +30 deg: max error 29.994 deg, RMS 23.796 deg.
  - Computed torque baseline J5 -30 deg: max error 29.991 deg, RMS 23.702 deg.

## SDK vs simulation result
- SDK/LOWCMD mean RMS error was 1.123 deg and mean max error was 1.455 deg. The closest simulation controller by mean absolute RMS difference was AugPD friction with 1.167 deg mean absolute RMS difference. SDK behavior is therefore useful as a baseline, but it is not identical to the torque-controller simulation.
- The SDK summary overwrite issue was avoided by parsing the 24 raw `sdk_lowcmd/workspace_j*_sdk_lowcmd.csv` files directly.

## M/C/N dynamics model validation
- The analytic dynamics comparison found max |M_python - M_sdk| = 0.00340829, max |N_python - N_sdk| = 0.0104184, max ||h_python - h_sdk|| = 0.00104424, max ||tau_python - tau_sdk|| = 0.0143673, and max |dM_python - fd_dM_sdk| = 0.0080013. These values indicate the local analytic M/C/N model is close to the Unitree SDK inverseDynamics result, with remaining mismatch small relative to the large gravity/torque terms in the hard poses.

## Real hardware limitation interpretation
- The real 100% forward-pose CPID friction2.5 test showed that the real arm has higher friction/deadband than the model assumed. Increasing friction compensation may help overcome static friction, but too much friction compensation can cause oscillation/chattering and actuator heating. Therefore, no more real-arm tests are performed. The remaining systematic comparison is done in simulation and SDK/LOWCMD analysis.

## Safety decision: why real tests were stopped
- Further hardware tuning would require pushing friction compensation and torque commands beyond the already-observed safe operating envelope. Because static friction/deadband, unmodeled actuator behavior, and thermal risk were not fully captured by the simulation model, real-arm tests were stopped and the final comparison was limited to logged simulation plus SDK/LOWCMD data.

## Final conclusion for report/poster
- In the logged simulation/SDK dataset, AugPD friction gave the best overall tracking accuracy by mean RMS error. Friction compensation was essential: AugPD with friction strongly outperformed AugPD without friction. CPID friction did not beat AugPD friction in this batch, indicating that the integral/friction tuning used here was more aggressive than necessary for the simulated benchmark. The analytic dynamics model matched the Unitree SDK inverseDynamics closely enough to support computed-torque analysis, but real-hardware friction/deadband limited safe transfer, so the final report should present simulation and SDK/LOWCMD results as the systematic comparison while explaining why hardware testing was stopped.

## Generated plot files
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/mean_rms_error_by_controller.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/mean_max_error_by_controller.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/mean_final_error_by_controller.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/mean_max_torque_by_controller.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/per_joint_rms_error_by_controller.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/worst_10_cases_by_max_error.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/sdk_vs_sim_error_comparison.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/tracking_j1_pos30.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/tracking_j2_pos30.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/tracking_j3_neg30.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/tracking_j4_neg30.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/tracking_j5_neg30.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/tracking_j5_pos30.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/tracking_j6_neg30.png`
- `/home/icesword/Desktop/torque_control/z1_project/logs/sim_compare_joint_controllers_20260613_053905/final_analysis_plots/tracking_j6_pos30.png`
