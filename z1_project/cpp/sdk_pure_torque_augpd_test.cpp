#include <unitree_arm_sdk/control/unitreeArm.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

using UNITREE_ARM::ArmFSMState;
using UNITREE_ARM::unitreeArm;

namespace {

constexpr int NDOF = 6;
constexpr double kDegToRad = M_PI / 180.0;

Vec6 vec6_from_array(const std::array<double, NDOF>& values) {
    Vec6 out;
    for (int i = 0; i < NDOF; ++i) {
        out[i] = values[i];
    }
    return out;
}

Vec6 clip_symmetric(const Vec6& value, double limit) {
    Vec6 out = value;
    for (int i = 0; i < NDOF; ++i) {
        out[i] = std::clamp(out[i], -limit, limit);
    }
    return out;
}

void scurve(const Vec6& q0, const Vec6& q1, double t, double T, Vec6& q, Vec6& dq, Vec6& ddq) {
    if (t <= 0.0) {
        q = q0;
        dq.setZero();
        ddq.setZero();
        return;
    }
    if (t >= T) {
        q = q1;
        dq.setZero();
        ddq.setZero();
        return;
    }

    const double s = std::clamp(t / T, 0.0, 1.0);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;
    const double s6 = s5 * s;
    const double s7 = s6 * s;

    const double b = 35.0 * s4 - 84.0 * s5 + 70.0 * s6 - 20.0 * s7;
    const double bd = (140.0 * s3 - 420.0 * s4 + 420.0 * s5 - 140.0 * s6) / T;
    const double bdd = (420.0 * s2 - 1680.0 * s3 + 2100.0 * s4 - 840.0 * s5) / (T * T);

    const Vec6 delta = q1 - q0;
    q = q0 + b * delta;
    dq = bd * delta;
    ddq = bdd * delta;
}

void write_csv_header(std::ofstream& out) {
    out << "t";
    for (int i = 1; i <= NDOF; ++i) out << ",q_actual_" << i;
    for (int i = 1; i <= NDOF; ++i) out << ",dq_actual_" << i;
    for (int i = 1; i <= NDOF; ++i) out << ",q_des_" << i;
    for (int i = 1; i <= NDOF; ++i) out << ",dq_des_" << i;
    for (int i = 1; i <= NDOF; ++i) out << ",ddq_des_" << i;
    for (int i = 1; i <= NDOF; ++i) out << ",tau_" << i;
    for (int i = 1; i <= NDOF; ++i) out << ",tau_ff_" << i;
    for (int i = 1; i <= NDOF; ++i) out << ",tau_fb_" << i;
    for (int i = 1; i <= NDOF; ++i) out << ",tau_total_raw_" << i;
    out << '\n';
}

void write_vec(std::ofstream& out, const Vec6& value) {
    for (int i = 0; i < NDOF; ++i) {
        out << ',' << value[i];
    }
}

}  // namespace

int main(int argc, char** argv) {
    double move_time = 6.0;
    double hold_time = 1.5;
    double tau_limit = 30.0;
    std::filesystem::path csv_path =
        "/home/icesword/Desktop/torque_control/z1_project/logs/sdk_pure_torque_labelrun_forward_plus5deg_augpd.csv";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(name + " requires a value");
            }
            return argv[++i];
        };
        if (arg == "--move-time") {
            move_time = std::stod(require_value(arg));
        } else if (arg == "--hold-time") {
            hold_time = std::stod(require_value(arg));
        } else if (arg == "--tau-limit") {
            tau_limit = std::stod(require_value(arg));
        } else if (arg == "--csv-log") {
            csv_path = require_value(arg);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (move_time <= 0.0 || hold_time < 0.0 || tau_limit <= 0.0) {
        throw std::runtime_error("invalid timing or torque limit");
    }

    std::cout << std::fixed << std::setprecision(6);
    unitreeArm arm(true);

    std::cout << "calling labelRun(\"forward\")\n";
    arm.sendRecvThread->start();
    const auto label_start = std::chrono::steady_clock::now();
    arm.labelRun("forward");
    const double label_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - label_start).count();
    std::cout << "labelRun returned after " << label_elapsed << " s\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    arm.setFsm(ArmFSMState::PASSIVE);
    arm.setFsm(ArmFSMState::LOWCMD);
    arm.sendRecvThread->shutdown();

    arm._ctrlComp->lowcmd->setZeroKp();
    arm._ctrlComp->lowcmd->setZeroKd();
    arm._ctrlComp->lowcmd->setGripperZeroGain();

    const double dt = arm._ctrlComp->dt;
    const Vec6 q_start = arm.lowstate->getQ();
    const Vec6 dq_start = arm.lowstate->getQd();
    Vec6 q_goal = q_start + Vec6::Constant(5.0 * kDegToRad);

    const Vec6 kp = vec6_from_array({120.0, 180.0, 180.0, 140.0, 90.0, 90.0});
    const Vec6 kd = vec6_from_array({8.0, 14.0, 14.0, 10.0, 5.0, 5.0});

    std::filesystem::create_directories(csv_path.parent_path());
    std::ofstream csv(csv_path);
    csv << std::setprecision(17);
    write_csv_header(csv);

    Vec6 max_error = Vec6::Zero();
    Vec6 max_tau = Vec6::Zero();
    Vec6 max_dq = Vec6::Zero();
    Vec6 final_error = Vec6::Zero();
    Vec6 final_q = q_start;
    std::size_t steps = 0;

    std::cout << "q_start = [" << q_start.transpose() << "]\n";
    std::cout << "dq_start = [" << dq_start.transpose() << "]\n";
    std::cout << "q_goal = [" << q_goal.transpose() << "]\n";
    std::cout << "zeroed Unitree lowcmd Kp/Kd; sending pure torque only\n";

    const auto loop_wall_start = std::chrono::steady_clock::now();
    const int total_steps = static_cast<int>(std::ceil((move_time + hold_time) / dt));
    auto next_tick = std::chrono::steady_clock::now();
    for (int step = 0; step <= total_steps; ++step) {
        const double t = step * dt;
        const Vec6 q_actual = arm.lowstate->getQ();
        const Vec6 dq_actual = arm.lowstate->getQd();

        Vec6 q_des;
        Vec6 dq_des;
        Vec6 ddq_des;
        scurve(q_start, q_goal, t, move_time, q_des, dq_des, ddq_des);

        const Vec6 tau_ff = arm._ctrlComp->armModel->inverseDynamics(q_des, dq_des, ddq_des, Vec6::Zero());
        const Vec6 tau_fb = kp.asDiagonal() * (q_des - q_actual) + kd.asDiagonal() * (dq_des - dq_actual);
        const Vec6 tau_raw = tau_ff + tau_fb;
        const Vec6 tau = clip_symmetric(tau_raw, tau_limit);

        // With lowcmd Kp/Kd zeroed in the SDK and sim bridge, q/dq are not a
        // servo target. Send measured q/dq so only tau contributes.
        arm.setArmCmd(q_actual, dq_actual, tau);
        arm.setGripperCmd(0.0, 0.0, 0.0);
        arm.sendRecv();

        const Vec6 error = q_des - q_actual;
        for (int i = 0; i < NDOF; ++i) {
            max_error[i] = std::max(max_error[i], std::abs(error[i]));
            max_tau[i] = std::max(max_tau[i], std::abs(tau[i]));
            max_dq[i] = std::max(max_dq[i], std::abs(dq_actual[i]));
        }
        final_error = error;
        final_q = q_actual;

        csv << t;
        write_vec(csv, q_actual);
        write_vec(csv, dq_actual);
        write_vec(csv, q_des);
        write_vec(csv, dq_des);
        write_vec(csv, ddq_des);
        write_vec(csv, tau);
        write_vec(csv, tau_ff);
        write_vec(csv, tau_fb);
        write_vec(csv, tau_raw);
        csv << '\n';
        ++steps;

        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
        std::this_thread::sleep_until(next_tick);
        if (std::chrono::steady_clock::now() > next_tick + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt))) {
            next_tick = std::chrono::steady_clock::now();
        }
    }

    const double loop_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_wall_start).count();

    std::cout << "actual_delta_deg = [" << ((final_q - q_start) / kDegToRad).transpose() << "]\n";
    std::cout << "command_delta_deg = [" << ((q_goal - q_start) / kDegToRad).transpose() << "]\n";
    std::cout << "final_error_deg = [" << (final_error / kDegToRad).transpose() << "]\n";
    std::cout << "max_error_deg = [" << (max_error / kDegToRad).transpose() << "]\n";
    std::cout << "max_tau_nm = [" << max_tau.transpose() << "]\n";
    std::cout << "max_dq_rad_s = [" << max_dq.transpose() << "]\n";
    std::cout << "effective_loop_rate = " << (steps / loop_elapsed) << " Hz\n";
    std::cout << "CSV path = " << csv_path << "\n";

    std::cout << "calling backToStart()\n";
    arm.sendRecvThread->start();
    const auto back_start = std::chrono::steady_clock::now();
    arm.backToStart();
    const double back_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - back_start).count();
    std::cout << "backToStart returned after " << back_elapsed << " s\n";
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}
