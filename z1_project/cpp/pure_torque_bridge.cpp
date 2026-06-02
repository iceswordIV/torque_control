#include <unitree_arm_sdk/control/unitreeArm.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <unistd.h>

using UNITREE_ARM::ArmFSMState;
using UNITREE_ARM::unitreeArm;

namespace {

std::atomic<bool> g_running{true};

void handle_signal(int) {
    g_running.store(false);
}

struct Options {
    double dt = 0.002;
    std::filesystem::path runtime_dir = std::string("/tmp/z1_torque_") + std::to_string(getuid());
    bool back_to_start_at_begin = false;
    bool back_to_start_at_end = false;
    bool has_gripper = true;
    bool custom_udp = false;
    std::string udp_to_ip = "127.0.0.1";
    unsigned int udp_to_port = 8071;
    unsigned int udp_own_port = 8072;
    std::size_t udp_timeout_us = 500000;
};

void print_usage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [--dt 0.002] [--runtime-dir /tmp/z1_torque_UID]"
              << " [--back-to-start] [--back-to-start-end] [--no-gripper]\n"
              << "       " << argv0 << " --gazebo-ports\n"
              << "       " << argv0 << " --udp-to-ip 127.0.0.1 --udp-to-port 8073"
              << " --udp-own-port 8074 [--udp-timeout-us 500000]\n";
}

Options parse_options(int argc, char** argv) {
    Options opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(name + " requires a value");
            }
            return argv[++i];
        };

        if (arg == "--dt") {
            opts.dt = std::stod(require_value(arg));
        } else if (arg == "--runtime-dir") {
            opts.runtime_dir = require_value(arg);
        } else if (arg == "--back-to-start") {
            opts.back_to_start_at_begin = true;
        } else if (arg == "--back-to-start-end") {
            opts.back_to_start_at_end = true;
        } else if (arg == "--no-gripper") {
            opts.has_gripper = false;
        } else if (arg == "--gazebo-ports") {
            opts.custom_udp = true;
            opts.udp_to_ip = "127.0.0.1";
            opts.udp_to_port = 8073;
            opts.udp_own_port = 8074;
        } else if (arg == "--udp-to-ip") {
            opts.custom_udp = true;
            opts.udp_to_ip = require_value(arg);
        } else if (arg == "--udp-to-port") {
            opts.custom_udp = true;
            opts.udp_to_port = static_cast<unsigned int>(std::stoul(require_value(arg)));
        } else if (arg == "--udp-own-port") {
            opts.custom_udp = true;
            opts.udp_own_port = static_cast<unsigned int>(std::stoul(require_value(arg)));
        } else if (arg == "--udp-timeout-us") {
            opts.custom_udp = true;
            opts.udp_timeout_us = static_cast<std::size_t>(std::stoull(require_value(arg)));
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
    if (opts.dt <= 0.0) {
        throw std::runtime_error("--dt must be positive");
    }
    if (opts.custom_udp && (opts.udp_to_port == 0 || opts.udp_own_port == 0)) {
        throw std::runtime_error("UDP ports must be non-zero");
    }
    return opts;
}

double now_seconds() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    const auto now = clock::now();
    return std::chrono::duration<double>(now - t0).count();
}

Vec6 zero_vec6() {
    return Vec6::Zero();
}

bool read_torque_command(const std::filesystem::path& path, Vec6& tau) {
    std::ifstream in(path);
    if (!in) {
        tau = zero_vec6();
        return false;
    }
    Vec6 value = zero_vec6();
    for (int i = 0; i < 6; ++i) {
        if (!(in >> value[i])) {
            tau = zero_vec6();
            return false;
        }
    }
    tau = value;
    return true;
}

void write_sensor_file(const std::filesystem::path& path, double timestamp, const Vec6& q, const Vec6& dq, const Vec6& tau) {
    const auto tmp = path.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        out << std::setprecision(17) << timestamp;
        for (int i = 0; i < 6; ++i) {
            out << ' ' << q[i];
        }
        for (int i = 0; i < 6; ++i) {
            out << ' ' << dq[i];
        }
        for (int i = 0; i < 6; ++i) {
            out << ' ' << tau[i];
        }
        out << '\n';
    }
    std::filesystem::rename(tmp, path);
}

void send_zero_torque(unitreeArm& arm, double dt) {
    Vec6 q = arm.lowstate->getQ();
    Vec6 dq = arm.lowstate->getQd();
    Vec6 zero = zero_vec6();
    for (int i = 0; i < 20; ++i) {
        q = arm.lowstate->getQ();
        dq = arm.lowstate->getQd();
        arm.setArmCmd(q, dq, zero);
        arm.setGripperCmd(0.0, 0.0, 0.0);
        arm.sendRecv();
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
}

std::unique_ptr<unitreeArm> create_arm(const Options& opts) {
    if (!opts.custom_udp) {
        return std::make_unique<unitreeArm>(opts.has_gripper);
    }

    auto* ctrl_comp = new UNITREE_ARM::CtrlComponents();
    ctrl_comp->dt = opts.dt;
    ctrl_comp->udp = new UNITREE_ARM::UDPPort(
        opts.udp_to_ip,
        opts.udp_to_port,
        opts.udp_own_port,
        UNITREE_ARM::RECVSTATE_LENGTH,
        UNITREE_ARM::BlockYN::NO,
        opts.udp_timeout_us);
    ctrl_comp->armModel = new UNITREE_ARM::Z1Model();
    ctrl_comp->armModel->addLoad(0.03);
    return std::make_unique<unitreeArm>(ctrl_comp);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options opts = parse_options(argc, argv);
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        std::filesystem::create_directories(opts.runtime_dir);
        const auto cmd_path = opts.runtime_dir / "z1_torque_cmd.txt";
        const auto sensor_path = opts.runtime_dir / "z1_sensor.txt";
        const auto stop_path = opts.runtime_dir / "z1_stop.txt";
        std::filesystem::remove(stop_path);

        std::cout << "runtime dir: " << opts.runtime_dir << "\n";
        if (opts.custom_udp) {
            std::cout << "UDP target " << opts.udp_to_ip << ":" << opts.udp_to_port
                      << ", own port " << opts.udp_own_port << "\n";
        }
        auto arm_holder = create_arm(opts);
        unitreeArm& arm = *arm_holder;

        if (opts.back_to_start_at_begin) {
            std::cout << "calling backToStart() before torque loop\n";
            arm.sendRecvThread->start();
            arm.backToStart();
            arm.sendRecvThread->shutdown();
        }

        arm.sendRecvThread->start();
        arm.setFsm(ArmFSMState::PASSIVE);
        arm.setFsm(ArmFSMState::LOWCMD);
        arm.sendRecvThread->shutdown();

        arm._ctrlComp->lowcmd->setZeroKp();
        arm._ctrlComp->lowcmd->setZeroKd();
        arm._ctrlComp->lowcmd->setGripperZeroGain();

        Vec6 tau_cmd = zero_vec6();
        std::size_t loops = 0;
        auto last_print = std::chrono::steady_clock::now();
        auto next_tick = std::chrono::steady_clock::now();

        while (g_running.load()) {
            if (std::filesystem::exists(stop_path)) {
                break;
            }

            read_torque_command(cmd_path, tau_cmd);
            Vec6 q_actual = arm.lowstate->getQ();
            Vec6 dq_actual = arm.lowstate->getQd();

            write_sensor_file(sensor_path, now_seconds(), q_actual, dq_actual, tau_cmd);

            arm.setArmCmd(q_actual, dq_actual, tau_cmd);
            arm.setGripperCmd(0.0, 0.0, 0.0);
            arm.sendRecv();

            ++loops;
            const auto now = std::chrono::steady_clock::now();
            if (now - last_print > std::chrono::seconds(1)) {
                const double rate = loops / std::chrono::duration<double>(now - last_print).count();
                std::cout << "rate " << std::fixed << std::setprecision(1) << rate
                          << " Hz, q [" << q_actual.transpose()
                          << "], dq [" << dq_actual.transpose()
                          << "], tau [" << tau_cmd.transpose() << "]\n";
                loops = 0;
                last_print = now;
            }

            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(opts.dt));
            std::this_thread::sleep_until(next_tick);
            if (std::chrono::steady_clock::now() > next_tick + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(opts.dt))) {
                next_tick = std::chrono::steady_clock::now();
            }
        }

        std::cout << "exiting torque loop, sending zero torque\n";
        send_zero_torque(arm, opts.dt);

        if (opts.back_to_start_at_end) {
            std::cout << "calling backToStart() on exit\n";
            arm.sendRecvThread->start();
            arm.backToStart();
            arm.sendRecvThread->shutdown();
        }

        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "pure_torque_bridge error: " << exc.what() << "\n";
        return 1;
    }
}
