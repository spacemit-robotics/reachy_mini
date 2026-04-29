/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <termios.h>
#include <unistd.h>
#include <grpcpp/grpcpp.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>



#include "reachy_kinematics.h"
#include "reachy_sim.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReaderWriter;
using grpc::Status;
using reachy::sim::ControlRequest;
using reachy::sim::ReachySimService;
using reachy::sim::SimState;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// -----------------------------------------------------------------------------
// Action Definitions (Keyframes)
// -----------------------------------------------------------------------------
struct ActionKeyframe {
    float roll, pitch, yaw;
    float ra, la;
    float duration;
};

const std::vector<ActionKeyframe> seq_nod = {
    {0, 15, 0, 0, 0, 0.15f}, {0, -10, 0, 0, 0, 0.12f}, {0, 12, 0, 0, 0, 0.12f},
    {0, -5, 0, 0, 0, 0.10f}, {0, 0, 0, 0, 0, 0.15f},
};

const std::vector<ActionKeyframe> seq_shake = {
    {0, 0, 25, 0, 0, 0.15f},  {0, 0, -25, 0, 0, 0.15f}, {0, 0, 20, 0, 0, 0.12f},
    {0, 0, -20, 0, 0, 0.12f}, {0, 0, 0, 0, 0, 0.15f},
};

const std::vector<ActionKeyframe> seq_dance = {
    {0, 0, 0, 1.2f, -0.3f, 0.12f}, {0, 0, 0, -0.3f, 1.2f, 0.12f}, {0, 0, 0, 1.2f, -0.3f, 0.12f},
    {0, 0, 0, -0.3f, 1.2f, 0.12f}, {0, 0, 0, 1.2f, -0.3f, 0.12f}, {0, 0, 0, -0.3f, 1.2f, 0.12f},
    {0, 0, 0, 0.0f, 0.0f, 0.15f},
};

// -----------------------------------------------------------------------------
// IK Helper: RPY (degrees) -> joint angles (radians)
// -----------------------------------------------------------------------------
static void buildPoseMatrix(float r, float p, float y, float T[16]) {
    auto d2r = [](float d) { return d * static_cast<float>(M_PI) / 180.0f; };
    float cr = cosf(d2r(r)), sr = sinf(d2r(r));
    float cp = cosf(d2r(p)), sp = sinf(d2r(p));
    float cy = cosf(d2r(y)), sy = sinf(d2r(y));

    T[0] = cy * cp;
    T[1] = cy * sp * sr - sy * cr;
    T[2] = cy * sp * cr + sy * sr;
    T[3] = 0;
    T[4] = sy * cp;
    T[5] = sy * sp * sr + cy * cr;
    T[6] = sy * sp * cr - cy * sr;
    T[7] = 0;
    T[8] = -sp;
    T[9] = cp * sr;
    T[10] = cp * cr;
    T[11] = 0;
    T[12] = 0;
    T[13] = 0;
    T[14] = 0;
    T[15] = 1.0f;
}

// 返回 true 表示 IK 成功，joints[0]=body_yaw, joints[1..6]=stewart
static bool solveIK(float roll, float pitch, float yaw, float joints[7]) {
    float pose[16];
    buildPoseMatrix(roll, pitch, yaw, pose);
    return reachy_calculate_ik(pose, joints) == 0;
}

// -----------------------------------------------------------------------------
// ControlClient Class
// -----------------------------------------------------------------------------
class ControlClient {
    public:
    explicit ControlClient(std::shared_ptr<Channel> channel)
        : stub_(ReachySimService::NewStub(channel)), running_(true) {
        r_ = p_ = y_ = ra_ = la_ = 0;
        step_size_ = 5.0f;
        reachy_reset_fk(NULL);
    }

    void run() {
        ClientContext context;
        auto stream = stub_->ControlStream(&context);

        // 启动输入读取线程
        std::thread input_thread([this]() { captureInput(); });

        auto last_time = std::chrono::steady_clock::now();
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            last_time = now;

            updateActions(dt);

            // 客户端完成 IK 解算，发送关节值
            ControlRequest req;
            {
                std::lock_guard<std::mutex> lock(mtx_);

                float joints[7] = {};
                if (solveIK(r_, p_, y_, joints)) {
                    req.set_body_yaw(joints[0]);
                    req.set_stewart_1(joints[1]);
                    req.set_stewart_2(joints[2]);
                    req.set_stewart_3(joints[3]);
                    req.set_stewart_4(joints[4]);
                    req.set_stewart_5(joints[5]);
                    req.set_stewart_6(joints[6]);
                } else {
                    // IK 失败时保持零位
                    req.set_body_yaw(0);
                    req.set_stewart_1(0);
                    req.set_stewart_2(0);
                    req.set_stewart_3(0);
                    req.set_stewart_4(0);
                    req.set_stewart_5(0);
                    req.set_stewart_6(0);
                }
                req.set_right_antenna(ra_);
                req.set_left_antenna(la_);
            }
            stream->Write(req);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));  // ~50Hz
        }

        stream->WritesDone();
        stream->Finish();
        input_thread.join();
    }

    private:
    void captureInput() {
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

        std::cout << "[*] Control Client Ready. "
                    << "Keys: WASD (P/Y), QE (Roll), ZX (LA), CV (RA), "
                    << "1-3 (Actions), ESC (Quit)" << std::endl;

        while (running_) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                std::lock_guard<std::mutex> lock(mtx_);
                if (c == 27) {
                    running_ = false;
                    break;
                }  // ESC
                processKey(c);
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    }

    void processKey(char c) {
        // QE (Roll)
        if (c == 'q') {
            r_ -= step_size_;
        } else if (c == 'e') {
            r_ += step_size_;
        // WASD
        } else if (c == 'w') {
            p_ -= step_size_;
        } else if (c == 's') {
            p_ += step_size_;
        } else if (c == 'a') {
            y_ -= step_size_;
        } else if (c == 'd') {
            y_ += step_size_;
        // ZX (Left Antenna)
        } else if (c == 'z') {
            la_ -= 0.1;
        } else if (c == 'x') {
            la_ += 0.1;
        // CV (Right Antenna)
        } else if (c == 'c') {
            ra_ -= 0.1;
        } else if (c == 'v') {
            ra_ += 0.1;
        // Home
        } else if (c == 'h') {
            r_ = p_ = y_ = ra_ = la_ = 0;
            active_seq_ = nullptr;
        // Actions
        } else if (c == '1') {
            startAction(seq_nod, 3);
        } else if (c == '2') {
            startAction(seq_shake, 3);
        } else if (c == '3') {
            startAction(seq_dance, 3);
        }

        // Clamping
        r_ = std::max(-25.0f, std::min(25.0f, static_cast<float>(r_)));
        p_ = std::max(-35.0f, std::min(35.0f, static_cast<float>(p_)));
        y_ = std::max(-170.0f, std::min(170.0f, static_cast<float>(y_)));
    }

    void startAction(const std::vector<ActionKeyframe> &seq, int repeats) {
        active_seq_ = &seq;
        seq_idx_ = 0;
        seq_timer_ = 0;
        seq_repeats_ = repeats;
        start_r_ = r_;
        start_p_ = p_;
        start_y_ = y_;
        start_ra_ = ra_;
        start_la_ = la_;
    }

    void updateActions(double dt) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!active_seq_)
            return;

        seq_timer_ += dt;
        const auto &kf = (*active_seq_)[seq_idx_];
        float t = std::min(1.0f, static_cast<float>(seq_timer_ / kf.duration));
        float s = t * t * (3.0f - 2.0f * t);  // Smoothstep

        r_ = start_r_ + s * (kf.roll - start_r_);
        p_ = start_p_ + s * (kf.pitch - start_p_);
        y_ = start_y_ + s * (kf.yaw - start_y_);
        ra_ = start_ra_ + s * (kf.ra - start_ra_);
        la_ = start_la_ + s * (kf.la - start_la_);

        if (t >= 1.0f) {
            seq_idx_++;
            if (seq_idx_ >= active_seq_->size()) {
                seq_repeats_--;
                if (seq_repeats_ > 0) {
                    seq_idx_ = 0;
                } else {
                    active_seq_ = nullptr;
                }
            }
            seq_timer_ = 0;
            start_r_ = r_;
            start_p_ = p_;
            start_y_ = y_;
            start_ra_ = ra_;
            start_la_ = la_;
        }
    }

    std::unique_ptr<ReachySimService::Stub> stub_;
    std::mutex mtx_;
    float r_, p_, y_, ra_, la_, step_size_;
    bool running_;

    // Action state
    const std::vector<ActionKeyframe> *active_seq_ = nullptr;
    int seq_idx_ = 0;
    double seq_timer_ = 0;
    int seq_repeats_ = 0;
    float start_r_, start_p_, start_y_, start_ra_, start_la_;
};

int main(int argc, char **argv) {
    std::string target_str = "192.168.88.100:50051";
    if (argc > 1) {
        target_str = argv[1];
        // 如果只输入了 IP，补全端口号
        if (target_str.find(":") == std::string::npos) {
            target_str += ":50051";
        }
    }

    std::cout << "[*] Connecting to Reachy Sim Server at: " << target_str << std::endl;

    ControlClient client(grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));
    client.run();
    return 0;
}
