/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*/

#include "motor_controller.h"
extern "C" {
#include <motor.h>
}

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 声明外部 C 运动学函数
extern "C" {
void reachy_reset_fk(const float *initial_pose);
int reachy_calculate_fk(const float joints[7], float out_pose[16], int num_iterations);
int motion_move_to_async(struct motor_dev **devs, float roll_deg, float pitch_deg, float yaw_deg,
                        float body_yaw_deg, float ant_r_deg, float ant_l_deg);
}

AsyncMotorController::AsyncMotorController(struct motor_dev **devs,
                                            uint32_t count)
    : motor_devs_(devs), motor_count_(count)
{
    // 初始化估算位姿为 0
    estimated_yaw_.store(0.0f);
    estimated_pitch_.store(0.0f);
    estimated_roll_.store(0.0f);
    estimated_body_.store(0.0f);
    smoothed_yaw_.store(0.0f);
    smoothed_pitch_.store(0.0f);
    smoothed_roll_.store(0.0f);
    smoothed_body_.store(0.0f);

    // 默认限速：20°/s (0.2°/tick)
    max_step_per_tick_.store(0.2f);
}

AsyncMotorController::~AsyncMotorController() {
    stop();
}

bool AsyncMotorController::start() {
    if (worker_thread_) {
        std::cerr << "Motor controller already running" << std::endl;
        return false;
    }

    shutdown_flag_ = false;
    worker_thread_ = std::make_unique<std::thread>([this] { worker_thread(); });

    std::cout << "Async motor controller started (non-blocking, interval: "
                << MIN_COMMAND_INTERVAL_MS << "ms)" << std::endl;
    return true;
}

void AsyncMotorController::stop() {
    if (!worker_thread_)
        return;

    shutdown_flag_ = true;
    target_cv_.notify_one();

    if (worker_thread_->joinable())
        worker_thread_->join();

    worker_thread_.reset();
    std::cout << "Async motor controller stopped" << std::endl;
}
void AsyncMotorController::sync_actual() {
    struct motor_state states[9];
    if (motor_get_states(motor_devs_, states, motor_count_) == 0) {
        // 提取关节角度
        float joints[7];
        // joints[0] 是 Body Yaw (ID 10)
        // joints[1..6] 是 Head Stewart (ID 11-16)
        std::cout << "[AsyncMotorCtrl] Raw motor angles (rad): ";
        for (int i = 0; i < motor_count_ && i < 9; i++) {
            joints[i] = states[i].pos;
            std::cout << "ID" << (10 + i) << "=" << joints[i] << " ";
        }
        std::cout << std::endl;
        // 调用 FK 计算当前的位姿
        float fk_pose[16];
        reachy_reset_fk(NULL);
        if (reachy_calculate_fk(joints, fk_pose, 10) == 0) {
            // 从位姿矩阵提取 RPY (参考 test_motion.c)
            float sp = -fk_pose[8];
            if (sp > 1.0f)
                sp = 1.0f;
            if (sp < -1.0f)
                sp = -1.0f;
            float pitch = asinf(sp);
            float cp = cosf(pitch);
            float roll = 0.0f, yaw = 0.0f;
            if (std::abs(cp) > 1e-6f) {
                roll = atan2f(fk_pose[9], fk_pose[10]);
                yaw = atan2f(fk_pose[4], fk_pose[0]);
            }

            // 物理限位保护阈值 (度)
            const float SYNC_THRESHOLD = 30.0f;
            internal_roll_ = roll * 180.0f / static_cast<float>(M_PI);
            internal_pitch_ = pitch * 180.0f / static_cast<float>(M_PI);
            internal_yaw_ = yaw * 180.0f / static_cast<float>(M_PI);

            // 检查计算出的 RPY 是否过于离谱（可能由于零位未校准）
            if (std::abs(internal_roll_) > SYNC_THRESHOLD ||
                std::abs(internal_pitch_) > SYNC_THRESHOLD ||
                std::abs(internal_yaw_) > SYNC_THRESHOLD) {
                std::cerr << "[AsyncMotorCtrl] WARNING: Sync pose extreme (R=" << internal_roll_
                            << " P=" << internal_pitch_ << " Y=" << internal_yaw_
                            << "), likely uncalibrated. Forcing 0 for safety." << std::endl;
                internal_roll_ = 0.0f;
                internal_pitch_ = 0.0f;
                internal_yaw_ = 0.0f;
            }

            internal_body_ = joints[0] * 180.0f / static_cast<float>(M_PI);
            if (std::abs(internal_body_) > SYNC_THRESHOLD) {
                internal_body_ = 0.0f;
            }

            // 天线简单同步
            if (motor_count_ >= 9) {
                internal_ant_r_ = states[7].pos * 180.0f / static_cast<float>(M_PI);
                internal_ant_l_ = states[8].pos * 180.0f / static_cast<float>(M_PI);
            }

            // 同步估算状态防止突变
            estimated_roll_.store(internal_roll_);
            estimated_pitch_.store(internal_pitch_);
            estimated_yaw_.store(internal_yaw_);
            smoothed_roll_.store(internal_roll_);
            smoothed_pitch_.store(internal_pitch_);
            smoothed_yaw_.store(internal_yaw_);
            smoothed_body_.store(internal_body_);

            // 关键点：将内部目标直接同步为实测位姿，确保启动瞬间 dy=0，不会发生任何偏转
            {
                std::lock_guard<std::mutex> lock(target_mutex_);
                current_target_.roll_deg = internal_roll_;
                current_target_.pitch_deg = internal_pitch_;
                current_target_.yaw_deg = internal_yaw_;
                current_target_.body_yaw_deg = internal_body_;
                current_target_.ant_r_deg = internal_ant_r_;
                current_target_.ant_l_deg = internal_ant_l_;
            }

            std::cout << "[AsyncMotorCtrl] Physical pose synced & target locked: R="
                        << internal_roll_ << " P=" << internal_pitch_ << " Y=" << internal_yaw_
                        << " B=" << internal_body_ << std::endl;
        }
    } else {
        std::cerr << "[AsyncMotorCtrl] Failed to get motor states for sync" << std::endl;
    }
}

void AsyncMotorController::set_target(float roll_deg, float pitch_deg,
                                        float yaw_deg, float body_yaw_deg,
                                        float ant_r_deg, float ant_l_deg)
{
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        current_target_.mode = TargetMode::RPY;
        current_target_.roll_deg = roll_deg;
        current_target_.pitch_deg = pitch_deg;
        current_target_.yaw_deg = yaw_deg;
        current_target_.body_yaw_deg = body_yaw_deg;
        current_target_.ant_r_deg = ant_r_deg;
        current_target_.ant_l_deg = ant_l_deg;
        target_updated_ = true;
    }
    target_cv_.notify_one();
}

void AsyncMotorController::set_joint_targets(const float joints[7], float ant_r_deg,
                                            float ant_l_deg) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    current_target_.mode = TargetMode::JOINTS;
    for (int i = 0; i < 7; i++) {
        current_target_.joints[i] = joints[i];
    }
    current_target_.ant_r_deg = ant_r_deg;
    current_target_.ant_l_deg = ant_l_deg;
    target_updated_ = true;
    target_cv_.notify_one();
}

void AsyncMotorController::set_speed_limit(float deg_per_sec) {
    // 将 deg/s 转换为 deg/tick (假设每 tick 是 MIN_COMMAND_INTERVAL_MS = 10ms)
    float step_per_tick = deg_per_sec * static_cast<float>(MIN_COMMAND_INTERVAL_MS) / 1000.0f;
    max_step_per_tick_.store(step_per_tick);
    std::cout << "[AsyncMotorCtrl] Speed limit updated: " << deg_per_sec
                << " deg/s (step=" << step_per_tick << ")" << std::endl;
}

bool AsyncMotorController::is_moving() const { return is_moving_.load(); }

// 返回估算的实际位置（经二次平滑处理）
float AsyncMotorController::get_actual_yaw() const
{
    return smoothed_yaw_.load();
}
float AsyncMotorController::get_actual_pitch() const
{
    return smoothed_pitch_.load();
}
float AsyncMotorController::get_actual_roll() const
{
    return smoothed_roll_.load();
}

float AsyncMotorController::get_actual_body() const
{
    return smoothed_body_.load();
}
void AsyncMotorController::worker_thread() {
    while (!shutdown_flag_) {
        // 等待 10ms 进行一次插值步进
        std::unique_lock<std::mutex> lock(target_mutex_);
        target_cv_.wait_for(lock, std::chrono::milliseconds(MIN_COMMAND_INTERVAL_MS));

        if (shutdown_flag_)
            break;

        // 复制当前全局目标 (无论是否在本帧更新)
        MotionTarget target = current_target_;
        lock.unlock();

        if (target.mode == TargetMode::RPY) {
            // ---- 1. 计算 RPY 步进平推 (Interpolation) ----
            float dy = target.yaw_deg - internal_yaw_;
            float dp = target.pitch_deg - internal_pitch_;
            float dr = target.roll_deg - internal_roll_;
            float db = target.body_yaw_deg - internal_body_;

            // 限幅单次步幅 (度)
            float max_step = max_step_per_tick_.load();
            internal_yaw_ += std::max(-max_step, std::min(max_step, dy));
            internal_pitch_ += std::max(-max_step, std::min(max_step, dp));
            internal_roll_ += std::max(-max_step, std::min(max_step, dr));
            internal_body_ += std::max(-max_step, std::min(max_step, db));

            // ---- 2. 发送指令 ----
            is_moving_ = true;
            motion_move_to_async(motor_devs_, internal_roll_, internal_pitch_, internal_yaw_,
                                internal_body_, internal_ant_r_, internal_ant_l_);
            is_moving_ = false;
        } else {
            // ---- 1. 直接注入关节空间指令 (忽略 RPY 插值) ----
            // 注意：舞蹈引擎通常以 100Hz 发送，这里直接透传或微小插值
            struct motor_cmd cmds[9];
            memset(cmds, 0, sizeof(cmds));
            for (int i = 0; i < 7; i++) {
                cmds[i].mode = MOTOR_MODE_POS;
                cmds[i].pos_des = target.joints[i];
            }
            // 天线处理
            cmds[7].mode = MOTOR_MODE_POS;
            cmds[7].pos_des = target.ant_r_deg * static_cast<float>(M_PI) / 180.0f;
            cmds[8].mode = MOTOR_MODE_POS;
            cmds[8].pos_des = target.ant_l_deg * static_cast<float>(M_PI) / 180.0f;

            motor_set_cmds(motor_devs_, cmds, 9);
        }

        // 天线平滑（通用）
        float dar = target.ant_r_deg - internal_ant_r_;
        float dal = target.ant_l_deg - internal_ant_l_;
        float max_step = max_step_per_tick_.load();
        internal_ant_r_ += std::max(-max_step, std::min(max_step, dar));
        internal_ant_l_ += std::max(-max_step, std::min(max_step, dal));

        // ---- 3. 更新反馈估算 (以平滑指令为基准进行物理模拟) ----
        float cur_est_y = estimated_yaw_.load();
        float cur_est_p = estimated_pitch_.load();
        float cur_est_r = estimated_roll_.load();
        float cur_est_b = estimated_body_.load();

        // 模拟电机跟随指令的延迟
        estimated_yaw_.store(cur_est_y + DECAY_RATE * (internal_yaw_ - cur_est_y));
        estimated_pitch_.store(cur_est_p +
                                DECAY_RATE * (internal_pitch_ - cur_est_p));
        estimated_roll_.store(cur_est_r +
                                DECAY_RATE * (internal_roll_ - cur_est_r));
        estimated_body_.store(cur_est_b + DECAY_RATE * (internal_body_ - cur_est_b));

        // 对外反馈的二次平滑
        smoothed_yaw_.store(smoothed_yaw_.load() +
                            FEEDBACK_SMOOTHING *
                                (estimated_yaw_.load() - smoothed_yaw_.load()));
        smoothed_pitch_.store(smoothed_pitch_.load() +
                                FEEDBACK_SMOOTHING * (estimated_pitch_.load() -
                                                    smoothed_pitch_.load()));
        smoothed_roll_.store(smoothed_roll_.load() +
                            FEEDBACK_SMOOTHING *
                                (estimated_roll_.load() - smoothed_roll_.load()));
        smoothed_body_.store(smoothed_body_.load() +
                            FEEDBACK_SMOOTHING * (estimated_body_.load() - smoothed_body_.load()));
    }
}

// ============================================================================
// C Interface for motor_controller (for use in C code)
// ============================================================================

extern "C"
{
    AsyncMotorController *async_motor_controller_create(struct motor_dev **devs,
                                                        uint32_t count)
    {
        return new AsyncMotorController(devs, count);
    }

    void async_motor_controller_destroy(AsyncMotorController *controller)
    {
        delete controller;
    }

    bool async_motor_controller_start(AsyncMotorController *controller)
    {
        return controller->start();
    }

    void async_motor_controller_stop(AsyncMotorController *controller)
    {
        controller->stop();
    }

    void async_motor_controller_sync_actual(AsyncMotorController *controller)
    {
        controller->sync_actual();
    }

    void async_motor_controller_set_target(AsyncMotorController *controller,
                                            float roll_deg, float pitch_deg,
                                            float yaw_deg, float body_yaw_deg,
                                            float ant_r_deg, float ant_l_deg)
    {
        controller->set_target(roll_deg, pitch_deg, yaw_deg, body_yaw_deg, ant_r_deg,
                                ant_l_deg);
    }

    void async_motor_controller_set_joint_targets(AsyncMotorController *controller,
                                                    const float joints[7], float ant_r_deg,
                                                    float ant_l_deg)
    {
        controller->set_joint_targets(joints, ant_r_deg, ant_l_deg);
    }

    void async_motor_controller_set_speed_limit(AsyncMotorController *controller, float deg_per_sec)
    {
        controller->set_speed_limit(deg_per_sec);
    }

    float async_motor_controller_get_actual_yaw(AsyncMotorController *controller)
    {
        return controller->get_actual_yaw();
    }

    float async_motor_controller_get_actual_pitch(
        AsyncMotorController *controller)
    {
        return controller->get_actual_pitch();
    }

    float async_motor_controller_get_actual_roll(AsyncMotorController *controller)
    {
        return controller->get_actual_roll();
    }

    float async_motor_controller_get_actual_body(AsyncMotorController *controller)
    {
        return controller->get_actual_body();
    }
}  // extern "C"
