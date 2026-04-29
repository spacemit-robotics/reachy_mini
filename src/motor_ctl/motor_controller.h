/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Async motor control - 异步电机控制
 * 在独立线程中处理电机运动，避免阻塞视频帧处理
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef __cplusplus
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#endif

#include "motion_api.h"
#include "motor.h"

#ifdef __cplusplus
class AsyncMotorController {
    public:
    enum class TargetMode { RPY, JOINTS };

    struct MotionTarget {
        TargetMode mode = TargetMode::RPY;
        // RPY Mode
        float roll_deg = 0.0f;
        float pitch_deg = 0.0f;
        float yaw_deg = 0.0f;
        float body_yaw_deg = 0.0f;

        // Joints Mode (弧度)
        float joints[7] = {0.0f};

        // Common
        float ant_r_deg = 0.0f;
        float ant_l_deg = 0.0f;
        int64_t timestamp_us = 0;
    };

    AsyncMotorController(struct motor_dev **devs, uint32_t count);
    ~AsyncMotorController();

    // 发送 RPY 目标角度到异步队列（非阻塞）
    void set_target(float roll_deg, float pitch_deg, float yaw_deg,
                    float body_yaw_deg = 0.0f, float ant_r_deg = 0.0f,
                    float ant_l_deg = 0.0f);

    // 发送关节空间目标（弧度）到异步队列（非阻塞）
    void set_joint_targets(const float joints[7], float ant_r_deg = 0.0f, float ant_l_deg = 0.0f);

    // 启动异步处理线程
    bool start();

    // 停止异步处理线程
    void stop();

    // 同步实际位姿：从电机读取当前位置并更新内部插值状态
    void sync_actual();

    // 动态调节最大角速度限制（度/秒）
    void set_speed_limit(float deg_per_sec);

    // 查询当前状态
    bool is_moving() const;

    // 获取估算的实际角度（度）— 指数衰减模拟物理运动延迟
    float get_actual_yaw() const;
    float get_actual_pitch() const;
    float get_actual_roll() const;
    float get_actual_body() const;

    private:
    void worker_thread();

    struct motor_dev **motor_devs_;
    uint32_t motor_count_;

    std::unique_ptr<std::thread> worker_thread_;
    std::atomic<bool> shutdown_flag_{false};
    std::atomic<bool> is_moving_{false};

    std::mutex target_mutex_;
    std::condition_variable target_cv_;
    MotionTarget current_target_;
    bool target_updated_{false};

    // 指数衰减位置估算（模拟电机物理运动延迟）
    std::atomic<float> estimated_yaw_{0.0f};
    std::atomic<float> estimated_pitch_{0.0f};
    std::atomic<float> estimated_roll_{0.0f};
    std::atomic<float> estimated_body_{0.0f};

    // 内部插值状态（Smooth Push）
    float internal_roll_{0.0f};
    float internal_pitch_{0.0f};
    float internal_yaw_{0.0f};
    float internal_body_{0.0f};
    float internal_ant_r_{0.0f};
    float internal_ant_l_{0.0f};

    // 平滑后的估算位置（供外部读取，作为跟随基准）
    std::atomic<float> smoothed_yaw_{0.0f};
    std::atomic<float> smoothed_pitch_{0.0f};
    std::atomic<float> smoothed_roll_{0.0f};
    std::atomic<float> smoothed_body_{0.0f};

    // 运行参数：动态步进限制
    std::atomic<float> max_step_per_tick_{0.5f};

    // 运动参数常量
    static constexpr int MIN_COMMAND_INTERVAL_MS =
        10;                                           // 最小命令间隔 10ms (~100Hz)
    static constexpr float DECAY_RATE = 0.90f;        // 大幅减小模拟延迟 (0.35 -> 0.90)
    static constexpr float FEEDBACK_SMOOTHING = 0.90f;  // 提高同步速率 (0.5 -> 0.90)
};
#else
typedef struct AsyncMotorController AsyncMotorController;
#endif

// ============================================================================
// C Interface for AsyncMotorController (for use in C code)
// ============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

    // Create and destroy motor controller
    AsyncMotorController *async_motor_controller_create(struct motor_dev **devs,
                                                        uint32_t count);
    void async_motor_controller_destroy(AsyncMotorController *controller);

    // Sync actual position from motors
    void async_motor_controller_sync_actual(AsyncMotorController *controller);

    // Start and stop motor controller
    bool async_motor_controller_start(AsyncMotorController *controller);
    void async_motor_controller_stop(AsyncMotorController *controller);

    // Set target position (RPY)
    void async_motor_controller_set_target(AsyncMotorController *controller,
                                            float roll_deg, float pitch_deg,
                                            float yaw_deg, float body_yaw_deg,
                                            float ant_r_deg, float ant_l_deg);

    // Set joint targets (radians)
    void async_motor_controller_set_joint_targets(AsyncMotorController *controller,
            const float joints[7], float ant_r_deg,
            float ant_l_deg);

    // Set speed limit (degrees per second)
    void async_motor_controller_set_speed_limit(AsyncMotorController *controller, float deg_per_sec);

    // Get actual position (smoothed feedback)
    float async_motor_controller_get_actual_yaw(AsyncMotorController *controller);
    float async_motor_controller_get_actual_pitch(AsyncMotorController *controller);
    float async_motor_controller_get_actual_roll(AsyncMotorController *controller);
    float async_motor_controller_get_actual_body(AsyncMotorController *controller);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROLLER_H
