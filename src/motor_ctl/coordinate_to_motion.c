/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*/

#include "coordinate_to_motion.h"

#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "motor.h"
#include "motor_controller.h"
#include "tracker_utils.h"
#include "vision_wrapper.h"

// ===========================================================================
// 人脸跟随角度计算 — 相机归一化焦距 (基于 HFOV=102°, VFOV=67°)
// ===========================================================================
#define FOCAL_X 0.8098f
#define FOCAL_Y 1.5109f

// 时间域参数（默认回退值，如果配置未设置）
#define FOCAL_X 0.8098f
#define FOCAL_Y 1.5109f

// 外推参数（无人脸时保持惯性）
#define EXTRAPOLATION_TIMEOUT_MS 100  // 超过 100ms 无坐标则停止外推

// Sigmoid 平滑死区函数
static float sigmoid_smooth(float error, float zone)
{
    if (fabsf(error) < zone * 0.3f)
    {
        // 死区中心：完全抑制
        return 0.0f;
    }
    float normalized = error / zone;
    // 使用适度的 sigmoid 曲线
    float sigmoid = 1.0f / (1.0f + expf(-3.5f * (fabsf(normalized) - 1.0f)));
    return error * sigmoid;
}

void motion_ctl_init(MotionController *mc, const TrackerConfig *config,
                    struct motor_dev **devs, int count,
                    void *async_controller)
{
    memset(mc, 0, sizeof(MotionController));

    // 复制配置
    if (config)
    {
        mc->config = *config;
    }

    mc->devs = devs;
    mc->motor_count = count;
    mc->async_controller = async_controller;

    uint64_t now = get_time_us();
    mc->state.last_update_us = now;
    mc->state.last_valid_us = now;

    printf("[motion_ctl] 初始化完成，参数化配置已加载\n");

    // 发送初始归零指令
    if (mc->async_controller)
    {
        AsyncMotorController *ctrl = (AsyncMotorController *)mc->async_controller;
        async_motor_controller_set_target(ctrl, 0, 0, 0, 0, 0, 0);
    }
}

void motion_ctl_update(MotionController *mc, float u, float v, bool valid)
{
    uint64_t now_us = get_time_us();
    float dt = (now_us - mc->state.last_update_us) / 1000000.0f;
    mc->state.last_update_us = now_us;

    // 限制 dt 范围：至少 10ms，最多 200ms
    if (dt < 0.01f)
        dt = 0.01f;
    if (dt > 0.2f)
        dt = 0.2f;

    if (!valid)
    {
        // 目标丢失：检查是否需要外推
        uint64_t time_since_last_valid = now_us - mc->state.last_valid_us;
        if (time_since_last_valid < EXTRAPOLATION_TIMEOUT_MS * 1000ULL)
        {
            // 保持惯性（按上次速度继续移动，但逐渐衰减）
            float decay =
                1.0f -
                (time_since_last_valid / (EXTRAPOLATION_TIMEOUT_MS * 1000.0f)) * 0.5f;
            mc->state.current_h_deg += mc->state.velocity_h * dt * decay;
            mc->state.current_v_deg += mc->state.velocity_v * dt * decay;

            // 硬限位
            if (mc->state.current_h_deg > 170.0f)
                mc->state.current_h_deg = 170.0f;
            if (mc->state.current_h_deg < -170.0f)
                mc->state.current_h_deg = -170.0f;
            if (mc->state.current_v_deg > 35.0f)
                mc->state.current_v_deg = 35.0f;
            if (mc->state.current_v_deg < -35.0f)
                mc->state.current_v_deg = -35.0f;

            // 发送电机指令
            if (mc->async_controller)
            {
                AsyncMotorController *ctrl =
                    (AsyncMotorController *)mc->async_controller;
                async_motor_controller_set_target(ctrl, 0, mc->state.current_v_deg,
                                                    mc->state.current_h_deg, 0, 0, 0);
            }
        } else {
            // 超时：停止外推，状态归零或衰减
            mc->state.frame_count = 0;
            mc->state.velocity_h = 0.0f;
            mc->state.velocity_v = 0.0f;
            mc->state.filtered_error_h *= 0.82f;
            mc->state.filtered_error_v *= 0.82f;
        }
        return;
    }

    // 有效目标：执行控制算法
    mc->state.frame_count++;
    mc->state.last_valid_us = now_us;

    // 1. Sigmoid 平滑死区
    float u_eff = sigmoid_smooth(u, mc->config.dead_zone);
    float v_eff = sigmoid_smooth(v, mc->config.dead_zone);

    // 2. 极小偏差判断
    const float MIN_ERROR_THRESHOLD = 0.018f;
    if (fabsf(u_eff) < MIN_ERROR_THRESHOLD &&
        fabsf(v_eff) < MIN_ERROR_THRESHOLD)
    {
        mc->state.filtered_error_h *= 0.82f;
        mc->state.filtered_error_v *= 0.82f;
        return;
    }

    // 3. 计算目标角度误差
    float target_delta_h = -atan2f(u_eff, FOCAL_X) * 180.0f / M_PI;
    float target_delta_v = atan2f(v_eff, FOCAL_Y) * 180.0f / M_PI;

    // 4. EMA 滤波
    float alpha_h = dt / (mc->config.ema_tc_h + dt);
    float alpha_v = dt / (mc->config.ema_tc_v + dt);
    mc->state.filtered_error_h =
        alpha_h * target_delta_h + (1.0f - alpha_h) * mc->state.filtered_error_h;
    mc->state.filtered_error_v =
        alpha_v * target_delta_v + (1.0f - alpha_v) * mc->state.filtered_error_v;

    // 5. 自适应增益：根据误差模长调整 Kp (weight)
    float error_deg =
        sqrtf(mc->state.filtered_error_h * mc->state.filtered_error_h +
                mc->state.filtered_error_v * mc->state.filtered_error_v);

    float weight_per_sec;
    if (error_deg > 10.0f)
    {
        weight_per_sec = mc->config.weight_max;
    } else if (error_deg > 2.0f) {
        float ratio = (error_deg - 2.0f) / 8.0f;
        weight_per_sec = mc->config.weight_base +
                        ratio * (mc->config.weight_max - mc->config.weight_base);
    } else {
        weight_per_sec = mc->config.weight_base;
    }

    // 起始抑制
    if (mc->state.frame_count < 3)
    {
        weight_per_sec *= 0.85f;
    }

    // 6. 导数项 (D)
    float delta_err_h =
        (mc->state.filtered_error_h - mc->state.prev_filtered_error_h);
    float delta_err_v =
        (mc->state.filtered_error_v - mc->state.prev_filtered_error_v);
    mc->state.prev_filtered_error_h = mc->state.filtered_error_h;
    mc->state.prev_filtered_error_v = mc->state.filtered_error_v;

    float adaptive_weight = weight_per_sec * dt;

    // 7. 物理状态同步
    float actual_h = mc->state.current_h_deg;
    float actual_v = mc->state.current_v_deg;

    if (mc->async_controller)
    {
        AsyncMotorController *ctrl = (AsyncMotorController *)mc->async_controller;
        actual_h = async_motor_controller_get_actual_yaw(ctrl);
        actual_v = async_motor_controller_get_actual_pitch(ctrl);
    }

    // 估算实时速度
    if (dt > 0.001f)
    {
        mc->state.velocity_h = (actual_h - mc->prev_yaw) / dt;
        mc->state.velocity_v = (actual_v - mc->prev_pitch) / dt;
    }
    mc->prev_yaw = actual_h;
    mc->prev_pitch = actual_v;

    // 8. PD 控制输出
    float new_h =
        mc->state.current_h_deg + (adaptive_weight * mc->state.filtered_error_h +
                                    mc->config.k_d * delta_err_h);
    float new_v =
        mc->state.current_v_deg +
        mc->config.pitch_scale * (adaptive_weight * mc->state.filtered_error_v +
                                    mc->config.k_d * delta_err_v);

    // 9. 指令-反馈超前限制
    float lead_h = new_h - actual_h;
    float lead_v = new_v - actual_v;
    if (fabsf(lead_h) > mc->config.max_command_lead)
        new_h = actual_h + (lead_h > 0 ? 1 : -1) * mc->config.max_command_lead;
    if (fabsf(lead_v) > mc->config.max_command_lead)
        new_v = actual_v + (lead_v > 0 ? 1 : -1) * mc->config.max_command_lead;

    // 10. 高级动态限制：速度刹车与过冲阻尼
    float error_sign_h = (mc->state.filtered_error_h > 0) ? 1.0f : -1.0f;
    float error_sign_v = (mc->state.filtered_error_v > 0) ? 1.0f : -1.0f;

    // 速度刹车（防止失控）
    const float BRAKE_THRESHOLD = 35.0f;
    if (fabsf(mc->state.velocity_h) > BRAKE_THRESHOLD)
    {
        float brake = 1.0f - 0.4f *
                            (fabsf(mc->state.velocity_h) - BRAKE_THRESHOLD) /
                            (mc->config.max_angle_speed - BRAKE_THRESHOLD);
        new_h = actual_h + (new_h - actual_h) * (brake < 0.4f ? 0.4f : brake);
    }
    if (fabsf(mc->state.velocity_v) > BRAKE_THRESHOLD)
    {
        float brake = 1.0f - 0.4f *
                            (fabsf(mc->state.velocity_v) - BRAKE_THRESHOLD) /
                            (mc->config.max_angle_speed - BRAKE_THRESHOLD);
        new_v = actual_v + (new_v - actual_v) * (brake < 0.4f ? 0.4f : brake);
    }

    // 过冲阻尼（震荡抑制）
    if (mc->state.velocity_h * error_sign_h < -mc->config.overshoot_threshold)
    {
        float damping = 1.0f - 0.55f * (fabsf(mc->state.velocity_h) /
                                        mc->config.max_angle_speed);
        new_h = actual_h + (new_h - actual_h) * (damping < 0.25f ? 0.25f : damping);
    }
    if (mc->state.velocity_v * error_sign_v < -mc->config.overshoot_threshold)
    {
        float damping = 1.0f - 0.55f * (fabsf(mc->state.velocity_v) /
                                        mc->config.max_angle_speed);
        new_v = actual_v + (new_v - actual_v) * (damping < 0.25f ? 0.25f : damping);
    }

    // 11. 终极速度限制
    float max_change = mc->config.max_angle_speed * dt;
    float diff_h = new_h - mc->state.current_h_deg;
    float diff_v = new_v - mc->state.current_v_deg;

    if (fabsf(diff_h) > max_change)
        diff_h = (diff_h > 0 ? max_change : -max_change);
    if (fabsf(diff_v) > max_change)
        diff_v = (diff_v > 0 ? max_change : -max_change);

    mc->state.current_h_deg += diff_h;
    mc->state.current_v_deg += diff_v;

    // 12. 硬限位封装
    if (mc->state.current_h_deg > 170.0f)
        mc->state.current_h_deg = 170.0f;
    if (mc->state.current_h_deg < -170.0f)
        mc->state.current_h_deg = -170.0f;
    if (mc->state.current_v_deg > 35.0f)
        mc->state.current_v_deg = 35.0f;
    if (mc->state.current_v_deg < -35.0f)
        mc->state.current_v_deg = -35.0f;

    // 13. 指令下发
    if (mc->async_controller)
    {
        AsyncMotorController *ctrl = (AsyncMotorController *)mc->async_controller;
        async_motor_controller_set_target(ctrl, 0, mc->state.current_v_deg,
                                            mc->state.current_h_deg, 0, 0, 0);
    }
}

void motion_ctl_reset(MotionController *mc)
{
    mc->state.current_h_deg = 0.0f;
    mc->state.current_v_deg = 0.0f;
    mc->state.filtered_error_h = 0.0f;
    mc->state.filtered_error_v = 0.0f;
    mc->state.prev_filtered_error_h = 0.0f;
    mc->state.prev_filtered_error_v = 0.0f;
    mc->prev_filtered_pitch = 0.0f;
    mc->state.velocity_h = 0.0f;
    mc->state.velocity_v = 0.0f;
    mc->prev_yaw = 0.0f;
    mc->prev_pitch = 0.0f;
    mc->state.frame_count = 0;
    mc->state.lost_frame_count = 0;
    uint64_t now = get_time_us();
    mc->state.last_update_us = now;
    mc->state.last_valid_us = now;
    printf("[motion_ctl] 状态已重置\n");
}

void motion_ctl_destroy(MotionController *mc)
{
    // 当前实现无需释放动态资源
    printf("[motion_ctl] 引擎已销毁\n");
}
