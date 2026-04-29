/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * tracker_common.h — 机器人视觉跟踪框架公共类型定义
 */

#ifndef TRACKER_COMMON_H
#define TRACKER_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * 跟随器控制配置
     * 包含 PD 控制器、滤波器和安全限制的所有超参数
     */
    typedef struct
    {
        // PD 控制项
        float weight_base;     // 基础歩进权重 (Kp init)
        float weight_max;      // 最大歩进上限 (Kp limit)
        float weight_per_sec;  // 权重随时间增长速率
        float k_d;             // 微分阻尼系数

        // 滤波器配置
        float ema_tc_h;  // 水平 (Yaw) EMA 滤波时间常数 (s)
        float ema_tc_v;  // 垂直 (Pitch) EMA 滤波时间常数 (s)

        // 阈值与死区
        float dead_zone;            // 归一化偏差死区 (0.0 ~ 1.0)
        float overshoot_threshold;  // 触发阻尼的角速度阈值
        float max_command_lead;     // 指令超前实际反馈的最大容忍度 (度)

        // 轴缩放
        float pitch_scale;      // 垂直轴增益缩放 (用于抵消大惯性)
        float max_angle_speed;  // 最大限制角速度 (度/秒)
    } TrackerConfig;

    /**
     * 实时目标坐标
     */
    typedef struct
    {
        float u;     // 归一化水平坐标 [-1.0, 1.0]
        float v;     // 归一化垂直坐标 [-1.0, 1.0]
        bool valid;  // 目标当前是否可见
    } TrackerTarget;

    /**
     * 运行状态跟踪器
     * 存储由于滤波、导数计算等产生的状态变量
     */
    typedef struct
    {
        // 滤波后的误差信号
        float filtered_error_h;
        float filtered_error_v;
        float prev_filtered_error_h;
        float prev_filtered_error_v;

        // 物理状态反馈
        float current_h_deg;  // 当前偏航角 (绝对)
        float current_v_deg;  // 当前俯仰角 (绝对)
        float velocity_h;     // 实时计算角速度
        float velocity_v;

        // 时间戳管理
        uint64_t last_update_us;
        uint64_t last_valid_us;

        // 状态计数
        int frame_count;
        int lost_frame_count;
    } TrackerState;

#ifdef __cplusplus
}
#endif

#endif  // TRACKER_COMMON_H
