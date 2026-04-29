/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef COORDINATE_TO_MOTION_H
#define COORDINATE_TO_MOTION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "tracker_common.h"

    struct motor_dev;

    /**
    * 运动控制器状态结构
    */
    typedef struct
    {
        TrackerConfig config;  // 控制配置（Kp, Kd, DeadZone 等）
        TrackerState state;    // 运行时状态（误差、速度、时间戳等）

        // 电机设备引用
        struct motor_dev **devs;
        int motor_count;

        // 异步电机控制器（如果启用）
        void *async_controller;  // AsyncMotorController*

        // 内部辅助状态
        float prev_yaw;
        float prev_pitch;
        float prev_filtered_pitch;
    } MotionController;

    /**
    * 初始化运动控制器
    * @param mc 控制器结构指针
    * @param config 配置对象
    * @param devs 电机设备数组
    * @param count 电机数量
    * @param async_controller 异步电机控制器指针（可选，传 NULL 则直接控制）
    */
    void motion_ctl_init(MotionController *mc, const TrackerConfig *config,
                        struct motor_dev **devs, int count,
                        void *async_controller);

    /**
    * 更新运动控制（同步调用，每帧调用一次）
    * @param mc 控制器结构指针
    * @param u 归一化水平偏差 [-1, 1]，负值表示目标在左侧
    * @param v 归一化垂直偏差 [-1, 1]，负值表示目标在下方
    * @param valid 坐标是否有效（检测到人脸）
    */
    void motion_ctl_update(MotionController *mc, float u, float v, bool valid);

    /**
    * 重置运动控制器状态
    */
    void motion_ctl_reset(MotionController *mc);

    /**
    * 销毁运动控制器（释放资源）
    */
    void motion_ctl_destroy(MotionController *mc);

#ifdef __cplusplus
}
#endif

#endif  // COORDINATE_TO_MOTION_H
