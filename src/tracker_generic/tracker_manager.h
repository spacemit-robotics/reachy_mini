/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TRACKER_MANAGER_H
#define TRACKER_MANAGER_H

#include <stdbool.h>
#include <sys/types.h>
#include "motor_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRACKER_TYPE_FACE = 0,
    TRACKER_TYPE_GESTURE,
    TRACKER_TYPE_MAX
} TrackerType;

/**
 * @brief 初始化 tracker 管理器
 * @param camera_id 相机 ID
 * @param motor_ctrl 异步电机控制器句柄 (用于切换控制权)
 * @return 0 成功, <0 失败
 */
int tracker_manager_init(int camera_id, AsyncMotorController *motor_ctrl);

/**
 * @brief 启动指定类型的 tracker
 * @param type tracker 类型
 * @return 0 成功, <0 失败
 */
int tracker_start(TrackerType type);

/**
 * @brief 停止指定类型的 tracker
 * @param type tracker 类型
 * @return 0 成功, <0 失败
 */
int tracker_stop(TrackerType type);

/**
 * @brief 停止所有正在运行的 tracker
 */
void tracker_stop_all(void);

/**
 * @brief 暂停所有运行中的 tracker (释放 NPU)
 */
void tracker_pause_all(void);

/**
 * @brief 恢复所有运行中的 tracker
 */
void tracker_resume_all(void);

/**
 * @brief 检查指定 tracker 是否正在运行
 */
bool tracker_is_running(TrackerType type);

#ifdef __cplusplus
}
#endif

#endif  // TRACKER_MANAGER_H
