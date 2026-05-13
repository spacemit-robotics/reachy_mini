/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef VOICE_CTL_H
#define VOICE_CTL_H

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
class AsyncMotorController;
#else
typedef struct AsyncMotorController AsyncMotorController;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// 定义状态常量
#define ACTION_NONE -1
#define ACTION_LIMIT_EXCEEDED -2

/**
 * @brief 初始化语音运动控制模块及底层电机
 *
 * @param serial_port 串口设备路径，如 "/dev/ttyACM0"
 * @param default_delay 初始化时的延迟时间（备用）
 * @return int 0 成功，<0 失败
 */
int voice_ctl_init(const char *serial_port, float default_delay);

/**
 * @brief 在 ASR 输出的文本中进行模糊匹配，查找意图
 *
 * @param text ASR 完整文本结果
 * @param out_keyword [输出] 匹配到的关键字缓冲区 (可选，传 NULL 则忽略)
 * @param max_len 缓冲区最大长度
 * @return int 匹配到的动作 ID。如果未匹配到有效运动意图，则返回 ACTION_NONE
 */
int voice_ctl_match(const char *text, char *out_keyword, size_t max_len);

/**
 * @brief 执行指定的动作 ID
 *
 * @param action_id 由 voice_ctl_match 返回的有效动作 ID
 * @return int 0 成功
 */
int voice_ctl_execute(int action_id);

/**
 * @brief 获取当前正在使用的异步电机控制器句柄
 *
 * @return AsyncMotorController* 控制器句柄，未初始化则返回 NULL
 */
AsyncMotorController *voice_ctl_get_controller(void);

/**
 * @brief 清理底层电机资源并回正
 */
void voice_ctl_cleanup(void);

/**
 * @brief 设置人脸跟踪使用的相机 ID
 *
 * @param camera_id 相机设备 ID (默认: 0)
 */
void voice_ctl_set_camera_id(int camera_id);

/**
 * @brief 暂停所有运行中的 tracker (释放 NPU 给语音链路使用)
 */
void voice_ctl_tracker_pause_all(void);

/**
 * @brief 恢复所有运行中的 tracker
 */
void voice_ctl_tracker_resume_all(void);

/**
 * @brief 检查是否有任何 tracker 正在运行
 *
 * @return bool true 表示正在运行
 */
bool voice_ctl_tracker_any_running(void);

#ifdef __cplusplus
}
#endif

#endif  // VOICE_CTL_H
