/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef DANCE_INTERFACE_H
#define DANCE_INTERFACE_H

#include "motor_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Headbanger Combo - 摇头摆脑组合
 *
 * @param ctrl 电机控制器句柄
 * @param cycles 循环次数
 * @param bpm 每分钟节拍数 (Beats Per Minute)
 */
void dance_move_headbanger(AsyncMotorController *ctrl, int cycles, float bpm);

/**
 * @brief Jackson Square - 杰克逊方块步
 *
 * @param ctrl 电机控制器句柄
 * @param cycles 循环次数
 * @param bpm 每分钟节拍数
 */
void dance_move_jackson_square(AsyncMotorController *ctrl, int cycles, float bpm);

/**
 * @brief Chicken Peck - 小鸡啄米
 *
 * @param ctrl 电机控制器句柄
 * @param cycles 循环次数
 * @param bpm 每分钟节拍数
 */
void dance_move_chicken_peck(AsyncMotorController *ctrl, int cycles, float bpm);

/**
 * @brief Uh-huh Tilt - 嗯哼歪头
 *
 * @param ctrl 电机控制器句柄
 * @param cycles 循环次数
 * @param bpm 每分钟节拍数
 */
void dance_move_uh_huh_tilt(AsyncMotorController *ctrl, int cycles, float bpm);

/**
 * @brief 请求停止当前舞蹈动作
 */
void dance_request_stop(void);

/**
 * @brief 检查是否已请求停止
 *
 * @return int 1 已请求，0 未请求
 */
int dance_should_stop(void);

#ifdef __cplusplus
}
#endif

#endif  // DANCE_INTERFACE_H
