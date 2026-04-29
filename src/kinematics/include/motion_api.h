#ifndef MOTION_API_H
#define MOTION_API_H

#include <stdbool.h>

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

// ========== 基础运动控制（多维度） ==========
// 所有角度输入单位均为度 (Degrees)
int motion_move_to(struct motor_dev **devs, float roll_deg, float pitch_deg,
                    float yaw_deg, float body_yaw_deg, float ant_r_deg,
                    float ant_l_deg);

// 非阻塞驱动接口（仅计算一次 IK 并直接发指令，无内部平滑循环）
int motion_move_to_async(struct motor_dev **devs, float roll_deg,
                    float pitch_deg, float yaw_deg, float body_yaw_deg,
                    float ant_r_deg, float ant_l_deg);

// ========== 各个动作的封装 API ==========
// 注意：传入的 angle 均为正数（表示该方向的绝对量度）
// 软限位将参考硬件极限，并在内部强制截断过大的输入

// 头部转动（Yaw - 左右转，限位 ±45°）
int head_turn_left(struct motor_dev **devs, float angle);
int head_turn_right(struct motor_dev **devs, float angle);

// 头部俯仰（Pitch - 上下看，限位 ±35°）
int head_look_down(struct motor_dev **devs, float angle);
int head_look_up(struct motor_dev **devs, float angle);

// 头部左右歪（Roll - 限位 ±25°）
int head_tilt_left(struct motor_dev **devs, float angle);
int head_tilt_right(struct motor_dev **devs, float angle);

// 身体左右转（Body Yaw - ID 10，限位 ±180°）
int body_turn_left(struct motor_dev **devs, float angle);
int body_turn_right(struct motor_dev **devs, float angle);

// 天线转动（Antennas - ID 17, 18，限位 ±180°）
int antenna_turn_forward(struct motor_dev **devs, float angle_r, float angle_l);
int antenna_turn_backward(struct motor_dev **devs, float angle_r,
                        float angle_l);

// 所有关节回中
int center_all(struct motor_dev **devs);

// ========== 获取当前姿态（上次成功执行的目标值，度） ==========
float motion_get_current_roll(void);
float motion_get_current_pitch(void);
float motion_get_current_yaw(void);

#ifdef __cplusplus
}
#endif

#endif  // MOTION_API_H
