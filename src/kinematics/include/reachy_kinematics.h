#ifndef REACHY_KINEMATICS_H
#define REACHY_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计算 Reachy Mini 逆运动学 (IK)
 *
 * @param head_pose_world 头部相对于世界坐标系的 4x4 矩阵 (行优先)
 * @param out_joints 输出 7 个关节角度 (弧度): [身体旋转, Stewart关节1...6]
 * @return 成功返回 0，超出硬件限位返回 -1
 */
int reachy_calculate_ik(const float head_pose_world[16], float out_joints[7]);

/**
 * @brief 计算 Reachy Mini 正运动学 (FK)
 *
 * @param joints 7 个关节角度 (弧度): [body_yaw, stewart_1...6]
 * @param out_pose 输出的 4x4 位姿矩阵 (行优先)
 * @param num_iterations 牛顿迭代次数
 * @return 0 成功, -1 失败
 */
int reachy_calculate_fk(const float joints[7], float out_pose[16], int num_iterations);

/**
 * @brief 重置 FK 迭代器的内部状态
 *
 * @param initial_pose 可为 NULL
 */
void reachy_reset_fk(const float *initial_pose);

#ifdef __cplusplus
}
#endif

#endif  // REACHY_KINEMATICS_H
