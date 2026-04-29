#include <math.h>
#include <stdio.h>

#include "kinematics_params.h"
#include "reachy_kinematics.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void mat4_to_rpy(const float T[16], float *roll, float *pitch,
                        float *yaw);
static void rpy_to_mat4(float roll, float pitch, float yaw, float tx, float ty,
                                                float tz, float T[16]);

// 4x4 矩阵乘法: C = A * B (均为行优先布局)
static void mat4_mul(const float A[16], const float B[16], float C[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i * 4 + j] = 0;
            for (int k = 0; k < 4; k++) {
                C[i * 4 + j] += A[i * 4 + k] * B[k * 4 + j];
            }
        }
    }
}

// 将角度归一化到 [-PI, PI] 区间
static float wrap_angle(float angle) {
    return angle - (2.0f * (float)M_PI) *
                                            floorf((angle + (float)M_PI) / (2.0f * (float)M_PI));
}

/**
    * @brief 计算 Reachy Mini 逆运动学 (IK)
    *
    * 重新实现以匹配 Rust 版本解算器 (reachy_mini_rust_kinematics)，
    * 使用半角切线代换公式。
    *
    * @param head_pose_world 头部相对于世界坐标系的 4x4 矩阵 (行优先)
    * @param out_joints 输出 7 个关节角度 (弧度): [身体旋转, Stewart关节1...6]
    * @return 成功返回 0，超出硬件限位返回 -1
    */
int reachy_calculate_ik(const float head_pose_world[16], float out_joints[7]) {
    float rs = MOTOR_ARM_LENGTH;
    float rp = ROD_LENGTH;

    // ==== 头身协同 (Automatic Body Yaw) ====
    float r_head, p_head, y_head;
    mat4_to_rpy(head_pose_world, &r_head, &p_head, &y_head);

    float max_relative_yaw = 30.0f * (float)M_PI / 180.0f;
    float max_body_yaw = 160.0f * (float)M_PI / 180.0f;

    float body_yaw = 0.0f;
    if (y_head > max_relative_yaw) {
        body_yaw = y_head - max_relative_yaw;
    } else if (y_head < -max_relative_yaw) {
        body_yaw = y_head + max_relative_yaw;
    }

    if (body_yaw > max_body_yaw)
        body_yaw = max_body_yaw;
    if (body_yaw < -max_body_yaw)
        body_yaw = -max_body_yaw;

    float rel_yaw = y_head - body_yaw;

    float tx = head_pose_world[3];
    float ty = head_pose_world[7];
    float tz = head_pose_world[11];

    float cb = cosf(-body_yaw);
    float sb = sinf(-body_yaw);
    float rel_tx = tx * cb - ty * sb;
    float rel_ty = tx * sb + ty * cb;
    float rel_tz = tz;

    float t_world_platform[16];
    rpy_to_mat4(r_head, p_head, rel_yaw, rel_tx, rel_ty, rel_tz, t_world_platform);

    // 1. 构建带有 Z 轴偏移的头部位姿
    t_world_platform[11] += HEAD_Z_OFFSET;

    // 记录解算出的身体旋转，作为后续关节指令的首个元素
    out_joints[0] = body_yaw;

    // 2. 遍历 6 根 Stewart 连杆
    for (int i = 0; i < 6; i++) {
        // 构建从分支点 branch_p 出发的平移矩阵 T_branch
        float T_branch[16] = {1, 0, 0, STEWART_PARAMS[i].branch_p[0],
                                                    0, 1, 0, STEWART_PARAMS[i].branch_p[1],
                                                    0, 0, 1, STEWART_PARAMS[i].branch_p[2],
                                                    0, 0, 0, 1};

        // 在 Rust 代码中，t_world_motor 是存储在分支中的 T_inv（已求逆）。
        // 然后计算 t_world_motor_inv = t_world_motor.inverse()，
        // 这将返回原始 JSON 中的 T_motor_world。
        //
        // 因此：branch_motor = T_motor_world * t_world_platform * T_branch
        //
        // kinematics_params.h 中的 T_inv 即为原始 JSON 中的 T_motor_world。

        float temp[16], branch_motor[16];
        mat4_mul(t_world_platform, T_branch, temp);
        mat4_mul(STEWART_PARAMS[i].T_inv, temp, branch_motor);

        float px = branch_motor[3];   // 行优先布局下的 (0,3)
        float py = branch_motor[7];   // (1,3)
        float pz = branch_motor[11];  // (2,3)

        // 半角切线代换 IK 公式 (与 Rust 版本一致)
        float px2 = px * px, py2 = py * py, pz2 = pz * pz;
        float rs2 = rs * rs, rp2 = rp * rp;

        float x = px2 + 2.0f * px * rs + py2 + pz2 - rp2 + rs2;

        // 平方根下的判别式
        float disc = -(px2 * px2) - 2.0f * px2 * py2 - 2.0f * px2 * pz2 +
                                    2.0f * px2 * rp2 + 2.0f * px2 * rs2 - (py2 * py2) -
                                    2.0f * py2 * pz2 + 2.0f * py2 * rp2 + 2.0f * py2 * rs2 -
                                    (pz2 * pz2) + 2.0f * pz2 * rp2 - 2.0f * pz2 * rs2 -
                                    (rp2 * rp2) + 2.0f * rp2 * rs2 - (rs2 * rs2);

        if (disc < 0.0f)
            disc = 0.0f;  // 消除数值误差

        // 解选择: JSON solution=0 -> Python 传入 -1, JSON solution=1 -> Python 传入
        // 1
        float sol = STEWART_PARAMS[i].solution ? 1.0f : -1.0f;

        float y = 2.0f * py * rs + sol * sqrtf(disc);

        out_joints[i + 1] = wrap_angle(2.0f * atan2f(y, x));
    }

    // 3. 硬件限位保护
    for (int i = 0; i < 7; i++) {
        if (!isfinite(out_joints[i]) || out_joints[i] < MOTOR_LIMITS[i].lower ||
            out_joints[i] > MOTOR_LIMITS[i].upper) {
            printf("[IK 调试] 关节 %d 超出限位: %f (限位区间: [%f, %f])\n", i, out_joints[i],
                    MOTOR_LIMITS[i].lower, MOTOR_LIMITS[i].upper);
            return -1;
        }
    }

    return 0;
}

// ================= 正运动学 (FK) =================

// FK 内部状态：当前猜测的位姿 (用于迭代的连续性)
static float fk_current_guess[16];
static int fk_initialized = 0;

/**
    * @brief 重置 FK 迭代器的内部状态
    *
    * 将初始猜测设为睡眠姿态或指定的位姿矩阵。
    * 在首次调用 FK 或者关节角度发生大幅跳变后调用。
    *
    * @param initial_pose 可为 NULL（使用默认睡眠姿态）
    */
void reachy_reset_fk(const float *initial_pose) {
    if (initial_pose) {
        for (int i = 0; i < 16; i++)
            fk_current_guess[i] = initial_pose[i];
    } else {
        for (int i = 0; i < 16; i++)
            fk_current_guess[i] = SLEEP_HEAD_POSE[i];
    }
    fk_initialized = 1;
}

// 从旋转矩阵提取 RPY 欧拉角 (XYZ 顺序)
static void mat4_to_rpy(const float T[16], float *roll, float *pitch,
                                                float *yaw) {
    // T[8] = -sin(pitch)
    float sp = -T[8];
    if (sp > 1.0f)
        sp = 1.0f;
    if (sp < -1.0f)
        sp = -1.0f;
    *pitch = asinf(sp);

    float cp = cosf(*pitch);
    if (fabsf(cp) > 1e-6f) {
        *roll = atan2f(T[9], T[10]);  // atan2(cp*sr, cp*cr)
        *yaw = atan2f(T[4], T[0]);    // atan2(sy*cp, cy*cp)
    } else {
        // 万向锁
        *roll = atan2f(-T[6], T[5]);
        *yaw = 0.0f;
    }
}

// 从 RPY + 平移构建 4x4 位姿矩阵
static void rpy_to_mat4(float roll, float pitch, float yaw, float tx, float ty,
                                                float tz, float T[16]) {
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);

    T[0] = cy * cp;
    T[1] = cy * sp * sr - sy * cr;
    T[2] = cy * sp * cr + sy * sr;
    T[3] = tx;
    T[4] = sy * cp;
    T[5] = sy * sp * sr + cy * cr;
    T[6] = sy * sp * cr - cy * sr;
    T[7] = ty;
    T[8] = -sp;
    T[9] = cp * sr;
    T[10] = cp * cr;
    T[11] = tz;
    T[12] = 0.0f;
    T[13] = 0.0f;
    T[14] = 0.0f;
    T[15] = 1.0f;
}

// 6x6 线性方程组求解 (高斯消元法，带部分主元)
static int solve_6x6(float A[6][6], float b[6], float x[6]) {
    // 前向消去
    for (int col = 0; col < 6; col++) {
        // 部分主元选取
        int max_row = col;
        float max_val = fabsf(A[col][col]);
        for (int row = col + 1; row < 6; row++) {
            if (fabsf(A[row][col]) > max_val) {
                max_val = fabsf(A[row][col]);
                max_row = row;
            }
        }
        if (max_val < 1e-12f)
            return -1;  // 奇异矩阵

        // 行交换
        if (max_row != col) {
            for (int j = 0; j < 6; j++) {
                float tmp = A[col][j];
                A[col][j] = A[max_row][j];
                A[max_row][j] = tmp;
            }
            float tmp = b[col];
            b[col] = b[max_row];
            b[max_row] = tmp;
        }

        // 消去
        for (int row = col + 1; row < 6; row++) {
            float factor = A[row][col] / A[col][col];
            for (int j = col; j < 6; j++) {
                A[row][j] -= factor * A[col][j];
            }
            b[row] -= factor * b[col];
        }
    }

    // 回代
    for (int i = 5; i >= 0; i--) {
        x[i] = b[i];
        for (int j = i + 1; j < 6; j++) {
            x[i] -= A[i][j] * x[j];
        }
        x[i] /= A[i][i];
    }
    return 0;
}

// 仅计算 Stewart 6 关节的 IK（不含硬件限位检查，供 FK 内部使用）
static int ik_stewart_only(const float head_pose[16], float body_yaw,
                                                        float out_stewart[6]) {
    float rs = MOTOR_ARM_LENGTH;
    float rp = ROD_LENGTH;

    float t_world_platform[16];
    for (int i = 0; i < 16; i++)
        t_world_platform[i] = head_pose[i];
    t_world_platform[11] += HEAD_Z_OFFSET;

    for (int i = 0; i < 6; i++) {
        float T_branch[16] = {1, 0, 0, STEWART_PARAMS[i].branch_p[0],
                                                    0, 1, 0, STEWART_PARAMS[i].branch_p[1],
                                                    0, 0, 1, STEWART_PARAMS[i].branch_p[2],
                                                    0, 0, 0, 1};

        float temp[16], branch_motor[16];
        mat4_mul(t_world_platform, T_branch, temp);
        mat4_mul(STEWART_PARAMS[i].T_inv, temp, branch_motor);

        float px = branch_motor[3];
        float py = branch_motor[7];
        float pz = branch_motor[11];

        float px2 = px * px, py2 = py * py, pz2 = pz * pz;
        float rs2 = rs * rs, rp2 = rp * rp;

        float x = px2 + 2.0f * px * rs + py2 + pz2 - rp2 + rs2;

        float disc = -(px2 * px2) - 2.0f * px2 * py2 - 2.0f * px2 * pz2 +
                                    2.0f * px2 * rp2 + 2.0f * px2 * rs2 - (py2 * py2) -
                                    2.0f * py2 * pz2 + 2.0f * py2 * rp2 + 2.0f * py2 * rs2 -
                                    (pz2 * pz2) + 2.0f * pz2 * rp2 - 2.0f * pz2 * rs2 -
                                    (rp2 * rp2) + 2.0f * rp2 * rs2 - (rs2 * rs2);

        if (disc < 0.0f)
            disc = 0.0f;

        float sol = STEWART_PARAMS[i].solution ? 1.0f : -1.0f;
        float y = 2.0f * py * rs + sol * sqrtf(disc);
        out_stewart[i] = wrap_angle(2.0f * atan2f(y, x));

        if (!isfinite(out_stewart[i]))
            return -1;
    }
    return 0;
}

/**
    * @brief 计算 Reachy Mini 正运动学 (FK)
    *
    * 使用牛顿迭代法 (数值雅可比) 求解 Stewart 平台正运动学。
    * 给定 7 个关节角度，输出 4x4 头部位姿矩阵。
    *
    * @param joints 7 个关节角度 (弧度): [body_yaw, stewart_1...6]
    * @param out_pose 输出的 4x4 位姿矩阵 (行优先，不含 Z 偏移)
    * @param num_iterations 牛顿迭代次数（建议 3-10）
    * @return 0 成功, -1 失败 (未收敛或奇异)
    */
int reachy_calculate_fk(const float joints[7], float out_pose[16],
                                                int num_iterations) {
    if (!fk_initialized) {
        reachy_reset_fk(NULL);
    }

    float body_yaw = joints[0];
    const float *target_stewart = &joints[1];  // 6 个目标 Stewart 关节角

    // 使用上次的猜测作为起点
    float guess[16];
    for (int i = 0; i < 16; i++)
        guess[i] = fk_current_guess[i];

    for (int iter = 0; iter < num_iterations; iter++) {
        // 1. 从当前猜测位姿提取 6 自由度参数 (roll, pitch, yaw, tx, ty, tz)
        float r, p, y;
        mat4_to_rpy(guess, &r, &p, &y);
        float tx = guess[3], ty = guess[7], tz = guess[11];

        // 2. 用当前猜测计算 IK，得到对应的 Stewart 关节角
        float ik_result[6];
        if (ik_stewart_only(guess, body_yaw, ik_result) != 0) {
            return -1;
        }

        // 3. 计算残差 (目标角 - 当前 IK 计算的角)
        float residual[6];
        float err_sq = 0.0f;
        for (int i = 0; i < 6; i++) {
            residual[i] = wrap_angle(target_stewart[i] - ik_result[i]);
            err_sq += residual[i] * residual[i];
        }

        // 4. 检查收敛
        if (err_sq < FK_CONVERGENCE_EPS) {
            break;
        }

        // 5. 构建数值雅可比矩阵 J[6][6]
        //    J[i][j] = d(ik_result[i]) / d(param[j])
        //    param = [roll, pitch, yaw, tx, ty, tz]
        float J[6][6];
        float params[6] = {r, p, y, tx, ty, tz};

        for (int j = 0; j < 6; j++) {
            float params_plus[6];
            for (int k = 0; k < 6; k++)
                params_plus[k] = params[k];
            params_plus[j] += FK_DELTA;

            float perturbed_pose[16];
            rpy_to_mat4(params_plus[0], params_plus[1], params_plus[2],
                                    params_plus[3], params_plus[4], params_plus[5],
                                    perturbed_pose);

            float ik_perturbed[6];
            if (ik_stewart_only(perturbed_pose, body_yaw, ik_perturbed) != 0) {
                return -1;
            }

            for (int i = 0; i < 6; i++) {
                J[i][j] = (ik_perturbed[i] - ik_result[i]) / FK_DELTA;
            }
        }

        // 6. 求解 J * delta_params = residual
        float delta[6];
        if (solve_6x6(J, residual, delta) != 0) {
            // 雅可比奇异，尝试微小扰动继续
            r += 0.001f;
            p += 0.001f;
            rpy_to_mat4(r, p, y, tx, ty, tz, guess);
            continue;
        }

        // 7. 更新参数
        r += delta[0];
        p += delta[1];
        y += delta[2];
        tx += delta[3];
        ty += delta[4];
        tz += delta[5];

        rpy_to_mat4(r, p, y, tx, ty, tz, guess);
    }

    // 保存当前结果作为下次迭代的初始猜测
    for (int i = 0; i < 16; i++) {
        fk_current_guess[i] = guess[i];
        out_pose[i] = guess[i];
    }

    return 0;
}
