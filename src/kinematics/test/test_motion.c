#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "kinematics_params.h"
#include "motor.h"

/**
 * @brief Reachy Mini 电机运动与运动学示例
 *
 * 该示例演示了如何结合运动学解算控制 Reachy Mini 的头部。
 * 包含安全检查机制：数值异常保护和基于 hardware_config.yaml 的硬件限位保护。
 */

#define MOTOR_COUNT 9  // ID 10 (Body Yaw) + ID 11-16 (Stewart) + ID 17, 18 (Antennas)
#define BAUDRATE 1000000
#define DEFAULT_PORT "/dev/ttyACM0"

#define CONTROL_PERIOD_US 10000  // 100Hz

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "reachy_kinematics.h"

/* 辅助函数：创建位姿矩阵 */
void create_pose(float roll, float pitch, float yaw, float head_pose[16]) {
    // 简单的 Rxyz 旋转矩阵 (假设无平移，或在 IK 中处理 offset)
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);

    head_pose[0] = cy * cp;
    head_pose[1] = cy * sp * sr - sy * cr;
    head_pose[2] = cy * sp * cr + sy * sr;
    head_pose[3] = 0.0f;

    head_pose[4] = sy * cp;
    head_pose[5] = sy * sp * sr + cy * cr;
    head_pose[6] = sy * sp * cr - cy * sr;
    head_pose[7] = 0.0f;

    head_pose[8] = -sp;
    head_pose[9] = cp * sr;
    head_pose[10] = cp * cr;
    head_pose[11] = 0.0f;

    head_pose[12] = 0.0f;
    head_pose[13] = 0.0f;
    head_pose[14] = 0.0f;
    head_pose[15] = 1.0f;
}

/* 安全检查函数 (仅检查前7个核心关节) */
bool check_safety(float joints[]) {
    for (int i = 0; i < 7; i++) {
        // 1. 数值有效性检查 (NaN/Inf)
        if (!isfinite(joints[i])) {
            fprintf(stderr, "\n[安全警报] 关节 %d 数值异常 (NaN/Inf)!\n", i);
            return false;
        }
        // 2. 硬件限位检查 (从 kinematics_params.h 获取)
        if (joints[i] < MOTOR_LIMITS[i].lower || joints[i] > MOTOR_LIMITS[i].upper) {
            fprintf(stderr, "\n[安全警报] 关节 %d 越界: %.3f rad (限位: [%.3f, %.3f])!\n", i,
                    joints[i], MOTOR_LIMITS[i].lower, MOTOR_LIMITS[i].upper);
            return false;
        }
    }
    return true;
}

/* ==================== FK 验证测试 ==================== */
void test_fk_roundtrip(void) {
    printf("\n========== 正运动学 (FK) 往返验证测试 ==========\n");

    // 测试用例: 不同的 RPY 姿态
    float test_poses[][3] = {
        {0.0f, 0.0f, 0.0f},                          // 中心
        {0.0f, 0.0f, (float)(20.0 * M_PI / 180.0)},  // Yaw 20°
        {0.0f, (float)(15.0 * M_PI / 180.0), 0.0f},  // Pitch 15°
        {(float)(10.0 * M_PI / 180.0), 0.0f, 0.0f},  // Roll 10°
        {(float)(5.0 * M_PI / 180.0), (float)(10.0 * M_PI / 180.0),
        (float)(-15.0 * M_PI / 180.0)},  // 组合姿态
    };
    int num_tests = sizeof(test_poses) / sizeof(test_poses[0]);

    reachy_reset_fk(NULL);  // 重置 FK 初始猜测

    int pass = 0, fail = 0;

    for (int t = 0; t < num_tests; t++) {
        float r = test_poses[t][0], p = test_poses[t][1], y = test_poses[t][2];

        // 1. 创建位姿 -> IK 计算关节角
        float pose[16];
        create_pose(r, p, y, pose);

        float joints[7];
        if (reachy_calculate_ik(pose, joints) != 0) {
            printf("  [%d] IK 失败 (R=%.1f° P=%.1f° Y=%.1f°) - 跳过\n", t, r * 180 / M_PI,
                    p * 180 / M_PI, y * 180 / M_PI);
            fail++;
            continue;
        }

        // 2. FK 从关节角反推位姿
        float fk_pose[16];
        if (reachy_calculate_fk(joints, fk_pose, 10) != 0) {
            printf("  [%d] FK 失败 (R=%.1f° P=%.1f° Y=%.1f°) - 跳过\n", t, r * 180 / M_PI,
                    p * 180 / M_PI, y * 180 / M_PI);
            fail++;
            continue;
        }

        // 3. 用 FK 输出的位姿再次 IK，比较关节角与原始关节角的差异
        float verify_joints[7];
        if (reachy_calculate_ik(fk_pose, verify_joints) != 0) {
            printf("  [%d] 验证 IK 失败\n", t);
            fail++;
            continue;
        }

        float max_err = 0.0f;
        for (int i = 1; i < 7; i++) {
            float err = fabsf(joints[i] - verify_joints[i]);
            if (err > max_err)
                max_err = err;
        }

        if (max_err < 0.01f) {
            printf("  [%d] 通过 ✓  R=%.1f° P=%.1f° Y=%.1f°  最大误差=%.6f rad\n", t, r * 180 / M_PI,
                    p * 180 / M_PI, y * 180 / M_PI, max_err);
            pass++;
        } else {
            printf("  [%d] 失败 ✗  R=%.1f° P=%.1f° Y=%.1f°  最大误差=%.6f rad\n", t, r * 180 / M_PI,
                    p * 180 / M_PI, y * 180 / M_PI, max_err);
            fail++;
        }
    }

    printf("---------- FK 测试结果: %d 通过, %d 失败 ----------\n\n", pass, fail);
}

/* ==================== 实时 FK 读取测试 ==================== */
void test_fk_realtime(struct motor_dev **devs, struct motor_state *states) {
    printf("\n========== 实时 FK 读取测试 ==========\n");
    printf("从电机读取当前关节角度，用 FK 计算头部位姿...\n");

    if (motor_get_states(devs, states, MOTOR_COUNT) != 0) {
        fprintf(stderr, "无法获取电机状态\n");
        return;
    }

    float joints[7];
    joints[0] = states[0].pos;  // Body Yaw (ID 10)
    for (int i = 1; i < 7; i++) {
        joints[i] = states[i].pos;  // Stewart 1-6 (ID 11-16)
    }

    printf("当前关节角度 (rad):\n  BodyYaw=%.4f", joints[0]);
    for (int i = 1; i < 7; i++) {
        printf("  S%d=%.4f", i, joints[i]);
    }
    printf("\n");

    float fk_pose[16];
    reachy_reset_fk(NULL);
    if (reachy_calculate_fk(joints, fk_pose, 10) == 0) {
        // 提取 RPY
        float sp = -fk_pose[8];
        if (sp > 1.0f)
            sp = 1.0f;
        if (sp < -1.0f)
            sp = -1.0f;
        float pitch = asinf(sp);
        float cp = cosf(pitch);
        float roll = 0.0f, yaw = 0.0f;
        if (fabsf(cp) > 1e-6f) {
            roll = atan2f(fk_pose[9], fk_pose[10]);
            yaw = atan2f(fk_pose[4], fk_pose[0]);
        }

        printf("FK 计算的头部位姿:\n");
        printf("  Roll  = %.2f°\n", roll * 180.0f / (float)M_PI);
        printf("  Pitch = %.2f°\n", pitch * 180.0f / (float)M_PI);
        printf("  Yaw   = %.2f°\n", yaw * 180.0f / (float)M_PI);
        printf("  位置  = (%.4f, %.4f, %.4f)\n", fk_pose[3], fk_pose[7], fk_pose[11]);
    } else {
        fprintf(stderr, "FK 计算失败\n");
    }
    printf("======================================\n\n");
}

int main(int argc, char *argv[]) {
    const char *port = DEFAULT_PORT;
    if (argc > 1) {
        port = argv[1];
    }

    // ====== 先运行 FK 纯数学验证（不需要硬件） ======
    test_fk_roundtrip();

    struct motor_dev *devs[MOTOR_COUNT];
    struct motor_cmd cmds[MOTOR_COUNT];
    struct motor_state states[MOTOR_COUNT];

    printf("[示例] 初始化 %d 个 Reachy Mini 电机 (端口: %s)...\n", MOTOR_COUNT, port);

    /* 1. 分配电机 (ID 10-18) */
    uint8_t motor_ids[MOTOR_COUNT] = {10, 11, 12, 13, 14, 15, 16, 17, 18};
    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint8_t id = motor_ids[i];
        devs[i] = motor_alloc_uart("drv_uart_rm", port, BAUDRATE, id, NULL);
        if (!devs[i]) {
            fprintf(stderr, "错误: 无法分配电机 ID %d\n", id);
            motor_free(devs, i);
            return -1;
        }
    }

    /* 2. 初始化电机 */
    if (motor_init(devs, MOTOR_COUNT) != 0) {
        fprintf(stderr, "错误: 初始化电机失败\n");
        motor_free(devs, MOTOR_COUNT);
        return -1;
    }

    printf("[示例] 电机就绪。\n");

    // ====== 实时 FK 测试（从真实电机读取角度并计算位姿） ======
    test_fk_realtime(devs, states);

    printf("[示例] 开始运动规划演示...\n");

    // 定义目标姿态序列 (Roll, Pitch, Yaw, BodyYaw, AntennaR, AntennaL)
    float targets[][6] = {
        {0, 0, 0, 0, 0, 0},                                                      // 中心
        {0, 0, (float)(30 * M_PI / 180.0), 0, 0, 0},                             // 向左看
        {0, 0, (float)(-30 * M_PI / 180.0), 0, 0, 0},                            // 向右看
        {0, (float)(20 * M_PI / 180.0), 0, 0, 0, 0},                             // 向下看
        {0, (float)(-20 * M_PI / 180.0), 0, 0, 0, 0},                            // 向上看
        {(float)(15 * M_PI / 180.0), 0, 0, 0, 0, 0},                             // 向左歪头
        {(float)(-15 * M_PI / 180.0), 0, 0, 0, 0, 0},                            // 向右歪头
        {0, 0, 0, 0, 0, 0},                                                      // 返回中心
        {0, 0, 0, (float)(30 * M_PI / 180.0), 0, 0},                             // 身体向左转动
        {0, 0, 0, (float)(-30 * M_PI / 180.0), 0, 0},                            // 身体向右转动
        {0, 0, 0, 0, 0, 0},                                                      // 返回中心
        {0, 0, 0, 0, (float)(45 * M_PI / 180.0), (float)(45 * M_PI / 180.0)},    // 天线向前转动
        {0, 0, 0, 0, (float)(-45 * M_PI / 180.0), (float)(-45 * M_PI / 180.0)},  // 天线向后转动
        {0, 0, 0, 0, (float)(45 * M_PI / 180.0),
        (float)(-45 * M_PI / 180.0)},  // 天线交错转动 (右前左后)
        {0, 0, 0, 0, (float)(-45 * M_PI / 180.0),
        (float)(45 * M_PI / 180.0)},  // 天线交错转动 (右后左前)
        {0, 0, 0, 0, 0, 0}             // 返回中心
    };
    int num_targets = sizeof(targets) / sizeof(targets[0]);

    for (int t = 0; t < num_targets; t++) {
        printf("[运动] 切换到目标姿态 %d: R=%.1f, P=%.1f, Y=%.1f, Body=%.1f, "
                "AntR=%.1f, AntL=%.1f\n",
                t, targets[t][0] * 180 / M_PI, targets[t][1] * 180 / M_PI,
                targets[t][2] * 180 / M_PI, targets[t][3] * 180 / M_PI, targets[t][4] * 180 / M_PI,
                targets[t][5] * 180 / M_PI);

        float current_pose[16];
        float joint_goals[MOTOR_COUNT];

        // 计算解算 (写入前7个关节)
        create_pose(targets[t][0], targets[t][1], targets[t][2], current_pose);
        if (reachy_calculate_ik(current_pose, joint_goals) != 0) {
            fprintf(stderr, "错误: 运动学解算失败\n");
            break;
        }

        // 设置 Body Yaw (ID 10) 目标
        joint_goals[0] = targets[t][3];
        // 设置 天线 目标 (ID 17, 18)
        joint_goals[7] = targets[t][4];
        joint_goals[8] = targets[t][5];

        // 安全检查 (仅检查前7个核心关节)
        if (!check_safety(joint_goals)) {
            fprintf(stderr, "检测到异常数值，中止动作保护运行。\n");
            break;
        }

        // 获取当前实际位置作为插值起点
        float start_joints[MOTOR_COUNT];
        if (motor_get_states(devs, states, MOTOR_COUNT) == 0) {
            for (int i = 0; i < MOTOR_COUNT; i++) {
                start_joints[i] = states[i].pos;
            }
        } else {
            fprintf(stderr, "警告: 无法获取电机状态，使用 0 作为起点\n");
            for (int i = 0; i < MOTOR_COUNT; i++)
                start_joints[i] = 0.0f;
        }

        /* 3. 动态计算步数与 S-曲线插值 (平滑过渡) */
        // 计算各关节最大运动距离，以决定步数（假设平均速度 1.0 rad/s）
        float max_travel = 0.0f;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            float travel = fabsf(joint_goals[i] - start_joints[i]);
            if (travel > max_travel)
                max_travel = travel;
        }

        // 基础步数计算：根据距离动态调整，最少 20 步 (0.2s)，最多 200 步 (2.0s)
        int steps = (int)(max_travel * 100);  // 100步/rad (100Hz 下即 1.0 rad/s)
        if (steps < 20)
            steps = 20;
        if (steps > 200)
            steps = 200;

        for (int s = 1; s <= steps; s++) {
            // S-曲线插值：0.5 * (1 - cos(pi * t))
            float phase = (float)s / (float)steps;
            float t_factor = 0.5f * (1.0f - cosf((float)M_PI * phase));

            for (int i = 0; i < MOTOR_COUNT; i++) {
                cmds[i].mode = MOTOR_MODE_POS;
                // 使用 S-曲线插值：start + (goal - start) * t_factor
                cmds[i].pos_des = start_joints[i] + (joint_goals[i] - start_joints[i]) * t_factor;
                cmds[i].vel_des = 0.0f;
                cmds[i].trq_des = 0.0f;
                cmds[i].kp = 0.0f;
                cmds[i].kd = 0.0f;
            }

            if (motor_set_cmds(devs, cmds, MOTOR_COUNT) != 0) {
                fprintf(stderr, "警告: 发送指令失败\n");
            }

            usleep(CONTROL_PERIOD_US);
        }
        sleep(1);  // 到达后观察1秒
    }

    // ====== 运动后再次测试 FK ======
    printf("\n[示例] 运动演示结束，再次读取 FK...\n");
    test_fk_realtime(devs, states);

    /* 4. 清理 */
    printf("[示例] 正在复位并释放资源...\n");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        cmds[i].pos_des = 0.0f;
    }
    motor_set_cmds(devs, cmds, MOTOR_COUNT);
    sleep(1);

    motor_free(devs, MOTOR_COUNT);
    printf("[示例] 已释放所有设备。\n");

    return 0;
}
