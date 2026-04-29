#ifndef KINEMATICS_PARAMS_H
#define KINEMATICS_PARAMS_H

#include <stdint.h>

#define MOTOR_ARM_LENGTH 0.040000f
#define ROD_LENGTH 0.085000f
#define HEAD_Z_OFFSET 0.177000f

typedef struct {
    float T_inv[16];    // Inverse of T_motor_world (from world to motor local)
    float branch_p[3];  // Top attachment point in head frame
    int solution;       // 0 or 1
} LegParams;

// Data extracted from kinematics_data.json
static const LegParams STEWART_PARAMS[6] = {
    {  // stewart_1
        .T_inv = {0.866025f, -0.500001f, -0.000002f, -0.010000f, 0.000004f,
                0.000003f, 1.000000f, -0.076633f, -0.500001f, -0.866025f,
                0.000005f, 0.036660f, 0.000000f, 0.000000f, 0.000000f,
                1.000000f},
        .branch_p = {0.020648f, 0.021764f, 0.000000f},
        .solution = 0},
    {  // stewart_2
        .T_inv = {-0.866021f, 0.500007f, 0.000002f, -0.010001f, -0.000004f,
                -0.000003f, -1.000000f, 0.076633f, -0.500007f, -0.866021f,
                0.000005f, 0.036660f, 0.000000f, 0.000000f, 0.000000f,
                1.000000f},
        .branch_p = {0.008524f, 0.028764f, 0.000000f},
        .solution = 1},
    {  // stewart_3
        .T_inv = {0.000006f, 1.000000f, -0.000000f, -0.010000f, -0.000001f,
                0.000000f, 1.000000f, -0.076633f, 1.000000f, -0.000006f,
                0.000001f, 0.036661f, 0.000000f, 0.000000f, 0.000000f,
                1.000000f},
        .branch_p = {-0.029172f, 0.007000f, 0.000000f},
        .solution = 0},
    {  // stewart_4
        .T_inv = {-0.000004f, -1.000000f, -0.000000f, -0.010000f, 0.000001f,
                -0.000000f, -1.000000f, 0.076633f, 1.000000f, -0.000004f,
                0.000001f, 0.036661f, 0.000000f, 0.000000f, 0.000000f,
                1.000000f},
        .branch_p = {-0.029172f, -0.007000f, 0.000000f},
        .solution = 1},
    {  // stewart_5
        .T_inv = {-0.866028f, -0.499995f, 0.000002f, -0.010000f, 0.000004f,
                -0.000003f, 1.000000f, -0.076633f, -0.499995f, 0.866028f,
                0.000005f, 0.036660f, 0.000000f, 0.000000f, 0.000000f,
                1.000000f},
        .branch_p = {0.008524f, -0.028764f, 0.000000f},
        .solution = 0},
    {  // stewart_6
        .T_inv = {0.866025f, 0.500001f, -0.000002f, -0.010000f, -0.000004f,
                0.000003f, -1.000000f, 0.076633f, -0.500001f, 0.866025f,
                0.000005f, 0.036660f, 0.000000f, 0.000000f, 0.000000f,
                1.000000f},
        .branch_p = {0.020648f, -0.021764f, 0.000000f},
        .solution = 1}};

typedef struct {
    float lower;
    float upper;
} JointLimit;

// Limits extracted from hardware_config.yaml
// rad = (raw - 2048) * (PI / 2048)
static const JointLimit MOTOR_LIMITS[7] = {
    {-3.141592f, 3.141592f},  // Body Yaw (ID 10): [0, 4095]
    {-0.837758f, 1.396263f},  // Stewart 1 (ID 11): [1502, 2958]
    {-1.396263f, 1.221730f},  // Stewart 2 (ID 12): [1138, 2844]
    {-0.837758f, 1.396263f},  // Stewart 3 (ID 13): [1502, 2958]
    {-1.396263f, 0.837758f},  // Stewart 4 (ID 14): [1138, 2594]
    {-1.221730f, 1.396263f},  // Stewart 5 (ID 15): [1252, 2958]
    {-1.396263f, 0.837758f}   // Stewart 6 (ID 16): [1138, 2594]
};

// ================= 正运动学 (FK) 配置 =================

// 睡眠姿态 (来自 Python SDK SLEEP_HEAD_POSE，行优先 4x4)
// 用于 FK 牛顿迭代法的初始猜测
static const float SLEEP_HEAD_POSE[16] = {
    0.911f,  0.004f,  0.413f, -0.021f, -0.004f, 1.000f, -0.001f, 0.001f,
    -0.413f, -0.001f, 0.911f, -0.044f, 0.000f,  0.000f, 0.000f,  1.000f};

// FK 牛顿法参数
#define FK_MAX_ITERATIONS 50      // 最大迭代次数
#define FK_CONVERGENCE_EPS 1e-6f  // 收敛阈值 (关节角残差平方和)
#define FK_DELTA 1e-4f            // 数值雅可比微分步长

#endif
