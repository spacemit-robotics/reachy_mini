#include "motion_api.h"

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "kinematics_params.h"

#define MOTOR_COUNT 9
#define CONTROL_PERIOD_US 10000  // 100Hz

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ================= 全局状态记录 (逻辑姿态) =================
// 记录当前目标姿态（全部为度数 Degrees），用于增量或单维度控制
static float current_r = 0.0f;
static float current_p = 0.0f;
static float current_y = 0.0f;
static float current_body = 0.0f;
static float current_ant_r = 0.0f;
static float current_ant_l = 0.0f;

// 声明外部运动学解算函数
extern int reachy_calculate_ik(const float head_pose_world[16],
                    float out_joints[7]);

// ================= 内部辅助函数 =================
static void create_pose(float roll, float pitch, float yaw,
                        float head_pose[16]) {
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

static bool check_safety(float joints[]) {
    for (int i = 0; i < 7; i++) {
    if (!isfinite(joints[i]))
            return false;
    if (joints[i] < MOTOR_LIMITS[i].lower ||
        joints[i] > MOTOR_LIMITS[i].upper) {
            return false;
    }
    }
    return true;
}

// ================= 核心执行 API =================
int motion_move_to(struct motor_dev **devs, float roll_deg, float pitch_deg,
                    float yaw_deg, float body_yaw_deg, float ant_r_deg,
                    float ant_l_deg) {
    if (!devs)
    return -1;

    // ==== 参考 hardware_config.yaml 限制最大传入参数 ====
    if (body_yaw_deg > 180.0f)
    body_yaw_deg = 180.0f;
    if (body_yaw_deg < -180.0f)
    body_yaw_deg = -180.0f;
    if (ant_r_deg > 180.0f)
    ant_r_deg = 180.0f;
    if (ant_r_deg < -180.0f)
    ant_r_deg = -180.0f;
    if (ant_l_deg > 180.0f)
    ant_l_deg = 180.0f;
    if (ant_l_deg < -180.0f)
    ant_l_deg = -180.0f;

    if (roll_deg > 25.0f)
    roll_deg = 25.0f;
    if (roll_deg < -25.0f)
    roll_deg = -25.0f;
    if (pitch_deg > 35.0f)
    pitch_deg = 35.0f;
    if (pitch_deg < -35.0f)
    pitch_deg = -35.0f;
    if (yaw_deg > 170.0f)
    yaw_deg = 170.0f;
    if (yaw_deg < -170.0f)
    yaw_deg = -170.0f;

    float r = roll_deg * (float)M_PI / 180.0f;
    float p = pitch_deg * (float)M_PI / 180.0f;
    float y = yaw_deg * (float)M_PI / 180.0f;
    float body = body_yaw_deg * (float)M_PI / 180.0f;
    float ar = ant_r_deg * (float)M_PI / 180.0f;
    float al = ant_l_deg * (float)M_PI / 180.0f;

    float current_pose[16];
    float joint_goals[MOTOR_COUNT];

    create_pose(r, p, y, current_pose);
    if (reachy_calculate_ik(current_pose, joint_goals) != 0) {
    return -1;
    }

    // 叠加外部指定的 body yaw，因为 reachy_calculate_ik
    // 内部可能已经解算出了用于协同的 body 角度
    joint_goals[0] += body;
    joint_goals[7] = ar;
    joint_goals[8] = al;

    if (!check_safety(joint_goals))
    return -1;

    current_r = roll_deg;
    current_p = pitch_deg;
    current_y = yaw_deg;
    current_body = body_yaw_deg;
    current_ant_r = ant_r_deg;
    current_ant_l = ant_l_deg;

    struct motor_state states[MOTOR_COUNT];
    float start_joints[MOTOR_COUNT];
    if (motor_get_states(devs, states, MOTOR_COUNT) == 0) {
    for (int i = 0; i < MOTOR_COUNT; i++)
            start_joints[i] = states[i].pos;
    } else {
    for (int i = 0; i < MOTOR_COUNT; i++)
            start_joints[i] = 0.0f;
    }

    float max_travel = 0.0f;  // 所有关节中运动幅度最大的值
    for (int i = 0; i < MOTOR_COUNT; i++) {
    float travel = fabsf(joint_goals[i] - start_joints[i]);
    if (travel > max_travel)
            max_travel = travel;
    }

    // 根据运动幅度计算步数，确保运动平滑
    //
    int steps = (int)(max_travel * 100);  // 步数
    if (steps < 20)
    steps = 20;
    if (steps > 200)
    steps = 200;

    struct motor_cmd cmds[MOTOR_COUNT];
    for (int s = 1; s <= steps; s++) {
    // 0.5 * (1 - cos(pi * phase)) 是一个平滑的加权函数，
    // 确保运动开始和结束时速度为0，中间速度逐渐增大再减小，
    // 形成平滑的加减速效果
    float phase = (float)s / (float)steps;
    float t_factor = 0.5f * (1.0f - cosf((float)M_PI * phase));

    for (int i = 0; i < MOTOR_COUNT; i++) {
            cmds[i].mode = MOTOR_MODE_POS;
            cmds[i].pos_des =
                    start_joints[i] + (joint_goals[i] - start_joints[i]) * t_factor;
            cmds[i].vel_des = 0.0f;
            cmds[i].trq_des = 0.0f;
            cmds[i].kp = 0.0f;
            cmds[i].kd = 0.0f;
    }
    motor_set_cmds(devs, cmds, MOTOR_COUNT);
    usleep(CONTROL_PERIOD_US);
    }

    return 0;
}

int motion_move_to_async(struct motor_dev **devs, float roll_deg,
                        float pitch_deg, float yaw_deg, float body_yaw_deg,
                        float ant_r_deg, float ant_l_deg) {
    if (!devs)
    return -1;

    // ==== 参考 hardware_config.yaml 限制最大传入参数 ====
    if (body_yaw_deg > 180.0f)
    body_yaw_deg = 180.0f;
    if (body_yaw_deg < -180.0f)
    body_yaw_deg = -180.0f;
    if (ant_r_deg > 180.0f)
    ant_r_deg = 180.0f;
    if (ant_r_deg < -180.0f)
    ant_r_deg = -180.0f;
    if (ant_l_deg > 180.0f)
    ant_l_deg = 180.0f;
    if (ant_l_deg < -180.0f)
    ant_l_deg = -180.0f;

    if (roll_deg > 25.0f)
    roll_deg = 25.0f;
    if (roll_deg < -25.0f)
    roll_deg = -25.0f;
    if (pitch_deg > 35.0f)
    pitch_deg = 35.0f;
    if (pitch_deg < -35.0f)
    pitch_deg = -35.0f;
    if (yaw_deg > 170.0f)
    yaw_deg = 170.0f;
    if (yaw_deg < -170.0f)
    yaw_deg = -170.0f;

    float r = roll_deg * (float)M_PI / 180.0f;
    float p = pitch_deg * (float)M_PI / 180.0f;
    float y = yaw_deg * (float)M_PI / 180.0f;
    float body = body_yaw_deg * (float)M_PI / 180.0f;
    float ar = ant_r_deg * (float)M_PI / 180.0f;
    float al = ant_l_deg * (float)M_PI / 180.0f;

    float current_pose[16];
    float joint_goals[MOTOR_COUNT];

    create_pose(r, p, y, current_pose);
    if (reachy_calculate_ik(current_pose, joint_goals) != 0) {
    return -1;
    }

    joint_goals[0] += body;
    joint_goals[7] = ar;
    joint_goals[8] = al;

    if (!check_safety(joint_goals))
    return -1;

    current_r = roll_deg;
    current_p = pitch_deg;
    current_y = yaw_deg;
    current_body = body_yaw_deg;
    current_ant_r = ant_r_deg;
    current_ant_l = ant_l_deg;

    struct motor_cmd cmds[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) {
    cmds[i].mode = MOTOR_MODE_POS;
    cmds[i].pos_des = joint_goals[i];
    cmds[i].vel_des = 0.0f;
    cmds[i].trq_des = 0.0f;
    cmds[i].kp = 0.0f;
    cmds[i].kd = 0.0f;
    }
    motor_set_cmds(devs, cmds, MOTOR_COUNT);

    return 0;
}

// ================= 各个动作的具体封装实现 =================

int head_turn_left(struct motor_dev **devs, float angle) {
    return motion_move_to(devs, current_r, current_p, angle, current_body,
                        current_ant_r, current_ant_l);
}

int head_turn_right(struct motor_dev **devs, float angle) {
    return motion_move_to(devs, current_r, current_p, -angle, current_body,
                        current_ant_r, current_ant_l);
}

int head_look_down(struct motor_dev **devs, float angle) {
    return motion_move_to(devs, current_r, angle, current_y, current_body,
                        current_ant_r, current_ant_l);
}

int head_look_up(struct motor_dev **devs, float angle) {
    return motion_move_to(devs, current_r, -angle, current_y, current_body,
                        current_ant_r, current_ant_l);
}

int head_tilt_left(struct motor_dev **devs, float angle) {
    return motion_move_to(devs, angle, current_p, current_y, current_body,
                        current_ant_r, current_ant_l);
}

int head_tilt_right(struct motor_dev **devs, float angle) {
    return motion_move_to(devs, -angle, current_p, current_y, current_body,
                        current_ant_r, current_ant_l);
}

int body_turn_left(struct motor_dev **devs, float angle) {
    return motion_move_to(devs, current_r, current_p, current_y, angle,
                        current_ant_r, current_ant_l);
}

int body_turn_right(struct motor_dev **devs, float angle) {
    return motion_move_to(devs, current_r, current_p, current_y, -angle,
                        current_ant_r, current_ant_l);
}

int antenna_turn_forward(struct motor_dev **devs, float angle_r,
                        float angle_l) {
    return motion_move_to(devs, current_r, current_p, current_y, current_body,
                        angle_r, angle_l);
}

int antenna_turn_backward(struct motor_dev **devs, float angle_r,
                                        float angle_l) {
    return motion_move_to(devs, current_r, current_p, current_y, current_body,
                        -angle_r, -angle_l);
}

int center_all(struct motor_dev **devs) {
    return motion_move_to(devs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

// ================= 姿态查询接口 =================
float motion_get_current_roll(void) { return current_r; }
float motion_get_current_pitch(void) { return current_p; }
float motion_get_current_yaw(void) { return current_y; }
