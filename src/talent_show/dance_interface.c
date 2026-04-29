/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dance_interface.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "motor_controller.h"
#include "reachy_kinematics.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile int g_dance_stop_requested = 0;

void dance_request_stop(void) {
    g_dance_stop_requested = 1;
}

int dance_should_stop(void) {
    return g_dance_stop_requested;
}

// ============================================================================
// 1. 基础数学助手 (Ported from Python rhythmic_motion.py)
// ============================================================================

static float ease_hermite(float t) {
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

static float oscillator_sin(float t_beats, float subcycles, float phase) {
    return sinf(2.0f * (float)M_PI * (subcycles * t_beats + phase));
}

// 模拟 Python 的 transient_motion
static float transient_peak(float t_beats, float duration, float repeat, float delay) {
    float t_in_period = fmodf(t_beats, repeat);
    if (t_in_period < delay)
        return 0.0f;
    float t_rel = (t_in_period - delay) / duration;
    if (t_rel < 0.0f || t_rel > 1.0f)
        return 0.0f;
    // 使用半弦波作为瞬态峰
    return sinf((float)M_PI * t_rel);
}

// ============================================================================
// 2. 轨迹计算与 IK 调用
// ============================================================================

typedef struct {
    float x, y, z;
    float roll, pitch, yaw;
} Pose6D;

static void pose_to_mat4(const Pose6D *p, float T[16]) {
    float cr = cosf(p->roll), sr = sinf(p->roll);
    float cp = cosf(p->pitch), sp = sinf(p->pitch);
    float cy = cosf(p->yaw), sy = sinf(p->yaw);

    T[0] = cy * cp;
    T[1] = cy * sp * sr - sy * cr;
    T[2] = cy * sp * cr + sy * sr;
    T[3] = p->x;
    T[4] = sy * cp;
    T[5] = sy * sp * sr + cy * cr;
    T[6] = sy * sp * cr - cy * sr;
    T[7] = p->y;
    T[8] = -sp;
    T[9] = cp * sr;
    T[10] = cp * cr;
    T[11] = p->z;
    T[12] = 0.0f;
    T[13] = 0.0f;
    T[14] = 0.0f;
    T[15] = 1.0f;
}

// 执行一帧并推送到控制器
static void execute_pose_step(AsyncMotorController *ctrl, const Pose6D *p, float ant_r,
                                float ant_l) {
    float T[16];
    float joints[7];
    pose_to_mat4(p, T);

    if (reachy_calculate_ik(T, joints) == 0) {
        async_motor_controller_set_joint_targets(ctrl, joints, ant_r, ant_l);
    }
}

// ============================================================================
// 3. 舞蹈动作函数封装
// ============================================================================

// --- 动作 1: Headbanger Combo (摇头摆脑) ---
void dance_move_headbanger(AsyncMotorController *ctrl, int cycles, float bpm) {
    float duration_beats = 4.0f;
    float total_beats = cycles * duration_beats;
    float hz = 100.0f;  // 10ms loop

    printf("[Dance] Starting Headbanger Combo (%d cycles, %.1f BPM)\n", cycles, bpm);

    g_dance_stop_requested = 0;
    for (float b = 0.0f; b < total_beats; b += (bpm / 60.0f / hz)) {
        if (g_dance_stop_requested)
            break;
        Pose6D p = {0};
        float osc = oscillator_sin(b, 1.0f, 0.0f);
        float pitch_amp = (osc < 0) ? 10.0f : 20.0f;  // 仰角幅度限缩至 10deg，防止撞肩
        p.pitch = pitch_amp * (float)M_PI / 180.0f * osc;
        p.z = 0.010f * oscillator_sin(b, 1.0f, 0.1f);

        float ant = 40.0f * oscillator_sin(b, 1.0f, 0.0f);
        execute_pose_step(ctrl, &p, ant, -ant);
        usleep(10000);
    }
}

// --- 动作 2: Jackson Square (杰克逊方块步) ---
void dance_move_jackson_square(AsyncMotorController *ctrl, int cycles, float bpm) {
    float leg_duration = 2.0f;
    float period = 10.0f;
    float total_beats = cycles * period;
    float hz = 100.0f;

    Pose6D path[5] = {
        {0, 0, -0.01f, 0, 0, 0},         // P0
        {0, 0.035f, 0.005f, 0, 0, 0},    // P1 (0.025 + -0.01)
        {0, -0.035f, 0.005f, 0, 0, 0},   // P2
        {0, -0.035f, -0.035f, 0, 0, 0},  // P3
        {0, 0.035f, -0.035f, 0, 0, 0}    // P4
    };

    printf("[Dance] Starting Jackson Square (%d cycles, %.1f BPM)\n", cycles, bpm);

    g_dance_stop_requested = 0;
    for (float b = 0.0f; b < total_beats; b += (bpm / 60.0f / hz)) {
        if (g_dance_stop_requested)
            break;
        float b_mod = fmodf(b, period);
        int leg = (int)(b_mod / leg_duration) % 5;
        float progress = fmodf(b_mod, leg_duration) / leg_duration;
        float eased = ease_hermite(progress);

        Pose6D p;
        Pose6D *s = &path[leg];
        Pose6D *e = &path[(leg + 1) % 5];

        p.x = s->x + (e->x - s->x) * eased;
        p.y = s->y + (e->y - s->y) * eased;
        p.z = s->z + (e->z - s->z) * eased;
        p.pitch = 0;
        p.yaw = 0;
        p.roll = 0;  // Removed roll twitch to fix jitter

        execute_pose_step(ctrl, &p, 45, 45);
        usleep(10000);
    }

    // Reset to neutral pose at the end
    Pose6D neutral = {0, 0, 0, 0, 0, 0};
    execute_pose_step(ctrl, &neutral, 0, 0);
    printf("[Dance] Jackson Square finished, reset to zero.\n");
}

// --- 动作 3: Chicken Peck (小鸡啄米) ---
void dance_move_chicken_peck(AsyncMotorController *ctrl, int cycles, float bpm) {
    float duration_beats = 4.0f;
    float total_beats = cycles * duration_beats;
    float hz = 100.0f;

    printf("[Dance] Starting Chicken Peck (%d cycles, %.1f BPM)\n", cycles, bpm);

    g_dance_stop_requested = 0;
    for (float b = 0.0f; b < total_beats; b += (bpm / 60.0f / hz)) {
        if (g_dance_stop_requested)
            break;
        Pose6D p = {0};
        // amplitude_m=0.02, duration=0.8, repeat=1.0
        float t_x = transient_peak(b, 0.8f, 1.0f, 0.0f);
        p.x = 0.02f * t_x;
        p.pitch = (0.1f) * t_x;  // 0.1 rad approx 5.7deg

        float ant = 30.0f * oscillator_sin(b, 1.0f, 0.0f);
        execute_pose_step(ctrl, &p, ant, -ant);
        usleep(10000);
    }
}

// --- 动作 4: Uh-huh Tilt (嗯哼歪头) ---
void dance_move_uh_huh_tilt(AsyncMotorController *ctrl, int cycles, float bpm) {
    float duration_beats = 4.0f;
    float total_beats = cycles * duration_beats;
    float hz = 100.0f;

    printf("[Dance] Starting Uh-huh Tilt (%d cycles, %.1f BPM)\n", cycles, bpm);

    g_dance_stop_requested = 0;
    for (float b = 0.0f; b < total_beats; b += (bpm / 60.0f / hz)) {
        if (g_dance_stop_requested)
            break;
        Pose6D p = {0};
        float osc = 15.0f * (float)M_PI / 180.0f * oscillator_sin(b, 0.5f, 0.0f);
        p.roll = osc;
        p.pitch = osc;

        execute_pose_step(ctrl, &p, 45, -45);
        usleep(10000);
    }
}
