/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "voice_ctl.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// 包含 reachy-mini 基础动作库和异步电机 API
#include "../talent_show/audio/audio_player.h"
#include "../talent_show/dance_interface.h"
#include "../talent_show/dance_player.h"
#include "motion_api.h"
#include "motor_controller.h"

// 内部驱动和管理变量
static struct motor_dev *g_devs[9];
static AsyncMotorController *g_async_motor_ctrl = NULL;
static bool g_initialized = false;

// 累计目标角度状态缓存 (用于增量步进与限位控制)
static float g_target_r = 0.0f;
static float g_target_p = 0.0f;
static float g_target_y = 0.0f;
static float g_target_body = 0.0f;
static float g_target_ant_r = 0.0f;
static float g_target_ant_l = 0.0f;

// 定义具体动作映射的数据结构
typedef int (*MotionFunc)(struct motor_dev **devs, float angle);

typedef struct {
    int action_id;
    const char *keywords[8];
    MotionFunc func;
    float param_angle;  // 调用时的参数值
    const char *desc;
} CommandMapping;

// ==========================================
// 兼容动作接口的包装函数
// ==========================================

static int center_all_wrap(struct motor_dev **devs, float angle) {
    (void)angle;  // 忽略角度参数
    center_all(devs);
    return 0;
}

static int head_turn_left_wrap(struct motor_dev **devs, float angle) {
    head_turn_left(devs, angle);
    return 0;
}
static int head_turn_right_wrap(struct motor_dev **devs, float angle) {
    head_turn_right(devs, angle);
    return 0;
}
static int head_look_up_wrap(struct motor_dev **devs, float angle) {
    head_look_up(devs, angle);
    return 0;
}
static int head_look_down_wrap(struct motor_dev **devs, float angle) {
    head_look_down(devs, angle);
    return 0;
}
static int head_tilt_left_wrap(struct motor_dev **devs, float angle) {
    head_tilt_left(devs, angle);
    return 0;
}
static int head_tilt_right_wrap(struct motor_dev **devs, float angle) {
    head_tilt_right(devs, angle);
    return 0;
}
static int body_turn_left_wrap(struct motor_dev **devs, float angle) {
    body_turn_left(devs, angle);
    return 0;
}
static int body_turn_right_wrap(struct motor_dev **devs, float angle) {
    body_turn_right(devs, angle);
    return 0;
}

// === 动作意图映射表 ===
static const CommandMapping CMD_TABLE[] = {
    {0, {"向左转", "左转", "往左看", "看左边", NULL}, head_turn_left, 42.0f, "头部左转"},
    {1, {"向右转", "右转", "往奏看", "看右边", NULL}, head_turn_right, 42.0f, "头部右转"},
    {2, {"抬头", "向上看", "往上看", "看上面", NULL}, head_look_up, 35.0f, "抬头"},
    {3, {"低头", "向下看", "往下看", "看下面", NULL}, head_look_down, 35.0f, "低头"},
    {4, {"左歪头", "歪头", NULL}, head_tilt_left, 25.0f, "左歪头"},
    {5, {"右歪头", NULL}, head_tilt_right, 25.0f, "右歪头"},
    {6, {"身体左转", "转身向左", NULL}, body_turn_left, 42.0f, "身体左转"},
    {7, {"身体右转", "转身向右", NULL}, body_turn_right, 42.0f, "身体右转"},
    {8, {"回正", "回中", "复位", "正前方", NULL}, center_all_wrap, 0.0f, "全身回中"},
    {9, {"跳一段甩头舞", "甩头舞", NULL}, NULL, 0.0f, "甩头舞"},
    {10, {"跳一段杰克逊", "杰克逊", "方块步", NULL}, NULL, 0.0f, "杰克逊方块步"},
    {11, {"跳一段小鸡啄米", "小鸡啄米", NULL}, NULL, 0.0f, "小鸡啄米"},
    {12, {"跳一段古典舞", "嗯哼歪头", "歪头舞", NULL}, NULL, 0.0f, "嗯哼歪头"},
};
static const int CMD_TABLE_SIZE = sizeof(CMD_TABLE) / sizeof(CMD_TABLE[0]);

// ==========================================
// 初始化与清理
// ==========================================

int voice_ctl_init(const char *serial_port, float default_delay) {
    if (g_initialized) {
        return 0;
    }

    printf("[VoiceCtl] 开始初始化电机模块，串口: %s\n", serial_port ? serial_port : "(null)");

    // 1. 初始化 9 个底层电机 (ID 10-18)
    int motors_found = 0;
    for (int i = 0; i < 9; i++) {
        g_devs[i] = motor_alloc_uart("drv_uart_xl330", serial_port ? serial_port : "/dev/ttyACM0",
                                    1000000, 10 + i, NULL);
        if (g_devs[i]) {
            motors_found++;
        } else {
            printf("[VoiceCtl] 警告: 初始化电机 ID %d 失败，跳过该电机\n", 10 + i);
        }
    }

    if (motors_found == 0) {
        printf("[VoiceCtl] 错误: 未发现任何可用电机\n");
        return -1;
    }

    // 初始化电机状态
    motor_init(g_devs, 9);

    // 2. 初始化 AsyncMotorController
    g_async_motor_ctrl = async_motor_controller_create(g_devs, 9);
    if (!g_async_motor_ctrl) {
        printf("[VoiceCtl] 错误: 创建 AsyncMotorController 失败\n");
        return -1;
    }

    // 3. 按照官方高度成功的 gesture_tracker 逻辑：
    // 在启动时不进行物理位姿同步，强制信任零位，这能有效防止因电机未校准引起的“剧烈补偿”。
    // 默认所有目标均为 0
    g_target_r = 0.0f;
    g_target_p = 0.0f;
    g_target_y = 0.0f;
    g_target_body = 0.0f;
    g_target_ant_r = 0.0f;
    g_target_ant_l = 0.0f;

    // 4. 关键点：初始化动作速度设为 20, 启动控制循环
    async_motor_controller_set_speed_limit(g_async_motor_ctrl, 20.0f);
    if (!async_motor_controller_start(g_async_motor_ctrl)) {
        printf("[VoiceCtl] 错误: AsyncWorker 线程启动失败\n");
        return -1;
    }

    // 给机器人 1.5 秒时间以平稳速度 (20deg/s) 缓慢重对齐到位，避免启动冲撞
    usleep(1500000);

    // 5. 初始化完成后，将常规动作速度提高到 50
    async_motor_controller_set_speed_limit(g_async_motor_ctrl, 50.0f);

    g_initialized = true;
    printf("[VoiceCtl] 动作控制模块准备就绪 (运行速度: 50deg/s)\n");

    return 0;
}

void voice_ctl_cleanup(void) {
    if (!g_initialized)
        return;

    printf("[VoiceCtl] 正在清理动作模块...\n");

    if (g_async_motor_ctrl) {
        // 先发送回正指令，等待一会儿再停止
        async_motor_controller_set_target(g_async_motor_ctrl, 0, 0, 0, 0, 0, 0);
        usleep(100000);
        async_motor_controller_stop(g_async_motor_ctrl);
        async_motor_controller_destroy(g_async_motor_ctrl);
        g_async_motor_ctrl = NULL;
    }

    motor_free(g_devs, 9);
    for (int i = 0; i < 9; i++) {
        g_devs[i] = NULL;
    }

    g_initialized = false;
}

AsyncMotorController *voice_ctl_get_controller(void) {
    return g_async_motor_ctrl;
}

// ==========================================
// 核心逻辑: 预判(匹配) 与 执行
// ==========================================

int voice_ctl_match(const char *text, char *out_keyword, size_t max_len) {
    if (!text || strlen(text) == 0)
        return ACTION_NONE;

    // 模糊匹配: 遍历表中的同义词数组
    for (int i = 0; i < CMD_TABLE_SIZE; i++) {
        for (int j = 0; CMD_TABLE[i].keywords[j] != NULL; j++) {
            if (strstr(text, CMD_TABLE[i].keywords[j]) != NULL) {
                printf("[VoiceCtl] [匹配命中] 文本 >>'%s'<< 命中了关键词 -> '%s' "
                        "(动作:%s)\n",
                        text, CMD_TABLE[i].keywords[j], CMD_TABLE[i].desc);

                // 如果提供了输出缓冲区，则复制匹配到的关键词
                if (out_keyword && max_len > 0) {
                    strncpy(out_keyword, CMD_TABLE[i].keywords[j], max_len - 1);
                    out_keyword[max_len - 1] = '\0';
                }

                return CMD_TABLE[i].action_id;
            }
        }
    }
    return ACTION_NONE;
}

int voice_ctl_execute(int action_id) {
    if (!g_initialized || !g_async_motor_ctrl) {
        printf("[VoiceCtl] 警告: 尚未初始化，无法执行动作\n");
        return -1;
    }

    if (action_id < 0 || action_id >= CMD_TABLE_SIZE)
        return -1;

    const CommandMapping *cmd = &CMD_TABLE[action_id];
    printf("[VoiceCtl] [执行动作] 意图匹配 -> %s (步进: %.1f)\n", cmd->desc, cmd->param_angle);

    float next_r = g_target_r;
    float next_p = g_target_p;
    float next_y = g_target_y;
    float next_body = g_target_body;
    float next_ant_r = g_target_ant_r;
    float next_ant_l = g_target_ant_l;

    // 1. 计算累计增量目标
    switch (action_id) {
    case 0:
        next_y += cmd->param_angle;
        break;  // 头部左转 (Yaw)
    case 1:
        next_y -= cmd->param_angle;
        break;  // 头部右转 (Yaw)
    case 2:
        next_p -= cmd->param_angle;
        break;  // 抬头 (Pitch)
    case 3:
        next_p += cmd->param_angle;
        break;  // 低头 (Pitch)
    case 4:
        next_r += cmd->param_angle;
        break;  // 左歪头 (Roll)
    case 5:
        next_r -= cmd->param_angle;
        break;  // 右歪头 (Roll)
    case 6:
        next_body += cmd->param_angle;
        break;  // 身体左转
    case 7:
        next_body -= cmd->param_angle;
        break;  // 身体右转
    case 8:
        next_r = 0;
        next_p = 0;
        next_y = 0;
        next_body = 0;
        next_ant_r = 0;
        next_ant_l = 0;
        break;  // 复位
    case 9:
    case 10:
    case 11:
    case 12: {
        /* 舞蹈动作统一走 dance_player */
        static const char *dance_names[] = {
            [9] = "headbanger", [10] = "jackson",
            [11] = "chicken",   [12] = "uh_huh_tilt",
        };
        const DanceRoutine *routine = dance_player_find(
            g_dance_routines, g_dance_routine_count, dance_names[action_id]);
        if (routine) {
            /* 使用默认音频配置（48kHz 播放） */
            DanceAudioConfig dac = {-1, -1, 16000, 1, 48000, 1};
            dance_player_execute(routine, g_async_motor_ctrl, &dac);
        }
        g_target_r = 0;
        g_target_p = 0;
        g_target_y = 0;
        g_target_body = 0;
        g_target_ant_r = 0;
        g_target_ant_l = 0;
        break;
    }
    default:
        break;
    }

    // 2. 边界检查 (Limit Checking)
    // 身: ±180° (内限 160°)
    if (next_body > 160.0f || next_body < -160.0f) {
        printf("[VoiceCtl] 警告: 身体转动拟达到极限 (拟设定: %.1f, 范围: ±160)\n", next_body);
        return ACTION_LIMIT_EXCEEDED;
    }
    // 头 Yaw: ±170°, Pitch: ±35°, Roll: ±25°
    if (next_y > 170.0f || next_y < -170.0f) {
        printf("[VoiceCtl] 警告: 头部偏航拟达到极限 (拟设定: %.1f, 范围: ±170)\n", next_y);
        return ACTION_LIMIT_EXCEEDED;
    }
    if (next_p > 35.0f || next_p < -35.0f) {
        printf("[VoiceCtl] 警告: 头部俯仰拟达到极限 (拟设定: %.1f, 范围: ±35)\n", next_p);
        return ACTION_LIMIT_EXCEEDED;
    }
    if (next_r > 25.0f || next_r < -25.0f) {
        printf("[VoiceCtl] 警告: 头部滚转拟达到极限 (拟设定: %.1f, 范围: ±25)\n", next_r);
        return ACTION_LIMIT_EXCEEDED;
    }

    // 3. 更新目标并推送
    g_target_r = next_r;
    g_target_p = next_p;
    g_target_y = next_y;
    g_target_body = next_body;
    g_target_ant_r = next_ant_r;
    g_target_ant_l = next_ant_l;

    async_motor_controller_set_target(g_async_motor_ctrl, g_target_r, g_target_p, g_target_y,
                                        g_target_body, g_target_ant_r, g_target_ant_l);

    return 0;
}
