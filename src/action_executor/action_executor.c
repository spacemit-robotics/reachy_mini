/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * LLM 自主编排动作系统 - 动作执行引擎实现
 */

#include "action_executor.h"

#include <cJSON.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "dance_player.h"
#include "tracker_manager.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ====================================================================
 * 全局静态分配
 * ==================================================================== */

/* 全局中断标志 */
volatile int g_action_abort = 0;

/* 刹车轨迹执行标志与二次中断标志 */
static volatile int g_brake_active = 0;
static volatile int g_brake_abort = 0;

/* 全局静态动作序列池 */
static ActionSequence g_action_pool;

/* ====================================================================
 * 辅助函数
 * ==================================================================== */

float action_clamp_angle(float angle, float limit) {
    if (angle > limit) return limit;
    if (angle < -limit) return -limit;
    return angle;
}

static int clamp_duration(int duration_ms) {
    if (duration_ms < MIN_DURATION_MS) return MIN_DURATION_MS;
    if (duration_ms > MAX_DURATION_MS) return MAX_DURATION_MS;
    return duration_ms;
}

float action_easing_eval(float t, EasingType easing) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;

    switch (easing) {
    case EASING_HERMITE:
        /* Hermite smooth step: 3t^2 - 2t^3 */
        return t * t * (3.0f - 2.0f * t);
    case EASING_EASE_IN:
        /* Quadratic ease in: t^2 */
        return t * t;
    case EASING_EASE_OUT:
        /* Quadratic ease out: 1 - (1-t)^2 */
        return 1.0f - (1.0f - t) * (1.0f - t);
    case EASING_LINEAR:
    default:
        return t;
    }
}

static EasingType parse_easing(const char *str) {
    if (!str) return EASING_LINEAR;
    if (strcasecmp(str, "hermite") == 0) return EASING_HERMITE;
    if (strcasecmp(str, "ease_in") == 0) return EASING_EASE_IN;
    if (strcasecmp(str, "ease_out") == 0) return EASING_EASE_OUT;
    return EASING_LINEAR;
}

static void init_action_node(ActionNode *node) {
    memset(node, 0, sizeof(ActionNode));
    node->type = ACTION_TYPE_POSE;
    node->easing = EASING_LINEAR;
    node->duration_ms = 500;
    node->audio_sync_ms = -1;
    node->cycles = 1;
    node->field_mask = 0;
}

/* ====================================================================
 * JSON 预处理: 提取第一个 {...} 块
 * ==================================================================== */

static const char *extract_json_block(const char *input, const char **end_ptr) {
    const char *start = strchr(input, '{');
    if (!start) return NULL;

    int depth = 0;
    const char *p = start;
    while (*p) {
        if (*p == '{') depth++;
        else if (*p == '}') {
            depth--;
            if (depth == 0) {
                if (end_ptr) *end_ptr = p + 1;
                return start;
            }
        }
        p++;
    }
    return NULL; /* 不完整的 JSON */
}

/* ====================================================================
 * JSON 解析 (递归展平 sequence)
 * ==================================================================== */

static int parse_action_node(cJSON *item, ActionSequence *seq, int depth);

static int parse_actions_array(
        cJSON *actions_arr, ActionSequence *seq,
        int repeat, int depth) {
    if (!cJSON_IsArray(actions_arr)) return ACTION_EXEC_ERR_PARSE;
    if (depth > SEQUENCE_NEST_MAX) return ACTION_EXEC_ERR_NEST;

    int count = cJSON_GetArraySize(actions_arr);
    if (count == 0) return ACTION_EXEC_OK;

    /* 限制 repeat */
    if (repeat > MAX_REPEAT) repeat = MAX_REPEAT;
    if (repeat < 1) repeat = 1;

    for (int r = 0; r < repeat; r++) {
        cJSON *child = NULL;
        cJSON_ArrayForEach(child, actions_arr) {
            int ret = parse_action_node(child, seq, depth);
            if (ret != ACTION_EXEC_OK) return ret;
        }
    }

    return ACTION_EXEC_OK;
}

static int parse_action_node(cJSON *item, ActionSequence *seq, int depth) {
    if (!cJSON_IsObject(item)) return ACTION_EXEC_ERR_PARSE;

    cJSON *type_item = cJSON_GetObjectItem(item, "type");
    if (!type_item || !cJSON_IsString(type_item)) return ACTION_EXEC_ERR_PARSE;

    const char *type_str = type_item->valuestring;

    /* sequence 类型: 递归展平 */
    if (strcasecmp(type_str, "sequence") == 0) {
        cJSON *sub_actions = cJSON_GetObjectItem(item, "actions");
        cJSON *repeat_item = cJSON_GetObjectItem(item, "repeat");
        int repeat = 1;
        if (repeat_item && cJSON_IsNumber(repeat_item)) {
            repeat = repeat_item->valueint;
        }
        return parse_actions_array(sub_actions, seq, repeat, depth + 1);
    }

    /* 非 sequence 类型: 分配节点 */
    if (seq->count >= ACTION_POOL_SIZE) {
        printf("[ActionExecutor] 错误: 动作池已满 (%d/%d)\n", seq->count, ACTION_POOL_SIZE);
        return ACTION_EXEC_ERR_POOL;
    }

    ActionNode *node = &seq->nodes[seq->count];
    init_action_node(node);

    /* 解析 type */
    if (strcasecmp(type_str, "pose") == 0) {
        node->type = ACTION_TYPE_POSE;
    } else if (strcasecmp(type_str, "move") == 0) {
        node->type = ACTION_TYPE_MOVE;
    } else if (strcasecmp(type_str, "dance") == 0) {
        node->type = ACTION_TYPE_DANCE;
    } else if (strcasecmp(type_str, "wait") == 0) {
        node->type = ACTION_TYPE_WAIT;
    } else if (strcasecmp(type_str, "tracker") == 0) {
        node->type = ACTION_TYPE_TRACKER;
    } else if (strcasecmp(type_str, "center") == 0) {
        node->type = ACTION_TYPE_CENTER;
    } else {
        printf("[ActionExecutor] 警告: 未知动作类型 '%s'，跳过\n", type_str);
        return ACTION_EXEC_OK; /* 跳过未知类型 */
    }

    /* 解析通用数值字段 */
    cJSON *val;

    val = cJSON_GetObjectItem(item, "roll");
    if (val && cJSON_IsNumber(val)) { node->roll = (float)val->valuedouble; node->field_mask |= FIELD_ROLL; }

    val = cJSON_GetObjectItem(item, "pitch");
    if (val && cJSON_IsNumber(val)) { node->pitch = (float)val->valuedouble; node->field_mask |= FIELD_PITCH; }

    val = cJSON_GetObjectItem(item, "yaw");
    if (val && cJSON_IsNumber(val)) { node->yaw = (float)val->valuedouble; node->field_mask |= FIELD_YAW; }

    val = cJSON_GetObjectItem(item, "body");
    if (val && cJSON_IsNumber(val)) { node->body = (float)val->valuedouble; node->field_mask |= FIELD_BODY; }

    val = cJSON_GetObjectItem(item, "ant_r");
    if (val && cJSON_IsNumber(val)) { node->ant_r = (float)val->valuedouble; node->field_mask |= FIELD_ANT_R; }

    val = cJSON_GetObjectItem(item, "ant_l");
    if (val && cJSON_IsNumber(val)) { node->ant_l = (float)val->valuedouble; node->field_mask |= FIELD_ANT_L; }

    /* move 类型的增量字段 - 仅非零值才覆盖，避免 delta_*:0 误清除绝对值字段 */
    if (node->type == ACTION_TYPE_MOVE) {
        val = cJSON_GetObjectItem(item, "delta_roll");
        if (val && cJSON_IsNumber(val) && val->valuedouble != 0.0) {
            node->roll = (float)val->valuedouble;
            node->field_mask |= FIELD_ROLL;
        }

        val = cJSON_GetObjectItem(item, "delta_pitch");
        if (val && cJSON_IsNumber(val) && val->valuedouble != 0.0) {
            node->pitch = (float)val->valuedouble;
            node->field_mask |= FIELD_PITCH;
        }

        val = cJSON_GetObjectItem(item, "delta_yaw");
        if (val && cJSON_IsNumber(val) && val->valuedouble != 0.0) {
            node->yaw = (float)val->valuedouble;
            node->field_mask |= FIELD_YAW;
        }

        val = cJSON_GetObjectItem(item, "delta_body");
        if (val && cJSON_IsNumber(val) && val->valuedouble != 0.0) {
            node->body = (float)val->valuedouble;
            node->field_mask |= FIELD_BODY;
        }

        val = cJSON_GetObjectItem(item, "delta_ant_r");
        if (val && cJSON_IsNumber(val) && val->valuedouble != 0.0) {
            node->ant_r = (float)val->valuedouble;
            node->field_mask |= FIELD_ANT_R;
        }

        val = cJSON_GetObjectItem(item, "delta_ant_l");
        if (val && cJSON_IsNumber(val) && val->valuedouble != 0.0) {
            node->ant_l = (float)val->valuedouble;
            node->field_mask |= FIELD_ANT_L;
        }
    }

    /* duration_ms */
    val = cJSON_GetObjectItem(item, "duration_ms");
    if (val && cJSON_IsNumber(val)) {
        node->duration_ms = val->valueint;
        node->field_mask |= FIELD_DURATION;
    }

    /* easing */
    val = cJSON_GetObjectItem(item, "easing");
    if (val && cJSON_IsString(val)) {
        node->easing = parse_easing(val->valuestring);
        node->field_mask |= FIELD_EASING;
    }

    /* dance 字段 */
    val = cJSON_GetObjectItem(item, "name");
    if (val && cJSON_IsString(val)) {
        strncpy(node->dance_name, val->valuestring, sizeof(node->dance_name) - 1);
        node->dance_name[sizeof(node->dance_name) - 1] = '\0';
    }

    val = cJSON_GetObjectItem(item, "cycles");
    if (val && cJSON_IsNumber(val)) { node->cycles = val->valueint; }

    val = cJSON_GetObjectItem(item, "bpm");
    if (val && cJSON_IsNumber(val)) { node->bpm = (float)val->valuedouble; }

    /* tracker 字段 */
    val = cJSON_GetObjectItem(item, "mode");
    if (val && cJSON_IsString(val)) {
        if (strcasecmp(val->valuestring, "gesture") == 0) {
            node->tracker_mode = TRACKER_MODE_GESTURE;
        } else {
            node->tracker_mode = TRACKER_MODE_FACE;
        }
    }

    val = cJSON_GetObjectItem(item, "action");
    if (val && cJSON_IsString(val)) {
        if (strcasecmp(val->valuestring, "stop") == 0) {
            node->tracker_action = TRACKER_ACTION_STOP;
        } else {
            node->tracker_action = TRACKER_ACTION_START;
        }
    }

    /* audio_sync_ms (预留) */
    val = cJSON_GetObjectItem(item, "audio_sync_ms");
    if (val && cJSON_IsNumber(val)) { node->audio_sync_ms = val->valueint; }

    seq->count++;
    return ACTION_EXEC_OK;
}

/* ====================================================================
 * 公开 API: 解析
 * ==================================================================== */

int action_executor_parse(const char *json_str, ActionSequence *seq) {
    if (!json_str || !seq) return ACTION_EXEC_ERR_PARAM;

    /* 清空序列 */
    memset(seq, 0, sizeof(ActionSequence));

    /* 预处理: 提取第一个 JSON 块 */
    const char *json_end = NULL;
    const char *json_start = extract_json_block(json_str, &json_end);
    if (!json_start) {
        printf("[ActionExecutor] 错误: 未找到有效的 JSON 块\n");
        return ACTION_EXEC_ERR_PARSE;
    }

    /* 复制 JSON 子串用于解析 (零运行时 malloc) */
    static char s_json_buf[8192]; /* 预留静态缓冲区 */
    size_t json_len = (size_t)(json_end - json_start);
    if (json_len >= sizeof(s_json_buf)) {
        printf("[ActionExecutor] 错误: JSON 块过长 (%zu bytes)\n", json_len);
        return ACTION_EXEC_ERR_PARAM; /* 长度超限返回 PARAM 错误 */
    }
    char *json_copy = s_json_buf;
    memcpy(json_copy, json_start, json_len);
    json_copy[json_len] = '\0';

    /* cJSON 解析 */
    cJSON *root = cJSON_Parse(json_copy);
    /* 静态缓冲区无需 free */

    if (!root) {
        printf("[ActionExecutor] 错误: JSON 解析失败: %s\n",
            cJSON_GetErrorPtr() ? cJSON_GetErrorPtr() : "unknown");
        return ACTION_EXEC_ERR_PARSE;
    }

    /* 查找 actions 数组 */
    cJSON *actions = cJSON_GetObjectItem(root, "actions");
    if (!actions || !cJSON_IsArray(actions)) {
        /* 尝试整体作为单个动作 */
        cJSON *type_item = cJSON_GetObjectItem(root, "type");
        if (type_item && cJSON_IsString(type_item)) {
            int ret = parse_action_node(root, seq, 0);
            cJSON_Delete(root);
            return ret;
        }
        printf("[ActionExecutor] 错误: JSON 中未找到 'actions' 数组\n");
        cJSON_Delete(root);
        return ACTION_EXEC_ERR_PARSE;
    }

    int ret = parse_actions_array(actions, seq, 1, 0);
    cJSON_Delete(root);

    if (ret == ACTION_EXEC_OK) {
        printf("[ActionExecutor] 解析成功: %d 个动作节点 (展平后)\n", seq->count);
    }

    return ret;
}

/* ====================================================================
 * 公开 API: 校验
 * ==================================================================== */

int action_executor_validate(ActionSequence *seq) {
    if (!seq) return ACTION_EXEC_ERR_PARAM;

    int total_duration = 0;

    for (int i = 0; i < seq->count; i++) {
        ActionNode *node = &seq->nodes[i];

        /* clamp 角度 */
        switch (node->type) {
        case ACTION_TYPE_POSE:
            node->roll = action_clamp_angle(node->roll, LIMIT_HEAD_ROLL);
            node->pitch = action_clamp_angle(node->pitch, LIMIT_HEAD_PITCH);
            node->yaw = action_clamp_angle(node->yaw, LIMIT_HEAD_YAW);
            node->body = action_clamp_angle(node->body, LIMIT_BODY);
            node->ant_r = action_clamp_angle(node->ant_r, LIMIT_ANTENNA);
            node->ant_l = action_clamp_angle(node->ant_l, LIMIT_ANTENNA);
            break;

        case ACTION_TYPE_MOVE:
            /* move 的增量值不做 clamp (执行时计算绝对值后再 clamp) */
            break;

        default:
            break;
        }

        /* clamp duration */
        if (node->type == ACTION_TYPE_POSE || node->type == ACTION_TYPE_MOVE ||
            node->type == ACTION_TYPE_WAIT || node->type == ACTION_TYPE_CENTER) {
            node->duration_ms = clamp_duration(node->duration_ms);
            total_duration += node->duration_ms;
        }

        /* dance cycles 限制 */
        if (node->type == ACTION_TYPE_DANCE) {
            if (node->cycles < 1) node->cycles = 1;
            if (node->cycles > 10) node->cycles = 10;
            /* 估算舞蹈时长 (按默认 BPM 和每周期拍数) */
            total_duration += node->cycles * 2000; /* 粗估 2s/cycle */
        }
    }

    if (total_duration > MAX_TOTAL_DURATION_MS) {
        printf("[ActionExecutor] 警告: 总时长 %dms 超过上限 %dms，截断\n",
            total_duration, MAX_TOTAL_DURATION_MS);
        /* 不返回错误，执行引擎会在超时后自动停止 */
    }

    return ACTION_EXEC_OK;
}

/* ====================================================================
 * 公开 API: 执行
 * ==================================================================== */

static void sleep_ms(int ms) {
    usleep(ms * 1000);
}

/**
 * 执行单个 pose/move 动作的插值过程
 * @param ctrl      异步电机控制器
 * @param target    目标位姿
 * @param start     起始位姿
 * @param duration_ms  过渡时间
 * @param easing    插值方式
 * @return          0 成功, -1 被中断
 */
static int execute_interpolated_move(
    AsyncMotorController *ctrl,
    const Pose6DOF *target,
    const Pose6DOF *start,
    int duration_ms,
    EasingType easing) {
    const int tick_ms = 10; /* 10ms 控制周期 */
    int steps = duration_ms / tick_ms;
    if (steps < 1) steps = 1;

    for (int s = 1; s <= steps; s++) {
        if (g_action_abort) return -1;

        float t = (float)s / (float)steps;
        float f = action_easing_eval(t, easing);

        float r = start->roll  + (target->roll  - start->roll)  * f;
        float p = start->pitch + (target->pitch - start->pitch) * f;
        float y = start->yaw   + (target->yaw   - start->yaw)   * f;
        float b = start->body  + (target->body  - start->body)  * f;
        float ar = start->ant_r + (target->ant_r - start->ant_r) * f;
        float al = start->ant_l + (target->ant_l - start->ant_l) * f;

        async_motor_controller_set_target(ctrl, r, p, y, b, ar, al);
        sleep_ms(tick_ms);
    }

    return 0;
}

int action_executor_run(ActionSequence *seq, AsyncMotorController *ctrl,
                        const Pose6DOF *initial_pose) {
    if (!seq || !ctrl) return ACTION_EXEC_ERR_PARAM;

    /* 当前位姿状态 */
    Pose6DOF current = {0};
    if (initial_pose) {
        current = *initial_pose;
    }

    int total_elapsed_ms = 0;

    for (seq->cursor = 0; seq->cursor < seq->count; seq->cursor++) {
        if (g_action_abort) {
            printf("[ActionExecutor] 执行被中断 (cursor=%d/%d)\n",
                seq->cursor, seq->count);
            return ACTION_EXEC_ERR_ABORT;
        }

        /* 总时长检查 */
        if (total_elapsed_ms > MAX_TOTAL_DURATION_MS) {
            printf("[ActionExecutor] 总时长超限，停止执行\n");
            break;
        }

        ActionNode *node = &seq->nodes[seq->cursor];

        switch (node->type) {
        case ACTION_TYPE_POSE: {
            /* 计算目标位姿 */
            Pose6DOF target = current;
            if (node->field_mask & FIELD_ROLL) target.roll = node->roll;
            if (node->field_mask & FIELD_PITCH) target.pitch = node->pitch;
            if (node->field_mask & FIELD_YAW) target.yaw = node->yaw;
            if (node->field_mask & FIELD_BODY) target.body = node->body;
            if (node->field_mask & FIELD_ANT_R) target.ant_r = node->ant_r;
            if (node->field_mask & FIELD_ANT_L) target.ant_l = node->ant_l;

            /* clamp */
            target.roll = action_clamp_angle(target.roll, LIMIT_HEAD_ROLL);
            target.pitch = action_clamp_angle(target.pitch, LIMIT_HEAD_PITCH);
            target.yaw = action_clamp_angle(target.yaw, LIMIT_HEAD_YAW);
            target.body = action_clamp_angle(target.body, LIMIT_BODY);
            target.ant_r = action_clamp_angle(target.ant_r, LIMIT_ANTENNA);
            target.ant_l = action_clamp_angle(target.ant_l, LIMIT_ANTENNA);

            printf("[ActionExecutor] POSE: r=%.1f p=%.1f y=%.1f b=%.1f "
                "ar=%.1f al=%.1f dur=%dms easing=%d\n",
                target.roll, target.pitch, target.yaw, target.body,
                target.ant_r, target.ant_l, node->duration_ms, node->easing);

            if (execute_interpolated_move(
                    ctrl, &target, &current,
                    node->duration_ms, node->easing) < 0) {
                return ACTION_EXEC_ERR_ABORT;
            }

            current = target;
            total_elapsed_ms += node->duration_ms;
            break;
        }

        case ACTION_TYPE_MOVE: {
            /* 增量运动 */
            Pose6DOF target = current;
            if (node->field_mask & FIELD_ROLL) target.roll += node->roll;
            if (node->field_mask & FIELD_PITCH) target.pitch += node->pitch;
            if (node->field_mask & FIELD_YAW) target.yaw += node->yaw;
            if (node->field_mask & FIELD_BODY) target.body += node->body;

            /* clamp */
            target.roll = action_clamp_angle(target.roll, LIMIT_HEAD_ROLL);
            target.pitch = action_clamp_angle(target.pitch, LIMIT_HEAD_PITCH);
            target.yaw = action_clamp_angle(target.yaw, LIMIT_HEAD_YAW);
            target.body = action_clamp_angle(target.body, LIMIT_BODY);

            printf("[ActionExecutor] MOVE: "
                "dr=%.1f dp=%.1f dy=%.1f db=%.1f dur=%dms\n",
                node->roll, node->pitch, node->yaw, node->body,
                node->duration_ms);

            if (execute_interpolated_move(
                    ctrl, &target, &current,
                    node->duration_ms, node->easing) < 0) {
                return ACTION_EXEC_ERR_ABORT;
            }

            current = target;
            total_elapsed_ms += node->duration_ms;
            break;
        }

        case ACTION_TYPE_DANCE: {
            printf("[ActionExecutor] DANCE: %s, cycles=%d\n",
                node->dance_name, node->cycles);

            const DanceRoutine *routine = dance_player_find(
                g_dance_routines, g_dance_routine_count, node->dance_name);
            if (routine) {
                DanceAudioConfig dac = {-1, -1, 16000, 1, 48000, 1};
                dance_player_execute(routine, ctrl, &dac);
            } else {
                printf("[ActionExecutor] 警告: 未找到舞蹈 '%s'\n",
                    node->dance_name);
            }

            /* dance 执行后位姿回零 */
            memset(&current, 0, sizeof(current));
            total_elapsed_ms += node->cycles * 2000; /* 粗估 */
            break;
        }

        case ACTION_TYPE_WAIT: {
            printf("[ActionExecutor] WAIT: %dms\n", node->duration_ms);
            int remaining = node->duration_ms;
            while (remaining > 0 && !g_action_abort) {
                int sleep_chunk = (remaining > 50) ? 50 : remaining;
                sleep_ms(sleep_chunk);
                remaining -= sleep_chunk;
            }
            if (g_action_abort) return ACTION_EXEC_ERR_ABORT;
            total_elapsed_ms += node->duration_ms;
            break;
        }

        case ACTION_TYPE_TRACKER: {
            if (node->tracker_action == TRACKER_ACTION_START) {
                printf("[ActionExecutor] TRACKER START: mode=%d\n", node->tracker_mode);
                if (node->tracker_mode == TRACKER_MODE_FACE) {
                    tracker_start(TRACKER_TYPE_FACE);
                } else {
                    tracker_start(TRACKER_TYPE_GESTURE);
                }
            } else {
                printf("[ActionExecutor] TRACKER STOP\n");
                tracker_stop_all();
            }
            break;
        }

        case ACTION_TYPE_CENTER: {
            int dur = (node->field_mask & FIELD_DURATION) ? node->duration_ms : 800;
            dur = clamp_duration(dur);
            printf("[ActionExecutor] CENTER: dur=%dms\n", dur);

            Pose6DOF zero = {0};
            if (execute_interpolated_move(
                    ctrl, &zero, &current,
                    dur, EASING_HERMITE) < 0) {
                return ACTION_EXEC_ERR_ABORT;
            }

            current = zero;
            total_elapsed_ms += dur;
            break;
        }

        default:
            printf("[ActionExecutor] 跳过未知动作类型: %d\n", node->type);
            break;
        }
    }

    printf("[ActionExecutor] 序列执行完成: %d 个动作, 总时长约 %dms\n",
        seq->count, total_elapsed_ms);
    return ACTION_EXEC_OK;
}

/* ====================================================================
 * 中断机制
 * ==================================================================== */

void action_executor_abort(void) {
    g_action_abort = 1;
    /* 如果刹车轨迹正在执行，标记二次中断 */
    if (g_brake_active) {
        g_brake_abort = 1;
    }
    /* 同时通知舞蹈子系统停止 */
    dance_request_stop();
    printf("[ActionExecutor] 收到中断信号\n");
}

void action_executor_reset_abort(void) {
    g_action_abort = 0;
}

/* ====================================================================
 * 平滑中断: Hermite 刹车轨迹
 * ==================================================================== */

void action_executor_abort_smooth(
    AsyncMotorController *ctrl,
    const Pose6DOF *current_pose,
    const Pose6DOF *current_vel) {
    if (!ctrl) return;

    /* 设置中断标志 */
    g_action_abort = 1;
    dance_request_stop();

    Pose6DOF pose = {0};
    Pose6DOF vel = {0};
    if (current_pose) pose = *current_pose;
    if (current_vel) vel = *current_vel;

    /* 计算最大分量速度 */
    float max_vel = fabsf(vel.roll);
    if (fabsf(vel.pitch) > max_vel) max_vel = fabsf(vel.pitch);
    if (fabsf(vel.yaw) > max_vel) max_vel = fabsf(vel.yaw);
    if (fabsf(vel.body) > max_vel) max_vel = fabsf(vel.body);

    /* 低速时直接回中 */
    if (max_vel < 5.0f) {
        printf("[ActionExecutor] 低速中断，直接回中\n");
        Pose6DOF zero = {0};
        execute_interpolated_move(ctrl, &zero, &pose, 1200, EASING_HERMITE);
        return;
    }

    /* 计算刹车时长: max(300ms, |V_max| / max_decel) */
    /* 假设最大减速度 400°/s^2 */
    float max_decel = 400.0f;
    int brake_ms = (int)(max_vel / max_decel * 1000.0f);
    if (brake_ms < 300) brake_ms = 300;
    if (brake_ms > 2000) brake_ms = 2000;

    printf("[ActionExecutor] 平滑刹车: max_vel=%.1f°/s, brake=%dms\n",
        max_vel, brake_ms);

    /*
     * Hermite 样条刹车:
     * 起点: current_pose, 起始速度: current_vel
     * 终点: (0,0,0,0,0,0), 终止速度: (0,0,0,0,0,0)
     *
     * Hermite 基函数:
     *   h00(t) = 2t^3 - 3t^2 + 1     (位置 P0)
     *   h10(t) = t^3 - 2t^2 + t       (切线 M0)
     *   h01(t) = -2t^3 + 3t^2         (位置 P1)
     *   h11(t) = t^3 - t^2            (切线 M1)
     *
     * P(t) = h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1
     * P1 = 0, M1 = 0 → P(t) = h00 * P0 + h10 * M0
     */

    const int tick_ms = 10;
    int steps = brake_ms / tick_ms;
    if (steps < 1) steps = 1;

    /* 速度转换为 Hermite 切线 (需要乘以总时间) */
    float T = (float)brake_ms / 1000.0f;

    /* 标记刹车轨迹执行中，重置二次中断标志 */
    g_brake_abort = 0;
    g_brake_active = 1;

    for (int s = 1; s <= steps; s++) {
        float t = (float)s / (float)steps;

        /* Hermite 基函数 */
        float h00 = 2.0f * t * t * t - 3.0f * t * t + 1.0f;
        float h10 = t * t * t - 2.0f * t * t + t;
        /* h01 = -2t^3 + 3t^2 = 1 - h00,  h11 = t^3 - t^2 → 不需要 (P1=M1=0) */

        float r = h00 * pose.roll  + h10 * vel.roll  * T;
        float p = h00 * pose.pitch + h10 * vel.pitch * T;
        float y = h00 * pose.yaw   + h10 * vel.yaw   * T;
        float b = h00 * pose.body  + h10 * vel.body  * T;
        float ar = h00 * pose.ant_r + h10 * vel.ant_r * T;
        float al = h00 * pose.ant_l + h10 * vel.ant_l * T;

        /* 安全 clamp */
        r = action_clamp_angle(r, LIMIT_HEAD_ROLL);
        p = action_clamp_angle(p, LIMIT_HEAD_PITCH);
        y = action_clamp_angle(y, LIMIT_HEAD_YAW);
        b = action_clamp_angle(b, LIMIT_BODY);
        ar = action_clamp_angle(ar, LIMIT_ANTENNA);
        al = action_clamp_angle(al, LIMIT_ANTENNA);

        async_motor_controller_set_target(ctrl, r, p, y, b, ar, al);
        sleep_ms(tick_ms);

        /* 二次 abort: 硬停兜底 (检查刹车中的二次中断) */
        if (g_brake_abort) {
            printf("[ActionExecutor] 二次中断，硬停\n");
            g_brake_active = 0;
            return;
        }
    }

    /* 确保最终到达零位 */
    async_motor_controller_set_target(ctrl, 0, 0, 0, 0, 0, 0);
    g_brake_active = 0;
    printf("[ActionExecutor] 刹车完成，已回中\n");
}

/* ====================================================================
 * 内存池获取
 * ==================================================================== */

ActionSequence *action_executor_get_pool(void) {
    return &g_action_pool;
}
