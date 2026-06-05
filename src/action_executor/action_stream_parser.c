/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * LLM 自主编排动作系统 - 流式 JSON 解析器实现
 */

#include "action_stream_parser.h"

#include <cJSON.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ====================================================================
 * 内部辅助: 触发错误回调
 * ==================================================================== */

static void trigger_error_callback(
        ActionStreamParser *parser,
        StreamErrorType error_type,
        const char *message) {
    parser->last_error = error_type;
    if (parser->error_callback) {
        parser->error_callback(error_type, message, parser->error_user_data);
    }
}

/* ====================================================================
 * 动作队列实现
 * ==================================================================== */

void action_queue_init(ActionQueue *queue) {
    memset(queue, 0, sizeof(ActionQueue));
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->finished = false;
    queue->error = false;
    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);
}

void action_queue_destroy(ActionQueue *queue) {
    pthread_mutex_destroy(&queue->mutex);
    pthread_cond_destroy(&queue->not_empty);
    pthread_cond_destroy(&queue->not_full);
}

bool action_queue_push(ActionQueue *queue, const ActionNode *node) {
    pthread_mutex_lock(&queue->mutex);

    /* 等待空间可用 */
    while (queue->count >= ACTION_QUEUE_CAPACITY && !g_action_abort) {
        pthread_cond_wait(&queue->not_full, &queue->mutex);
    }

    if (g_action_abort) {
        pthread_mutex_unlock(&queue->mutex);
        return false;
    }

    queue->items[queue->tail] = *node;
    queue->tail = (queue->tail + 1) % ACTION_QUEUE_CAPACITY;
    queue->count++;

    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
    return true;
}

bool action_queue_pop(ActionQueue *queue, ActionNode *out_node) {
    pthread_mutex_lock(&queue->mutex);

    /* 等待数据或完成信号 */
    while (queue->count == 0 && !queue->finished && !g_action_abort) {
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    if (queue->count == 0) {
        pthread_mutex_unlock(&queue->mutex);
        return false;
    }

    *out_node = queue->items[queue->head];
    queue->head = (queue->head + 1) % ACTION_QUEUE_CAPACITY;
    queue->count--;

    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
    return true;
}

void action_queue_finish(ActionQueue *queue) {
    pthread_mutex_lock(&queue->mutex);
    queue->finished = true;
    pthread_cond_broadcast(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
}

void action_queue_set_error(ActionQueue *queue) {
    pthread_mutex_lock(&queue->mutex);
    queue->error = true;
    queue->finished = true;
    pthread_cond_broadcast(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
}

bool action_queue_has_error(ActionQueue *queue) {
    pthread_mutex_lock(&queue->mutex);
    bool err = queue->error;
    pthread_mutex_unlock(&queue->mutex);
    return err;
}

void action_queue_clear(ActionQueue *queue) {
    pthread_mutex_lock(&queue->mutex);
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->finished = false;
    queue->error = false;
    pthread_cond_broadcast(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
}

/* ====================================================================
 * 流式解析器: 内部辅助
 * ==================================================================== */

/**
 * 尝试将 buffer 中的一段 JSON 对象解析为 ActionNode 并入队
 */
static bool try_parse_and_enqueue(
        ActionStreamParser *parser,
        const char *json_str, int json_len) {
    /* 临时 null-terminate */
    char *tmp = (char *)malloc(json_len + 1);
    if (!tmp) {
        printf("[StreamParser] 错误: malloc 失败 (%d bytes)\n", json_len + 1);
        parser->failed_count++;
        trigger_error_callback(parser, STREAM_ERROR_JSON_PARSE,
            "内存分配失败");
        return false;
    }
    memcpy(tmp, json_str, json_len);
    tmp[json_len] = '\0';

    cJSON *obj = cJSON_Parse(tmp);
    free(tmp);

    if (!obj) {
        printf("[StreamParser] 警告: 单个动作对象解析失败\n");
        parser->failed_count++;
        trigger_error_callback(parser, STREAM_ERROR_JSON_PARSE,
            "JSON 解析失败");
        return false;
    }

    /* 复用 action_executor 的解析逻辑 */
    ActionSequence temp_seq;
    memset(&temp_seq, 0, sizeof(temp_seq));

    /* 通过包装成 {"actions": [obj]} 来复用 action_executor_parse */
    /* 但更高效的方式是直接解析单个对象 */

    /* 内联解析单个动作对象字段 */
    ActionNode node;
    memset(&node, 0, sizeof(node));
    node.easing = EASING_LINEAR;
    node.duration_ms = 500;
    node.audio_sync_ms = -1;
    node.cycles = 1;

    cJSON *type_item = cJSON_GetObjectItem(obj, "type");
    if (!type_item || !cJSON_IsString(type_item)) {
        cJSON_Delete(obj);
        return false;
    }

    const char *type_str = type_item->valuestring;

    /* 解析类型 */
    if (strcasecmp(type_str, "pose") == 0) node.type = ACTION_TYPE_POSE;
    else if (strcasecmp(type_str, "move") == 0) node.type = ACTION_TYPE_MOVE;
    else if (strcasecmp(type_str, "dance") == 0) node.type = ACTION_TYPE_DANCE;
    else if (strcasecmp(type_str, "wait") == 0) node.type = ACTION_TYPE_WAIT;
    else if (strcasecmp(type_str, "tracker") == 0) node.type = ACTION_TYPE_TRACKER;
    else if (strcasecmp(type_str, "center") == 0) node.type = ACTION_TYPE_CENTER;
    else if (strcasecmp(type_str, "sequence") == 0) {
        /* sequence 在流式模式下不支持 (扁平 schema 策略) */
        printf("[StreamParser] 警告: 流式模式不支持 sequence 类型，跳过\n");
        cJSON_Delete(obj);
        return true; /* 不是错误，只是跳过 */
    } else {
        printf("[StreamParser] 警告: 未知类型 '%s'\n", type_str);
        cJSON_Delete(obj);
        return true;
    }

    /* 解析数值字段 */
    cJSON *val;

    val = cJSON_GetObjectItem(obj, "roll");
    if (val && cJSON_IsNumber(val)) { node.roll = (float)val->valuedouble; node.field_mask |= FIELD_ROLL; }

    val = cJSON_GetObjectItem(obj, "pitch");
    if (val && cJSON_IsNumber(val)) { node.pitch = (float)val->valuedouble; node.field_mask |= FIELD_PITCH; }

    val = cJSON_GetObjectItem(obj, "yaw");
    if (val && cJSON_IsNumber(val)) { node.yaw = (float)val->valuedouble; node.field_mask |= FIELD_YAW; }

    val = cJSON_GetObjectItem(obj, "body");
    if (val && cJSON_IsNumber(val)) { node.body = (float)val->valuedouble; node.field_mask |= FIELD_BODY; }

    val = cJSON_GetObjectItem(obj, "ant_r");
    if (val && cJSON_IsNumber(val)) { node.ant_r = (float)val->valuedouble; node.field_mask |= FIELD_ANT_R; }

    val = cJSON_GetObjectItem(obj, "ant_l");
    if (val && cJSON_IsNumber(val)) { node.ant_l = (float)val->valuedouble; node.field_mask |= FIELD_ANT_L; }

    /* move 增量字段 */
    if (node.type == ACTION_TYPE_MOVE) {
        val = cJSON_GetObjectItem(obj, "delta_roll");
        if (val && cJSON_IsNumber(val)) { node.roll = (float)val->valuedouble; node.field_mask |= FIELD_ROLL; }
        val = cJSON_GetObjectItem(obj, "delta_pitch");
        if (val && cJSON_IsNumber(val)) { node.pitch = (float)val->valuedouble; node.field_mask |= FIELD_PITCH; }
        val = cJSON_GetObjectItem(obj, "delta_yaw");
        if (val && cJSON_IsNumber(val)) { node.yaw = (float)val->valuedouble; node.field_mask |= FIELD_YAW; }
        val = cJSON_GetObjectItem(obj, "delta_body");
        if (val && cJSON_IsNumber(val)) { node.body = (float)val->valuedouble; node.field_mask |= FIELD_BODY; }
    }

    val = cJSON_GetObjectItem(obj, "duration_ms");
    if (val && cJSON_IsNumber(val)) { node.duration_ms = val->valueint; node.field_mask |= FIELD_DURATION; }

    val = cJSON_GetObjectItem(obj, "easing");
    if (val && cJSON_IsString(val)) {
        if (strcasecmp(val->valuestring, "hermite") == 0) node.easing = EASING_HERMITE;
        else if (strcasecmp(val->valuestring, "ease_in") == 0) node.easing = EASING_EASE_IN;
        else if (strcasecmp(val->valuestring, "ease_out") == 0) node.easing = EASING_EASE_OUT;
        node.field_mask |= FIELD_EASING;
    }

    val = cJSON_GetObjectItem(obj, "name");
    if (val && cJSON_IsString(val)) {
        strncpy(node.dance_name, val->valuestring, sizeof(node.dance_name) - 1);
    }

    val = cJSON_GetObjectItem(obj, "cycles");
    if (val && cJSON_IsNumber(val)) node.cycles = val->valueint;

    val = cJSON_GetObjectItem(obj, "bpm");
    if (val && cJSON_IsNumber(val)) node.bpm = (float)val->valuedouble;

    val = cJSON_GetObjectItem(obj, "mode");
    if (val && cJSON_IsString(val)) {
        node.tracker_mode =
            (strcasecmp(val->valuestring, "gesture") == 0)
            ? TRACKER_MODE_GESTURE : TRACKER_MODE_FACE;
    }

    val = cJSON_GetObjectItem(obj, "action");
    if (val && cJSON_IsString(val)) {
        node.tracker_action =
            (strcasecmp(val->valuestring, "stop") == 0)
            ? TRACKER_ACTION_STOP : TRACKER_ACTION_START;
    }

    cJSON_Delete(obj);

    /* 校验并 clamp */
    if (node.type == ACTION_TYPE_POSE) {
        node.roll = action_clamp_angle(node.roll, LIMIT_HEAD_ROLL);
        node.pitch = action_clamp_angle(node.pitch, LIMIT_HEAD_PITCH);
        node.yaw = action_clamp_angle(node.yaw, LIMIT_HEAD_YAW);
        node.body = action_clamp_angle(node.body, LIMIT_BODY);
        node.ant_r = action_clamp_angle(node.ant_r, LIMIT_ANTENNA);
        node.ant_l = action_clamp_angle(node.ant_l, LIMIT_ANTENNA);
    }

    if (node.duration_ms < MIN_DURATION_MS) node.duration_ms = MIN_DURATION_MS;
    if (node.duration_ms > MAX_DURATION_MS) node.duration_ms = MAX_DURATION_MS;

    /* 入队 */
    if (!action_queue_push(parser->queue, &node)) {
        parser->failed_count++;
        trigger_error_callback(parser, STREAM_ERROR_QUEUE_FULL,
            "动作队列已满或被中断");
        return false;
    }

    parser->parsed_count++;
    printf("[StreamParser] 已解析并入队动作 #%d (type=%d)\n",
        parser->parsed_count, node.type);

    return true;
}

/* ====================================================================
 * 流式解析器: 公开 API
 * ==================================================================== */

ActionStreamParser *action_stream_parser_create(ActionQueue *queue) {
    ActionStreamParser *parser = (ActionStreamParser *)calloc(1, sizeof(ActionStreamParser));
    if (!parser) return NULL;

    parser->queue = queue;
    parser->state = STREAM_STATE_INIT;
    parser->buf_len = 0;
    parser->brace_depth = 0;
    parser->bracket_depth = 0;
    parser->obj_start = -1;
    parser->in_string = false;
    parser->escape_next = false;
    parser->parsed_count = 0;
    parser->failed_count = 0;
    parser->error_callback = NULL;
    parser->error_user_data = NULL;
    parser->last_error = STREAM_ERROR_NONE;

    return parser;
}

void action_stream_parser_feed(
        ActionStreamParser *parser,
        const char *chunk, size_t len) {
    if (!parser || !chunk || len == 0) return;
    if (parser->state == STREAM_STATE_DONE ||
        parser->state == STREAM_STATE_ERROR) return;

    for (size_t i = 0; i < len; i++) {
        char c = chunk[i];

        /* 缓冲区溢出保护 */
        if (parser->buf_len >= (int)sizeof(parser->buffer) - 1) {
            printf("[StreamParser] 错误: 缓冲区溢出\n");
            parser->state = STREAM_STATE_ERROR;
            trigger_error_callback(parser, STREAM_ERROR_BUFFER_OVERFLOW,
                "JSON 缓冲区溢出，单个动作对象过大");
            action_queue_set_error(parser->queue);
            return;
        }

        /* 字符串状态追踪 */
        if (parser->escape_next) {
            parser->escape_next = false;
            parser->buffer[parser->buf_len++] = c;
            continue;
        }

        if (c == '\\' && parser->in_string) {
            parser->escape_next = true;
            parser->buffer[parser->buf_len++] = c;
            continue;
        }

        if (c == '"') {
            parser->in_string = !parser->in_string;
        }

        if (parser->in_string) {
            parser->buffer[parser->buf_len++] = c;
            continue;
        }

        /* 非字符串内的括号追踪 */
        switch (parser->state) {
        case STREAM_STATE_INIT:
            if (c == '{') {
                parser->state = STREAM_STATE_FIND_ACTIONS;
                parser->brace_depth = 1;
                parser->buffer[parser->buf_len++] = c;
            }
            break;

        case STREAM_STATE_FIND_ACTIONS:
            parser->buffer[parser->buf_len++] = c;
            if (c == '[') {
                parser->bracket_depth++;
                /* 检查是否是 actions 数组的开始 */
                /* 简化: 假设第一个 [ 就是 actions 数组 */
                parser->state = STREAM_STATE_IN_ARRAY;
            } else if (c == '{') {
                parser->brace_depth++;
            } else if (c == '}') {
                parser->brace_depth--;
                if (parser->brace_depth == 0) {
                    /* 顶层 JSON 闭合但没找到数组 → 可能是单个动作对象 */
                    parser->buffer[parser->buf_len] = '\0';
                    try_parse_and_enqueue(parser, parser->buffer, parser->buf_len);
                    parser->state = STREAM_STATE_DONE;
                }
            }
            break;

        case STREAM_STATE_IN_ARRAY:
            if (c == '{') {
                parser->brace_depth++;
                if (parser->brace_depth == 2) {
                    /* 进入一个动作对象，记录起始位置 */
                    parser->obj_start = parser->buf_len;
                    parser->state = STREAM_STATE_IN_OBJECT;
                    parser->buffer[parser->buf_len++] = c;
                }
                /* brace_depth != 2 时忽略（不应该发生） */
            } else if (c == ']') {
                parser->bracket_depth--;
                if (parser->bracket_depth == 0) {
                    parser->state = STREAM_STATE_DONE;
                }
                /* 数组结束符无需缓存 */
            }
            /* 跳过对象间的逗号、空白等字符，不写入缓冲区 */
            break;

        case STREAM_STATE_IN_OBJECT:
            parser->buffer[parser->buf_len++] = c;
            if (c == '{') {
                parser->brace_depth++;
            } else if (c == '}') {
                parser->brace_depth--;
                if (parser->brace_depth == 1) {
                    /* 一个完整的动作对象结束 */
                    int obj_len = parser->buf_len - parser->obj_start;
                    try_parse_and_enqueue(
                        parser,
                        &parser->buffer[parser->obj_start],
                        obj_len);

                    /* 清除已消费内容，回绕缓冲区防止溢出 */
                    parser->buf_len = 0;
                    parser->obj_start = -1;
                    parser->state = STREAM_STATE_IN_ARRAY;
                }
            }
            break;

        case STREAM_STATE_DONE:
        case STREAM_STATE_ERROR:
            return;
        }
    }
}

void action_stream_parser_finish(ActionStreamParser *parser) {
    if (!parser) return;

    /* 如果还有未完成的对象在缓冲区，尝试解析 */
    if (parser->state == STREAM_STATE_IN_OBJECT && parser->obj_start >= 0) {
        printf("[StreamParser] 警告: 流结束时有未完成的动作对象\n");
        trigger_error_callback(parser, STREAM_ERROR_INCOMPLETE,
            "LLM 输出截断，存在未完成的动作对象");
    }

    parser->state = STREAM_STATE_DONE;
    action_queue_finish(parser->queue);

    printf("[StreamParser] 流解析完成: 成功 %d 个, 失败 %d 个\n",
        parser->parsed_count, parser->failed_count);
}

void action_stream_parser_destroy(ActionStreamParser *parser) {
    if (parser) {
        free(parser);
    }
}

void action_stream_parser_set_error_callback(
        ActionStreamParser *parser,
        StreamErrorCallback callback,
        void *user_data) {
    if (!parser) return;
    parser->error_callback = callback;
    parser->error_user_data = user_data;
}

StreamErrorType action_stream_parser_get_last_error(ActionStreamParser *parser) {
    if (!parser) return STREAM_ERROR_NONE;
    return parser->last_error;
}

void action_stream_parser_get_stats(
        ActionStreamParser *parser,
        int *out_success,
        int *out_failed) {
    if (!parser) {
        if (out_success) *out_success = 0;
        if (out_failed) *out_failed = 0;
        return;
    }
    if (out_success) *out_success = parser->parsed_count;
    if (out_failed) *out_failed = parser->failed_count;
}

void action_stream_parser_reset(ActionStreamParser *parser) {
    if (!parser) return;

    /* 重置解析状态 */
    parser->state = STREAM_STATE_INIT;
    parser->buf_len = 0;
    parser->brace_depth = 0;
    parser->bracket_depth = 0;
    parser->obj_start = -1;
    parser->in_string = false;
    parser->escape_next = false;

    /* 重置统计 */
    parser->parsed_count = 0;
    parser->failed_count = 0;
    parser->last_error = STREAM_ERROR_NONE;

    /* 重置队列 */
    if (parser->queue) {
        action_queue_clear(parser->queue);
    }

    printf("[StreamParser] 解析器已重置，可继续使用\n");
}

bool action_stream_parser_has_error(ActionStreamParser *parser) {
    if (!parser) return false;
    return parser->state == STREAM_STATE_ERROR ||
            parser->last_error != STREAM_ERROR_NONE;
}
