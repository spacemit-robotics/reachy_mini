/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * LLM 自主编排动作系统 - 流式 JSON 解析器
 *
 * 功能:
 * - 增量接收 LLM 输出的 token 流
 * - 当 actions 数组中一个完整元素被解析出来时，立即入队
 * - 线程安全的动作队列 (ring buffer)
 */

#ifndef ACTION_STREAM_PARSER_H
#define ACTION_STREAM_PARSER_H

#include <stddef.h>
#include <stdbool.h>
#include <pthread.h>

#include "action_executor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 * 动作队列 (Ring Buffer, 线程安全)
 * ==================================================================== */

#define ACTION_QUEUE_CAPACITY 32

typedef struct {
    ActionNode items[ACTION_QUEUE_CAPACITY];
    int head;           /* 消费者读取位置 */
    int tail;           /* 生产者写入位置 */
    int count;          /* 当前元素数量 */
    bool finished;      /* 生产者标记完成 */
    bool error;         /* 解析错误标志 */
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
} ActionQueue;

/**
 * 初始化动作队列
 */
void action_queue_init(ActionQueue *queue);

/**
 * 销毁动作队列
 */
void action_queue_destroy(ActionQueue *queue);

/**
 * 入队 (生产者调用, 阻塞式)
 * @return true 成功, false 队列已关闭或 abort
 */
bool action_queue_push(ActionQueue *queue, const ActionNode *node);

/**
 * 出队 (消费者调用, 阻塞式)
 * @return true 成功取出一个节点, false 队列为空且已完成
 */
bool action_queue_pop(ActionQueue *queue, ActionNode *out_node);

/**
 * 标记生产完成
 */
void action_queue_finish(ActionQueue *queue);

/**
 * 标记解析错误
 */
void action_queue_set_error(ActionQueue *queue);

/**
 * 查询是否有错误
 */
bool action_queue_has_error(ActionQueue *queue);

/**
 * 清空队列
 */
void action_queue_clear(ActionQueue *queue);

/* ====================================================================
 * 流式解析器
 * ==================================================================== */

/**
 * 解析器状态
 */
typedef enum {
    STREAM_STATE_INIT = 0,       /* 等待第一个 { */
    STREAM_STATE_FIND_ACTIONS,   /* 在顶层寻找 "actions" 键 */
    STREAM_STATE_IN_ARRAY,       /* 在 actions 数组内 */
    STREAM_STATE_IN_OBJECT,      /* 在一个动作对象内 */
    STREAM_STATE_DONE,           /* 解析完成 */
    STREAM_STATE_ERROR,          /* 解析错误 */
} StreamParserState;

/**
 * 错误类型枚举
 */
typedef enum {
    STREAM_ERROR_NONE = 0,
    STREAM_ERROR_BUFFER_OVERFLOW,   /* 缓冲区溢出 */
    STREAM_ERROR_JSON_PARSE,        /* JSON 解析失败 */
    STREAM_ERROR_INCOMPLETE,        /* 流结束时有未完成的对象 */
    STREAM_ERROR_QUEUE_FULL,        /* 队列满且被中断 */
} StreamErrorType;

/**
 * 错误回调函数类型
 * @param error_type  错误类型
 * @param message     错误描述
 * @param user_data   用户数据
 */
typedef void (*StreamErrorCallback)(
    StreamErrorType error_type,
    const char *message,
    void *user_data);

typedef struct {
    ActionQueue *queue;         /* 目标队列 */
    StreamParserState state;

    /* 缓冲区: 累积当前正在解析的 JSON 对象/块 */
    char buffer[4096];
    int buf_len;

    /* 括号深度追踪 */
    int brace_depth;            /* {} 深度 */
    int bracket_depth;          /* [] 深度 */

    /* 在对象内的起始位置 */
    int obj_start;              /* 当前对象的起始位置 (buffer 内偏移) */

    /* 顶层 JSON 解析用 */
    bool in_string;             /* 当前在字符串内 */
    bool escape_next;           /* 下一个字符被转义 */

    /* 统计 */
    int parsed_count;           /* 已成功解析的动作数 */
    int failed_count;           /* 解析失败的动作数 */

    /* 错误回调 */
    StreamErrorCallback error_callback;
    void *error_user_data;

    /* 最后的错误类型 */
    StreamErrorType last_error;
} ActionStreamParser;

/**
 * 创建流式解析器
 * @param queue  目标动作队列
 * @return       解析器实例
 */
ActionStreamParser *action_stream_parser_create(ActionQueue *queue);

/**
 * 喂入 LLM 输出的 token 片段
 * @param parser  解析器实例
 * @param chunk   数据片段
 * @param len     数据长度
 */
void action_stream_parser_feed(
        ActionStreamParser *parser,
        const char *chunk, size_t len);

/**
 * 通知 LLM 输出结束
 * @param parser  解析器实例
 */
void action_stream_parser_finish(ActionStreamParser *parser);

/**
 * 销毁解析器
 * @param parser  解析器实例
 */
void action_stream_parser_destroy(ActionStreamParser *parser);

/**
 * 设置错误回调 (用于通知上层应用处理错误)
 * @param parser    解析器实例
 * @param callback  回调函数
 * @param user_data 用户数据 (传入回调)
 */
void action_stream_parser_set_error_callback(
    ActionStreamParser *parser,
    StreamErrorCallback callback,
    void *user_data);

/**
 * 获取最后的错误类型
 * @param parser  解析器实例
 * @return        错误类型
 */
StreamErrorType action_stream_parser_get_last_error(ActionStreamParser *parser);

/**
 * 获取解析统计信息
 * @param parser        解析器实例
 * @param out_success   [输出] 成功解析数
 * @param out_failed    [输出] 解析失败数
 */
void action_stream_parser_get_stats(
    ActionStreamParser *parser,
    int *out_success,
    int *out_failed);

/**
 * 重置解析器状态 (用于自动恢复)
 * 不重新创建实例，清除错误状态并重置到初始状态
 * @param parser  解析器实例
 */
void action_stream_parser_reset(ActionStreamParser *parser);

/**
 * 检查解析器是否处于错误状态
 * @param parser  解析器实例
 * @return        true 表示处于错误状态
 */
bool action_stream_parser_has_error(ActionStreamParser *parser);

#ifdef __cplusplus
}
#endif

#endif /* ACTION_STREAM_PARSER_H */
