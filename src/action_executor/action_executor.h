/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * LLM 自主编排动作系统 - 动作执行引擎
 *
 * 设计要点:
 * - 静态内存池，零运行时 malloc
 * - 角度 clamp、速度上限、duration 下限
 * - 嵌套深度限制、总时长限制
 * - 平滑中断 (Hermite 刹车轨迹)
 */

#ifndef ACTION_EXECUTOR_H
#define ACTION_EXECUTOR_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "motor_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 * 常量定义
 * ==================================================================== */

#define ACTION_POOL_SIZE       64   /* 预分配动作节点池容量 */
#define SEQUENCE_NEST_MAX      3    /* 嵌套深度上限 */
#define MAX_REPEAT             20   /* sequence.repeat 上限 */
#define MAX_TOTAL_DURATION_MS  60000 /* 单次编排总时长上限 (60s) */
#define MIN_DURATION_MS        50   /* 最小 duration */
#define MAX_DURATION_MS        5000 /* 最大 duration */
#define MAX_ANGULAR_VEL        200.0f /* 最大角速度 °/s */

/* 角度限位 */
#define LIMIT_HEAD_YAW         170.0f
#define LIMIT_HEAD_PITCH       35.0f
#define LIMIT_HEAD_ROLL        25.0f
#define LIMIT_BODY             160.0f
#define LIMIT_ANTENNA          90.0f

/* 执行引擎错误码 */
#define ACTION_EXEC_OK         0
#define ACTION_EXEC_ERR_PARSE  -1   /* JSON 解析失败 */
#define ACTION_EXEC_ERR_VALID  -2   /* 校验失败 */
#define ACTION_EXEC_ERR_ABORT  -3   /* 被中断 */
#define ACTION_EXEC_ERR_PARAM  -4   /* 参数错误 */
#define ACTION_EXEC_ERR_POOL   -5   /* 内存池耗尽 */
#define ACTION_EXEC_ERR_NEST   -6   /* 嵌套过深 */

/* ====================================================================
 * 动作类型
 * ==================================================================== */

typedef enum {
    ACTION_TYPE_POSE = 0,     /* 绝对姿态目标 */
    ACTION_TYPE_MOVE,         /* 相对增量运动 */
    ACTION_TYPE_DANCE,        /* 调用已注册舞蹈 */
    ACTION_TYPE_WAIT,         /* 等待 */
    ACTION_TYPE_TRACKER,      /* 启停跟踪 */
    ACTION_TYPE_CENTER,       /* 回中 */
    ACTION_TYPE_SEQUENCE,     /* 嵌套子序列 */
    ACTION_TYPE_MAX
} ActionType;

/* 插值方式 */
typedef enum {
    EASING_LINEAR = 0,        /* 线性插值 (默认) */
    EASING_HERMITE,           /* Hermite 平滑 (起止速度为零) */
    EASING_EASE_IN,           /* 缓入 */
    EASING_EASE_OUT,          /* 缓出 */
} EasingType;

/* Tracker 模式 */
typedef enum {
    TRACKER_MODE_FACE = 0,
    TRACKER_MODE_GESTURE,
} TrackerMode;

/* Tracker 操作 */
typedef enum {
    TRACKER_ACTION_START = 0,
    TRACKER_ACTION_STOP,
} TrackerAction;

/* ====================================================================
 * 动作节点 (展平后的统一结构)
 * ==================================================================== */

typedef struct {
    ActionType type;

    /* pose / move 通用字段 */
    float roll;               /* 绝对/增量 roll (°) */
    float pitch;              /* 绝对/增量 pitch (°) */
    float yaw;                /* 绝对/增量 yaw (°) */
    float body;               /* 绝对/增量 body yaw (°) */
    float ant_r;              /* 右天线角度 (°) */
    float ant_l;              /* 左天线角度 (°) */
    int32_t duration_ms;      /* 过渡时间 */
    EasingType easing;        /* 插值方式 */

    /* dance 字段 */
    char dance_name[32];      /* 舞蹈名称 */
    int32_t cycles;           /* 舞蹈循环次数 */
    float bpm;                /* BPM (0 = 使用默认) */

    /* tracker 字段 */
    TrackerMode tracker_mode;
    TrackerAction tracker_action;

    /* sequence 展平标记 (解析时已展平，此处仅记录重复信息) */
    /* 不需要，解析时 inline expansion */

    /* TTS 音频同步 (预留, P6 阶段) */
    int32_t audio_sync_ms;    /* -1 = 不同步 */

    /* 内部标记: 该节点是否设置了各个字段 (用于 pose 类型的部分更新) */
    uint16_t field_mask;
} ActionNode;

/* field_mask 位定义 */
#define FIELD_ROLL      (1 << 0)
#define FIELD_PITCH     (1 << 1)
#define FIELD_YAW       (1 << 2)
#define FIELD_BODY      (1 << 3)
#define FIELD_ANT_R     (1 << 4)
#define FIELD_ANT_L     (1 << 5)
#define FIELD_DURATION   (1 << 6)
#define FIELD_EASING    (1 << 7)

/* ====================================================================
 * 动作序列 (静态内存池)
 * ==================================================================== */

typedef struct {
    ActionNode nodes[ACTION_POOL_SIZE];  /* 预分配动作节点池 */
    int count;                           /* 当前使用数量 */
    int cursor;                          /* 执行游标 */
} ActionSequence;

/* ====================================================================
 * 执行状态 (当前位姿缓存)
 * ==================================================================== */

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float body;
    float ant_r;
    float ant_l;
} Pose6DOF;

/* ====================================================================
 * 全局中断标志
 * ==================================================================== */

extern volatile int g_action_abort;

/* ====================================================================
 * 公开 API
 * ==================================================================== */

/**
 * 从 JSON 字符串解析动作序列到静态内存池
 * @param json   JSON 字符串 (符合设计文档 5.1 schema)
 * @param seq    输出: 解析后的动作序列 (静态分配, 调用者无需 free)
 * @return       ACTION_EXEC_OK 成功, 否则错误码
 */
int action_executor_parse(const char *json, ActionSequence *seq);

/**
 * 校验动作序列的合法性 (角度范围、时长合理性等)
 * 会自动 clamp 超限值
 * @param seq    待校验的动作序列
 * @return       ACTION_EXEC_OK 成功, 否则错误码
 */
int action_executor_validate(ActionSequence *seq);

/**
 * 执行动作序列
 * @param seq    已解析并校验的动作序列
 * @param ctrl   异步电机控制器
 * @param initial_pose  当前姿态 (NULL 则假定零位)
 * @return       ACTION_EXEC_OK 成功, ACTION_EXEC_ERR_ABORT 被中断
 */
int action_executor_run(ActionSequence *seq, AsyncMotorController *ctrl,
                        const Pose6DOF *initial_pose);

/**
 * 设置中断标志 (安全停止, 平滑刹车)
 */
void action_executor_abort(void);

/**
 * 平滑中断: 根据当前速度生成 Hermite 刹车轨迹后回中
 * @param ctrl   异步电机控制器
 * @param current_pose  当前估算位姿
 * @param current_vel   当前估算角速度 (°/s, 各轴)
 */
void action_executor_abort_smooth(
    AsyncMotorController *ctrl,
    const Pose6DOF *current_pose,
    const Pose6DOF *current_vel);

/**
 * 重置中断标志
 */
void action_executor_reset_abort(void);

/**
 * 获取全局静态动作序列实例
 * @return 指向全局 ActionSequence 的指针
 */
ActionSequence *action_executor_get_pool(void);

/* ====================================================================
 * 插值辅助函数 (供外部测试)
 * ==================================================================== */

/**
 * 根据 easing 类型计算插值因子
 * @param t      归一化时间 [0, 1]
 * @param easing 插值方式
 * @return       插值后的因子 [0, 1]
 */
float action_easing_eval(float t, EasingType easing);

/**
 * 角度 clamp
 */
float action_clamp_angle(float angle, float limit);

#ifdef __cplusplus
}
#endif

#endif /* ACTION_EXECUTOR_H */
