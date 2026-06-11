/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * 动作执行引擎单元测试
 * 编译: gcc -o test_action_executor test_action_executor.c ../action_executor.c -lcjson -I..
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "action_executor.h"

/* ====================================================================
 * 测试辅助
 * ==================================================================== */

static int test_count = 0;
static int pass_count = 0;

#define TEST(name) do { \
    test_count++; \
    printf("[TEST %d] %s ... ", test_count, name); \
} while (0)

#define PASS() do { \
    pass_count++; \
    printf("PASS\n"); \
} while (0)

#define FAIL(msg) do { \
    printf("FAIL: %s\n", msg); \
} while (0)

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) { FAIL("assertion failed: " #a " != " #b); return; } \
} while (0)

#define ASSERT_FLOAT_EQ(a, b, eps) do { \
    if (fabsf((a) - (b)) > (eps)) { \
        printf("FAIL: %s = %f, expected %f\n", #a, (float)(a), (float)(b)); \
        return; \
    } \
} while (0)

/* ====================================================================
 * 测试用例
 * ==================================================================== */

static void test_parse_simple_pose(void) {
    TEST("parse simple pose action");
    ActionSequence seq;
    const char *json = "{\"actions\":[{\"type\":\"pose\",\"yaw\":30.0,\"pitch\":-10.0,\"duration_ms\":500,\"easing\":\"hermite\"}]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    ASSERT_EQ(seq.count, 1);
    ASSERT_EQ(seq.nodes[0].type, ACTION_TYPE_POSE);
    ASSERT_FLOAT_EQ(seq.nodes[0].yaw, 30.0f, 0.01f);
    ASSERT_FLOAT_EQ(seq.nodes[0].pitch, -10.0f, 0.01f);
    ASSERT_EQ(seq.nodes[0].duration_ms, 500);
    ASSERT_EQ(seq.nodes[0].easing, EASING_HERMITE);
    PASS();
}

static void test_parse_multiple_actions(void) {
    TEST("parse multiple actions");
    ActionSequence seq;
    const char *json = "{\"actions\":["
        "{\"type\":\"pose\",\"yaw\":25,\"duration_ms\":350,\"easing\":\"hermite\"},"
        "{\"type\":\"wait\",\"duration_ms\":1000},"
        "{\"type\":\"center\",\"duration_ms\":500}"
        "]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    ASSERT_EQ(seq.count, 3);
    ASSERT_EQ(seq.nodes[0].type, ACTION_TYPE_POSE);
    ASSERT_EQ(seq.nodes[1].type, ACTION_TYPE_WAIT);
    ASSERT_EQ(seq.nodes[2].type, ACTION_TYPE_CENTER);
    PASS();
}

static void test_parse_sequence_flatten(void) {
    TEST("parse sequence with flatten");
    ActionSequence seq;
    const char *json = "{\"actions\":["
        "{\"type\":\"sequence\",\"repeat\":3,\"actions\":["
            "{\"type\":\"pose\",\"yaw\":25,\"duration_ms\":300},"
            "{\"type\":\"pose\",\"yaw\":-25,\"duration_ms\":300}"
        "]},"
        "{\"type\":\"center\",\"duration_ms\":500}"
        "]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    /* 3 repetitions * 2 actions + 1 center = 7 */
    ASSERT_EQ(seq.count, 7);
    ASSERT_FLOAT_EQ(seq.nodes[0].yaw, 25.0f, 0.01f);
    ASSERT_FLOAT_EQ(seq.nodes[1].yaw, -25.0f, 0.01f);
    ASSERT_FLOAT_EQ(seq.nodes[2].yaw, 25.0f, 0.01f);
    PASS();
}

static void test_parse_move(void) {
    TEST("parse move action");
    ActionSequence seq;
    const char *json = "{\"actions\":[{\"type\":\"move\",\"delta_roll\":5.0,\"delta_pitch\":-10.0,\"duration_ms\":500}]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    ASSERT_EQ(seq.count, 1);
    ASSERT_EQ(seq.nodes[0].type, ACTION_TYPE_MOVE);
    ASSERT_FLOAT_EQ(seq.nodes[0].roll, 5.0f, 0.01f);
    ASSERT_FLOAT_EQ(seq.nodes[0].pitch, -10.0f, 0.01f);
    PASS();
}

static void test_parse_dance(void) {
    TEST("parse dance action");
    ActionSequence seq;
    const char *json = "{\"actions\":[{\"type\":\"dance\",\"name\":\"headbanger\",\"cycles\":2}]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    ASSERT_EQ(seq.count, 1);
    ASSERT_EQ(seq.nodes[0].type, ACTION_TYPE_DANCE);
    ASSERT_EQ(strcmp(seq.nodes[0].dance_name, "headbanger"), 0);
    ASSERT_EQ(seq.nodes[0].cycles, 2);
    PASS();
}

static void test_parse_tracker(void) {
    TEST("parse tracker action");
    ActionSequence seq;
    const char *json = "{\"actions\":[{\"type\":\"tracker\",\"mode\":\"face\",\"action\":\"start\"}]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    ASSERT_EQ(seq.count, 1);
    ASSERT_EQ(seq.nodes[0].type, ACTION_TYPE_TRACKER);
    ASSERT_EQ(seq.nodes[0].tracker_mode, TRACKER_MODE_FACE);
    ASSERT_EQ(seq.nodes[0].tracker_action, TRACKER_ACTION_START);
    PASS();
}

static void test_parse_with_noise(void) {
    TEST("parse JSON with surrounding text");
    ActionSequence seq;
    const char *json = "好的，这是动作序列：{\"actions\":[{\"type\":\"center\",\"duration_ms\":500}]} 以上就是结果。";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    ASSERT_EQ(seq.count, 1);
    ASSERT_EQ(seq.nodes[0].type, ACTION_TYPE_CENTER);
    PASS();
}

static void test_parse_invalid_json(void) {
    TEST("parse invalid JSON");
    ActionSequence seq;
    const char *json = "这不是 JSON";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_ERR_PARSE);
    PASS();
}

static void test_parse_single_action(void) {
    TEST("parse single action without actions array");
    ActionSequence seq;
    const char *json = "{\"type\":\"pose\",\"yaw\":15,\"duration_ms\":300}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    ASSERT_EQ(seq.count, 1);
    PASS();
}

static void test_validate_clamp(void) {
    TEST("validate clamps out-of-range angles");
    ActionSequence seq;
    const char *json = "{\"actions\":[{\"type\":\"pose\",\"yaw\":200.0,\"pitch\":-50.0,\"roll\":30.0,\"duration_ms\":30}]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);

    ret = action_executor_validate(&seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);

    /* Check clamped values */
    ASSERT_FLOAT_EQ(seq.nodes[0].yaw, 170.0f, 0.01f);
    ASSERT_FLOAT_EQ(seq.nodes[0].pitch, -35.0f, 0.01f);
    ASSERT_FLOAT_EQ(seq.nodes[0].roll, 25.0f, 0.01f);
    ASSERT_EQ(seq.nodes[0].duration_ms, MIN_DURATION_MS);
    PASS();
}

static void test_easing_functions(void) {
    TEST("easing function values");

    /* All easing functions should map 0→0 and 1→1 */
    ASSERT_FLOAT_EQ(action_easing_eval(0.0f, EASING_LINEAR), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_easing_eval(1.0f, EASING_LINEAR), 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_easing_eval(0.5f, EASING_LINEAR), 0.5f, 0.001f);

    ASSERT_FLOAT_EQ(action_easing_eval(0.0f, EASING_HERMITE), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_easing_eval(1.0f, EASING_HERMITE), 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_easing_eval(0.5f, EASING_HERMITE), 0.5f, 0.001f);

    ASSERT_FLOAT_EQ(action_easing_eval(0.0f, EASING_EASE_IN), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_easing_eval(1.0f, EASING_EASE_IN), 1.0f, 0.001f);

    ASSERT_FLOAT_EQ(action_easing_eval(0.0f, EASING_EASE_OUT), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_easing_eval(1.0f, EASING_EASE_OUT), 1.0f, 0.001f);

    /* Hermite should be symmetric */
    float h_low = action_easing_eval(0.25f, EASING_HERMITE);
    float h_high = action_easing_eval(0.75f, EASING_HERMITE);
    ASSERT_FLOAT_EQ(h_low + h_high, 1.0f, 0.001f);

    PASS();
}

static void test_field_mask(void) {
    TEST("field mask partial update");
    ActionSequence seq;
    /* Only set yaw, not roll/pitch */
    const char *json = "{\"actions\":[{\"type\":\"pose\",\"yaw\":30.0,\"duration_ms\":500}]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_OK);
    ASSERT_EQ(seq.nodes[0].field_mask & FIELD_YAW, FIELD_YAW);
    ASSERT_EQ(seq.nodes[0].field_mask & FIELD_ROLL, 0);
    ASSERT_EQ(seq.nodes[0].field_mask & FIELD_PITCH, 0);
    PASS();
}

static void test_nest_depth_limit(void) {
    TEST("nesting depth limit");
    ActionSequence seq;
    /* 4 levels of nesting (exceeds SEQUENCE_NEST_MAX=3) */
    const char *json = "{\"actions\":["
        "{\"type\":\"sequence\",\"repeat\":1,\"actions\":["
            "{\"type\":\"sequence\",\"repeat\":1,\"actions\":["
                "{\"type\":\"sequence\",\"repeat\":1,\"actions\":["
                    "{\"type\":\"sequence\",\"repeat\":1,\"actions\":["
                        "{\"type\":\"pose\",\"yaw\":10,\"duration_ms\":100}"
                    "]}"
                "]}"
            "]}"
        "]}"
        "]}";

    int ret = action_executor_parse(json, &seq);
    ASSERT_EQ(ret, ACTION_EXEC_ERR_NEST);
    PASS();
}

static void test_clamp_angle(void) {
    TEST("angle clamp function");
    ASSERT_FLOAT_EQ(action_clamp_angle(200.0f, 170.0f), 170.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_clamp_angle(-200.0f, 170.0f), -170.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_clamp_angle(50.0f, 170.0f), 50.0f, 0.001f);
    ASSERT_FLOAT_EQ(action_clamp_angle(0.0f, 25.0f), 0.0f, 0.001f);
    PASS();
}

/* ====================================================================
 * 主函数
 * ==================================================================== */

int main(void) {
    printf("========================================\n");
    printf("  Action Executor 单元测试\n");
    printf("========================================\n\n");

    test_parse_simple_pose();
    test_parse_multiple_actions();
    test_parse_sequence_flatten();
    test_parse_move();
    test_parse_dance();
    test_parse_tracker();
    test_parse_with_noise();
    test_parse_invalid_json();
    test_parse_single_action();
    test_validate_clamp();
    test_easing_functions();
    test_field_mask();
    test_nest_depth_limit();
    test_clamp_angle();

    printf("\n========================================\n");
    printf("  结果: %d/%d 测试通过\n", pass_count, test_count);
    printf("========================================\n");

    return (pass_count == test_count) ? 0 : 1;
}
