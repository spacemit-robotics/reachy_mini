/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * gesture_motion.c — 手势 (palm) 跟随运动控制
 *
 * 重构版：使用 TrackerApp 助手和 TrackerConfig 参数化配置。
 */

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "tracker_app_helper.h"

static volatile int g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

// 获取当前时间（微秒）
static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

#define GESTURE_PALM_LABEL 2
#define MAX_DETECTIONS 20
#define GESTURE_CONF_THRESHOLD 0.5f

/**
 * 从检测结果中选择最佳 palm 手势
 */
static int select_best_palm(const FaceDetectionResult *results, int count, float conf_threshold,
                            float last_cx, float last_cy) {
    int best = -1;
    float min_dist_sq = 1e6f;
    float max_area = 0.0f;

    for (int i = 0; i < count; i++) {
        if (results[i].class_id != GESTURE_PALM_LABEL || results[i].score < conf_threshold)
            continue;

        float cx = (results[i].x1 + results[i].x2) / 2.0f;
        float cy = (results[i].y1 + results[i].y2) / 2.0f;
        float area = (results[i].x2 - results[i].x1) * (results[i].y2 - results[i].y1);

        if (last_cx > 0) {
            float d2 = (cx - last_cx) * (cx - last_cx) + (cy - last_cy) * (cy - last_cy);
            if (d2 < min_dist_sq) {
                min_dist_sq = d2;
                best = i;
            }
        } else {
            if (area > max_area) {
                max_area = area;
                best = i;
            }
        }
    }
    return best;
}

static void print_usage(const char *program_name) {
    printf("Usage: %s <config_yaml> [options]\n", program_name);
    printf("  YOLOv5-Gesture 手势跟随 (新架构重构版)\n");
    printf("Options:\n");
    printf("  --model-path <path>   覆盖 yaml 中的 model_path\n");
    printf("  --camera-id <i>       相机设备 ID (默认: 0)\n");
    printf("  --no-gui              禁用 GUI 窗口\n");
    printf("  --control             启用电机跟踪控制\n");
    printf("  --port <p>            电机串口路路径 (默认: /dev/ttyACM0)\n");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

#ifndef CONFIG_DIR
#define CONFIG_DIR "."
#endif

    const char *config_path = argv[1];
    static char resolved_config[512];
    // 如果不是绝对路径且当前目录下找不到，尝试用编译时的 CONFIG_DIR 拼接
    if (config_path[0] != '/' && access(config_path, F_OK) != 0) {
        snprintf(resolved_config, sizeof(resolved_config), "%s/%s", CONFIG_DIR, config_path);
        config_path = resolved_config;
    }
    const char *model_path_override = NULL;
    int camera_id = 0;
    bool no_gui = false;
    bool control_motor = false;
    const char *motor_port = "/dev/ttyACM0";

    static struct option long_options[] = {
        {"model-path", required_argument, 0, 'm'},
        {"camera-id", required_argument, 0, 'c'},
        {"no-gui", no_argument, 0, 'n'},
        {"control", no_argument, 0, 'C'},
        {"port", required_argument, 0, 'p'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "m:c:nCp:h", long_options, NULL)) != -1) {
        switch (opt) {
        case 'm':
            model_path_override = optarg;
            break;
        case 'c':
            camera_id = atoi(optarg);
            break;
        case 'n':
            no_gui = true;
            break;
        case 'C':
            control_motor = true;
            break;
        case 'p':
            motor_port = optarg;
            break;
        default:
            print_usage(argv[0]);
            return 0;
        }
    }

    // 设置手势跟踪专用的控制参数（低延迟反馈优化版）
    TrackerConfig gesture_cfg = {
        .weight_base = 0.85f,
        .weight_max = 1.50f,
        .weight_per_sec = 4.0f,
        .k_d = 0.15f,
        .ema_tc_h = 0.10f,  // 较快滤波 (60ms)
        .ema_tc_v = 0.12f,
        .dead_zone = 0.08f,
        .overshoot_threshold = 8.0f,
        .max_command_lead = 6.0f,  // 较宽提前量
        .pitch_scale = 0.7f,
        .max_angle_speed = 60.0f
    };

    TrackerApp app;
    if (!tracker_app_init(&app, config_path, model_path_override, camera_id,
            control_motor ? motor_port : NULL, &gesture_cfg)) {
        return 1;
    }

    // // --- 解除底层异步电机的硬限速限制 (增加到 60 deg/s 以兼顾平滑) ---
    // if (app.motor_initialized && app.async_motor_ctrl) {
    //     printf("[Performance] 解除底层限速，设置为 %.1f deg/s\n", gesture_cfg.max_angle_speed);
    //     async_motor_controller_set_speed_limit(app.async_motor_ctrl, gesture_cfg.max_angle_speed);
    // }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("开始手势跟随循环，按 Ctrl+C 退出\n");
    FaceDetectionResult detections[MAX_DETECTIONS];
    int count = 0;
    int frame = 0;
    float last_cx = -1.0f, last_cy = -1.0f;
    uint64_t last_loop_us = get_time_us();

    while (g_running) {
        if (!camera_grab_frame(app.camera, 100))
            continue;
        frame++;

        uint64_t now = get_time_us();
        float loop_dt = (now - last_loop_us) / 1000000.0f;
        float fps = (loop_dt > 0) ? 1.0f / loop_dt : 0.0f;
        last_loop_us = now;

        if (!vision_detect_faces(app.vision, app.camera, detections, MAX_DETECTIONS, &count)) {
            continue;
        }

        if (app.motor_initialized) {
            int idx = select_best_palm(detections, count, GESTURE_CONF_THRESHOLD, last_cx, last_cy);
            if (idx >= 0) {
                int w, h;
                camera_get_size(app.camera, &w, &h);
                float cx = (detections[idx].x1 + detections[idx].x2) / 2.0f;
                float cy = (detections[idx].y1 + detections[idx].y2) / 2.0f;
                last_cx = cx;
                last_cy = cy;

                motion_ctl_update(&app.motion_ctrl,
                    (cx / w) * 2.0f - 1.0f, (cy / h) * 2.0f - 1.0f, true);
            } else {
                last_cx = -1.0f;
                motion_ctl_update(&app.motion_ctrl, 0, 0, false);
            }
        }

        if (!no_gui)
            vision_display_results(app.camera, detections, count, fps);
        if (frame % 60 == 0)
            printf("Frame %d: FPS=%.1f, Found=%d\n", frame, fps, count);
    }

    printf("\n[GestureTracker] 收到退出信号，正在释放电机...\n");
    tracker_app_cleanup(&app);
    return 0;
}
