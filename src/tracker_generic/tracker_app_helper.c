/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * tracker_app_helper.c — 视觉跟踪应用生命周期管理助手实现
 */

#include "tracker_app_helper.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor.h"

#define INFERENCE_WIDTH 640
#define INFERENCE_HEIGHT 480
#define MOTOR_COUNT 9

bool tracker_app_init(TrackerApp *app, const char *vision_config, const char *model_path,
                    int camera_id, const char *motor_port, const TrackerConfig *tracker_config) {
    memset(app, 0, sizeof(TrackerApp));
    app->motor_count = MOTOR_COUNT;

    // 1. 初始化视觉服务
    printf("[TrackerApp] 初始化视觉服务: %s\n", vision_config);
    app->vision = vision_service_init(vision_config, model_path);
    if (!app->vision) {
        fprintf(stderr, "Error: 无法初始化视觉服务\n");
        return false;
    }

    // 2. 打开相机
    printf("[TrackerApp] 打开相机 %d...\n", camera_id);
    app->camera = camera_open(camera_id, INFERENCE_WIDTH, INFERENCE_HEIGHT);
    if (!app->camera) {
        fprintf(stderr, "Error: 无法打开相机 %d\n", camera_id);
        vision_service_destroy(app->vision);
        return false;
    }

    // 3. 预热相机
    if (!camera_grab_frame(app->camera, 5000)) {
        fprintf(stderr, "Error: 相机抓帧超时\n");
        tracker_app_cleanup(app);
        return false;
    }

    // 4. 初始化电机（如果提供了端口）
    if (motor_port) {
        printf("[TrackerApp] 初始化电机 on %s...\n", motor_port);
        uint8_t motor_ids[MOTOR_COUNT] = {10, 11, 12, 13, 14, 15, 16, 17, 18};
        app->motor_devs = (struct motor_dev **)calloc(MOTOR_COUNT, sizeof(struct motor_dev *));
        bool all_alloc = true;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            app->motor_devs[i] =
                motor_alloc_uart("drv_uart_xl330", motor_port, 1000000, motor_ids[i], NULL);
            if (!app->motor_devs[i]) {
                all_alloc = false;
                break;
            }
        }

        if (all_alloc && motor_init(app->motor_devs, MOTOR_COUNT) == 0) {
            app->async_motor_ctrl = async_motor_controller_create(app->motor_devs, MOTOR_COUNT);
            if (app->async_motor_ctrl && async_motor_controller_start(app->async_motor_ctrl)) {
                // 恢复视觉跟随所需的响应速度 (默认构造为20°/s，跟踪需要50°/s)
                async_motor_controller_set_speed_limit(app->async_motor_ctrl, 50.0f);
                motion_ctl_init(&app->motion_ctrl, tracker_config, app->motor_devs, MOTOR_COUNT,
                                app->async_motor_ctrl);
                app->motor_initialized = true;
                printf("[TrackerApp] 电机系统初始化成功\n");
            }
        } else {
            fprintf(stderr, "Error: 电机初始化失败\n");
        }
    }

    return true;
}

void tracker_app_cleanup(TrackerApp *app) {
    printf("[TrackerApp] 清理资源...\n");
    if (app->motor_initialized) {
        motion_ctl_destroy(&app->motion_ctrl);
        if (app->async_motor_ctrl) {
            async_motor_controller_stop(app->async_motor_ctrl);
            async_motor_controller_destroy(app->async_motor_ctrl);
        }
        if (app->motor_devs) {
            motor_free(app->motor_devs, app->motor_count);
            free(app->motor_devs);
        }
    }
    if (app->camera)
        camera_close(app->camera);
    if (app->vision)
        vision_service_destroy(app->vision);
    memset(app, 0, sizeof(TrackerApp));
}
