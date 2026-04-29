/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*/


/*
* tracker_app_helper.h — 视觉跟踪应用生命周期管理助手
*/

#ifndef TRACKER_APP_HELPER_H
#define TRACKER_APP_HELPER_H

#include "coordinate_to_motion.h"
#include "motor_controller.h"
#include "tracker_common.h"
#include "vision_wrapper.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        VisionServiceHandle vision;
        CameraHandle camera;
        AsyncMotorController *async_motor_ctrl;
        MotionController motion_ctrl;
        struct motor_dev **motor_devs;
        int motor_count;
        bool motor_initialized;
    } TrackerApp;

    /**
    * 初始化跟踪应用基础架构
    */
    bool tracker_app_init(TrackerApp *app, const char *vision_config,
                            const char *model_path, int camera_id,
                            const char *motor_port,
                            const TrackerConfig *tracker_config);

    /**
    * 清理跟踪应用资源
    */
    void tracker_app_cleanup(TrackerApp *app);

#ifdef __cplusplus
}
#endif

#endif  // TRACKER_APP_HELPER_H
