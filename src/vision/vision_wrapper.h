/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef VISION_WRAPPER_H
#define VISION_WRAPPER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

    /**
    * 人脸检测结果
    */
    typedef struct
    {
        float x1, y1, x2, y2;  // 边界框坐标
        float score;           // 置信度
        int class_id;          // 类别 ID
    } FaceDetectionResult;

    /**
    * 视觉服务句柄（不透明指针）
    */
    typedef void *VisionServiceHandle;

    /**
    * 相机句柄（不透明指针）
    */
    typedef void *CameraHandle;

    /**
    * 初始化视觉服务
    * @param config_path 配置文件路径
    * @param model_path_override 模型路径覆盖（可选，传 NULL 使用配置文件中的路径）
    * @return 视觉服务句柄，失败返回 NULL
    */
    VisionServiceHandle vision_service_init(const char *config_path, const char *model_path_override);

    /**
    * 销毁视觉服务
    */
    void vision_service_destroy(VisionServiceHandle handle);

    /**
    * 获取最后的错误信息
    */
    const char *vision_service_last_error(VisionServiceHandle handle);

    /**
    * 打开相机
    * @param camera_id 相机设备 ID
    * @param width 目标宽度
    * @param height 目标高度
    * @return 相机句柄，失败返回 NULL
    */
    CameraHandle camera_open(int camera_id, int width, int height);

    /**
    * 关闭相机
    */
    void camera_close(CameraHandle handle);

    /**
    * 从相机抓取一帧
    * @param handle 相机句柄
    * @param timeout_ms 超时时间（毫秒）
    * @return 成功返回 true，失败返回 false
    */
    bool camera_grab_frame(CameraHandle handle, int timeout_ms);

    /**
    * 对当前帧进行人脸检测
    * @param vision_handle 视觉服务句柄
    * @param camera_handle 相机句柄
    * @param results 输出结果数组
    * @param max_results 最大结果数
    * @param result_count 实际检测到的人脸数
    * @return 成功返回 true，失败返回 false
    */
    bool vision_detect_faces(VisionServiceHandle vision_handle,
                            CameraHandle camera_handle,
                            FaceDetectionResult *results,
                            int max_results,
                            int *result_count);

    /**
    * 选择最佳人脸（面积最大 + 置信度最高）
    * @param results 检测结果数组
    * @param count 结果数量
    * @param conf_threshold 置信度阈值
    * @return 最佳人脸索引，无有效人脸返回 -1
    */
    int vision_select_best_face(const FaceDetectionResult *results, int count, float conf_threshold);

    /**
    * 显示检测结果（可选，用于调试）
    * @param camera_handle 相机句柄
    * @param results 检测结果数组
    * @param count 结果数量
    * @param fps 当前帧率
    */
    void vision_display_results(CameraHandle camera_handle,
                                const FaceDetectionResult *results,
                                int count,
                                double fps);

    /**
    * 获取相机帧的宽度和高度
    */
    void camera_get_size(CameraHandle handle, int *width, int *height);

#ifdef __cplusplus
}
#endif

#endif  // VISION_WRAPPER_H
