/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*/

#include "vision_wrapper.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "vision_service.h"

// 内部结构体（C++ 实现）
struct VisionServiceContext
{
    std::unique_ptr<VisionService> service;
    std::string last_error;
};

struct CameraContext
{
    cv::VideoCapture cap;
    cv::Mat current_frame;
    std::mutex frame_mutex;
    std::thread capture_thread;
    std::atomic<bool> stop_capture{false};
    std::atomic<uint64_t> frame_seq{0};
    uint64_t last_consumed_seq{0};
    std::condition_variable frame_cv;
    std::chrono::steady_clock::time_point last_display_time;  // 上次显示时间
    int width;
    int height;
};

// 抓帧线程函数
static void camera_capture_thread_func(CameraContext *ctx)
{
    while (!ctx->stop_capture)
    {
        cv::Mat frame;
        if (ctx->cap.read(frame))
        {
            if (!frame.empty())
            {
                // 如果尺寸不匹配，进行缩放
                if (frame.cols != ctx->width || frame.rows != ctx->height)
                {
                    cv::Mat resized;
                    cv::resize(frame, resized, cv::Size(ctx->width, ctx->height), 0, 0,
                                cv::INTER_LINEAR);
                    frame = resized;
                }

                {
                    std::lock_guard<std::mutex> lock(ctx->frame_mutex);
                    // 零拷贝优化：使用赋值（浅拷贝，增加引用计数）代替 copyTo
                    ctx->current_frame = frame;
                    ctx->frame_seq++;
                }
                ctx->frame_cv.notify_all();
            }
        } else {
            // 如果读取失败，或者是驱动级无数据，休眠 10ms 避免耗尽单核
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // 适度节流：强制休眠 8ms 以平衡 CPU 负载与处理通量 (对应最高 ~100 FPS)
        // 配合 blocking read，这能确保主循环有更高几率拿到最新帧，同时 CPU
        // 不会空转。
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        std::this_thread::yield();
    }
}

// 初始化视觉服务
VisionServiceHandle vision_service_init(const char *config_path,
                                        const char *model_path_override)
{
    try
    {
        auto ctx = new VisionServiceContext();
        std::string model_override = model_path_override ? model_path_override : "";
        ctx->service = VisionService::Create(config_path, model_override, true);

        if (!ctx->service)
        {
            ctx->last_error = VisionService::LastCreateError();
            delete ctx;
            return nullptr;
        }

        return static_cast<VisionServiceHandle>(ctx);
    }
    catch (const std::exception &e)
    {
        return nullptr;
    }
}

void vision_service_destroy(VisionServiceHandle handle)
{
    if (handle)
    {
        auto ctx = static_cast<VisionServiceContext *>(handle);
        delete ctx;
    }
}

const char *vision_service_last_error(VisionServiceHandle handle)
{
    if (!handle)
        return "Invalid handle";
    auto ctx = static_cast<VisionServiceContext *>(handle);
    return ctx->last_error.c_str();
}

// 打开相机
CameraHandle camera_open(int camera_id, int width, int height)
{
    try
    {
        auto ctx = new CameraContext();
        ctx->cap.open(camera_id);

        if (!ctx->cap.isOpened())
        {
            delete ctx;
            return nullptr;
        }

        // 设置相机格式为 MJPG
        ctx->cap.set(cv::CAP_PROP_FOURCC,
                    cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        ctx->cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        ctx->cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        ctx->cap.set(cv::CAP_PROP_FPS, 30);

        // 增加曝光控制以减少运动模糊 (降低曝光时间)
        // 注意：具体数值取决于 V4L2 驱动实现
        ctx->cap.set(cv::CAP_PROP_AUTO_EXPOSURE,
                    1);  // 1 = 手动模式 (V4L2_EXPOSURE_MANUAL)
        ctx->cap.set(cv::CAP_PROP_EXPOSURE,
                    150);  // 设定较低曝光值以获得更短快门 (减少模糊)

        ctx->width = width;
        ctx->height = height;
        ctx->last_display_time = std::chrono::steady_clock::now();

        // 启动抓帧线程
        ctx->stop_capture = false;
        ctx->frame_seq = 0;
        ctx->last_consumed_seq = 0;
        ctx->capture_thread = std::thread(camera_capture_thread_func, ctx);

        return static_cast<CameraHandle>(ctx);
    }
    catch (const std::exception &e)
    {
        return nullptr;
    }
}

void camera_close(CameraHandle handle)
{
    if (handle)
    {
        auto ctx = static_cast<CameraContext *>(handle);

        // 停止线程
        ctx->stop_capture = true;
        ctx->frame_cv.notify_all();
        if (ctx->capture_thread.joinable())
        {
            ctx->capture_thread.join();
        }

        ctx->cap.release();
        delete ctx;
    }
}

bool camera_grab_frame(CameraHandle handle, int timeout_ms)
{
    if (!handle)
        return false;

    auto ctx = static_cast<CameraContext *>(handle);

    // 等待直到产生新的帧序号
    std::unique_lock<std::mutex> lock(ctx->frame_mutex);
    uint64_t old_seq = ctx->last_consumed_seq;

    if (ctx->frame_cv.wait_for(
            lock, std::chrono::milliseconds(timeout_ms), [ctx, old_seq]
            { return ctx->frame_seq > old_seq || ctx->stop_capture; }))
    {
        ctx->last_consumed_seq = ctx->frame_seq;
        return true;
    }

    return false;
}

bool vision_detect_faces(VisionServiceHandle vision_handle,
                        CameraHandle camera_handle,
                        FaceDetectionResult *results, int max_results,
                        int *result_count)
{
    if (!vision_handle || !camera_handle || !results || !result_count)
    {
        return false;
    }

    auto vision_ctx = static_cast<VisionServiceContext *>(vision_handle);
    auto camera_ctx = static_cast<CameraContext *>(camera_handle);

    cv::Mat frame;
    {
        // 从抓帧线程中获取最新的一帧
        std::lock_guard<std::mutex> lock(camera_ctx->frame_mutex);
        if (camera_ctx->current_frame.empty())
        {
            *result_count = 0;
            return false;
        }
        // 零拷贝优化：获取引用
        frame = camera_ctx->current_frame;
    }

    std::vector<VisionServiceResult> cpp_results;
    VisionServiceStatus status =
        vision_ctx->service->InferImage(frame, &cpp_results);

    if (status != VISION_SERVICE_OK)
    {
        vision_ctx->last_error = vision_ctx->service->LastError();
        *result_count = 0;
        return false;
    }

    // 转换结果
    *result_count = std::min(static_cast<int>(cpp_results.size()), max_results);
    for (int i = 0; i < *result_count; i++)
    {
        results[i].x1 = cpp_results[i].x1;
        results[i].y1 = cpp_results[i].y1;
        results[i].x2 = cpp_results[i].x2;
        results[i].y2 = cpp_results[i].y2;
        results[i].score = cpp_results[i].score;
        results[i].class_id = cpp_results[i].label;  // 使用 label 字段
    }

    return true;
}

int vision_select_best_face(const FaceDetectionResult *results, int count,
                            float conf_threshold)
{
    if (!results || count <= 0)
        return -1;

    int best = -1;
    float best_score = -1.0f;
    float max_area = 0.0f;

    // 先算最大面积用于归一化
    for (int i = 0; i < count; i++)
    {
        float area =
            (results[i].x2 - results[i].x1) * (results[i].y2 - results[i].y1);
        if (area > max_area)
            max_area = area;
    }

    if (max_area <= 0.0f)
        return -1;

    for (int i = 0; i < count; i++)
    {
        if (results[i].score < conf_threshold)
            continue;

        float area =
            (results[i].x2 - results[i].x1) * (results[i].y2 - results[i].y1);
        float score = results[i].score * 0.7f + (area / max_area) * 0.3f;

        if (score > best_score)
        {
            best_score = score;
            best = i;
        }
    }

    return best;
}

void vision_display_results(CameraHandle camera_handle,
                            const FaceDetectionResult *results, int count,
                            double fps)
{
    if (!camera_handle)
        return;

    auto ctx = static_cast<CameraContext *>(camera_handle);

    // 显示降频优化：限制最高 10Hz (100ms)
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            now - ctx->last_display_time)
            .count() < 100)
    {
        return;
    }
    ctx->last_display_time = now;

    cv::Mat display;
    {
        std::lock_guard<std::mutex> lock(ctx->frame_mutex);
        if (ctx->current_frame.empty())
            return;
        // 渲染时才克隆图像，避免影响主推理逻辑
        display = ctx->current_frame.clone();
    }

    // 绘制检测框
    for (int i = 0; i < count; i++)
    {
        cv::rectangle(display, cv::Point(results[i].x1, results[i].y1),
                        cv::Point(results[i].x2, results[i].y2),
                        cv::Scalar(0, 255, 0), 2);

        char label[64];
        snprintf(label, sizeof(label), "%.2f", results[i].score);
        cv::putText(display, label, cv::Point(results[i].x1, results[i].y1 - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    // 绘制 FPS
    char fps_buf[64];
    snprintf(fps_buf, sizeof(fps_buf), "FPS: %.1f", fps);
    cv::putText(display, fps_buf, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1);

    // 无显示环境时跳过 GUI，防止 GTK 初始化崩溃
    static bool display_available = (getenv("DISPLAY") != nullptr || getenv("WAYLAND_DISPLAY") != nullptr);
    if (!display_available)
        return;

    cv::imshow("YOLOv5-Face Detection", display);
    cv::waitKey(1);
}

void camera_get_size(CameraHandle handle, int *width, int *height)
{
    if (!handle || !width || !height)
        return;

    auto ctx = static_cast<CameraContext *>(handle);
    *width = ctx->width;
    *height = ctx->height;
}
