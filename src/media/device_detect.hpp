/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DEVICE_DETECT_HPP
#define DEVICE_DETECT_HPP

#include <string>

/**
 * @brief Reachy Mini 多媒体设备自动探测结果
 */
struct ReachyDeviceInfo {
    int audio_card_id = -1;        // ALSA 声卡编号 (-1 表示未找到)
    int camera_id = -1;            // V4L2 摄像头设备编号 (-1 表示未找到)
    std::string audio_card_name;   // 声卡名称 (如 "card0")
    std::string camera_node;       // 摄像头设备节点 (如 "/dev/video13")

    // ALSA 硬件参数 (由 queryAudioHardwareConfig 填充)
    int hw_channels = -1;          // 硬件声道数 (-1 表示未查询)
    int hw_sample_rate = -1;       // 硬件采样率 (-1 表示未查询)
};

/**
 * @brief 通过 udev 自动探测 Reachy Mini USB 复合设备
 *
 * 扫描系统中所有 USB 设备，匹配 "Reachy_Mini" 型号名称，
 * 提取对应的 ALSA 声卡 ID 和 V4L2 摄像头设备节点。
 *
 * @param info [输出] 探测结果
 * @return true 至少找到一个设备, false 探测失败或无匹配设备
 */
bool detectReachyMiniDevices(ReachyDeviceInfo &info);

/**
 * @brief 打印探测结果到标准输出
 */
void printDeviceInfo(const ReachyDeviceInfo &info);

/**
 * @brief 查询 ALSA 声卡硬件参数 (声道数、采样率)
 *
 * 通过打开 PCM capture 设备获取硬件支持的声道数和采样率，
 * 结果写入 info.hw_channels 和 info.hw_sample_rate。
 *
 * @param info [输入/输出] 需要 audio_card_id 已填充
 * @return true 查询成功, false 失败
 */
bool queryAudioHardwareConfig(ReachyDeviceInfo &info);

/**
 * @brief 初始化 Reachy Mini 声卡音量
 *
 * 设置 PCM,0 为 100%，PCM,1 为 80%。
 * 需要在探测到声卡后、开始录音/播放前调用。
 *
 * @param info 已填充 audio_card_id 的设备信息
 */
void initAudioVolume(const ReachyDeviceInfo &info);

#endif  // DEVICE_DETECT_HPP
