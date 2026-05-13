/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "device_detect.hpp"

#include <libudev.h>
#include <alsa/asoundlib.h>

#include <climits>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

bool detectReachyMiniDevices(ReachyDeviceInfo &info) {
    struct udev *udev = udev_new();
    if (!udev) {
        std::cerr << "[DeviceDetect] 错误: 无法创建 udev 上下文\n";
        return false;
    }

    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_scan_devices(enumerate);
    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *entry;

    bool found_any = false;
    int min_video_id = INT_MAX;  // 取编号最小的视频节点

    udev_list_entry_foreach(entry, devices) {
        const char *path = udev_list_entry_get_name(entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);
        if (!dev)
            continue;

        const char *model = udev_device_get_property_value(dev, "ID_MODEL");
        const char *subsystem = udev_device_get_subsystem(dev);

        if (model && subsystem &&
            std::string(model).find("Reachy_Mini") != std::string::npos) {
            // 音频设备 (ALSA sound subsystem)
            if (std::string(subsystem) == "sound") {
                const char *sysname = udev_device_get_sysname(dev);
                // 匹配 "cardN" 格式的节点
                if (sysname && std::string(sysname).find("card") == 0) {
                    std::string card_str(sysname + 4);  // 跳过 "card" 前缀
                    int card_num = std::atoi(card_str.c_str());
                    info.audio_card_id = card_num;
                    info.audio_card_name = sysname;
                    found_any = true;
                }
            }

            // 摄像头设备 (Video4Linux subsystem)
            if (std::string(subsystem) == "video4linux") {
                const char *devnode = udev_device_get_devnode(dev);
                if (devnode) {
                    std::string node_str(devnode);
                    // 只取 /dev/videoN 格式的节点
                    size_t pos = node_str.find("video");
                    if (pos != std::string::npos) {
                        std::string num_str = node_str.substr(pos + 5);
                        int video_id = std::atoi(num_str.c_str());
                        // 取编号最小的节点（通常是主视频流，非元数据节点）
                        if (video_id < min_video_id) {
                            min_video_id = video_id;
                            info.camera_id = video_id;
                            info.camera_node = node_str;
                            found_any = true;
                        }
                    }
                }
            }
        }

        udev_device_unref(dev);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return found_any;
}

void printDeviceInfo(const ReachyDeviceInfo &info) {
    std::cout << "[DeviceDetect] --- Reachy Mini 硬件探测结果 ---\n";
    if (info.audio_card_id >= 0) {
        std::cout << "[DeviceDetect] 音频声卡: " << info.audio_card_name
            << " (ID: " << info.audio_card_id << ")\n";
        if (info.hw_channels > 0) {
            std::cout << "[DeviceDetect] 声道数: " << info.hw_channels << " ch\n";
        }
        if (info.hw_sample_rate > 0) {
            std::cout << "[DeviceDetect] 采样率: " << info.hw_sample_rate << " Hz\n";
        }
    } else {
        std::cout << "[DeviceDetect] 音频声卡: 未找到\n";
    }
    if (info.camera_id >= 0) {
        std::cout << "[DeviceDetect] 摄像头: " << info.camera_node
            << " (ID: " << info.camera_id << ")\n";
    } else {
        std::cout << "[DeviceDetect] 摄像头: 未找到\n";
    }
    std::cout << "[DeviceDetect] --------------------------------\n";
}

bool queryAudioHardwareConfig(ReachyDeviceInfo &info) {
    if (info.audio_card_id < 0) {
        return false;
    }
    std::string pcm_name = "hw:" + std::to_string(info.audio_card_id);
    snd_pcm_t *handle = nullptr;
    snd_pcm_hw_params_t *params = nullptr;
    // 打开 PCM capture 设备查询硬件参数
    int err = snd_pcm_open(&handle, pcm_name.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        std::cerr << "[DeviceDetect] 警告: 无法打开 PCM 设备 " << pcm_name
            << " (" << snd_strerror(err) << ")\n";
        return false;
    }
    snd_pcm_hw_params_alloca(&params);
    err = snd_pcm_hw_params_any(handle, params);
    if (err < 0) {
        std::cerr << "[DeviceDetect] 警告: 无法获取硬件参数 (" << snd_strerror(err) << ")\n";
        snd_pcm_close(handle);
        return false;
    }
    // 获取声道数 (取最大值作为硬件原生声道数)
    unsigned int max_channels = 0;
    snd_pcm_hw_params_get_channels_max(params, &max_channels);
    if (max_channels > 0 && max_channels <= 32) {
        info.hw_channels = static_cast<int>(max_channels);
    }
    // 获取采样率 (取最大值作为硬件原生采样率)
    unsigned int max_rate = 0;
    int dir = 0;
    snd_pcm_hw_params_get_rate_max(params, &max_rate, &dir);
    if (max_rate > 0 && max_rate <= 192000) {
        info.hw_sample_rate = static_cast<int>(max_rate);
    }
    snd_pcm_close(handle);
    std::cout << "[DeviceDetect] ALSA 硬件查询 (" << pcm_name << "): "
        << info.hw_channels << " ch, " << info.hw_sample_rate << " Hz\n";
    return true;
}

// 音量初始化
// ============================================================================

/**
 * 设置指定声卡的 mixer 元素音量
 * @param card_id   ALSA 声卡编号
 * @param elem_name 控制元素名称 (如 "PCM")
 * @param index     元素索引 (对应 amixer 中的 ,0 或 ,1)
 * @param percent   音量百分比 (0-100)
 */
static void setMixerVolume(int card_id, const char *elem_name, int index, int percent) {
    std::string device = "hw:" + std::to_string(card_id);

    snd_mixer_t *handle = nullptr;
    if (snd_mixer_open(&handle, 0) < 0) {
        std::cerr << "[DeviceDetect] 警告: 无法打开 mixer\n";
        return;
    }
    if (snd_mixer_attach(handle, device.c_str()) < 0) {
        std::cerr << "[DeviceDetect] 警告: mixer attach 失败 (" << device << ")\n";
        snd_mixer_close(handle);
        return;
    }
    snd_mixer_selem_register(handle, nullptr, nullptr);
    snd_mixer_load(handle);

    snd_mixer_selem_id_t *sid = nullptr;
    snd_mixer_selem_id_alloca(&sid);
    snd_mixer_selem_id_set_index(sid, index);
    snd_mixer_selem_id_set_name(sid, elem_name);

    snd_mixer_elem_t *elem = snd_mixer_find_selem(handle, sid);
    if (!elem) {
        std::cerr << "[DeviceDetect] 警告: 未找到控制元素 " << elem_name
            << "," << index << "\n";
    } else {
        int64_t min = 0, max = 0;
        snd_mixer_selem_get_playback_volume_range(elem, &min, &max);
        int64_t volume = min + (percent * (max - min) / 100);
        snd_mixer_selem_set_playback_volume_all(elem, volume);
        std::cout << "[DeviceDetect] 音量设置: " << device << " ["
            << elem_name << "," << index << "] -> " << percent << "%\n";
    }

    snd_mixer_close(handle);
}

void initAudioVolume(const ReachyDeviceInfo &info) {
    if (info.audio_card_id < 0) {
        return;
    }
    // amixer -c N sset 'PCM',0 100%
    setMixerVolume(info.audio_card_id, "PCM", 0, 100);
    // amixer -c N sset 'PCM',1 80%
    setMixerVolume(info.audio_card_id, "PCM", 1, 80);
}
