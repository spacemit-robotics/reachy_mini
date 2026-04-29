/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取音频文件的总时长（秒）
 *
 * @param path 音频文件路径
 * @return float 时长（秒），失败返回 -1.0
 */
float audio_get_duration(const char *path);

/**
 * @brief 异步播放音频文件
 *
 * @param path 音频文件路径
 * @return int 0 成功，-1 失败
 */
int audio_play_async(const char *path);

/**
 * @brief 列出所有可用的音频设备
 */
void audio_list_devices(void);

/**
 * @brief 异步播放音频文件 (带完整参数支持)
 *
 * @param path 音频文件路径
 * @param src_rate 指定源采样率 (设为 0 则从文件头自动检测)
 * @param src_channels 指定源声道数 (设为 0 则从文件头自动检测)
 * @param target_rate 目标采样率 (如 48000)
 * @param target_channels 目标声道数 (1 为单声道, 2 为双声道)
 * @param output_device_id 输出设备 ID (设为 -1 则使用系统默认)
 * @return int 0 成功，-1 失败
 */
int audio_play_async_full(const char *path, int src_rate, int src_channels, int target_rate,
                            int target_channels, int output_device_id);

/**
 * @brief 异步播放音频文件 (带目标参数支持)
 *
 * @param path 音频文件路径
 * @param sample_rate 目标采样率 (如 48000)
 * @param channels 目标声道数 (1 为单声道, 2 为双声道)
 * @return int 0 成功，-1 失败
 */
int audio_play_async_ext(const char *path, int sample_rate, int channels);

/**
 * @brief 停止当前播放
 */
void audio_stop(void);

#ifdef __cplusplus
}
#endif

#endif  // AUDIO_PLAYER_H
