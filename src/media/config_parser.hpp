/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

<<<<<<< HEAD
#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP
=======
#ifndef MEDIA_CONFIG_PARSER_HPP
#define MEDIA_CONFIG_PARSER_HPP
>>>>>>> 5c03a11 (feat: add config file for start parameters)

#include <string>

/**
 * @brief 从 YAML 配置文件加载的参数集合
 *
 * 值为 sentinel 表示配置文件中未指定该项：
 *   int 类型: INT_MIN 表示未设置
 *   float 类型: -1.0f 表示未设置
 *   string 类型: 空字符串表示未设置
 *   bool 类型: 通过 _set 标志判断
 */
struct ConfigFromFile {
    // audio
    int input_device;
    int output_device;
    int capture_rate;
    int capture_channels;
    int playback_rate;
    int playback_channels;

    // llm
    std::string llm_url;
    std::string llm_model;
    int max_tokens;

    // tts
    std::string tts_engine;

    // vad
    float vad_threshold;
    float silence_duration;

    // motor
    std::string motor_port;

    // camera
    int camera_id;

    // mcp
    std::string mcp_config_path;

    // debug
    bool save_audio;
    bool save_audio_set;
    std::string audio_file;

    ConfigFromFile();
};

/**
 * @brief 从 YAML 文件加载配置
 *
 * @param path YAML 文件路径
 * @param out  输出配置结构
 * @return true 加载成功, false 文件不存在或解析失败
 */
bool loadConfigFromYaml(const std::string &path, ConfigFromFile &out);

<<<<<<< HEAD
#endif  // CONFIG_PARSER_HPP
=======
#endif  // MEDIA_CONFIG_PARSER_HPP
>>>>>>> 5c03a11 (feat: add config file for start parameters)
