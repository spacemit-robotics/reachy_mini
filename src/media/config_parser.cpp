/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "config_parser.hpp"

#include <climits>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

// Sentinel 值
static constexpr int NOT_SET_INT = INT_MIN;
static constexpr float NOT_SET_FLOAT = -1.0f;

ConfigFromFile::ConfigFromFile()
    : input_device(NOT_SET_INT),
        output_device(NOT_SET_INT),
        capture_rate(NOT_SET_INT),
        capture_channels(NOT_SET_INT),
        playback_rate(NOT_SET_INT),
        playback_channels(NOT_SET_INT),
        max_tokens(NOT_SET_INT),
        vad_threshold(NOT_SET_FLOAT),
        silence_duration(NOT_SET_FLOAT),
        camera_id(NOT_SET_INT),
        save_audio(false),
        save_audio_set(false) {}

// ============================================================================
// 轻量 YAML 解析器 (仅支持本项目的简单 key: value 格式)
// 支持:
//   - 顶层 section (如 "audio:")
//   - 缩进的 key: value 对
//   - # 注释
//   - 字符串值 (带或不带引号)
//   - 数值和布尔值
// ============================================================================

static std::string trim(const std::string &s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

static std::string stripQuotes(const std::string &s) {
    if (s.size() >= 2 &&
        ((s.front() == '"' && s.back() == '"') ||
        (s.front() == '\'' && s.back() == '\''))) {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

bool loadConfigFromYaml(const std::string &path, ConfigFromFile &out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }

    // 解析为 section.key -> value 的平面映射
    std::unordered_map<std::string, std::string> kv_map;
    std::string current_section;
    std::string line;

    while (std::getline(file, line)) {
        // 去除行尾注释 (但保留引号内的 #)
        size_t comment_pos = std::string::npos;
        bool in_quotes = false;
        for (size_t i = 0; i < line.size(); ++i) {
            if (line[i] == '"' || line[i] == '\'') in_quotes = !in_quotes;
            if (line[i] == '#' && !in_quotes) {
                comment_pos = i;
                break;
            }
        }
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        std::string trimmed = trim(line);
        if (trimmed.empty()) continue;

        // 检查是否是 section 头 (无缩进，以 : 结尾，冒号后无值)
        size_t colon_pos = trimmed.find(':');
        if (colon_pos != std::string::npos) {
            std::string after_colon = trim(trimmed.substr(colon_pos + 1));
            // 判断是否为 section: 行首无空格且冒号后无值
            bool is_indented = (!line.empty() && (line[0] == ' ' || line[0] == '\t'));
            if (after_colon.empty() && !is_indented) {
                current_section = trim(trimmed.substr(0, colon_pos));
                continue;
            }

            // 普通 key: value
            std::string key = trim(trimmed.substr(0, colon_pos));
            std::string value = stripQuotes(trim(trimmed.substr(colon_pos + 1)));

            std::string full_key = current_section.empty()
                ? key
                : (current_section + "." + key);
            kv_map[full_key] = value;
        }
    }

    file.close();

    // 映射到 ConfigFromFile 结构
    auto getInt = [&](const std::string &key) -> int {
        auto it = kv_map.find(key);
        if (it == kv_map.end() || it->second.empty()) return NOT_SET_INT;
        return std::atoi(it->second.c_str());
    };

    auto getFloat = [&](const std::string &key) -> float {
        auto it = kv_map.find(key);
        if (it == kv_map.end() || it->second.empty()) return NOT_SET_FLOAT;
        return std::atof(it->second.c_str());
    };

    auto getString = [&](const std::string &key) -> std::string {
        auto it = kv_map.find(key);
        if (it == kv_map.end()) return "";
        return it->second;
    };

    auto getBool = [&](const std::string &key, bool &value, bool &was_set) {
        auto it = kv_map.find(key);
        if (it == kv_map.end()) return;
        std::string v = it->second;
        if (v == "true" || v == "True" || v == "yes" || v == "1") {
            value = true;
            was_set = true;
        } else if (v == "false" || v == "False" || v == "no" || v == "0") {
            value = false;
            was_set = true;
        }
    };

    // audio
    out.input_device = getInt("audio.input_device");
    out.output_device = getInt("audio.output_device");
    out.capture_rate = getInt("audio.capture_rate");
    out.capture_channels = getInt("audio.capture_channels");
    out.playback_rate = getInt("audio.playback_rate");
    out.playback_channels = getInt("audio.playback_channels");

    // llm
    out.llm_url = getString("llm.url");
    out.llm_model = getString("llm.model");
    out.max_tokens = getInt("llm.max_tokens");

    // tts
    out.tts_engine = getString("tts.engine");

    // vad
    out.vad_threshold = getFloat("vad.threshold");
    out.silence_duration = getFloat("vad.silence_duration");

    // motor
    out.motor_port = getString("motor.port");

    // camera
    out.camera_id = getInt("camera.id");

    // mcp
    out.mcp_config_path = getString("mcp.config_path");

    // debug
    getBool("debug.save_audio", out.save_audio, out.save_audio_set);
    out.audio_file = getString("debug.audio_file");

    std::cout << "[ConfigParser] 已加载配置文件: " << path << "\n";
    return true;
}
