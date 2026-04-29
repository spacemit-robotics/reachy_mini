/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "voice_common.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// ============================================================================
// Global state
// ============================================================================

std::atomic<bool> g_running{true};
std::atomic<bool> g_processing{false};
std::atomic<bool> g_barge_in{false};
std::mutex g_process_thread_mutex;
std::unique_ptr<std::thread> g_process_thread;

void voiceSignalHandler(int sig) {
    (void)sig;
    std::cout << "\n" << getTimestamp() << " [退出中...]" << std::endl;
    g_running = false;
}

// ============================================================================
// Timestamp
// ============================================================================

std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;

    std::tm tm_now;
    localtime_r(&time_t_now, &tm_now);

    std::ostringstream oss;
    oss << "[" << std::setfill('0') << std::setw(2) << tm_now.tm_hour << ":"
        << std::setw(2) << tm_now.tm_min << ":" << std::setw(2) << tm_now.tm_sec
        << "." << std::setw(3) << ms.count() << "]";
    return oss.str();
}

// ============================================================================
// TTS engine selection
// ============================================================================

const std::vector<std::pair<std::string, std::string>> kKokoroVoices = {
    // Chinese female
    {"zf_xiaobei", "xiaobei"},
    {"zf_xiaoni", "xiaoni"},
    {"zf_xiaoxiao", "xiaoxiao"},
    {"zf_xiaoyi", "xiaoyi"},
    // Chinese male
    {"zm_yunxi", "yunxi"},
    {"zm_yunyang", "yunyang"},
    {"zm_yunjian", "yunjian"},
    {"zm_yunfan", "yunfan"},
    // American English female
    {"af_heart", "heart"},
    {"af_alloy", "alloy"},
    {"af_aoede", "aoede"},
    {"af_bella", "bella"},
    {"af_jessica", "jessica"},
    {"af_kore", "kore"},
    {"af_nicole", "nicole"},
    {"af_nova", "nova"},
    {"af_river", "river"},
    {"af_sarah", "sarah"},
    {"af_sky", "sky"},
    // American English male
    {"am_adam", "adam"},
    {"am_echo", "echo"},
    {"am_eric", "eric"},
    {"am_fenrir", "fenrir"},
    {"am_liam", "liam"},
    {"am_michael", "michael"},
    {"am_onyx", "onyx"},
    {"am_puck", "puck"},
    // British English female
    {"bf_alice", "alice"},
    {"bf_emma", "emma"},
    {"bf_isabella", "isabella"},
    {"bf_lily", "lily"},
    // British English male
    {"bm_daniel", "daniel"},
    {"bm_fable", "fable"},
    {"bm_george", "george"},
    {"bm_lewis", "lewis"},
};

std::string resolveVoiceName(const std::string& input) {
    if (input.empty()) return input;

    if (input.find('_') != std::string::npos) {
        return input;
    }

    std::vector<std::string> matches;
    for (const auto& [full, shortname] : kKokoroVoices) {
        if (shortname == input) {
            matches.push_back(full);
        }
    }

    if (matches.size() == 1) {
        std::cout << "音色: " << input << " -> " << matches[0] << std::endl;
        return matches[0];
    }

    if (matches.size() > 1) {
        std::cerr << "错误: 音色名 '" << input << "' 有多个匹配:\n";
        for (const auto& m : matches) {
            std::cerr << "  " << m << "\n";
        }
        std::cerr << "请使用完整名称，如 --tts kokoro:" << matches[0] << "\n";
        exit(1);
    }

    std::cerr << "警告: 未知音色 '" << input << "'，将直接使用该名称\n"
            << "使用 --list-voices 查看可用音色列表\n";
    return input;
}

void printVoiceList() {
    std::cout
        << "Kokoro 可用音色列表:\n"
        << "\n"
        << "中文女声 (zf_):\n"
        << "  zf_xiaobei      小北 (默认)\n"
        << "  zf_xiaoni       小妮\n"
        << "  zf_xiaoxiao     小小\n"
        << "  zf_xiaoyi       小一\n"
        << "\n"
        << "中文男声 (zm_):\n"
        << "  zm_yunxi        云希\n"
        << "  zm_yunyang      云阳\n"
        << "  zm_yunjian      云健\n"
        << "  zm_yunfan       云帆\n"
        << "\n"
        << "美式英语女声 (af_):\n"
        << "  af_heart        Heart\n"
        << "  af_alloy        Alloy\n"
        << "  af_aoede        Aoede\n"
        << "  af_bella        Bella\n"
        << "  af_jessica      Jessica\n"
        << "  af_kore         Kore\n"
        << "  af_nicole       Nicole\n"
        << "  af_nova         Nova\n"
        << "  af_river        River\n"
        << "  af_sarah        Sarah\n"
        << "  af_sky          Sky\n"
        << "\n"
        << "美式英语男声 (am_):\n"
        << "  am_adam         Adam\n"
        << "  am_echo         Echo\n"
        << "  am_eric         Eric\n"
        << "  am_fenrir       Fenrir\n"
        << "  am_liam         Liam\n"
        << "  am_michael      Michael\n"
        << "  am_onyx         Onyx\n"
        << "  am_puck         Puck\n"
        << "\n"
        << "英式英语女声 (bf_):\n"
        << "  bf_alice        Alice\n"
        << "  bf_emma         Emma\n"
        << "  bf_isabella     Isabella\n"
        << "  bf_lily         Lily\n"
        << "\n"
        << "英式英语男声 (bm_):\n"
        << "  bm_daniel       Daniel\n"
        << "  bm_fable        Fable\n"
        << "  bm_george       George\n"
        << "  bm_lewis        Lewis\n"
        << "\n"
        << "用法: --tts kokoro:<voice>  支持短名 (xiaobei) 和全名 (zf_xiaobei)\n"
        << std::endl;
}

EngineSelection parseEngine(const std::string& spec) {
    EngineSelection sel;
    sel.backend = SpacemiT::BackendType::MATCHA_ZH;

    auto colon = spec.find(':');
    std::string engine =
        (colon != std::string::npos) ? spec.substr(0, colon) : spec;
    std::string variant =
        (colon != std::string::npos) ? spec.substr(colon + 1) : "";

    if (engine == "matcha") {
        if (variant.empty() || variant == "zh") {
            sel.backend = SpacemiT::BackendType::MATCHA_ZH;
        } else if (variant == "en") {
            sel.backend = SpacemiT::BackendType::MATCHA_EN;
        } else if (variant == "zh-en" || variant == "zhen") {
            sel.backend = SpacemiT::BackendType::MATCHA_ZH_EN;
        } else {
            std::cerr << "错误: 未知 Matcha 变体 '" << variant << "'\n"
                    << "可用变体: zh, en, zh-en\n";
            exit(1);
        }
        return sel;
    }

    if (engine == "kokoro") {
        sel.backend = SpacemiT::BackendType::KOKORO;
        sel.voice = resolveVoiceName(variant);
        return sel;
    }

    std::cerr << "错误: 未知引擎 '" << engine << "'\n"
            << "可用引擎: matcha, kokoro\n"
            << "用法: --tts matcha:zh 或 --tts kokoro:zf_xiaobei\n";
    exit(1);
}

// ============================================================================
// Audio conversion utilities
// ============================================================================

std::vector<float> pcm16BytesToFloat(const std::vector<uint8_t>& bytes) {
    size_t num_samples = bytes.size() / 2;
    std::vector<float> output(num_samples);
    const int16_t* samples = reinterpret_cast<const int16_t*>(bytes.data());

    for (size_t i = 0; i < num_samples; ++i) {
        output[i] = samples[i] / 32768.0f;
    }

    return output;
}

void saveWav(const std::string& filename, const std::vector<int16_t>& data,
            int sample_rate) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "无法创建文件: " << filename << std::endl;
        return;
    }

    uint32_t data_size = static_cast<uint32_t>(data.size() * sizeof(int16_t));
    uint32_t file_size = 36 + data_size;

    // RIFF header
    file.write("RIFF", 4);
    file.write(reinterpret_cast<const char*>(&file_size), 4);
    file.write("WAVE", 4);

    // fmt chunk
    file.write("fmt ", 4);
    uint32_t fmt_size = 16;
    uint16_t audio_format = 1;  // PCM
    uint16_t num_channels = 1;
    uint32_t sr = static_cast<uint32_t>(sample_rate);
    uint32_t byte_rate = sr * 2;
    uint16_t block_align = 2;
    uint16_t bits_per_sample = 16;

    file.write(reinterpret_cast<const char*>(&fmt_size), 4);
    file.write(reinterpret_cast<const char*>(&audio_format), 2);
    file.write(reinterpret_cast<const char*>(&num_channels), 2);
    file.write(reinterpret_cast<const char*>(&sr), 4);
    file.write(reinterpret_cast<const char*>(&byte_rate), 4);
    file.write(reinterpret_cast<const char*>(&block_align), 2);
    file.write(reinterpret_cast<const char*>(&bits_per_sample), 2);

    // data chunk
    file.write("data", 4);
    file.write(reinterpret_cast<const char*>(&data_size), 4);
    file.write(reinterpret_cast<const char*>(data.data()), data_size);
}
