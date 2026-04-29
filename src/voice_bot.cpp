/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * 语音对话系统 Demo (非 AEC 模式)
 *
 * 使用独立的 AudioCapture 和 AudioPlayer，适用于硬件自带 AEC 的场景
 * 录音默认 16kHz/1ch，直连 VAD/STT，无需重采样
 * 播放使用独立采样率（默认 48kHz）
 * 支持 barge-in（用户打断 TTS 播放）
 *
 * 用法:
 *   ./voice_chat [--tts matcha:zh|matcha:en|matcha:zh-en|kokoro|kokoro:<voice>]
 * [--model qwen2.5:0.5b] [--input-device 0] [--output-device 0]
 */

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// Audio capture/playback
#include "audio_base.hpp"

// Resampler (capture rate <-> 16kHz, TTS rate -> playback rate)
#include "audio_resampler.hpp"

// Shared modules
#include "engine_init.hpp"
#include "voice_common.hpp"
#include "voice_pipeline.hpp"

// Reachy Voice Control
#include "voice_ctl.h"

// Talent Show - Dance Player
#include "talent_show/dance_player.h"

// ============================================================================
// 参数配置
// ============================================================================

struct Config {
    std::string tts_type = "matcha:zh";
    bool list_voices = false;
    std::string llm_model = "qwen2.5:0.5b";
    std::string llm_url = "";
    int input_device = -1;
    int output_device = -1;
    float vad_threshold = 0.8f;
    float silence_duration = 0.5f;
    int max_tokens = 150;
    bool list_devices = false;

    // Audio config (independent capture/playback)
    int capture_rate = 16000;
    int capture_channels = 1;
    int playback_rate = 48000;
    int playback_channels = 1;

    // 调试：音频录制
    bool save_audio = false;
    std::string audio_file = "voice_debug.wav";

    // Motor Controller configuration
    std::string motor_port = "/dev/ttyACM0";

    // MCP 配置
    std::string mcp_config_path = "";

    // 跟踪 --model 是否被显式指定
    bool llm_model_set = false;
};

Config parseArgs(int argc, char *argv[]) {
    Config cfg;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--tts") == 0 && i + 1 < argc) {
            cfg.tts_type = argv[++i];
        } else if (strcmp(argv[i], "--model") == 0 && i + 1 < argc) {
            cfg.llm_model = argv[++i];
            cfg.llm_model_set = true;
        } else if (
            (strcmp(argv[i], "--llm-url") == 0 || strcmp(argv[i], "--llm_url") == 0) &&
            i + 1 < argc) {
            cfg.llm_url = argv[++i];
        } else if (
            (strcmp(argv[i], "--input-device") == 0 || strcmp(argv[i], "-i") == 0) &&
            i + 1 < argc) {
            cfg.input_device = std::stoi(argv[++i]);
        } else if (
            (strcmp(argv[i], "--output-device") == 0 || strcmp(argv[i], "-o") == 0) &&
            i + 1 < argc) {
            cfg.output_device = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--list-devices") == 0 || strcmp(argv[i], "-l") == 0) {
            cfg.list_devices = true;
        } else if (strcmp(argv[i], "--capture-rate") == 0 && i + 1 < argc) {
            cfg.capture_rate = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--capture-channels") == 0 && i + 1 < argc) {
            cfg.capture_channels = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--playback-rate") == 0 && i + 1 < argc) {
            cfg.playback_rate = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--playback-channels") == 0 && i + 1 < argc) {
            cfg.playback_channels = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--motor-port") == 0 && i + 1 < argc) {
            cfg.motor_port = argv[++i];
        } else if (strcmp(argv[i], "--save-audio") == 0) {
            cfg.save_audio = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                cfg.audio_file = argv[++i];
            }
        } else if (strcmp(argv[i], "--mcp-config") == 0 && i + 1 < argc) {
            cfg.mcp_config_path = argv[++i];
        } else if (strcmp(argv[i], "--list-voices") == 0) {
            cfg.list_voices = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            std::cout << "用法: " << argv[0] << " [选项]\n";
            std::cout << "\n音频设备:\n";
            std::cout << "  -i, --input-device <id>       输入设备索引 (默认: 系统默认)\n";
            std::cout << "  -o, --output-device <id>      输出设备索引 (默认: 系统默认)\n";
            std::cout << "  -l, --list-devices            列出可用音频设备\n";
            std::cout << "\n音频参数:\n";
            std::cout << "  --capture-rate <hz>           录音采样率 (默认: 16000)\n";
            std::cout << "  --capture-channels <n>        录音声道数 (默认: 1)\n";
            std::cout << "  --playback-rate <hz>          播放采样率 (默认: 48000)\n";
            std::cout << "  --playback-channels <n>       播放声道数 (默认: 1)\n";
            std::cout << "\nLLM:\n";
            std::cout << "  --model <name>                LLM模型 (默认: qwen2.5:0.5b)\n";
            std::cout << "  --llm-url <url>               LLM API地址 (必填)\n";
            std::cout << "\nTTS:\n";
            std::cout << "  --tts <engine>                TTS后端 (默认: matcha:zh)\n";
            std::cout << "                                matcha:zh / matcha:en / matcha:zh-en\n";
            std::cout << "                                kokoro / kokoro:<voice>\n";
            std::cout << "  --list-voices                 列出 Kokoro 可用音色\n";
            std::cout << "\n调试:\n";
            std::cout << "  --save-audio [file]           保存录音 (默认: voice_debug.wav)\n";
            std::cout << "  --motor-port <port>           电机串口路径 (默认: /dev/ttyACM0)\n";
            std::cout << "\nMCP:\n";
            std::cout << "  --mcp-config <path>           MCP配置文件 (启用工具调用)\n";
            std::cout << "\n其他:\n";
            std::cout << "  -h, --help                    显示帮助\n";
            exit(0);
        }
    }
    return cfg;
}

// ============================================================================
// 列出音频设备
// ============================================================================

void listAudioDevices() {
    std::cout << getTimestamp() << " ========================================\n";
    std::cout << getTimestamp() << "            可用音频设备\n";
    std::cout << getTimestamp() << " ========================================\n\n";

    std::cout << getTimestamp() << " 输入设备 (麦克风):\n";
    auto input_devices = SpacemitAudio::AudioCapture::ListDevices();
    if (input_devices.empty()) {
        std::cout << getTimestamp() << "   (无可用设备)\n";
    } else {
        for (const auto &dev : input_devices) {
            std::cout << getTimestamp() << "   [" << dev.first << "] " << dev.second << "\n";
        }
    }

    std::cout << getTimestamp() << " \n输出设备 (扬声器):\n";
    auto output_devices = SpacemitAudio::AudioPlayer::ListDevices();
    if (output_devices.empty()) {
        std::cout << getTimestamp() << "   (无可用设备)\n";
    } else {
        for (const auto &dev : output_devices) {
            std::cout << getTimestamp() << "   [" << dev.first << "] " << dev.second << "\n";
        }
    }

    std::cout << getTimestamp() << " \n使用方法:\n";
    std::cout << getTimestamp() << "   voice_chat -i <输入设备ID> -o <输出设备ID>\n";
    std::cout << getTimestamp() << " ========================================\n";
}

// ============================================================================
// 主程序
// ============================================================================

int main(int argc, char *argv[]) {
    signal(SIGINT, voiceSignalHandler);

    Config cfg = parseArgs(argc, argv);

    if (cfg.list_devices) {
        listAudioDevices();
        return 0;
    }

    if (cfg.list_voices) {
        printVoiceList();
        return 0;
    }

    if (cfg.llm_url.empty()) {
        std::cerr << "错误: 必须通过 --llm-url 指定 LLM API 地址\n";
        return 1;
    }

    std::cout << getTimestamp() << " ========================================\n";
    std::cout << getTimestamp() << "    语音对话系统 (非 AEC 模式)\n";
    std::cout << getTimestamp() << " ========================================\n";
    std::cout << getTimestamp() << " TTS后端: " << cfg.tts_type << "\n";
    std::cout << getTimestamp() << " LLM模型: " << cfg.llm_model << "\n";
    std::cout << getTimestamp() << " LLM URL: " << cfg.llm_url << "\n";
    std::cout << getTimestamp() << " 录音: " << cfg.capture_rate << " Hz / "
        << cfg.capture_channels << " ch\n";
    std::cout << getTimestamp() << " 播放: " << cfg.playback_rate << " Hz / "
        << cfg.playback_channels << " ch\n";
    std::cout << getTimestamp() << " 按 Ctrl+C 退出\n";
    std::cout << getTimestamp() << " ========================================\n\n";

    // 初始化电机控制模块
    if (voice_ctl_init(cfg.motor_port.c_str(), 0.0f) < 0) {
        std::cerr << getTimestamp() << " 警告: 电机初始化失败, 执行运动指令时可能受限\n";
    }

    // -------------------------------------------------------------------------
    // 1-4. 初始化引擎
    // -------------------------------------------------------------------------
    auto llm_result =
        initLLM(cfg.llm_model, cfg.llm_url, "You are a helpful assistant.", cfg.max_tokens);
    if (!llm_result.llm)
        return 1;
    auto llm = llm_result.llm;
    auto system_prompt = llm_result.system_prompt;

    auto vad = initVAD(cfg.vad_threshold);
    if (!vad)
        return 1;

    auto asr = initASR();
    if (!asr)
        return 1;

    auto tts_result = initTTS(cfg.tts_type);
    if (!tts_result.tts)
        return 1;
    auto tts = tts_result.tts;
    int tts_sample_rate = tts_result.sample_rate;

    // -------------------------------------------------------------------------
    // 5. 初始化音频设备
    // -------------------------------------------------------------------------
    std::cout << getTimestamp() << " [5/5] 初始化音频设备..." << std::flush;

    SpacemitAudio::AudioCapture capture(cfg.input_device);
    SpacemitAudio::AudioPlayer player(cfg.output_device);

    // 初始化录音重采样器（capture_rate != 16kHz 时使用）
    std::unique_ptr<Resampler> capture_resampler;
    if (cfg.capture_rate != 16000) {
        Resampler::Config rconf;
        rconf.input_sample_rate = cfg.capture_rate;
        rconf.output_sample_rate = 16000;
        rconf.channels = 1;
        rconf.method = (cfg.capture_rate > 16000)
            ? ResampleMethod::LINEAR_DOWNSAMPLE
            : ResampleMethod::LINEAR_UPSAMPLE;
        capture_resampler = std::make_unique<Resampler>(rconf);
        if (!capture_resampler->initialize()) {
            std::cerr << "\n" << getTimestamp() << " 错误: 录音重采样器初始化失败\n";
            return 1;
        }
    }

    // 初始化播放重采样器（TTS 采样率 != 播放采样率时使用）
    std::unique_ptr<Resampler> playback_resampler;
    if (tts_sample_rate != cfg.playback_rate) {
        Resampler::Config rconf;
        rconf.input_sample_rate = tts_sample_rate;
        rconf.output_sample_rate = cfg.playback_rate;
        rconf.channels = 1;
        rconf.method = (tts_sample_rate < cfg.playback_rate)
            ? ResampleMethod::LINEAR_UPSAMPLE
            : ResampleMethod::LINEAR_DOWNSAMPLE;
        playback_resampler = std::make_unique<Resampler>(rconf);
        if (!playback_resampler->initialize()) {
            std::cerr << "\n" << getTimestamp() << " 错误: 播放重采样器初始化失败\n";
            return 1;
        }
    }

    std::cout << " OK\n";
    if (cfg.capture_rate == 16000 && cfg.capture_channels == 1) {
        std::cout << getTimestamp() << " 录音管道: 16kHz/1ch -> 直连 VAD/STT (零重采样)\n";
    } else {
        std::cout << getTimestamp() << " 录音管道: " << cfg.capture_rate << "Hz/"
            << cfg.capture_channels << "ch -> ";
        if (cfg.capture_channels > 1)
            std::cout << "混音->mono -> ";
        if (cfg.capture_rate != 16000)
            std::cout << "重采样->16kHz -> ";
        std::cout << "VAD/STT\n";
    }
    if (tts_sample_rate == cfg.playback_rate) {
        std::cout << getTimestamp() << " 播放管道: TTS(" << tts_sample_rate << "Hz) -> 直连播放\n";
    } else {
        std::cout << getTimestamp() << " 播放管道: TTS(" << tts_sample_rate << "Hz) -> 重采样->"
            << cfg.playback_rate << "Hz -> 播放\n";
    }
    std::cout << "\n";

    // -------------------------------------------------------------------------
    // 6. 初始化 MCP (可选)
    // -------------------------------------------------------------------------
#ifdef USE_MCP
    if (cfg.mcp_config_path.empty()) {
#ifdef DEFAULT_MCP_CONFIG_PATH
        cfg.mcp_config_path = DEFAULT_MCP_CONFIG_PATH;
        std::cout << getTimestamp() << " [MCP] 使用默认配置文件: " << cfg.mcp_config_path << "\n";
#endif
    }
    MCPInitResult mcp;
    initMCP(
        cfg.mcp_config_path, llm, system_prompt, mcp, cfg.llm_url,
        cfg.llm_model_set ? cfg.llm_model : "");
#endif

    // -------------------------------------------------------------------------
    // 播放队列和播放线程
    // -------------------------------------------------------------------------
    std::queue<std::vector<uint8_t>> playback_queue;
    std::mutex playback_mutex;
    std::condition_variable playback_cv;
    std::atomic<bool> is_playing{false};
    std::atomic<bool> playback_paused{false};  // 舞蹈期间暂停播放线程

    if (!player.Start(cfg.playback_rate, cfg.playback_channels)) {
        std::cerr << getTimestamp() << " 错误: 无法启动播放设备\n";
        return 1;
    }

    // 播放线程：从队列取数据写入 AudioPlayer，队列空时写静音防止 ALSA XRUN
    std::thread playback_thread([&]() {
        const size_t silence_frames = cfg.playback_rate / 50;  // 20ms
        const size_t silence_bytes = silence_frames * cfg.playback_channels * sizeof(int16_t);
        const std::vector<uint8_t> silence(silence_bytes, 0);

        while (g_running) {
            std::vector<uint8_t> chunk;
            {
                std::unique_lock<std::mutex> lock(playback_mutex);
                playback_cv.wait_for(lock, std::chrono::milliseconds(20), [&] {
                    return !playback_queue.empty() || !g_running;
                });
                if (!g_running)
                    break;
                if (!playback_queue.empty()) {
                    chunk = std::move(playback_queue.front());
                    playback_queue.pop();
                }
            }
            if (playback_paused) {
                is_playing = false;
                continue;
            }
            if (chunk.empty()) {
                player.Write(silence);
                is_playing = false;
            } else {
                is_playing = true;
                player.Write(chunk);
                {
                    std::lock_guard<std::mutex> lock(playback_mutex);
                    if (playback_queue.empty()) {
                        is_playing = false;
                    }
                }
            }
        }
    });

    // 入队播放数据：float mono -> resample -> expand channels -> PCM16 bytes ->
    // enqueue
    auto enqueuePlayback = [&](const std::vector<float> &float_samples, int src_rate) {
        std::vector<float> resampled;
        if (src_rate != cfg.playback_rate && playback_resampler) {
            resampled = playback_resampler->process(float_samples);
        } else {
            resampled = float_samples;
        }

        size_t total_samples;
        if (cfg.playback_channels > 1) {
            total_samples = resampled.size() * cfg.playback_channels;
        } else {
            total_samples = resampled.size();
        }

        std::vector<uint8_t> pcm_bytes(total_samples * 2);
        int16_t *out = reinterpret_cast<int16_t *>(pcm_bytes.data());

        if (cfg.playback_channels > 1) {
            for (size_t i = 0; i < resampled.size(); ++i) {
                int16_t sample =
                    static_cast<int16_t>(std::clamp(resampled[i], -1.0f, 1.0f) * 32767.0f);
                for (int ch = 0; ch < cfg.playback_channels; ++ch) {
                    out[i * cfg.playback_channels + ch] = sample;
                }
            }
        } else {
            for (size_t i = 0; i < resampled.size(); ++i) {
                out[i] = static_cast<int16_t>(std::clamp(resampled[i], -1.0f, 1.0f) * 32767.0f);
            }
        }

        {
            const size_t chunk_bytes =
                (cfg.playback_rate / 50) * cfg.playback_channels * sizeof(int16_t);
            std::lock_guard<std::mutex> lock(playback_mutex);
            for (size_t offset = 0; offset < pcm_bytes.size(); offset += chunk_bytes) {
                size_t len = std::min(chunk_bytes, pcm_bytes.size() - offset);
                playback_queue.push(
                    std::vector<uint8_t>(
                        pcm_bytes.data() + offset, pcm_bytes.data() + offset + len));
            }
        }
        playback_cv.notify_one();
    };

    auto clearPlayback = [&]() {
        {
            std::lock_guard<std::mutex> lock(playback_mutex);
            std::queue<std::vector<uint8_t>> empty;
            playback_queue.swap(empty);
        }
        is_playing = false;
    };

    // -------------------------------------------------------------------------
    // 状态变量
    // -------------------------------------------------------------------------
    std::vector<float> audio_buffer;
    std::mutex buffer_mutex;
    int silence_frames_count = 0;
    const int silence_frames_threshold = static_cast<int>(cfg.silence_duration * 16000 / 512);
    bool is_speaking = false;
    int frame_count = 0;

    const size_t PRE_BUFFER_FRAMES = 30;
    std::deque<std::vector<float>> pre_buffer;

    int barge_in_confirm_frames = 0;
    const int BARGE_IN_CONFIRM_THRESHOLD = 5;

    std::atomic<bool> barge_in_recording{false};

    std::vector<int16_t> recorded_audio;
    std::mutex record_mutex;

    const size_t VAD_FRAME_SIZE = 512;
    std::vector<float> vad_frame_buffer;

    // -------------------------------------------------------------------------
    // 构造 VoicePipelineContext
    // -------------------------------------------------------------------------
    VoicePipelineContext pipeline_ctx;
    pipeline_ctx.llm = llm;
    pipeline_ctx.tts = tts;
    pipeline_ctx.vad = vad;
    pipeline_ctx.tts_sample_rate = tts_sample_rate;
    pipeline_ctx.system_prompt = system_prompt;
    pipeline_ctx.enqueue_playback = enqueuePlayback;
    pipeline_ctx.is_playing = [&]() { return is_playing.load(); };
    pipeline_ctx.clear_playback = clearPlayback;
    pipeline_ctx.audio_buffer = &audio_buffer;
    pipeline_ctx.buffer_mutex = &buffer_mutex;
    pipeline_ctx.silence_frames = &silence_frames_count;
    pipeline_ctx.is_speaking = &is_speaking;
    pipeline_ctx.barge_in_recording = &barge_in_recording;
    pipeline_ctx.vad_frame_buffer = &vad_frame_buffer;
    pipeline_ctx.pre_buffer = &pre_buffer;
#ifdef USE_MCP
    PendingDance pending_dance;  // 待处理的舞蹈动作标志
    pipeline_ctx.mcp_manager = mcp.manager.get();
    pipeline_ctx.action_provider = mcp.action_provider.get();  // 新增
    pipeline_ctx.llm_tools_json = &mcp.llm_tools_json;
    pipeline_ctx.tools_mutex = &mcp.tools_mutex;
    pipeline_ctx.conversation_messages = &mcp.conversation_messages;
    pipeline_ctx.conversation_mutex = &mcp.conversation_mutex;
    pipeline_ctx.mcp_enabled = mcp.enabled;
    pipeline_ctx.pending_dance = &pending_dance;
    if (mcp.action_provider) {
        mcp.action_provider->setPendingDance(&pending_dance);
    }
#endif

    // -------------------------------------------------------------------------
    // 准备舞蹈音频配置 (DanceAudioConfig)
    // -------------------------------------------------------------------------
    DanceAudioConfig dance_audio_cfg;
    dance_audio_cfg.input_device = cfg.input_device;
    dance_audio_cfg.output_device = cfg.output_device;
    dance_audio_cfg.capture_rate = cfg.capture_rate;
    dance_audio_cfg.capture_channels = cfg.capture_channels;
    dance_audio_cfg.playback_rate = cfg.playback_rate;
    dance_audio_cfg.playback_channels = cfg.playback_channels;

    // -------------------------------------------------------------------------
    // 设置录音回调
    // -------------------------------------------------------------------------
    capture.SetCallback([&](const uint8_t *data, size_t size) {
        if (!g_running)
            return;

        // PCM16 little-endian -> float
        size_t num_samples = size / 2;
        const int16_t *pcm = reinterpret_cast<const int16_t *>(data);
        std::vector<float> float_samples(num_samples);
        for (size_t i = 0; i < num_samples; ++i) {
            float_samples[i] = pcm[i] / 32768.0f;
        }

        // 多声道混音到 mono
        if (cfg.capture_channels > 1) {
            size_t frames = num_samples / cfg.capture_channels;
            std::vector<float> mono(frames);
            for (size_t i = 0; i < frames; ++i) {
                float sum = 0.0f;
                for (int ch = 0; ch < cfg.capture_channels; ++ch) {
                    sum += float_samples[i * cfg.capture_channels + ch];
                }
                mono[i] = sum / cfg.capture_channels;
            }
            float_samples = std::move(mono);
        }

        // 重采样到 16kHz（如果需要）
        std::vector<float> samples_16k;
        if (capture_resampler) {
            samples_16k = capture_resampler->process(float_samples);
        } else {
            samples_16k = std::move(float_samples);
        }

        if (samples_16k.empty())
            return;

        // 录制音频（用于调试）
        if (cfg.save_audio) {
            std::lock_guard<std::mutex> lock(record_mutex);
            for (float s : samples_16k) {
                recorded_audio.push_back(
                    static_cast<int16_t>(std::clamp(s, -1.0f, 1.0f) * 32767.0f));
            }
        }

        // 累积音频到 VAD 帧缓冲区
        vad_frame_buffer.insert(vad_frame_buffer.end(), samples_16k.begin(), samples_16k.end());

        while (vad_frame_buffer.size() >= VAD_FRAME_SIZE && g_running) {
            std::vector<float> vad_frame(
                vad_frame_buffer.begin(), vad_frame_buffer.begin() + VAD_FRAME_SIZE);
            vad_frame_buffer.erase(
                vad_frame_buffer.begin(), vad_frame_buffer.begin() + VAD_FRAME_SIZE);

            auto vad_result = vad->Detect(vad_frame);
            float vad_prob = vad_result ? vad_result->GetProbability() : 0.0f;

            frame_count++;
            if (frame_count % 10 == 0 && !g_processing) {
                std::cout << "\r" << getTimestamp() << " [VAD] prob=" << std::fixed
                    << std::setprecision(2) << vad_prob
                    << " speaking=" << (is_speaking ? "Y" : "N")
                    << " buffer=" << audio_buffer.size()
                    << " playing=" << (is_playing.load() ? "Y" : "N") << "      "
                    << std::flush;
            }

            // TTS 播放期间：检测 barge-in
            if (g_processing) {
                if (barge_in_recording && is_speaking) {
                    std::unique_lock<std::mutex> lock(buffer_mutex);
                    audio_buffer.insert(audio_buffer.end(), vad_frame.begin(), vad_frame.end());

                    if (vad_prob <= cfg.vad_threshold) {
                        silence_frames_count++;
                    } else {
                        silence_frames_count = 0;
                    }
                    continue;
                }

                if (is_playing.load() && vad_prob > cfg.vad_threshold) {
                    barge_in_confirm_frames++;
                    pre_buffer.push_back(vad_frame);
                    if (pre_buffer.size() > PRE_BUFFER_FRAMES + BARGE_IN_CONFIRM_THRESHOLD) {
                        pre_buffer.pop_front();
                    }

                    if (barge_in_confirm_frames >= BARGE_IN_CONFIRM_THRESHOLD) {
                        std::cout << "\n" << getTimestamp()
                            << " [Barge-in] 用户打断 (连续" << barge_in_confirm_frames
                            << "帧, prob=" << vad_prob << ")，停止播放\n";
                        clearPlayback();
                        g_barge_in = true;
                        barge_in_recording = true;
                        barge_in_confirm_frames = 0;

                        std::unique_lock<std::mutex> lock(buffer_mutex);
                        is_speaking = true;
                        audio_buffer.clear();
                        for (const auto &frame : pre_buffer) {
                            audio_buffer.insert(audio_buffer.end(), frame.begin(), frame.end());
                        }
                        pre_buffer.clear();
                        silence_frames_count = 0;
                    }
                } else {
                    barge_in_confirm_frames = 0;
                    pre_buffer.push_back(vad_frame);
                    if (pre_buffer.size() > PRE_BUFFER_FRAMES) {
                        pre_buffer.pop_front();
                    }
                }
                continue;
            }

            std::unique_lock<std::mutex> lock(buffer_mutex);

            if (vad_prob > cfg.vad_threshold) {
                if (!is_speaking) {
                    is_speaking = true;
                    audio_buffer.clear();

                    for (const auto &frame : pre_buffer) {
                        audio_buffer.insert(audio_buffer.end(), frame.begin(), frame.end());
                    }
                    pre_buffer.clear();

                    std::cout << "\n" << getTimestamp()
                        << " [VAD] 开始说话 (prob=" << vad_prob << ")...\n";
                }
                audio_buffer.insert(audio_buffer.end(), vad_frame.begin(), vad_frame.end());
                silence_frames_count = 0;
            } else if (is_speaking) {
                audio_buffer.insert(audio_buffer.end(), vad_frame.begin(), vad_frame.end());
                silence_frames_count++;

                if (silence_frames_count >= silence_frames_threshold) {
                    is_speaking = false;
                    barge_in_recording = false;
                    std::cout << "\n" << getTimestamp() << " [VAD] 停止说话，触发识别\n";

                    if (audio_buffer.size() > 8000) {
                        // 关键修复：将缓冲区内存克隆到局部变量并立即释放锁，避免与处理线程收尾时的锁需求冲突产生死锁
                        std::vector<float> audio_to_process = audio_buffer;
                        audio_buffer.clear();
                        lock.unlock();

                        std::cout << getTimestamp() << " [ASR] 开始识别...\n";
                        auto result = asr->Recognize(audio_to_process, 16000);
                        if (result && !result->IsEmpty()) {
                            std::string text = result->GetText();
                            std::cout << getTimestamp() << " [ASR] 识别完成: \"" << text << "\"\n";

                            // 1. 将意图映射转换为动作 ID，并获取匹配到的关键词
                            char matched_keyword[64] = {0};
                            int pending_action = voice_ctl_match(
                                text.c_str(), matched_keyword, sizeof(matched_keyword));
                            std::string keyword_str(matched_keyword);

                            {
                                std::lock_guard<std::mutex> lock2(g_process_thread_mutex);
                                if (g_process_thread && g_process_thread->joinable()) {
                                    g_process_thread->join();
                                }
                                g_process_thread = std::make_unique<std::thread>(
                                    [&pipeline_ctx, text, pending_action, keyword_str,
                                        &dance_audio_cfg, &player, &cfg, &playback_paused]() {
                                    if (pending_action != ACTION_NONE && !keyword_str.empty()) {
                                        // 2a. 命中关键词：直接语音回复 "好的，[关键词]"，绕过
                                        // LLM
                                        std::string reply = "好的，" + keyword_str;
                                        std::cout << getTimestamp()
                                            << " [关键词命中] 语音回复: \"" << reply << "\"\n";

                                        auto tts_result = pipeline_ctx.tts->Call(reply);
                                        if (tts_result && tts_result->IsSuccess()) {
                                            auto audio_bytes = tts_result->GetAudioData();
                                            pipeline_ctx.enqueue_playback(
                                                pcm16BytesToFloat(audio_bytes),
                                                pipeline_ctx.tts_sample_rate);
                                        }

                                        // 等待播报完成
                                        while (pipeline_ctx.is_playing() && g_running
                                            && !g_barge_in) {
                                            std::this_thread::sleep_for(
                                                std::chrono::milliseconds(50));
                                        }

                                        // 播报完成，激活肢体动作
                                        if (!g_barge_in) {
                                            if (pending_action >= 9 && pending_action <= 12) {
                                                const char *dance_name = nullptr;
                                                if (pending_action == 9)
                                                    dance_name = "headbanger";
                                                else if (pending_action == 10)
                                                    dance_name = "jackson";
                                                else if (pending_action == 11)
                                                    dance_name = "chicken";
                                                else if (pending_action == 12)
                                                    dance_name = "uh_huh_tilt";

                                                const DanceRoutine *routine = dance_player_find(
                                                    g_dance_routines, g_dance_routine_count,
                                                    dance_name);
                                                if (routine) {
                                                    std::cout << getTimestamp()
                                                        << " [DancePlayer] 执行同步舞蹈: "
                                                        << routine->name << "\n";
                                                    // 暂停播放线程，关闭流释放 ALSA 设备
                                                    pipeline_ctx.clear_playback();
                                                    playback_paused = true;
                                                    player.Close();
                                                    dance_player_execute(
                                                        routine, voice_ctl_get_controller(),
                                                        &dance_audio_cfg);
                                                    // 重新打开并恢复播放流
                                                    player.Start(
                                                        cfg.playback_rate, cfg.playback_channels);
                                                    playback_paused = false;
                                                    // 回正
                                                    voice_ctl_execute(8);
                                                }
                                            } else {
                                                int res = voice_ctl_execute(pending_action);
                                                if (res == ACTION_LIMIT_EXCEEDED) {
                                                    std::string limit_reply = "我已经到极限啦";
                                                    std::cout << getTimestamp()
                                                        << " [限位触发] 语音回复: \""
                                                        << limit_reply << "\"\n";
                                                    auto tts_limit =
                                                        pipeline_ctx.tts->Call(limit_reply);
                                                    if (tts_limit && tts_limit->IsSuccess()) {
                                                        auto audio_limit =
                                                            tts_limit->GetAudioData();
                                                        pipeline_ctx.enqueue_playback(
                                                            pcm16BytesToFloat(audio_limit),
                                                            pipeline_ctx.tts_sample_rate);
                                                    }
                                                }
                                            }
                                        }

                                        // 清理缓冲区 (参考 processText 内部逻辑)
                                        if (!g_barge_in) {
                                            {
                                                std::lock_guard<std::mutex> lock(
                                                    *pipeline_ctx.buffer_mutex);
                                                pipeline_ctx.audio_buffer->clear();
                                                pipeline_ctx.pre_buffer->clear();
                                                *pipeline_ctx.silence_frames = 0;
                                                *pipeline_ctx.is_speaking = false;
                                            }
                                            *pipeline_ctx.barge_in_recording = false;
                                            pipeline_ctx.vad_frame_buffer->clear();
                                            pipeline_ctx.vad->Reset();
                                        }
                                    } else {
                                        // 2b. 未命中关键词：常规 LLM 流程
                                        processText(pipeline_ctx, text);

#ifdef USE_MCP
                                        // 检查是否有 MCP 触发的待处理舞蹈动作
                                        int mcp_dance_action = -1;
                                        if (pipeline_ctx.pending_dance) {
                                            std::lock_guard<std::mutex> lock(
                                                pipeline_ctx.pending_dance->mutex);
                                            mcp_dance_action =
                                                pipeline_ctx.pending_dance->action_id.exchange(-1);
                                        }

                                        if (mcp_dance_action >= 9 && mcp_dance_action <= 12 &&
                                            !g_barge_in) {
                                            const char *dance_name = nullptr;
                                            if (mcp_dance_action == 9)
                                                dance_name = "headbanger";
                                            else if (mcp_dance_action == 10)
                                                dance_name = "jackson";
                                            else if (mcp_dance_action == 11)
                                                dance_name = "chicken";
                                            else if (mcp_dance_action == 12)
                                                dance_name = "uh_huh_tilt";

                                            const DanceRoutine *routine = dance_player_find(
                                                g_dance_routines, g_dance_routine_count,
                                                dance_name);
                                            if (routine) {
                                                std::cout
                                                    << getTimestamp()
                                                    << " [MCP舞蹈] 执行同步舞蹈: " << routine->name
                                                    << "\n";
                                                // 暂停播放线程，关闭流释放 ALSA 设备
                                                pipeline_ctx.clear_playback();
                                                playback_paused = true;
                                                player.Close();
                                                dance_player_execute(
                                                    routine, voice_ctl_get_controller(),
                                                    &dance_audio_cfg);
                                                // 重新打开并恢复播放流
                                                player.Start(
                                                    cfg.playback_rate, cfg.playback_channels);
                                                playback_paused = false;
                                                // 回正
                                                voice_ctl_execute(8);
                                            }
                                        }
#endif

                                        // 播报完成，按需激活肢体动作（如果在讲话高频被打断，静默丢弃）
                                        if (pending_action != ACTION_NONE && !g_barge_in) {
                                            if (pending_action >= 9 && pending_action <= 12) {
                                                const char *dance_name = nullptr;
                                                if (pending_action == 9)
                                                    dance_name = "headbanger";
                                                else if (pending_action == 10)
                                                    dance_name = "jackson";
                                                else if (pending_action == 11)
                                                    dance_name = "chicken";
                                                else if (pending_action == 12)
                                                    dance_name = "uh_huh_tilt";

                                                const DanceRoutine *routine = dance_player_find(
                                                    g_dance_routines, g_dance_routine_count,
                                                    dance_name);
                                                if (routine) {
                                                    std::cout << getTimestamp()
                                                        << " [DancePlayer] 执行同步舞蹈: "
                                                        << routine->name << "\n";
                                                    // 暂停播放线程，关闭流释放 ALSA 设备
                                                    pipeline_ctx.clear_playback();
                                                    playback_paused = true;
                                                    player.Close();
                                                    dance_player_execute(
                                                        routine, voice_ctl_get_controller(),
                                                        &dance_audio_cfg);
                                                    // 重新打开并恢复播放流
                                                    player.Start(
                                                        cfg.playback_rate, cfg.playback_channels);
                                                    playback_paused = false;
                                                    // 回正
                                                    voice_ctl_execute(8);
                                                }
                                            } else {
                                                int res = voice_ctl_execute(pending_action);
                                                if (res == ACTION_LIMIT_EXCEEDED) {
                                                    std::string limit_reply = "我已经到极限啦";
                                                    std::cout << getTimestamp()
                                                        << " [限位触发] 语音回复: \""
                                                        << limit_reply << "\"\n";
                                                    auto tts_limit =
                                                        pipeline_ctx.tts->Call(limit_reply);
                                                    if (tts_limit && tts_limit->IsSuccess()) {
                                                        auto audio_limit =
                                                            tts_limit->GetAudioData();
                                                        pipeline_ctx.enqueue_playback(
                                                            pcm16BytesToFloat(audio_limit),
                                                            pipeline_ctx.tts_sample_rate);
                                                    }
                                                }
                                            }
                                        } else if (g_barge_in && pending_action != ACTION_NONE) {
                                            std::cout << getTimestamp()
                                                << " [VoiceCtl] "
                                                << "检测到高频打断指令，直接丢弃积压动作: "
                                                << pending_action << "\n";
                                        }
                                    }
                                });
                            }
                        } else {
                            std::cout << getTimestamp() << " [ASR] 识别完成: (无结果)\n";
                        }
                    }

                    audio_buffer.clear();
                    silence_frames_count = 0;
                }
            } else {
                pre_buffer.push_back(vad_frame);
                if (pre_buffer.size() > PRE_BUFFER_FRAMES) {
                    pre_buffer.pop_front();
                }
            }
        }
    });

    // -------------------------------------------------------------------------
    // 开始对话
    // -------------------------------------------------------------------------
    std::cout << getTimestamp() << " [等待语音输入...]\n" << std::flush;

    if (!capture.Start(cfg.capture_rate, cfg.capture_channels)) {
        std::cerr << getTimestamp() << " 错误: 无法启动录音设备\n";
        player.Stop();
        player.Close();
        return 1;
    }

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    {
        std::lock_guard<std::mutex> lock(g_process_thread_mutex);
        if (g_process_thread && g_process_thread->joinable()) {
            g_process_thread->join();
        }
    }

    capture.Stop();
    capture.Close();

    player.Stop();
    playback_cv.notify_all();
    if (playback_thread.joinable()) {
        playback_thread.join();
    }
    player.Close();

#ifdef USE_MCP
    if (mcp.enabled) {
        if (mcp.registry_poll_thread.joinable()) {
            mcp.registry_poll_thread.join();
        }
        if (mcp.manager) {
            mcp.manager->stopAll();
        }
        std::cout << getTimestamp() << " [MCP] 已清理\n";
    }
#endif

    if (cfg.save_audio && !recorded_audio.empty()) {
        std::cout << getTimestamp() << " [保存音频] " << cfg.audio_file << " ("
            << recorded_audio.size() << " samples, "
            << (recorded_audio.size() / 16000.0f) << " 秒)\n";
        saveWav(cfg.audio_file, recorded_audio, 16000);
    }

    voice_ctl_cleanup();

    std::cout << "\n" << getTimestamp() << " [已退出]\n";
    return 0;
}
