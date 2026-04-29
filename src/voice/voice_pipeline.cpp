/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "voice_pipeline.hpp"

#include <deque>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "text_buffer.hpp"
#include "voice_common.hpp"
#include "voice_ctl.h"

#ifdef USE_MCP
#include <nlohmann/json.hpp>

#include "mcp_action_provider.hpp"
#include "mcp_helper.hpp"
using json = nlohmann::json;

/**
 * 尝试通过 ActionProvider 将 MCP tool call 映射为本地电机动作。
 */
static bool tryLocalToolCall(VoicePipelineContext &ctx, const std::string &tool_name,
                                const json &tool_args, std::string &result_text) {
    if (ctx.action_provider) {
        return ctx.action_provider->executeTool(tool_name, tool_args, result_text);
    }
    return false;
}

#endif  // USE_MCP

void processText(VoicePipelineContext &ctx, const std::string &text) {
    if (text.empty())
        return;

    g_processing = true;
    g_barge_in = false;

    std::cout << "\n" << getTimestamp() << " [你]: " << text << "\n";

    TextBuffer text_buffer;
    std::string full_response;
    int sentence_count = 0;
    int total_duration_ms = 0;
    int total_processing_ms = 0;

    auto synthesizeSentence = [&](const std::string &sentence) {
        if (sentence.empty() || g_barge_in || !g_running)
            return;

        sentence_count++;
        auto result = ctx.tts->Call(sentence);
        if (result && result->IsSuccess() && !g_barge_in) {
            total_duration_ms += result->GetDurationMs();
            total_processing_ms += result->GetProcessingTimeMs();
            std::cout << getTimestamp() << " [TTS] 句" << sentence_count << ": \"" << sentence
                        << "\" " << result->GetDurationMs() << "ms, "
                        << "RTF=" << std::fixed << std::setprecision(3) << result->GetRTF() << "\n";
            auto audio_bytes = result->GetAudioData();
            if (!audio_bytes.empty()) {
                auto float_samples = pcm16BytesToFloat(audio_bytes);
                ctx.enqueue_playback(float_samples, ctx.tts_sample_rate);
            }
        }
    };

#ifdef USE_MCP
    if (ctx.mcp_enabled) {
        {
            std::lock_guard<std::mutex> lock(*ctx.conversation_mutex);
            ctx.conversation_messages->push_back(spacemit_llm::ChatMessage::User(text));
        }

        const int MAX_TOOL_ROUNDS = 5;
        int round = 0;
        std::string last_tool_name;
        int consecutive_same_tool = 0;

        while (round++ < MAX_TOOL_ROUNDS && g_running && !g_barge_in) {
            std::cout << getTimestamp() << " [LLM] 第 " << round << " 轮...\n";
            std::cout << getTimestamp() << " [AI]: " << std::flush;

            std::string current_tools;
            std::vector<spacemit_llm::ChatMessage> current_messages;
            {
                std::lock_guard<std::mutex> lock(*ctx.tools_mutex);
                current_tools = *ctx.llm_tools_json;
            }
            {
                std::lock_guard<std::mutex> lock(*ctx.conversation_mutex);
                current_messages = *ctx.conversation_messages;
            }

            auto result = ctx.llm->chat_stream(
                current_messages,
                [&](const std::string &chunk, bool is_done, const std::string &error) -> bool {
                    if (g_barge_in || !g_running)
                        return false;
                    if (!error.empty()) {
                        std::cerr << "\n" << getTimestamp() << " [LLM错误] " << error << std::endl;
                        return false;
                    }
                    if (is_done)
                        return true;

                    if (!chunk.empty()) {
                        std::cout << chunk << std::flush;
                        full_response += chunk;
                        text_buffer.addText(chunk);

                        while (text_buffer.hasSentence() && !g_barge_in) {
                            std::string sentence = text_buffer.getNextSentence();
                            synthesizeSentence(sentence);
                        }
                    }
                    return true;
                },
                current_tools);

            std::cout << std::endl;

            if (result.HasToolCalls()) {
                std::cout << getTimestamp() << " [Tool Call] 检测到工具调用\n";

                {
                    std::lock_guard<std::mutex> lock(*ctx.conversation_mutex);
                    ctx.conversation_messages->push_back(spacemit_llm::ChatMessage::Assistant(
                        result.content, result.tool_calls_json));
                }

                try {
                    auto tool_calls = json::parse(result.tool_calls_json);
                    for (const auto &tc : tool_calls) {
                        std::string tool_name = tc["function"]["name"];
                        json tool_args = tc["function"]["arguments"];

                        if (tool_args.is_string()) {
                            try {
                                tool_args = json::parse(tool_args.get<std::string>());
                            } catch (...) {
                            }
                        }

                        std::string result_text;

                        // 检测连续重复调用同一工具
                        if (tool_name == last_tool_name) {
                            consecutive_same_tool++;
                        } else {
                            consecutive_same_tool = 0;
                            last_tool_name = tool_name;
                        }

                        if (consecutive_same_tool >= 1) {
                            std::cout << getTimestamp() << " [MCP] 检测到重复调用 " << tool_name
                                << "，强制终止工具循环\n";
                            result_text = "工具已执行过，不要重复调用。请直接用语言回复用户。";
                            std::string tc_id = tc.value("id", "");
                            {
                                std::lock_guard<std::mutex> lock(*ctx.conversation_mutex);
                                ctx.conversation_messages->push_back(
                                    spacemit_llm::ChatMessage::Tool(result_text, tc_id));
                            }
                            // 强制跳出工具循环，进入文本生成
                            round = MAX_TOOL_ROUNDS;
                            continue;
                        }

                        // 优先尝试本地工具拦截（由 ActionProvider 处理）
                        if (tryLocalToolCall(ctx, tool_name, tool_args, result_text)) {
                            std::cout << getTimestamp() << " [LocalTool] 本地执行: " << tool_name
                                << " -> " << result_text << "\n";
                        } else {
                            // 走网络 MCP 调用
                            std::string server = ctx.mcp_manager->findServerForTool(tool_name);
                            std::cout << getTimestamp() << " [MCP] 调用: " << tool_name << " @ "
                                << server << " 参数: " << tool_args.dump() << std::endl;

                            auto tool_result = ctx.mcp_manager->callTool(tool_name, tool_args);

                            if (tool_result.success && !tool_result.contents.empty()) {
                                result_text = tool_result.contents[0];
                            } else if (!tool_result.error.empty()) {
                                result_text = "错误: " + tool_result.error;
                            } else {
                                result_text = tool_result.rawResult.dump();
                            }

                            std::cout << getTimestamp() << " [MCP] 结果: " << result_text
                                << std::endl;
                        }

                        std::string tc_id = tc.value("id", "");
                        {
                            std::lock_guard<std::mutex> lock(*ctx.conversation_mutex);
                            ctx.conversation_messages->push_back(
                                spacemit_llm::ChatMessage::Tool(result_text, tc_id));
                        }
                    }
                } catch (const std::exception &e) {
                    std::cerr << getTimestamp() << " [MCP] 工具调用解析错误: " << e.what()
                                << std::endl;
                }

                full_response.clear();
                text_buffer.clear();
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(*ctx.conversation_mutex);
                ctx.conversation_messages->push_back(
                    spacemit_llm::ChatMessage::Assistant(result.content));
            }

            if (!g_barge_in) {
                std::string remaining = text_buffer.getNextSentence();
                if (!remaining.empty()) {
                    synthesizeSentence(remaining);
                }
                text_buffer.stop();
                while (text_buffer.hasSentence() && !g_barge_in) {
                    synthesizeSentence(text_buffer.getNextSentence());
                }
            }
            break;
        }

        // 对话历史滑动窗口：保留最近 8 条消息（不含 system message）
        {
            std::lock_guard<std::mutex> lock(*ctx.conversation_mutex);
            const size_t MAX_HISTORY_MESSAGES = 8;

            // 找到第一条非 system 消息的位置
            size_t system_count = 0;
            for (const auto &msg : *ctx.conversation_messages) {
                if (msg.role == spacemit_llm::ChatMessage::Role::SYSTEM) {
                    system_count++;
                } else {
                    break;
                }
            }

            // 如果非 system 消息超过限制，从头部删除旧消息
            size_t non_system_count = ctx.conversation_messages->size() - system_count;
            if (non_system_count > MAX_HISTORY_MESSAGES) {
                size_t to_remove = non_system_count - MAX_HISTORY_MESSAGES;
                ctx.conversation_messages->erase(ctx.conversation_messages->begin() + system_count,
                                                ctx.conversation_messages->begin() + system_count +
                                                    to_remove);
                std::cout << getTimestamp() << " [对话历史] 清理了 " << to_remove
                            << " 条旧消息，保留最近 " << MAX_HISTORY_MESSAGES << " 条\n";
            }
        }
    } else  // NOLINT(readability/braces)
#endif  // USE_MCP
    {
        // 非 MCP 模式：直接 LLM 流式对话
            std::cout << getTimestamp() << " [AI]: " << std::flush;

            ctx.llm->chat_stream(
                {spacemit_llm::ChatMessage::User(text)},
                [&](const std::string &chunk, bool is_done, const std::string &error) -> bool {
                    if (g_barge_in || !g_running)
                        return false;
                    if (!error.empty()) {
                        std::cerr << "\n" << getTimestamp() << " [LLM错误] " << error << std::endl;
                        return false;
                    }
                    if (is_done)
                        return true;

                    if (!chunk.empty()) {
                        std::cout << chunk << std::flush;
                        full_response += chunk;
                        text_buffer.addText(chunk);

                        while (text_buffer.hasSentence() && !g_barge_in) {
                            std::string sentence = text_buffer.getNextSentence();
                            synthesizeSentence(sentence);
                        }
                    }
                    return true;
                });

            std::cout << std::endl;

            if (!g_barge_in) {
                std::string remaining = text_buffer.getNextSentence();
                if (!remaining.empty()) {
                    synthesizeSentence(remaining);
                }
                text_buffer.stop();
                while (text_buffer.hasSentence() && !g_barge_in) {
                    synthesizeSentence(text_buffer.getNextSentence());
                }
            }
        }

        if (sentence_count > 0) {
            float avg_rtf = total_duration_ms > 0
                                ? static_cast<float>(total_processing_ms) / total_duration_ms
                                : 0.0f;
            std::cout << getTimestamp() << " [TTS] 流式合成完成 (" << sentence_count << " 句, "
                        << "音频=" << std::fixed << std::setprecision(1)
                        << (total_duration_ms / 1000.0f) << "s, "
                        << "耗时=" << (total_processing_ms / 1000.0f) << "s, "
                        << "RTF=" << std::setprecision(3) << avg_rtf << ")\n";
        }

        // Wait for playback to finish
        while (ctx.is_playing() && g_running && !g_barge_in) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Clean up buffers
        if (!g_barge_in) {
            {
                std::lock_guard<std::mutex> lock(*ctx.buffer_mutex);
                ctx.audio_buffer->clear();
                ctx.pre_buffer->clear();
                *ctx.silence_frames = 0;
                *ctx.is_speaking = false;
            }
            *ctx.barge_in_recording = false;
            ctx.vad_frame_buffer->clear();
            ctx.vad->Reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << getTimestamp() << " [TTS] 播放完成，缓冲区已清理\n";
        } else {
            std::cout << getTimestamp() << " [TTS] Barge-in 打断，保留音频缓冲区\n";
            g_barge_in = false;
        }

        g_processing = false;
        std::cout << getTimestamp() << " [等待语音输入...]\n" << std::flush;
    }
