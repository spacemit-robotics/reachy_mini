/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ENGINE_INIT_HPP
#define ENGINE_INIT_HPP

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "asr_service.h"
#include "llm_service.h"
#include "tts_service.h"
#include "vad_service.h"

#ifdef USE_MCP
#include <mcp_service.hpp>

#include "mcp_action_provider.hpp"
#include "mcp_helper.hpp"
#endif

struct LLMInitResult {
    std::shared_ptr<spacemit_llm::LLMService> llm;
    std::string system_prompt;
};

LLMInitResult initLLM(const std::string &llm_model, const std::string &llm_url,
                    const std::string &default_system_prompt, int max_tokens);

std::shared_ptr<SpacemiT::VadEngine> initVAD(float vad_threshold);

std::shared_ptr<SpacemiT::AsrEngine> initASR();

struct TTSInitResult {
    std::shared_ptr<SpacemiT::TtsEngine> tts;
    int sample_rate;
};

TTSInitResult initTTS(const std::string &tts_type);

#ifdef USE_MCP
struct MCPInitResult {
    bool enabled = false;
    std::unique_ptr<mcp::MCPManager> manager;
    std::unique_ptr<MCPActionProvider> action_provider;  // 新增
    std::string llm_tools_json;
    std::vector<spacemit_llm::ChatMessage> conversation_messages;
    std::thread registry_poll_thread;
    std::mutex tools_mutex;
    std::mutex conversation_mutex;
    bool tools_hint_added = false;
    std::set<std::string> known_servers;
    MCPConfig config;
};

void initMCP(const std::string &mcp_config_path, std::shared_ptr<spacemit_llm::LLMService> &llm,
            std::string &system_prompt, MCPInitResult &result, const std::string &cli_llm_url = "",
            const std::string &cli_llm_model = "");
#endif

#endif  // ENGINE_INIT_HPP
