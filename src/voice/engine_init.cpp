/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "engine_init.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "voice_common.hpp"

#ifdef USE_MCP
#include <nlohmann/json.hpp>

#include "mcp_helper.hpp"
using json = nlohmann::json;
#endif

LLMInitResult initLLM(const std::string &llm_model, const std::string &llm_url,
                    const std::string &default_system_prompt, int max_tokens) {
    LLMInitResult result;
    result.system_prompt = default_system_prompt;

    const char *key_env = std::getenv("OPENAI_API_KEY");
    std::string api_key = key_env ? key_env : "";
    result.llm = std::make_shared<spacemit_llm::LLMService>(llm_model, llm_url, api_key,
                                                            result.system_prompt, max_tokens);

    std::cout << getTimestamp() << " [1/5] LLM 后端: " << llm_url << " OK\n";

    return result;
}

std::shared_ptr<SpacemiT::VadEngine> initVAD(float vad_threshold) {
    std::cout << getTimestamp() << " [2/5] 初始化 VAD..." << std::flush;

    auto vad_config = SpacemiT::VadConfig::Preset("silero")
                        .withTriggerThreshold(vad_threshold)
                        .withStopThreshold(vad_threshold - 0.15f);

    auto vad = std::make_shared<SpacemiT::VadEngine>(vad_config);
    if (!vad->IsInitialized()) {
        std::cerr << "\n" << getTimestamp() << " 错误: VAD 初始化失败\n";
        return nullptr;
    }
    std::cout << " OK (" << vad->GetEngineName() << ")\n";
    return vad;
}

std::shared_ptr<SpacemiT::AsrEngine> initASR() {
    std::cout << getTimestamp() << " [3/5] 初始化 ASR..." << std::flush;

    SpacemiT::AsrConfig asr_cfg = SpacemiT::AsrConfig::Preset("sensevoice");
    asr_cfg.provider = "cpu";

    auto asr = std::make_shared<SpacemiT::AsrEngine>(asr_cfg);
    if (!asr->IsInitialized()) {
        std::cerr << "\n" << getTimestamp() << " 错误: ASR 初始化失败\n";
        return nullptr;
    }
    std::cout << " OK (Provider: " << asr_cfg.provider << ")\n";
    return asr;
}

TTSInitResult initTTS(const std::string &tts_type) {
    TTSInitResult result;
    std::cout << getTimestamp() << " [4/5] 初始化 TTS (" << tts_type << ")..." << std::flush;

    auto selection = parseEngine(tts_type);

    SpacemiT::TtsConfig tts_cfg;
    tts_cfg.backend = selection.backend;
    if (!selection.voice.empty()) {
        tts_cfg.voice = selection.voice;
    }

    result.tts = std::make_shared<SpacemiT::TtsEngine>(tts_cfg);
    if (!result.tts->IsInitialized()) {
        std::cerr << "\n" << getTimestamp() << " 错误: TTS 初始化失败\n";
        result.tts = nullptr;
        return result;
    }

    result.sample_rate = result.tts->GetSampleRate();
    std::cout << " OK (" << result.tts->GetEngineName() << ", " << result.sample_rate << " Hz)\n";
    return result;
}

// ============================================================================
// MCP 初始化
// ============================================================================

#ifdef USE_MCP

void initMCP(const std::string &mcp_config_path, std::shared_ptr<spacemit_llm::LLMService> &llm,
            std::string &system_prompt, MCPInitResult &result, const std::string &cli_llm_url,
            const std::string &cli_llm_model) {
    if (mcp_config_path.empty()) {
        result.enabled = false;
        return;
    }

    std::cout << getTimestamp() << " [MCP] 加载配置: " << mcp_config_path << "\n";

    if (!loadMCPConfig(mcp_config_path, result.config)) {
        std::cerr << getTimestamp() << " [MCP] 配置加载失败，禁用 MCP\n";
        result.enabled = false;
        return;
    }

    result.enabled = true;

    // 覆盖 system_prompt
    if (!result.config.system_prompt.empty()) {
        system_prompt = result.config.system_prompt;
        llm->update_prompt(system_prompt);
        std::cout << getTimestamp() << " [MCP] 已更新 system_prompt\n";
    }

    // CLI 参数覆盖配置文件中的 LLM 设置
    if (!cli_llm_url.empty()) {
        result.config.url = cli_llm_url;
    }
    if (!cli_llm_model.empty()) {
        result.config.model = cli_llm_model;
    }

    // 创建 MCPManager 和 MCPActionProvider
    result.manager = std::make_unique<mcp::MCPManager>();
    result.action_provider = std::make_unique<MCPActionProvider>();

    // 注册工具变更回调：当外部 MCP 服务器上线/下线时，合并本地工具并更新 LLM 工具列表
    result.manager->onToolChange([&result](const std::vector<mcp::Tool> &tools) {
        // 合并外部工具 + 本地 ActionProvider 工具
        auto merged = tools;
        if (result.action_provider) {
            auto local_tools = result.action_provider->getTools();
            merged.insert(merged.end(), local_tools.begin(), local_tools.end());
        }

        std::lock_guard<std::mutex> lock(result.tools_mutex);
        const bool had_tools_before =
            !result.llm_tools_json.empty() && result.llm_tools_json != "[]";
        result.llm_tools_json = convertMCPToolsToString(merged);
        std::cout << "\n"
                << getTimestamp() << " [MCP] 工具列表已更新: " << merged.size() << " 个工具\n";

        // 首次获得工具时，注入系统消息提示 LLM 使用工具
        if (!had_tools_before && !merged.empty() && !result.tools_hint_added) {
            std::string tools_list_str;
            for (const auto &tool : merged) {
                tools_list_str += "- " + tool.name + ": " + tool.description + "\n";
            }

            std::lock_guard<std::mutex> conversation_lock(result.conversation_mutex);
            result.conversation_messages.push_back(spacemit_llm::ChatMessage::System(
                "现在已经加载了以下 MCP 工具，可以用于实际控制设备或调用后端服务：\n" +
                tools_list_str +
                "\n从现在开始，凡是与这些设备或服务控制 / 查询相关的请求，必须优先调用对应的 MCP "
                "工具完成操作，"
                "不要只用自然语言假装已经完成。"));
            result.tools_hint_added = true;
        }
    });

    // 添加配置文件中的服务器
    for (const auto &srv : result.config.servers) {
        if (srv.type == "stdio") {
            mcp::StdioConfig sc;
            sc.command = srv.command;
            sc.args = srv.args;
            result.manager->addStdioServer(srv.name, sc);
            result.known_servers.insert(srv.name);
            std::cout << getTimestamp() << " [MCP] 添加服务器: " << srv.name
                    << " (stdio: " << srv.command << ")\n";
        } else if (srv.type == "http") {
            mcp::HttpConfig hc;
            hc.url = srv.url;
            result.manager->addHttpServer(srv.name, hc);
            result.known_servers.insert(srv.name);
            std::cout << getTimestamp() << " [MCP] 添加服务器: " << srv.name
                    << " (http: " << srv.url << ")\n";
        } else if (srv.type == "socket") {
            mcp::UnixSocketConfig uc;
            uc.socketPath = srv.socketPath;
            result.manager->addUnixSocketServer(srv.name, uc);
            result.known_servers.insert(srv.name);
            std::cout << getTimestamp() << " [MCP] 添加服务器: " << srv.name
                    << " (socket: " << srv.socketPath << ")\n";
        }
    }

    // 从注册中心获取服务
    if (!result.config.registry_url.empty()) {
        auto registry_services = fetchServicesFromRegistry(result.config.registry_url);
        for (const auto &srv : registry_services) {
            if (result.known_servers.find(srv.name) == result.known_servers.end()) {
                result.known_servers.insert(srv.name);
                mcp::HttpConfig hc;
                hc.url = srv.url;
                result.manager->addHttpServer(srv.name, hc);
                std::cout << getTimestamp() << " [MCP] 添加服务器: " << srv.name
                        << " (http: " << srv.url << ")\n";
            }
        }
    }

    std::cout << getTimestamp() << " [MCP] 启动服务器...\n";
    result.manager->startAll();

    // 辅助函数：合并外部和本地工具并更新 llm_tools_json
    auto updateToolsJson = [&result]() {
        auto tools = result.manager->getAllTools();
        auto local_tools = result.action_provider->getTools();
        tools.insert(tools.end(), local_tools.begin(), local_tools.end());

        std::lock_guard<std::mutex> lock(result.tools_mutex);
        result.llm_tools_json = convertMCPToolsToString(tools);
        return tools.size();
    };

    // 初始加载工具（至少包含本地工具）
    size_t total_tools = updateToolsJson();

    std::chrono::milliseconds server_wait_timeout(10000);
    for (const auto &srv : result.config.servers) {
        if (srv.type == "stdio") {
            server_wait_timeout =
                std::max(server_wait_timeout, std::chrono::milliseconds(srv.startup_timeout));
        }
    }

    if (result.manager->waitForAnyServer(server_wait_timeout)) {
        total_tools = updateToolsJson();
        std::cout << getTimestamp() << " [MCP] 已连接 " << result.manager->readyServerCount()
                << " 个服务器, 共 " << total_tools << " 个工具 ("
                << result.action_provider->getTools().size() << " 本地)\n";
    } else {
        std::cout << getTimestamp() << " [MCP] 警告: 无可用服务器，但已启用 "
                << result.action_provider->getTools().size() << " 个本地工具\n";
    }

    {
        std::lock_guard<std::mutex> conversation_lock(result.conversation_mutex);
        result.conversation_messages.push_back(
            spacemit_llm::ChatMessage::System(result.config.system_prompt));

        // 如果已有工具（含本地工具）且尚未注入 hint，立即注入工具提示
        // 这对纯本地工具场景（无外部 MCP 服务器）至关重要，因为 onToolChange 不会被触发
        if (total_tools > 0 && !result.tools_hint_added) {
            auto all_tools = result.manager->getAllTools();
            auto local_tools = result.action_provider->getTools();
            all_tools.insert(all_tools.end(), local_tools.begin(), local_tools.end());

            std::string tools_list_str;
            for (const auto &tool : all_tools) {
                tools_list_str += "- " + tool.name + ": " + tool.description + "\n";
            }

            result.conversation_messages.push_back(spacemit_llm::ChatMessage::System(
                "现在已经加载了以下工具，可以用于实际控制机器人：\n" +
                tools_list_str +
                "\n当用户要求做动作（如转头、点头、跳舞等）时，必须调用对应的工具完成操作，"
                "不要只用自然语言假装已经完成。"));
            result.tools_hint_added = true;
            std::cout << getTimestamp() << " [MCP] 已注入工具提示 (" << all_tools.size()
                    << " 个工具)\n";
        }
    }

    // 注册中心轮询线程
    if (!result.config.registry_url.empty()) {
        std::cout << getTimestamp() << " [MCP] 启动注册中心轮询: " << result.config.registry_url
                << "\n";
        result.registry_poll_thread = std::thread([&result, &updateToolsJson]() {
            while (g_running) {
                auto services = fetchServicesFromRegistry(result.config.registry_url);

                std::set<std::string> registry_services;
                std::map<std::string, std::string> service_urls;
                for (const auto &srv : services) {
                    registry_services.insert(srv.name);
                    service_urls[srv.name] = srv.url;
                }

                std::vector<std::string> to_remove;
                for (const auto &name : result.known_servers) {
                    auto status = result.manager->getStatus(name);
                    if (registry_services.find(name) == registry_services.end()) {
                        if (status.state == mcp::ServerState::Error ||
                            status.state == mcp::ServerState::Disconnected) {
                            to_remove.push_back(name);
                            std::cout << "\n"
                                    << getTimestamp() << " [MCP] 服务已下线: " << name << "\n";
                        }
                    } else if (status.state == mcp::ServerState::Error ||
                                status.state == mcp::ServerState::Disconnected) {
                        std::cout << "\n" << getTimestamp() << " [MCP] 尝试重连: " << name << "\n";
                        result.manager->startServer(name);
                    }
                }

                for (const auto &name : to_remove) {
                    result.manager->removeServer(name);
                    result.known_servers.erase(name);
                }

                bool new_services_added = false;
                for (const auto &srv : services) {
                    if (result.known_servers.find(srv.name) == result.known_servers.end()) {
                        mcp::HttpConfig hc;
                        hc.url = srv.url;
                        result.manager->addHttpServer(srv.name, hc);
                        result.manager->startServer(srv.name);
                        result.known_servers.insert(srv.name);
                        new_services_added = true;
                        std::cout << "\n"
                                << getTimestamp() << " [MCP] 发现新服务: " << srv.name << " ("
                                << srv.url << ")\n";
                    }
                }

                if (!to_remove.empty() || new_services_added) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    size_t count = updateToolsJson();
                    std::cout << getTimestamp() << " [MCP] 工具列表已更新: " << count
                            << " 个工具\n";
                }

                std::this_thread::sleep_for(
                    std::chrono::seconds(result.config.registry_poll_interval));
            }
        });
    }

    std::cout << getTimestamp() << " [MCP] 初始化完成\n\n";
}

#endif  // USE_MCP
