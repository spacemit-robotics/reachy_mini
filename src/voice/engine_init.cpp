/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "engine_init.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17)
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

void warmupLLM(std::shared_ptr<spacemit_llm::LLMService> llm,
                const std::string &tools_json,
                const std::vector<spacemit_llm::ChatMessage> *conversation_messages) {
    if (!llm) return;
    std::cout << getTimestamp() << " [LLM] 预热中..." << std::flush;
    // 使用完整的 conversation context（包含 system prompt）进行预热
    // 这样可以预热整个 prompt 处理流程，消除首轮对话的冷启动延迟
    std::vector<spacemit_llm::ChatMessage> warmup_msgs;
    if (conversation_messages && !conversation_messages->empty()) {
        // 复制 system message(s) 以保持相同的 context
        warmup_msgs = *conversation_messages;
    }
    // 添加预热用的 user message
    warmup_msgs.push_back(spacemit_llm::ChatMessage::User("你好，请用一句话介绍你自己"));

    std::string dummy;
    int token_count = 0;
    auto start = std::chrono::steady_clock::now();
    llm->chat_stream(warmup_msgs, [&dummy, &token_count](const std::string &chunk, bool, const std::string &) {
        dummy += chunk;
        token_count++;
        return true;
    }, tools_json);  // 传入 tools schema
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << " OK (" << token_count << " tokens, " << elapsed << "ms)\n";
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

void warmupTTS(std::shared_ptr<SpacemiT::TtsEngine> tts) {
    if (!tts) return;
    std::cout << getTimestamp() << " [TTS] 预热中..." << std::flush;
    // 合成一个短句触发模型加载
    auto result = tts->Call("你好");
    if (result && result->IsSuccess()) {
        std::cout << " OK\n";
    } else {
        std::cout << " (跳过)\n";
    }
}

// ============================================================================
// MCP 初始化
// ============================================================================

#ifdef USE_MCP

void initMCP(const std::string &mcp_config_path_in, std::shared_ptr<spacemit_llm::LLMService> &llm,
            std::string &system_prompt, MCPInitResult &result, const std::string &cli_llm_url,
            const std::string &cli_llm_model) {
    std::string mcp_config_path = mcp_config_path_in;
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

    // CLI 参数覆盖配置文件中的 LLM 设置
    if (!cli_llm_url.empty()) {
        result.config.url = cli_llm_url;
    }
    if (!cli_llm_model.empty()) {
        result.config.model = cli_llm_model;
    }

    // 创建 MCPManager 和 MCPActionProvider，并检测小模型
    // 小模型定义: 参数量 ≤3B 的模型，使用扁平 Schema 和精简工具集
    bool is_small_model = false;
    std::string model_name = result.config.model;
    std::transform(model_name.begin(), model_name.end(), model_name.begin(), ::tolower);

    // 精确匹配小模型标识，避免误匹配 13b/33b/q3_k_m 等
    // 支持格式: "0.5b", "1.5b", "3b", ":0.5b", ":1.5b", ":3b", "-0.5b", "-1.5b", "-3b"
    // 注意: pos == 0 (模型名以 tag 开头) 也被视为合法匹配，例如 "3b-instruct" 是小模型
    auto is_small_model_tag = [](const std::string &name, const std::string &tag) -> bool {
        size_t pos = name.find(tag);
        if (pos == std::string::npos) return false;
        // 检查 tag 前一个字符：必须是分隔符或字符串开头
        // pos == 0 时表示模型名以 tag 开头，这是合法的小模型命名（如 "3b-instruct"）
        if (pos > 0) {
            char prev = name[pos - 1];
            // "0.5b"/"1.5b" 的 tag 本身包含数字前缀，无需额外允许数字
            // "3b" 前不能是数字，否则 "13b" 会误判
            if (prev != ':' && prev != '-' && prev != '_' && prev != '.') {
                return false;
            }
        }
        // 检查 tag 后一个字符：必须是分隔符或字符串结尾
        size_t end_pos = pos + tag.length();
        if (end_pos < name.length()) {
            char next = name[end_pos];
            if (next != '-' && next != '_' && next != ':' && next != '.') {
                return false;
            }
        }
        return true;
    };

    if (is_small_model_tag(model_name, "0.5b") ||
        is_small_model_tag(model_name, "1.5b") ||
        is_small_model_tag(model_name, "3b")) {
        is_small_model = true;
        std::cout << getTimestamp() << " [MCP] 检测到小模型: " << model_name << "，启用扁平 Schema" << std::endl;

        // 尝试加载小模型专属配置（仅覆盖 system_prompt 和 tools_whitelist）
        // 基于目录推导配置路径，而非字符串匹配
        std::filesystem::path config_path(mcp_config_path);
        std::filesystem::path small_config_path = config_path.parent_path() / "mcp_robot_small.json";

        // 避免重复加载（如果用户已指定小模型配置）
        if (config_path.filename() != "mcp_robot_small.json") {
            std::cout << getTimestamp() << " [MCP] 尝试加载小模型专属配置: " << small_config_path.string() << std::endl;
            MCPConfig small_config;
            if (loadMCPConfig(small_config_path.string(), small_config)) {
                // 仅覆盖 prompt 和白名单，保留原 config 的 servers/registry
                if (!small_config.system_prompt.empty()) {
                    result.config.system_prompt = small_config.system_prompt;
                }
                if (!small_config.tools_whitelist.empty()) {
                    result.config.tools_whitelist = small_config.tools_whitelist;
                }
                std::cout << getTimestamp() << " [MCP] 成功加载小模型专属配置" << std::endl;
            } else {
                std::cout << getTimestamp() << " [MCP] 未找到小模型专属配置 (" << small_config_path.string()
                    << ")，继续使用默认配置" << std::endl;
            }
        }
    }

    // 覆盖 system_prompt
    if (!result.config.system_prompt.empty()) {
        system_prompt = result.config.system_prompt;
        llm->update_prompt(system_prompt);
        std::cout << getTimestamp() << " [MCP] 已更新 system_prompt\n";
    }

    result.manager = std::make_unique<mcp::MCPManager>();
    result.action_provider = std::make_unique<MCPActionProvider>(
        is_small_model, result.config.tools_whitelist);

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

        // 首次获得工具时，记录标记
        if (!had_tools_before && !merged.empty() && !result.tools_hint_added) {
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

    // 仅在配置了外部服务器时才阻塞等待连接，避免无服务器场景下空等 10 秒
    if (!result.config.servers.empty()) {
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
            std::cout << getTimestamp() << " [MCP] 警告: 外部服务器连接超时，仅使用 "
                    << result.action_provider->getTools().size() << " 个本地工具\n";
        }
    } else {
        std::cout << getTimestamp() << " [MCP] 无外部服务器，直接使用 "
                << total_tools << " 个本地工具\n";
    }

    {
        std::lock_guard<std::mutex> conversation_lock(result.conversation_mutex);
        result.conversation_messages.push_back(
            spacemit_llm::ChatMessage::System(result.config.system_prompt));

        // 如果已有工具（含本地工具）且尚未注入 hint，进行标记
        if (total_tools > 0 && !result.tools_hint_added) {
            result.tools_hint_added = true;
            std::cout << getTimestamp() << " [MCP] 使用 tools schema 提供工具信息，已跳过额外 prompt 拼接\n";
        }
    }

    // 注册中心轮询线程
    if (!result.config.registry_url.empty()) {
        std::cout << getTimestamp() << " [MCP] 启动注册中心轮询: " << result.config.registry_url
                << "\n";
        // 注意: updateToolsJson 是 initMCP 的局部 lambda，必须按值捕获 (拷贝)，
        // 否则 initMCP 返回后引用悬空，导致 heap corruption → std::bad_alloc
        result.registry_poll_thread = std::thread([&result, updateToolsJson]() {
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
