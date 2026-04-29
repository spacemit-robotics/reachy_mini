/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MCP_HELPER_HPP
#define MCP_HELPER_HPP

#ifdef USE_MCP

#include <mcp_service.hpp>

#include <string>
#include <vector>

struct MCPConfig {
    std::string backend = "openai";
    std::string url = "";
    std::string model = "qwen2.5:0.5b";
    int timeout = 120;
    std::string system_prompt =
        "你是一个智能助手，可以使用工具帮助用户。请用中文回复。";
    std::string registry_url = "";
    int registry_poll_interval = 5;

    struct ServerEntry {
        std::string name;
        std::string type;  // "stdio", "socket", "http"
        std::string command;
        std::vector<std::string> args;
        int startup_timeout = 30000;
        int request_timeout = 30000;
        std::string socketPath;
        std::string url;
    };
    std::vector<ServerEntry> servers;
};

bool loadMCPConfig(const std::string& path, MCPConfig& config);
std::string convertMCPToolsToString(const std::vector<mcp::Tool>& tools);
std::vector<MCPConfig::ServerEntry> fetchServicesFromRegistry(
    const std::string& registry_url);

#endif  // USE_MCP

#endif  // MCP_HELPER_HPP
