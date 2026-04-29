// Copyright (c) 2025 Pollen Robotics / SpacemiT
// SPDX-License-Identifier: Apache-2.0
//
// MCP configuration loading and tool description utilities.

#include "mcp_helper.hpp"

#ifdef USE_MCP
#include <curl/curl.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "voice_common.hpp"

using json = nlohmann::json;

// curl write callback
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    static_cast<std::string *>(userp)->append(static_cast<char *>(contents), size * nmemb);
    return size * nmemb;
}

bool loadMCPConfig(const std::string &config_path, MCPConfig &config) {
    std::ifstream file(config_path);
    if (!file.is_open()) {
        std::cerr << getTimestamp() << " [MCP] 无法打开配置文件: " << config_path << "\n";
        return false;
    }

    bool success = false;
    try {
        json j = json::parse(file);

        if (j.contains("system_prompt")) {
            config.system_prompt = j["system_prompt"].get<std::string>();
        }

        success = true;

        if (j.contains("servers")) {
            for (auto &[name, server_json] : j["servers"].items()) {
                MCPConfig::ServerEntry entry;
                entry.name = name;
                entry.type = server_json.value("type", "stdio");
                entry.command = server_json.value("command", "");
                entry.url = server_json.value("url", "");

                if (server_json.contains("args")) {
                    for (auto &arg : server_json["args"]) {
                        entry.args.push_back(arg.get<std::string>());
                    }
                }
                // Note: ServerEntry struct does not currently have an 'env' member
                config.servers.push_back(entry);
            }
        }

        if (j.contains("registry_url")) {
            config.registry_url = j["registry_url"].get<std::string>();
        }
    } catch (const std::exception &e) {
        std::cerr << getTimestamp() << " [MCP] 配置解析错误: " << e.what() << "\n";
    }

    return success;
}

std::string convertMCPToolsToString(const std::vector<mcp::Tool> &tools) {
    json tools_json = json::array();
    for (const auto &tool : tools) {
        json tool_obj;
        tool_obj["type"] = "function";
        tool_obj["function"]["name"] = tool.name;
        tool_obj["function"]["description"] = tool.description;
        if (!tool.inputSchema.empty()) {
            try {
                tool_obj["function"]["parameters"] = tool.inputSchema;
            } catch (...) {
                tool_obj["function"]["parameters"] = json::object();
            }
        } else {
            tool_obj["function"]["parameters"] = json::object();
        }
        tools_json.push_back(tool_obj);
    }
    return tools_json.dump();
}

std::vector<MCPConfig::ServerEntry> fetchServicesFromRegistry(const std::string &registry_url) {
    std::vector<MCPConfig::ServerEntry> entries;

    CURL *curl = curl_easy_init();
    if (!curl)
        return entries;

    std::string response;
    std::string url = registry_url + "/services";

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);

    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        std::cerr << getTimestamp() << " [MCP] Registry 请求失败: " << curl_easy_strerror(res)
                << "\n";
        return entries;
    }

    try {
        json services = json::parse(response);
        for (auto &[name, info] : services.items()) {
            MCPConfig::ServerEntry entry;
            entry.name = name;
            entry.type = info.value("type", "http");
            entry.url = info.value("url", "");
            entry.command = info.value("command", "");
            if (info.contains("args")) {
                for (auto &arg : info["args"]) {
                    entry.args.push_back(arg.get<std::string>());
                }
            }
            entries.push_back(entry);
        }
    } catch (const std::exception &e) {
        std::cerr << getTimestamp() << " [MCP] Registry 解析错误: " << e.what() << "\n";
    }

    return entries;
}

#endif  // USE_MCP
