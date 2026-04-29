/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MCP_ACTION_PROVIDER_HPP
#define MCP_ACTION_PROVIDER_HPP

#ifdef USE_MCP

#include <atomic>
#include <map>
#include <string>
#include <vector>
#include <mcp_service.hpp>

// Forward declaration
struct PendingDance;

/**
 * @brief MCPActionProvider 负责管理机器人本地动作工具的定义与执行。
 *
 * 它可以绕过网络通信，直接通过 C++ 调用底层电机动作。
 */
class MCPActionProvider {
public:
    MCPActionProvider();
    ~MCPActionProvider() = default;

    /**
     * @brief 设置待处理舞蹈标志的指针。
     */
    void setPendingDance(PendingDance *pending_dance) { pending_dance_ = pending_dance; }

    /**
     * @brief 获取所有本地动作的工具定义。
     */
    const std::vector<mcp::Tool> &getTools() const;

    /**
     * @brief 尝试执行本地动作工具。
     *
     * @param name 工具名称
     * @param args 工具参数 (JSON)
     * @param out_result [输出] 执行结果描述
     * @return true 如果工具被识别并执行（无论成功失败），false 表示不属于本地工具
     */
    bool executeTool(const std::string &name, const mcp::json &args, std::string &out_result);

private:
    void initTools();

    std::vector<mcp::Tool> tools_;
    std::map<std::string, int> tool_to_action_id_;
    PendingDance *pending_dance_ = nullptr;
};

#endif  // USE_MCP

#endif  // MCP_ACTION_PROVIDER_HPP
