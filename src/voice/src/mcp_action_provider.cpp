/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcp_action_provider.hpp"

#ifdef USE_MCP

#include <iostream>
#include <string>
#include <vector>

#include "voice_common.hpp"
#include "voice_ctl.h"
#include "voice_pipeline.hpp"

MCPActionProvider::MCPActionProvider() {
    initTools();
}

void MCPActionProvider::initTools() {
    auto add_tool = [&](const std::string &name, const std::string &desc, int action_id,
                        const mcp::json &schema = mcp::json::object()) {
        mcp::Tool tool;
        tool.name = name;
        tool.description = desc;
        if (!schema.empty()) {
            tool.inputSchema = schema;
        } else {
            tool.inputSchema = {{"type", "object"}, {"properties", mcp::json::object()}};
        }
        tools_.push_back(tool);
        tool_to_action_id_[name] = action_id;
    };

    mcp::json angle_schema = {
        {"type", "object"},
        {"properties",
        {{"angle", {{"type", "number"}, {"description", "角度 (度)"}, {"default", 15}}}}}};

    // 注册所有动作工具
    add_tool("head_turn_left", "控制机器人头部向左转动", 0, angle_schema);
    add_tool("head_turn_right", "控制机器人头部向右转动", 1, angle_schema);
    add_tool("head_look_up", "控制机器人头部向上仰", 2, angle_schema);
    add_tool("head_look_down", "控制机器人头部向下俯", 3, angle_schema);
    add_tool("head_tilt_left", "控制机器人头部向左侧倾斜 (Roll)", 4, angle_schema);
    add_tool("head_tilt_right", "控制机器人头部向右侧倾斜 (Roll)", 5, angle_schema);
    add_tool("body_turn_left", "控制机器人身体向左转动", 6, angle_schema);
    add_tool("body_turn_right", "控制机器人身体向右转动", 7, angle_schema);
    add_tool("reset_pose", "将机器人复位到初始中性姿态", 8);

    // 舞蹈动作
    add_tool("dance_headbanger", "执行摇滚风格的点头动作", 9);
    add_tool("dance_jackson", "执行迈克尔·杰克逊风格的舞蹈动作", 10);
    add_tool("dance_chicken", "执行小鸡啄米风格的舞蹈动作", 11);
    add_tool("dance_uh_huh_tilt", "执行点赞/附和风格的倾斜动作", 12);
}

const std::vector<mcp::Tool> &MCPActionProvider::getTools() const {
    return tools_;
}

bool MCPActionProvider::executeTool(const std::string &name, const mcp::json & /*args*/,
                                    std::string &out_result) {
    auto it = tool_to_action_id_.find(name);
    if (it == tool_to_action_id_.end()) {
        return false;
    }

    int action_id = it->second;
    std::cout << getTimestamp() << " [ActionProvider] 执行工具: " << name << " (ID: " << action_id
            << ")\n";

    // 舞蹈动作 (ID 9-12) 需要通过关键词路径执行，以正确管理音频设备
    if (action_id >= 9 && action_id <= 12) {
        if (pending_dance_) {
            std::lock_guard<std::mutex> lock(pending_dance_->mutex);
            pending_dance_->action_id.store(action_id);
            std::cout << getTimestamp() << " [ActionProvider] 舞蹈动作已标记为待处理 (ID: "
                    << action_id << ")，将由主线程执行\n";
            out_result = "舞蹈动作已准备就绪，正在执行。";
        } else {
            std::cerr << getTimestamp()
                    << " [ActionProvider] 错误: pending_dance 未设置，无法执行舞蹈\n";
            out_result = "舞蹈系统未就绪，无法执行。";
        }
        return true;
    }

    // 非舞蹈动作直接执行
    int res = voice_ctl_execute(action_id);
    if (res == ACTION_LIMIT_EXCEEDED) {
        out_result = "动作已到达极限位置，无法继续执行。请直接用语言回复用户，不要再调用工具。";
    } else if (res < 0) {
        out_result = "电机控制器异常，执行失败。请直接用语言回复用户。";
    } else {
        out_result = "动作已成功执行完毕。请直接用简短的语言回复用户，不要重复调用同一个工具。";
    }

    return true;
}

#endif  // USE_MCP
