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

    // ========================================================================
    // 头部水平转动（左右看）- 用于"看左边"、"看右边"、"向左看"等
    // ========================================================================
    add_tool("head_turn_left",
            "头部向左转，看向左边。触发词：看左边、向左看、左转头、往左看",
            0, angle_schema);
    add_tool("head_turn_right",
            "头部向右转，看向右边。触发词：看右边、向右看、右转头、往右看",
            1, angle_schema);

    // ========================================================================
    // 头部垂直俯仰（上下看）- 用于"抬头"、"低头"、"看上面"等
    // ========================================================================
    add_tool("head_look_up",
            "抬头，向上看。触发词：抬头、向上看、看上面、仰头",
            2, angle_schema);
    add_tool("head_look_down",
            "低头，向下看。触发词：低头、向下看、看下面、俯头",
            3, angle_schema);

    // ========================================================================
    // 头部侧倾（歪头）- 用于"歪头"、"侧头"等
    // ========================================================================
    add_tool("head_tilt_left",
            "向左歪头，头部向左侧倾斜。触发词：左歪头、向左歪、左侧头",
            4, angle_schema);
    add_tool("head_tilt_right",
            "向右歪头，头部向右侧倾斜。触发词：右歪头、向右歪、右侧头",
            5, angle_schema);

    // ========================================================================
    // 身体转动
    // ========================================================================
    add_tool("body_turn_left",
            "身体向左转。触发词：左转身、身体向左转",
            6, angle_schema);
    add_tool("body_turn_right",
            "身体向右转。触发词：右转身、身体向右转",
            7, angle_schema);

    // ========================================================================
    // 复位
    // ========================================================================
    add_tool("reset_pose",
            "复位到初始姿态。触发词：复位、回正、归位",
            8);

    // ========================================================================
    // 舞蹈动作
    // ========================================================================
    add_tool("dance_headbanger",
            "跳摇滚舞，摇头晃脑。触发词：摇滚舞、headbanger",
            9);
    add_tool("dance_jackson",
            "跳杰克逊舞。触发词：杰克逊舞、jackson",
            10);
    add_tool("dance_chicken",
            "跳小鸡舞。触发词：小鸡舞、chicken",
            11);
    add_tool("dance_uh_huh_tilt",
            "点头附和。触发词：点头、uh huh",
            12);

    // ========================================================================
    // 视觉跟踪
    // ========================================================================
    add_tool("face_follow_start",
            "启动人脸跟随。触发词：跟着我、看着我、人脸跟随",
            13);
    add_tool("gesture_follow_start",
            "启动手势跟随。触发词：跟着手、手势跟随",
            15);
    add_tool("tracker_stop",
            "停止跟随。触发词：别跟了、停止跟随、不要跟了",
            14);
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
            out_result = "OK";
        } else {
            std::cerr << getTimestamp()
                    << " [ActionProvider] 错误: pending_dance 未设置，无法执行舞蹈\n";
            out_result = "执行失败，舞蹈系统未就绪。";
        }
        return true;
    }

    // 非舞蹈动作直接执行
    int res = voice_ctl_execute(action_id);
    if (res == ACTION_LIMIT_EXCEEDED) {
        out_result = "已到达极限位置，无法继续。";
    } else if (res < 0) {
        out_result = "执行失败，电机异常。";
    } else {
        out_result = "OK";
    }

    return true;
}

#endif  // USE_MCP
