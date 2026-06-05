/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcp_action_provider.hpp"

#ifdef USE_MCP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "voice_common.hpp"
#include "voice_ctl.h"
#include "voice_pipeline.hpp"

#include "action_executor.h"

MCPActionProvider::MCPActionProvider(
    bool is_small_model,
    const std::vector<std::string> &tools_whitelist)
    : is_small_model_(is_small_model), tools_whitelist_(tools_whitelist) {
    initTools();
    // 应用白名单过滤：仅保留白名单中的工具
    if (!tools_whitelist_.empty()) {
        std::vector<mcp::Tool> filtered;
        for (const auto &tool : tools_) {
            for (const auto &name : tools_whitelist_) {
                if (tool.name == name) {
                    filtered.push_back(tool);
                    break;
                }
            }
        }
        std::cout << getTimestamp() << " [MCP] 工具白名单过滤: " << tools_.size()
            << " → " << filtered.size() << " 个工具\n";
        tools_ = std::move(filtered);
    }
}

void MCPActionProvider::initTools() {
    initLegacyTools();
    initAtomicTools();
    if (!is_small_model_) {
        initChoreographyTool();
    }
    initFlatChoreographyTool();
}

/* ====================================================================
 * 向后兼容的旧版工具 (基于 action_id)
 * ==================================================================== */

void MCPActionProvider::initLegacyTools() {
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

    add_tool("head_turn_left", "头部向左转。触发词：看左边、向左看", 0, angle_schema);
    add_tool("head_turn_right", "头部向右转。触发词：看右边、向右看", 1, angle_schema);
    add_tool("head_look_up", "抬头，向上看。触发词：抬头、向上看", 2, angle_schema);
    add_tool("head_look_down", "低头，向下看。触发词：低头、向下看", 3, angle_schema);
    add_tool("head_tilt_left", "向左歪头。触发词：左歪头", 4, angle_schema);
    add_tool("head_tilt_right", "向右歪头。触发词：右歪头", 5, angle_schema);
    add_tool("body_turn_left", "身体向左转。触发词：身体左转", 6, angle_schema);
    add_tool("body_turn_right", "身体向右转。触发词：身体右转", 7, angle_schema);
    add_tool("reset_pose", "复位到初始姿态。触发词：复位、回正", 8);
    add_tool("dance_headbanger", "跳摇滚舞", 9);
    add_tool("dance_jackson", "跳杰克逊舞", 10);
    add_tool("dance_chicken", "跳小鸡舞", 11);
    add_tool("dance_uh_huh_tilt", "点头附和", 12);
    add_tool("face_follow_start", "启动人脸跟随", 13);
    add_tool("gesture_follow_start", "启动手势跟随", 15);
    add_tool("tracker_stop", "停止跟随", 14);
}

/* ====================================================================
 * 新增原子动作工具 (适合 ≤3B 端侧模型)
 * ==================================================================== */

void MCPActionProvider::initAtomicTools() {
    /* pose_head: 设置头部姿态 (绝对角度) */
    {
        mcp::Tool tool;
        tool.name = "pose_head";
        tool.description = "设置头部姿态。角度为绝对值，duration 控制过渡时间。"
                            "roll: 滚转角(±25°), pitch: 俯仰角(±35°), yaw: 偏航角(±170°)";
        tool.inputSchema = {
            {"type", "object"},
            {"properties", {
                {"roll", {{"type", "number"}, {"minimum", -25}, {"maximum", 25}, {"description", "头部滚转角(°)"}}},
                {"pitch", {{"type", "number"}, {"minimum", -35}, {"maximum", 35}, {"description", "头部俯仰角(°)"}}},
                {"yaw", {{"type", "number"}, {"minimum", -170}, {"maximum", 170}, {"description", "头部偏航角(°)"}}},
                {"duration_ms", {{"type", "integer"}, {"minimum", 50}, {"maximum", 5000}, {"default", 500}}}
            }}
        };
        tools_.push_back(tool);
    }

    /* pose_body: 设置躯干旋转角度 */
    {
        mcp::Tool tool;
        tool.name = "pose_body";
        tool.description = "设置躯干旋转角度。angle: ±160°";
        tool.inputSchema = {
            {"type", "object"},
            {"properties", {
                {"angle", {{"type", "number"}, {"minimum", -160}, {"maximum", 160}, {"description", "躯干旋转角(°)"}}},
                {"duration_ms", {{"type", "integer"}, {"minimum", 50}, {"maximum", 5000}, {"default", 500}}}
            }}
        };
        tools_.push_back(tool);
    }

    /* antenna_right: 单独控制右天线 */
    {
        mcp::Tool tool;
        tool.name = "antenna_right";
        tool.description = "控制右边天线。角度范围: -90到90。正数向前，负数向后。";
        tool.inputSchema = {
            {"type", "object"},
            {"required", {"angle"}},
            {"properties", {
                {"angle", {{"type", "number"}, {"minimum", -90}, {"maximum", 90}}}
            }}
        };
        tools_.push_back(tool);
    }

    /* antenna_left: 单独控制左天线 */
    {
        mcp::Tool tool;
        tool.name = "antenna_left";
        tool.description = "控制左边天线。角度范围: -90到90。正数向前，负数向后。";
        tool.inputSchema = {
            {"type", "object"},
            {"required", {"angle"}},
            {"properties", {
                {"angle", {{"type", "number"}, {"minimum", -90}, {"maximum", 90}}}
            }}
        };
        tools_.push_back(tool);
    }

    /* play_emotion: 播放情感表达动作 */
    {
        mcp::Tool tool;
        tool.name = "play_emotion";
        tool.description = "播放一个情感表达动作。"
                            "happy: 快速点头+天线摆动, shy: 缓慢低头, curious: 歪头+前倾, "
                            "thinking: 微微抬头, surprised: 快速后仰, sleepy: 缓慢低头+天线下垂";
        tool.inputSchema = {
            {"type", "object"},
            {"required", {"emotion"}},
            {"properties", {
                {"emotion", {{"type", "string"}, {"enum", {"happy", "shy", "curious", "thinking", "surprised", "sleepy"}}}},
                {"intensity", {{"type", "number"}, {"minimum", 0.1}, {"maximum", 1.0}, {"default", 0.7}}}
            }}
        };
        tools_.push_back(tool);
    }

    /* center_all: 所有关节回到零位 */
    {
        mcp::Tool tool;
        tool.name = "center_all";
        tool.description = "所有关节回到零位。";
        tool.inputSchema = {
            {"type", "object"},
            {"properties", {
                {"duration_ms", {{"type", "integer"}, {"minimum", 50}, {"maximum", 3000}, {"default", 800}}}
            }}
        };
        tools_.push_back(tool);
    }

    /* start_tracker */
    {
        mcp::Tool tool;
        tool.name = "start_tracker";
        tool.description = "启动视觉跟踪模式。mode: face 或 gesture";
        tool.inputSchema = {
            {"type", "object"},
            {"required", {"mode"}},
            {"properties", {
                {"mode", {{"type", "string"}, {"enum", {"face", "gesture"}}}}
            }}
        };
        tools_.push_back(tool);
    }

    /* stop_tracker */
    {
        mcp::Tool tool;
        tool.name = "stop_tracker";
        tool.description = "停止所有视觉跟踪。";
        tool.inputSchema = {{"type", "object"}, {"properties", mcp::json::object()}};
        tools_.push_back(tool);
    }
}

/* ====================================================================
 * 复合编排工具 (适合 ≥7B 模型 / 云端)
 * ==================================================================== */

void MCPActionProvider::initChoreographyTool() {
    mcp::Tool tool;
    tool.name = "execute_choreography";
    tool.description =
        "执行一段机器人动作编排序列。支持多步骤组合、嵌套循环、插值控制。"
        "动作类型: pose(绝对姿态), move(增量运动), dance(预设舞蹈), "
        "wait(等待), tracker(跟踪), center(回中), sequence(子序列)。"
        "可用舞蹈: headbanger, jackson, chicken, uh_huh_tilt。"
        "插值方式: linear, hermite, ease_in, ease_out。";
    tool.inputSchema = {
        {"type", "object"},
        {"required", {"actions"}},
        {"properties", {
            {"actions", {
                {"type", "array"},
                {"maxItems", 64},
                {"items", {
                    {"type", "object"},
                    {"required", {"type"}},
                    {"properties", {
                        {"type", {{"type", "string"}, {"enum", {"pose", "move", "dance", "wait", "tracker", "center", "sequence"}}}},
                        {"roll", {{"type", "number"}, {"minimum", -25}, {"maximum", 25}}},
                        {"pitch", {{"type", "number"}, {"minimum", -35}, {"maximum", 35}}},
                        {"yaw", {{"type", "number"}, {"minimum", -170}, {"maximum", 170}}},
                        {"body", {{"type", "number"}, {"minimum", -160}, {"maximum", 160}}},
                        {"ant_r", {{"type", "number"}, {"minimum", -90}, {"maximum", 90}}},
                        {"ant_l", {{"type", "number"}, {"minimum", -90}, {"maximum", 90}}},
                        {"duration_ms", {{"type", "integer"}, {"minimum", 50}, {"maximum", 5000}}},
                        {"easing", {{"type", "string"}, {"enum", {"linear", "hermite", "ease_in", "ease_out"}}}},
                        {"delta_roll", {{"type", "number"}}},
                        {"delta_pitch", {{"type", "number"}}},
                        {"delta_yaw", {{"type", "number"}}},
                        {"delta_body", {{"type", "number"}}},
                        {"name", {{"type", "string"}, {"enum", {"headbanger", "jackson", "chicken", "uh_huh_tilt"}}}},
                        {"cycles", {{"type", "integer"}, {"minimum", 1}, {"maximum", 10}}},
                        {"mode", {{"type", "string"}, {"enum", {"face", "gesture"}}}},
                        {"action", {{"type", "string"}, {"enum", {"start", "stop"}}}},
                        {"repeat", {{"type", "integer"}, {"minimum", 1}, {"maximum", 20}}},
                        {"actions", {{"type", "array"}, {"maxItems", 16}}}
                    }}
                }}
            }}
        }}
    };
    tools_.push_back(tool);
}

void MCPActionProvider::initFlatChoreographyTool() {
    mcp::Tool tool;
    tool.name = "robot_action";
    tool.description =
        "执行单个机器人动作。\n"
        "【type 选项】\n"
        "- pose: 设置绝对姿态，需传 roll/pitch/yaw/body/ant_r/ant_l 中的一个或多个\n"
        "- move: 增量移动，需传 delta_roll/delta_pitch/delta_yaw/delta_body/delta_ant_r/delta_ant_l\n"
        "- dance: 跳舞，必须传 name (headbanger/jackson/chicken/uh_huh_tilt)\n"
        "- center: 回到零位\n"
        "- wait: 等待，需传 duration_ms";
    tool.inputSchema = {
        {"type", "object"},
        {"required", {"type"}},
        {"properties", {
            {"type", {{"type", "string"}, {"enum", {"pose", "move", "dance", "center", "wait"}}}},
            {"roll", {{"type", "number"}, {"minimum", -25}, {"maximum", 25}, {"description", "绝对滚转角(°)，pose用"}}},
            {"pitch", {{"type", "number"}, {"minimum", -35}, {"maximum", 35}, {"description", "绝对俯仰角(°)，pose用"}}},
            {"yaw", {{"type", "number"}, {"minimum", -170}, {"maximum", 170}, {"description", "绝对偏航角(°)，pose用"}}},
            {"body", {{"type", "number"}, {"minimum", -160}, {"maximum", 160}, {"description", "绝对躯干旋转角(°)，pose用"}}},
            {"ant_r", {{"type", "number"}, {"minimum", -90}, {"maximum", 90}, {"description", "右天线绝对角度(°)，pose用"}}},
            {"ant_l", {{"type", "number"}, {"minimum", -90}, {"maximum", 90}, {"description", "左天线绝对角度(°)，pose用"}}},
            {"delta_roll", {{"type", "number"}, {"description", "增量滚转角(°)，move用"}}},
            {"delta_pitch", {{"type", "number"}, {"description", "增量俯仰角(°)，move用"}}},
            {"delta_yaw", {{"type", "number"}, {"description", "增量偏航角(°)，move用"}}},
            {"delta_body", {{"type", "number"}, {"description", "增量躯干旋转角(°)，move用"}}},
            {"delta_ant_r", {{"type", "number"}, {"description", "右天线增量角度(°)，move用"}}},
            {"delta_ant_l", {{"type", "number"}, {"description", "左天线增量角度(°)，move用"}}},
            {"duration_ms", {{"type", "integer"}, {"minimum", 50}, {"maximum", 5000}, {"default", 500}}},
            {"easing", {{"type", "string"}, {"enum", {"linear", "hermite", "ease_in", "ease_out"}}, {"default", "linear"}}},
            {"name", {{"type", "string"}, {"enum", {"headbanger", "jackson", "chicken", "uh_huh_tilt"}}, {"description", "舞蹈名称，dance必填"}}},
            {"cycles", {{"type", "integer"}, {"minimum", 1}, {"maximum", 10}, {"default", 1}}}
        }}
    };
    tools_.push_back(tool);
}

const std::vector<mcp::Tool> &MCPActionProvider::getTools() const {
    return tools_;
}

/* ====================================================================
 * 情感动作预设 JSON (用于 play_emotion)
 * ==================================================================== */

static const char *get_emotion_json(const std::string &emotion, float intensity) {
    /* 根据情感类型返回对应的动作序列 JSON */
    /* intensity 用于缩放角度和时间 */
    static char json_buf[512];

    float scale = intensity;
    int fast_dur = (int)(200 / scale);
    int slow_dur = (int)(800 / scale);

    if (emotion == "happy") {
        snprintf(json_buf, sizeof(json_buf),
            "{\"actions\":[{\"type\":\"sequence\",\"repeat\":3,\"actions\":["
            "{\"type\":\"pose\",\"pitch\":%.1f,\"ant_r\":%.1f,\"ant_l\":%.1f,\"duration_ms\":%d,\"easing\":\"hermite\"},"
            "{\"type\":\"pose\",\"pitch\":%.1f,\"ant_r\":%.1f,\"ant_l\":%.1f,\"duration_ms\":%d,\"easing\":\"hermite\"}"
            "]},{\"type\":\"center\",\"duration_ms\":500}]}",
            -15.0f * scale, 45.0f * scale, -45.0f * scale, fast_dur,
            10.0f * scale, -30.0f * scale, 30.0f * scale, fast_dur);
    } else if (emotion == "shy") {
        snprintf(json_buf, sizeof(json_buf),
            "{\"actions\":[{\"type\":\"pose\",\"pitch\":%.1f,\"roll\":%.1f,"
            "\"ant_r\":%.1f,\"ant_l\":%.1f,\"duration_ms\":%d,\"easing\":\"ease_in\"}]}",
            20.0f * scale, -8.0f * scale, -20.0f * scale, 20.0f * scale, slow_dur);
    } else if (emotion == "curious") {
        snprintf(json_buf, sizeof(json_buf),
            "{\"actions\":[{\"type\":\"pose\",\"roll\":%.1f,\"pitch\":%.1f,"
            "\"ant_r\":%.1f,\"ant_l\":%.1f,\"duration_ms\":%d,\"easing\":\"hermite\"}]}",
            15.0f * scale, -10.0f * scale, 60.0f * scale, -60.0f * scale, slow_dur);
    } else if (emotion == "thinking") {
        snprintf(json_buf, sizeof(json_buf),
            "{\"actions\":[{\"type\":\"pose\",\"pitch\":%.1f,\"yaw\":%.1f,"
            "\"duration_ms\":%d,\"easing\":\"hermite\"},"
            "{\"type\":\"wait\",\"duration_ms\":1000}]}",
            -10.0f * scale, 20.0f * scale, slow_dur);
    } else if (emotion == "surprised") {
        snprintf(json_buf, sizeof(json_buf),
            "{\"actions\":[{\"type\":\"pose\",\"pitch\":%.1f,"
            "\"ant_r\":%.1f,\"ant_l\":%.1f,\"duration_ms\":%d,\"easing\":\"ease_out\"}]}",
            -25.0f * scale, 70.0f * scale, -70.0f * scale, fast_dur);
    } else if (emotion == "sleepy") {
        snprintf(json_buf, sizeof(json_buf),
            "{\"actions\":[{\"type\":\"pose\",\"pitch\":%.1f,"
            "\"ant_r\":%.1f,\"ant_l\":%.1f,\"duration_ms\":%d,\"easing\":\"ease_in\"}]}",
            25.0f * scale, -40.0f * scale, 40.0f * scale, (int)(1200 / scale));
    } else {
        snprintf(json_buf, sizeof(json_buf),
            "{\"actions\":[{\"type\":\"center\",\"duration_ms\":500}]}");
    }

    return json_buf;
}

/* ====================================================================
 * 工具执行分发
 * ==================================================================== */

bool MCPActionProvider::executeTool(const std::string &name, const mcp::json &args,
                                    std::string &out_result) {
    /* 新增原子动作工具 */
    if (name == "pose_head") return executePoseHead(args, out_result);
    if (name == "pose_body") return executePoseBody(args, out_result);
    if (name == "antenna_right") return executeAntennaRight(args, out_result);
    if (name == "antenna_left") return executeAntennaLeft(args, out_result);
    if (name == "play_emotion") return executePlayEmotion(args, out_result);
    if (name == "center_all") return executeCenterAll(args, out_result);
    if (name == "execute_choreography") return executeChoreography(args, out_result);
    if (name == "robot_action") return executeRobotAction(args, out_result);

    if (name == "start_tracker") {
        std::string mode = args.value("mode", "face");
        if (mode == "gesture") {
            voice_ctl_execute(15);
        } else {
            voice_ctl_execute(13);
        }
        out_result = "OK";
        return true;
    }
    if (name == "stop_tracker") {
        voice_ctl_execute(14);
        out_result = "OK";
        return true;
    }

    /* 旧版工具 (通过 action_id 查找) */
    auto it = tool_to_action_id_.find(name);
    if (it == tool_to_action_id_.end()) {
        return false;
    }

    int action_id = it->second;
    std::cout << getTimestamp() << " [ActionProvider] 执行工具: " << name << " (ID: " << action_id
            << ")\n";

    // 舞蹈动作 (ID 9-12) 需要通过主线程执行
    if (action_id >= 9 && action_id <= 12) {
        if (pending_dance_) {
            std::lock_guard<std::mutex> lock(pending_dance_->mutex);
            pending_dance_->action_id.store(action_id);
            std::cout << getTimestamp() << " [ActionProvider] 舞蹈动作已标记为待处理 (ID: "
                    << action_id << ")\n";
            out_result = "OK";
        } else {
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

/* ====================================================================
 * 原子动作工具实现
 * ==================================================================== */

bool MCPActionProvider::executePoseHead(const mcp::json &args, std::string &out_result) {
    float roll = 0, pitch = 0, yaw = 0, body = 0, ant_r = 0, ant_l = 0;
    voice_ctl_get_current_pose(&roll, &pitch, &yaw, &body, &ant_r, &ant_l);

    if (args.contains("roll")) roll = args["roll"].get<float>();
    if (args.contains("pitch")) pitch = args["pitch"].get<float>();
    if (args.contains("yaw")) yaw = args["yaw"].get<float>();
    int dur = args.value("duration_ms", 500);

    std::ostringstream json;
    json << "{\"actions\":[{\"type\":\"pose\""
        << ",\"roll\":" << roll
        << ",\"pitch\":" << pitch
        << ",\"yaw\":" << yaw
        << ",\"duration_ms\":" << dur
        << ",\"easing\":\"hermite\"}]}";

    int ret = voice_ctl_execute_choreography(json.str().c_str());
    if (ret == 0) {
        std::ostringstream ss;
        ss << "OK, 当前姿态: roll=" << roll << "° pitch=" << pitch << "° yaw=" << yaw << "°";
        out_result = ss.str();
    } else {
        out_result = "执行失败";
    }
    return true;
}

bool MCPActionProvider::executePoseBody(const mcp::json &args, std::string &out_result) {
    float angle = args.value("angle", 0.0f);
    int dur = args.value("duration_ms", 500);

    std::ostringstream json;
    json << "{\"actions\":[{\"type\":\"pose\""
        << ",\"body\":" << angle
        << ",\"duration_ms\":" << dur
        << ",\"easing\":\"hermite\"}]}";

    int ret = voice_ctl_execute_choreography(json.str().c_str());
    out_result = (ret == 0) ? "OK" : "执行失败";
    return true;
}

bool MCPActionProvider::executeAntennaRight(const mcp::json &args, std::string &out_result) {
    float angle = args.value("angle", 0.0f);

    std::ostringstream json;
    json << "{\"actions\":[{\"type\":\"pose\""
        << ",\"ant_r\":" << angle
        << ",\"duration_ms\":300"
        << ",\"easing\":\"hermite\"}]}";

    int ret = voice_ctl_execute_choreography(json.str().c_str());
    out_result = (ret == 0) ? "OK" : "执行失败";
    return true;
}

bool MCPActionProvider::executeAntennaLeft(const mcp::json &args, std::string &out_result) {
    float angle = args.value("angle", 0.0f);

    std::ostringstream json;
    json << "{\"actions\":[{\"type\":\"pose\""
        << ",\"ant_l\":" << angle
        << ",\"duration_ms\":300"
        << ",\"easing\":\"hermite\"}]}";

    int ret = voice_ctl_execute_choreography(json.str().c_str());
    out_result = (ret == 0) ? "OK" : "执行失败";
    return true;
}

bool MCPActionProvider::executePlayEmotion(const mcp::json &args, std::string &out_result) {
    std::string emotion = args.value("emotion", "happy");
    float intensity = args.value("intensity", 0.7f);

    const char *json = get_emotion_json(emotion, intensity);
    std::cout << getTimestamp()
        << " [ActionProvider] play_emotion: " << emotion
        << " (intensity=" << intensity << ")\n";

    int ret = voice_ctl_execute_choreography(json);
    out_result = (ret == 0) ? "OK" : "执行失败";
    return true;
}

bool MCPActionProvider::executeCenterAll(const mcp::json &args, std::string &out_result) {
    int dur = args.value("duration_ms", 800);

    std::ostringstream json;
    json << "{\"actions\":[{\"type\":\"center\",\"duration_ms\":" << dur << "}]}";

    int ret = voice_ctl_execute_choreography(json.str().c_str());
    out_result = (ret == 0) ? "OK" : "执行失败";
    return true;
}

bool MCPActionProvider::executeChoreography(const mcp::json &args, std::string &out_result) {
    if (!args.contains("actions") || !args["actions"].is_array() || args["actions"].empty()) {
        std::cout << getTimestamp() << " [ActionProvider] 降级：小模型缺少 actions 数组，随机调用原子工具\n";
        if (rand() % 2 == 0) {
            const char* emotions[] = {"happy", "shy", "curious", "thinking", "surprised", "sleepy"};
            std::string emotion = emotions[rand() % 6];
            mcp::json emo_args = {{"emotion", emotion}, {"intensity", 0.8f}};
            return executePlayEmotion(emo_args, out_result);
        } else {
            const char* dances[] = {"headbanger", "jackson", "chicken", "uh_huh_tilt"};
            std::string dance_name = dances[rand() % 4];
            int action_id = -1;
            if (dance_name == "headbanger") action_id = 9;
            else if (dance_name == "jackson") action_id = 10;
            else if (dance_name == "chicken") action_id = 11;
            else if (dance_name == "uh_huh_tilt") action_id = 12;

            if (action_id != -1 && pending_dance_) {
                pending_dance_->action_id.store(action_id);
                out_result = "OK";
                return true;
            }
        }
    }

    /* 将整个 args 作为 JSON 传递给执行引擎 */
    std::string json_str = args.dump();
    std::cout << getTimestamp()
        << " [ActionProvider] execute_choreography: "
        << json_str.substr(0, 200) << "...\n";

    int ret = voice_ctl_execute_choreography(json_str.c_str());
    if (ret == 0) {
        float r, p, y, b, ar, al;
        voice_ctl_get_current_pose(&r, &p, &y, &b, &ar, &al);
        std::ostringstream ss;
        ss << "动作编排执行完成。当前姿态: yaw=" << y << "° pitch=" << p << "° roll=" << r << "°";
        out_result = ss.str();
    } else if (ret == ACTION_EXEC_ERR_ABORT) {
        out_result = "动作被用户中断";
    } else {
        out_result = "动作编排执行失败";
    }
    return true;
}

bool MCPActionProvider::executeRobotAction(const mcp::json &args, std::string &out_result) {
    mcp::json action = mcp::json::object();
    for (auto it = args.begin(); it != args.end(); ++it) {
        action[it.key()] = it.value();
    }

    // 小模型兼容性：如果缺少 type 字段，根据其他参数推断
    if (!action.contains("type") || action["type"].get<std::string>().empty()) {
        std::string inferred_type;
        if (action.contains("name")) {
            inferred_type = "dance";
        } else if (action.contains("delta_roll") || action.contains("delta_pitch") ||
                action.contains("delta_yaw") || action.contains("delta_body") ||
                action.contains("delta_ant_r") || action.contains("delta_ant_l")) {
            inferred_type = "move";
        } else if (action.contains("roll") || action.contains("pitch") ||
                action.contains("yaw") || action.contains("body") ||
                action.contains("ant_r") || action.contains("ant_l")) {
            inferred_type = "pose";  // 有绝对值参数，推断为 pose
        } else {
            inferred_type = "center";
        }
        action["type"] = inferred_type;
        std::cout << getTimestamp() << " [ActionProvider] 推断 type=" << inferred_type << "\n";
    }

    std::string action_type = action.value("type", "");

    // dance 类型兼容性修复：如果没有 name 字段，随机选择一个舞蹈
    if (action_type == "dance") {
        if (!action.contains("name") || action["name"].get<std::string>().empty()) {
            const char* dances[] = {"headbanger", "jackson", "chicken", "uh_huh_tilt"};
            action["name"] = dances[rand() % 4];
            std::cout << getTimestamp() << " [ActionProvider] dance 缺少 name，随机选择: "
                    << action["name"].get<std::string>() << "\n";
        }
    }

    // move 类型兼容性修复：将绝对值字段自动转换为增量字段
    // 因为小模型常常混淆 pose/move 的参数格式
    if (action_type == "move") {
        auto maybe_convert_to_delta = [&action](const char *abs_key, const char *delta_key) {
            if (action.contains(abs_key) && !action.contains(delta_key)) {
                float val = action[abs_key].get<float>();
                if (val != 0.0f) {
                    action[delta_key] = val;
                    action.erase(abs_key);
                }
            } else if (action.contains(delta_key)) {
                float delta_val = action[delta_key].get<float>();
                if (delta_val == 0.0f && action.contains(abs_key)) {
                    float abs_val = action[abs_key].get<float>();
                    if (abs_val != 0.0f) {
                        action[delta_key] = abs_val;
                        action.erase(abs_key);
                    }
                }
            }
        };
        maybe_convert_to_delta("roll", "delta_roll");
        maybe_convert_to_delta("pitch", "delta_pitch");
        maybe_convert_to_delta("yaw", "delta_yaw");
        maybe_convert_to_delta("body", "delta_body");
        maybe_convert_to_delta("ant_r", "delta_ant_r");
        maybe_convert_to_delta("ant_l", "delta_ant_l");
    }

    mcp::json choreo = mcp::json::object();
    choreo["actions"] = mcp::json::array({action});

    std::string json_str = choreo.dump();
    std::cout << getTimestamp() << " [ActionProvider] execute_robot_action (flat): " << json_str << "\n";

    int ret = voice_ctl_execute_choreography(json_str.c_str());
    if (ret == 0) {
        float r, p, y, b, ar, al;
        voice_ctl_get_current_pose(&r, &p, &y, &b, &ar, &al);
        std::ostringstream ss;
        ss << "原子动作执行完成。当前姿态: yaw=" << y << "° pitch=" << p << "° roll=" << r << "°";
        out_result = ss.str();
    } else if (ret == ACTION_EXEC_ERR_ABORT) {
        out_result = "动作被用户中断";
    } else {
        out_result = "原子动作执行失败";
    }
    return true;
}

#endif  // USE_MCP
