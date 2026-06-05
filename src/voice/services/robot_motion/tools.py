#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#
# MCP Tool Handlers - 动作工具注册与处理

import json
import logging

try:
    from mcp.server import Server
    from mcp.types import Tool, TextContent
except ImportError:
    pass

from bridge import EngineBridge

logger = logging.getLogger("reachy-motion-mcp.tools")

# ============================================================================
# 情感预设动作序列
# ============================================================================

EMOTION_PRESETS = {
    "happy": {
        "actions": [
            {"type": "sequence", "repeat": 3, "actions": [
                {"type": "pose", "pitch": -15, "ant_r": 45, "ant_l": -45,
                 "duration_ms": 200, "easing": "hermite"},
                {"type": "pose", "pitch": 10, "ant_r": -30, "ant_l": 30,
                 "duration_ms": 200, "easing": "hermite"},
            ]},
            {"type": "center", "duration_ms": 500},
        ]
    },
    "shy": {
        "actions": [
            {"type": "pose", "pitch": 20, "roll": -8, "ant_r": -20, "ant_l": 20,
             "duration_ms": 800, "easing": "ease_in"},
        ]
    },
    "curious": {
        "actions": [
            {"type": "pose", "roll": 15, "pitch": -10, "ant_r": 60, "ant_l": -60,
             "duration_ms": 800, "easing": "hermite"},
        ]
    },
    "thinking": {
        "actions": [
            {"type": "pose", "pitch": -10, "yaw": 20,
             "duration_ms": 800, "easing": "hermite"},
            {"type": "wait", "duration_ms": 1000},
        ]
    },
    "surprised": {
        "actions": [
            {"type": "pose", "pitch": -25, "ant_r": 70, "ant_l": -70,
             "duration_ms": 200, "easing": "ease_out"},
        ]
    },
    "sleepy": {
        "actions": [
            {"type": "pose", "pitch": 25, "ant_r": -40, "ant_l": 40,
             "duration_ms": 1200, "easing": "ease_in"},
        ]
    },
}


# 角度类字段名集合 (用于 intensity 缩放)
_ANGLE_KEYS = {"roll", "pitch", "yaw", "body", "ant_r", "ant_l",
               "delta_roll", "delta_pitch", "delta_yaw", "delta_body"}


def _scale_preset(preset: dict, intensity: float) -> dict:
    """按 intensity 系数缩放预设动作中的角度值和时长。

    intensity=1.0 表示原始幅度；intensity=0.5 表示角度减半、时长缩短为 70%。
    """
    import copy
    intensity = max(0.1, min(1.0, intensity))
    scaled = copy.deepcopy(preset)

    def _scale_action(action: dict):
        for key in _ANGLE_KEYS:
            if key in action:
                action[key] = round(action[key] * intensity, 1)
        if "duration_ms" in action:
            # 时长按 sqrt(intensity) 缩放 — 低强度时动作更缓慢但不至于过长
            action["duration_ms"] = max(50, int(action["duration_ms"] * (intensity ** 0.5)))
        # 递归处理嵌套 actions (如 sequence 类型)
        for sub in action.get("actions", []):
            _scale_action(sub)

    for action in scaled.get("actions", []):
        _scale_action(action)
    return scaled


def register_tools(server: "Server", bridge: EngineBridge):
    """注册所有 MCP 工具"""

    @server.list_tools()
    async def list_tools() -> list[Tool]:
        return [
            # ============================================================
            # 原子动作工具 (适合 ≤3B 端侧模型)
            # ============================================================
            Tool(
                name="pose_head",
                description="设置头部姿态。角度为绝对值，duration 控制过渡时间。",
                inputSchema={
                    "type": "object",
                    "properties": {
                        "roll": {"type": "number", "minimum": -25, "maximum": 25,
                                 "description": "头部滚转角(°)"},
                        "pitch": {"type": "number", "minimum": -35, "maximum": 35,
                                  "description": "头部俯仰角(°)"},
                        "yaw": {"type": "number", "minimum": -170, "maximum": 170,
                                "description": "头部偏航角(°)"},
                        "duration_ms": {"type": "integer", "minimum": 50,
                                        "maximum": 5000, "default": 500},
                    },
                },
            ),
            Tool(
                name="pose_body",
                description="设置躯干旋转角度。",
                inputSchema={
                    "type": "object",
                    "properties": {
                        "angle": {"type": "number", "minimum": -160, "maximum": 160,
                                  "description": "躯干旋转角(°)"},
                        "duration_ms": {"type": "integer", "minimum": 50,
                                        "maximum": 5000, "default": 500},
                    },
                },
            ),
            Tool(
                name="pose_antenna",
                description="设置左右天线角度。",
                inputSchema={
                    "type": "object",
                    "properties": {
                        "right": {"type": "number", "minimum": -90, "maximum": 90},
                        "left": {"type": "number", "minimum": -90, "maximum": 90},
                        "duration_ms": {"type": "integer", "minimum": 50,
                                        "maximum": 5000, "default": 300},
                    },
                },
            ),
            Tool(
                name="play_dance",
                description="播放一段预设舞蹈动作。",
                inputSchema={
                    "type": "object",
                    "required": ["name"],
                    "properties": {
                        "name": {"type": "string",
                                 "enum": ["headbanger", "jackson", "chicken", "uh_huh_tilt"]},
                        "cycles": {"type": "integer", "minimum": 1, "maximum": 10,
                                   "default": 2},
                    },
                },
            ),
            Tool(
                name="play_emotion",
                description="播放一个情感表达动作。"
                            "happy=开心, shy=害羞, curious=好奇, "
                            "thinking=思考, surprised=惊讶, sleepy=困倦",
                inputSchema={
                    "type": "object",
                    "required": ["emotion"],
                    "properties": {
                        "emotion": {"type": "string",
                                    "enum": ["happy", "shy", "curious",
                                             "thinking", "surprised", "sleepy"]},
                        "intensity": {"type": "number", "minimum": 0.1,
                                      "maximum": 1.0, "default": 0.7},
                    },
                },
            ),
            Tool(
                name="center_all",
                description="所有关节回到零位。",
                inputSchema={
                    "type": "object",
                    "properties": {
                        "duration_ms": {"type": "integer", "minimum": 50,
                                        "maximum": 3000, "default": 800},
                    },
                },
            ),
            Tool(
                name="start_tracker",
                description="启动视觉跟踪模式。",
                inputSchema={
                    "type": "object",
                    "required": ["mode"],
                    "properties": {
                        "mode": {"type": "string", "enum": ["face", "gesture"]},
                    },
                },
            ),
            Tool(
                name="stop_tracker",
                description="停止所有视觉跟踪。",
                inputSchema={"type": "object", "properties": {}},
            ),
            # ============================================================
            # 复合编排工具 (适合 ≥7B 模型 / 云端)
            # ============================================================
            Tool(
                name="execute_choreography",
                description="执行一段机器人动作编排序列。"
                            "支持多步骤组合、嵌套循环、插值控制。"
                            "动作类型: pose, move, dance, wait, tracker, center, sequence。"
                            "可用舞蹈: headbanger, jackson, chicken, uh_huh_tilt。"
                            "插值: linear, hermite, ease_in, ease_out。",
                inputSchema={
                    "type": "object",
                    "required": ["actions"],
                    "properties": {
                        "actions": {
                            "type": "array",
                            "maxItems": 64,
                            "items": {
                                "type": "object",
                                "required": ["type"],
                                "properties": {
                                    "type": {"enum": ["pose", "move", "dance", "wait",
                                                      "tracker", "center", "sequence"]},
                                    "roll": {"type": "number", "minimum": -25, "maximum": 25},
                                    "pitch": {"type": "number", "minimum": -35, "maximum": 35},
                                    "yaw": {"type": "number", "minimum": -170, "maximum": 170},
                                    "body": {"type": "number", "minimum": -160, "maximum": 160},
                                    "ant_r": {"type": "number", "minimum": -90, "maximum": 90},
                                    "ant_l": {"type": "number", "minimum": -90, "maximum": 90},
                                    "duration_ms": {"type": "integer", "minimum": 50,
                                                    "maximum": 5000},
                                    "easing": {"enum": ["linear", "hermite",
                                                        "ease_in", "ease_out"]},
                                    "name": {"type": "string"},
                                    "cycles": {"type": "integer", "minimum": 1, "maximum": 10},
                                    "mode": {"enum": ["face", "gesture"]},
                                    "action": {"enum": ["start", "stop"]},
                                    "repeat": {"type": "integer", "minimum": 1, "maximum": 20},
                                    "actions": {"type": "array", "maxItems": 16},
                                },
                            },
                        },
                    },
                },
            ),
        ]

    @server.call_tool()
    async def call_tool(name: str, arguments: dict) -> list[TextContent]:
        logger.info(f"Tool call: {name}({json.dumps(arguments, ensure_ascii=False)[:200]})")

        try:
            if name == "pose_head":
                actions = {"actions": [{
                    "type": "pose",
                    **{k: v for k, v in arguments.items() if k in
                       ("roll", "pitch", "yaw", "duration_ms")},
                    "easing": "hermite",
                }]}
                result = await bridge.execute_actions(actions)

            elif name == "pose_body":
                actions = {"actions": [{
                    "type": "pose",
                    "body": arguments.get("angle", 0),
                    "duration_ms": arguments.get("duration_ms", 500),
                    "easing": "hermite",
                }]}
                result = await bridge.execute_actions(actions)

            elif name == "pose_antenna":
                actions = {"actions": [{
                    "type": "pose",
                    "ant_r": arguments.get("right", 0),
                    "ant_l": arguments.get("left", 0),
                    "duration_ms": arguments.get("duration_ms", 300),
                    "easing": "hermite",
                }]}
                result = await bridge.execute_actions(actions)

            elif name == "play_dance":
                actions = {"actions": [{
                    "type": "dance",
                    "name": arguments["name"],
                    "cycles": arguments.get("cycles", 2),
                }]}
                result = await bridge.execute_actions(actions)

            elif name == "play_emotion":
                emotion = arguments.get("emotion", "happy")
                intensity = arguments.get("intensity", 0.7)
                preset = EMOTION_PRESETS.get(emotion, EMOTION_PRESETS["happy"])
                # 按 intensity 缩放角度值和时长
                scaled = _scale_preset(preset, intensity)
                result = await bridge.execute_actions(scaled)

            elif name == "center_all":
                actions = {"actions": [{
                    "type": "center",
                    "duration_ms": arguments.get("duration_ms", 800),
                }]}
                result = await bridge.execute_actions(actions)

            elif name == "start_tracker":
                actions = {"actions": [{
                    "type": "tracker",
                    "mode": arguments.get("mode", "face"),
                    "action": "start",
                }]}
                result = await bridge.execute_actions(actions)

            elif name == "stop_tracker":
                actions = {"actions": [{
                    "type": "tracker",
                    "action": "stop",
                }]}
                result = await bridge.execute_actions(actions)

            elif name == "execute_choreography":
                actions = arguments.get("actions")
                if not isinstance(actions, list) or len(actions) == 0:
                    # 小模型 JSON 能力弱，无法正确生成 actions 数组
                    # 回退策略：从情感预设中随机挑选一个执行，让机器人"随便动动"
                    import random
                    fallback_emotion = random.choice(list(EMOTION_PRESETS.keys()))
                    fallback_preset = _scale_preset(
                        EMOTION_PRESETS[fallback_emotion], intensity=0.6
                    )
                    logger.info(f"execute_choreography fallback: playing '{fallback_emotion}'")
                    result = await bridge.execute_actions(fallback_preset)
                else:
                    # 云端大模型生成的合法复杂动作序列
                    result = await bridge.execute_actions(arguments)

            else:
                return [TextContent(type="text", text=f"未知工具: {name}")]

            # 获取执行后的位姿
            pose = await bridge.get_pose()
            pose_str = (f"yaw={pose.get('head', {}).get('yaw', 0):.1f}° "
                        f"pitch={pose.get('head', {}).get('pitch', 0):.1f}° "
                        f"roll={pose.get('head', {}).get('roll', 0):.1f}°")

            status = result.get("status", "unknown")
            if status == "ok":
                return [TextContent(type="text",
                                    text=f"动作执行完成，当前姿态: {pose_str}")]
            else:
                msg = result.get("message", "未知错误")
                return [TextContent(type="text", text=f"执行失败: {msg}")]

        except Exception as e:
            logger.error(f"Tool execution error: {e}")
            return [TextContent(type="text", text=f"执行异常: {str(e)}")]
