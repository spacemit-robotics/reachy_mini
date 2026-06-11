#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#
# MCP Resource Handlers - 机器人状态资源暴露

import json
import logging

try:
    from mcp.server import Server
    from mcp.types import Resource
except ImportError:
    pass

from bridge import EngineBridge

logger = logging.getLogger("reachy-motion-mcp.resources")


def register_resources(server: "Server", bridge: EngineBridge):
    """注册 MCP Resources"""

    @server.list_resources()
    async def list_resources() -> list[Resource]:
        return [
            Resource(
                uri="robot://status/pose",
                name="当前姿态",
                description="机器人当前的头部和躯干角度",
                mimeType="application/json",
            ),
            Resource(
                uri="robot://status/tracker",
                name="跟踪状态",
                description="当前是否有活跃的视觉跟踪任务",
                mimeType="application/json",
            ),
        ]

    @server.read_resource()
    async def read_resource(uri: str) -> str:
        if uri == "robot://status/pose":
            pose = await bridge.get_pose()
            return json.dumps(pose, ensure_ascii=False, indent=2)

        elif uri == "robot://status/tracker":
            tracker = await bridge.get_tracker_status()
            return json.dumps(tracker, ensure_ascii=False, indent=2)

        else:
            return json.dumps({"error": f"未知资源: {uri}"})
