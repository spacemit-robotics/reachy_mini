#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#
# Reachy Mini Motion MCP Server
#
# 轻量级 MCP Server，桥接 LLM Agent 与本地 C 动作执行引擎。
# 通过 Unix Domain Socket 与 action_executor 通信。
#
# 启动方式 (符合 MCP stdio 规范):
#   python3 server.py
#
# 环境变量:
#   ENGINE_SOCKET: C 引擎 Unix Socket 路径 (默认: /tmp/reachy_action_executor.sock)

import asyncio
import logging
import os
import sys

from tools import register_tools
from resources import register_resources
from bridge import EngineBridge

# 尝试导入 MCP SDK
try:
    from mcp.server import Server
    from mcp.server.stdio import stdio_server
except ImportError:
    print("错误: 需要安装 mcp-python-sdk", file=sys.stderr)
    print("  pip install mcp", file=sys.stderr)
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
logger = logging.getLogger("reachy-motion-mcp")


def create_server() -> Server:
    """创建并配置 MCP Server"""
    server = Server("reachy-motion")

    # 初始化与 C 引擎的通信桥
    socket_path = os.environ.get("ENGINE_SOCKET", "/tmp/reachy_action_executor.sock")
    bridge = EngineBridge(socket_path)

    # 注册工具和资源
    register_tools(server, bridge)
    register_resources(server, bridge)

    return server


async def main():
    """MCP Server 入口"""
    server = create_server()
    logger.info("Reachy Motion MCP Server 启动中...")

    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())


if __name__ == "__main__":
    asyncio.run(main())
