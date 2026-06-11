#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
#
# Engine Bridge - 与 C 动作执行引擎的 Unix Socket 通信桥接层

import asyncio
import json
import logging
import struct

logger = logging.getLogger("reachy-motion-mcp.bridge")

# 通信协议: 简单的长度前缀 JSON
# [4 bytes: payload length (big-endian)] [payload: JSON bytes]

# 命令类型
CMD_EXECUTE = "execute"
CMD_ABORT = "abort"
CMD_GET_POSE = "get_pose"
CMD_GET_TRACKER = "get_tracker"


class EngineBridge:
    """与 C action_executor 引擎的 Unix Socket 通信桥"""

    def __init__(self, socket_path: str):
        self.socket_path = socket_path
        self._reader = None
        self._writer = None
        self._lock = asyncio.Lock()

    async def connect(self) -> bool:
        """连接到 C 引擎 Unix Socket"""
        try:
            self._reader, self._writer = await asyncio.open_unix_connection(
                self.socket_path
            )
            logger.info(f"已连接到引擎: {self.socket_path}")
            return True
        except (ConnectionRefusedError, FileNotFoundError) as e:
            logger.warning(f"无法连接到引擎 ({self.socket_path}): {e}")
            return False

    async def disconnect(self):
        """断开连接"""
        if self._writer:
            self._writer.close()
            await self._writer.wait_closed()
            self._writer = None
            self._reader = None

    async def _send_command(self, cmd: dict) -> dict | None:
        """发送命令并等待响应"""
        async with self._lock:
            if not self._writer:
                if not await self.connect():
                    return None

            try:
                payload = json.dumps(cmd).encode("utf-8")
                self._writer.write(struct.pack(">I", len(payload)) + payload)
                await self._writer.drain()

                # 读取响应
                header = await self._reader.readexactly(4)
                resp_len = struct.unpack(">I", header)[0]
                resp_data = await self._reader.readexactly(resp_len)
                return json.loads(resp_data.decode("utf-8"))

            except (ConnectionResetError, BrokenPipeError, asyncio.IncompleteReadError) as e:
                logger.error(f"通信错误: {e}")
                await self.disconnect()
                return None

    async def execute_actions(self, actions_json: dict) -> dict:
        """执行动作序列"""
        result = await self._send_command({
            "cmd": CMD_EXECUTE,
            "data": actions_json,
        })
        if result is None:
            # 降级: 返回模拟结果 (引擎未启动时)
            return {"status": "error", "message": "引擎未连接"}
        return result

    async def abort(self) -> dict:
        """中断当前执行"""
        result = await self._send_command({"cmd": CMD_ABORT})
        return result or {"status": "ok"}

    async def get_pose(self) -> dict:
        """获取当前位姿"""
        result = await self._send_command({"cmd": CMD_GET_POSE})
        if result is None:
            return {
                "head": {"roll": 0, "pitch": 0, "yaw": 0},
                "body": 0,
                "antenna": {"right": 0, "left": 0},
                "tracker_active": None,
            }
        return result

    async def get_tracker_status(self) -> dict:
        """获取跟踪器状态"""
        result = await self._send_command({"cmd": CMD_GET_TRACKER})
        return result or {"active": None}
