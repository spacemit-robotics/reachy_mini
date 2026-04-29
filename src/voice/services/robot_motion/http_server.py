#!/usr/bin/env python3
# Copyright (c) 2025 Pollen Robotics / SpacemiT
# SPDX-License-Identifier: Apache-2.0
#
# Robot Motion MCP Server for Reachy Mini
# Provides tool definitions for LLM-driven motor control.
# In practice, these tools are intercepted locally by voice_pipeline.cpp
# (tryLocalToolCall) and never actually reach this server at runtime.
# This server exists for:
#   1. Tool schema advertisement (MCP discovery)
#   2. Fallback execution if local interception is disabled
#   3. Testing/debugging tool calls independently

"""
Robot Motion MCP Server - exposes Reachy Mini motor actions as MCP tools.

Usage:
    python http_server.py [--port 8100]

The server advertises tools that map to voice_ctl action IDs:
    action 0: head_turn_left
    action 1: head_turn_right
    action 2: head_look_up
    action 3: head_look_down
    action 4: head_tilt_left
    action 5: head_tilt_right
    action 6: body_turn_left
    action 7: body_turn_right
    action 8: reset_pose
    action 9: dance_headbanger
    action 10: dance_jackson
    action 11: dance_chicken
    action 12: dance_uh_huh_tilt
"""

import argparse
import json
from http.server import HTTPServer, BaseHTTPRequestHandler


# Tool definitions with JSON Schema parameters
TOOLS = [
    {
        "name": "head_turn_left",
        "description": "Turn the robot's head to the left",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Angle in degrees (default 20)",
                    "default": 20
                }
            }
        }
    },
    {
        "name": "head_turn_right",
        "description": "Turn the robot's head to the right",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Angle in degrees (default 20)",
                    "default": 20
                }
            }
        }
    },
    {
        "name": "head_look_up",
        "description": "Tilt the robot's head upward",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Angle in degrees (default 15)",
                    "default": 15
                }
            }
        }
    },
    {
        "name": "head_look_down",
        "description": "Tilt the robot's head downward",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Angle in degrees (default 15)",
                    "default": 15
                }
            }
        }
    },
    {
        "name": "head_tilt_left",
        "description": "Tilt (roll) the robot's head to the left",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Angle in degrees (default 15)",
                    "default": 15
                }
            }
        }
    },
    {
        "name": "head_tilt_right",
        "description": "Tilt (roll) the robot's head to the right",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Angle in degrees (default 15)",
                    "default": 15
                }
            }
        }
    },
    {
        "name": "body_turn_left",
        "description": "Rotate the robot's body to the left",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Angle in degrees (default 30)",
                    "default": 30
                }
            }
        }
    },
    {
        "name": "body_turn_right",
        "description": "Rotate the robot's body to the right",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Angle in degrees (default 30)",
                    "default": 30
                }
            }
        }
    },
    {
        "name": "reset_pose",
        "description": "Reset the robot to its neutral/home position",
        "inputSchema": {
            "type": "object",
            "properties": {}
        }
    },
    {
        "name": "dance_headbanger",
        "description": "Perform the headbanger dance move",
        "inputSchema": {
            "type": "object",
            "properties": {}
        }
    },
    {
        "name": "dance_jackson",
        "description": "Perform the Jackson dance move",
        "inputSchema": {
            "type": "object",
            "properties": {}
        }
    },
    {
        "name": "dance_chicken",
        "description": "Perform the chicken dance move",
        "inputSchema": {
            "type": "object",
            "properties": {}
        }
    },
    {
        "name": "dance_uh_huh_tilt",
        "description": "Perform the uh-huh head tilt dance move",
        "inputSchema": {
            "type": "object",
            "properties": {}
        }
    },
]


class MCPHandler(BaseHTTPRequestHandler):
    """Simple JSON-RPC 2.0 handler for MCP protocol."""

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length)

        try:
            request = json.loads(body)
        except json.JSONDecodeError:
            self._send_error(-32700, "Parse error")
            return

        method = request.get("method", "")
        req_id = request.get("id")
        params = request.get("params", {})

        if method == "initialize":
            self._send_result(req_id, {
                "protocolVersion": "2024-11-05",
                "capabilities": {"tools": {}},
                "serverInfo": {
                    "name": "robot_motion",
                    "version": "1.0.0"
                }
            })
        elif method == "tools/list":
            self._send_result(req_id, {"tools": TOOLS})
        elif method == "tools/call":
            tool_name = params.get("name", "")
            tool_args = params.get("arguments", {})
            result = self._execute_tool(tool_name, tool_args)
            self._send_result(req_id, {
                "content": [{"type": "text", "text": result}]
            })
        elif method == "notifications/initialized":
            # No response needed for notifications
            self._send_result(req_id, {})
        else:
            self._send_error(-32601, f"Method not found: {method}",
                             req_id)

    def _execute_tool(self, name: str, args: dict) -> str:
        """
        Fallback execution - in production these are intercepted locally.
        Returns a description of what would happen.
        """
        angle = args.get("angle", "default")
        if name.startswith("dance_"):
            return f"执行舞蹈动作: {name}"
        elif name == "reset_pose":
            return "机器人已复位到初始姿态"
        else:
            return f"执行动作: {name}, 角度: {angle}°"

    def _send_result(self, req_id, result):
        response = json.dumps({
            "jsonrpc": "2.0",
            "id": req_id,
            "result": result
        })
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(response.encode())

    def _send_error(self, code, message, req_id=None):
        response = json.dumps({
            "jsonrpc": "2.0",
            "id": req_id,
            "error": {"code": code, "message": message}
        })
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(response.encode())

    def log_message(self, format, *args):
        """Suppress default logging."""
        pass


def main():
    parser = argparse.ArgumentParser(
        description="Robot Motion MCP Server for Reachy Mini")
    parser.add_argument("--port", type=int, default=8100,
                        help="HTTP port (default: 8100)")
    args = parser.parse_args()

    server = HTTPServer(("0.0.0.0", args.port), MCPHandler)
    print(f"[robot_motion] MCP server listening on port {args.port}")
    print(f"[robot_motion] {len(TOOLS)} tools registered")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[robot_motion] Shutting down")
        server.shutdown()


if __name__ == "__main__":
    main()
