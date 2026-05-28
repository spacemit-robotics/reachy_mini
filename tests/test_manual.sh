#!/bin/bash
reachy_voice_bot start
echo "reachy_voice_bot started, will run for 120 seconds. Please complete the voice control test within 90 seconds after the welcome prompt plays. "
sleep 120
echo "Checking status..."
reachy_voice_bot status
echo "Waiting for 3 seconds..."
sleep 3
echo "Stopping reachy_voice_bot..."
reachy_voice_bot stop

# 给进程 2 秒时间清理退出
sleep 2

echo "Checking for leaked processes..."
leaked=""

if pgrep "reachy_voice_bot" > /dev/null; then
    leaked="$leaked reachy_voice_bot"
fi

if pgrep "face_tracker" > /dev/null; then
    leaked="$leaked face_tracker"
fi

if pgrep "gesture_tracker" > /dev/null; then
    leaked="$leaked gesture_tracker"
fi

if [ -n "$leaked" ]; then
    echo "!!! 进程泄漏 !!! :$leaked"
    # 可选: 强制清理残留进程，防止影响后续测试
    pkill -9 -f "reachy_voice_bot" 2>/dev/null || true
    pkill -9 -f "face_tracker" 2>/dev/null || true
    pkill -9 -f "gesture_tracker" 2>/dev/null || true
    exit 1
else
    echo "清理成功，无进程泄漏。"
    exit 0
fi