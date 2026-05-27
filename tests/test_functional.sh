#!/bin/bash
# tests/test_functional.sh
# Reachy Mini 模块功能性自动化测试 (functional + pr)

# ==========================================
# 1. 基础测试框架定义
# ==========================================
TOTAL_TESTS=0
FAILED_TESTS=0
TEST_SIM_SERVER_PID=""

# 日志输出格式
function log_info() { echo -e "\033[32m[INFO] $1\033[0m"; }
function log_err() { echo -e "\033[31m[ERROR] $1\033[0m"; }

# 断言：检查退出码
function assert_exit_code() {
    local expected=$1
    local actual=$2
    local msg=$3
    if [ "$expected" != "$actual" ]; then
        log_err "FAIL: $msg (Expected: $expected, Actual: $actual)"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    fi
    return 0
}

# 断言：检查输出是否包含某字符串
function assert_contains() {
    local output="$1"
    local keyword="$2"
    local msg="$3"
    if ! echo "$output" | grep -q -E "$keyword"; then
        log_err "FAIL: $msg"
        log_err "未找到关键字: '$keyword'。 实际输出:\n$output"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    fi
    return 0
}

# 资源清理 (trap)
# shellcheck disable=SC2317,SC2329
function cleanup() {
    log_info "执行资源清理..."
    rm -f /tmp/test_bad_config.yaml
    if [ -n "$TEST_SIM_SERVER_PID" ] && kill -0 "$TEST_SIM_SERVER_PID" 2>/dev/null; then
        kill -9 "$TEST_SIM_SERVER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# ==========================================
# 2. 方案实现
# ==========================================

function test_scenario_1_invalid_device() {
    ((TOTAL_TESTS++))
    log_info "--- 开始执行方案1: 非法外设注入验证 ---"
    
    # 程序容错性设计为非法串口下也会继续运行（不启动电机），因此使用 timeout 控制
    local output
    output=$(timeout 3 reachy_voice_bot --motor-port /dev/not_exist_mock_port 2>&1)
    
    # 检索日志是否有找不到、打不开串口的信息
    if echo "$output" | grep -q -E "Failed to open port|No such file or directory|NotFound|电机初始化失败|无效外设"; then
        log_info "成功验证非法串口，不启动电机"
        return 0
    else
        log_err "FAIL: 注入非法串口时，未在日志中检索到相关报错信息。"
        log_err "部分日志输出: \n$(echo "$output" | head -n 10)"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    fi
}

function test_scenario_2_simulation_loop() {
    ((TOTAL_TESTS++))
    log_info "--- 开始执行方案2: 仿真端与客户端 gRPC 闭环验证 ---"

    # 检查是否为 x86_64 的 ubuntu 系统
    if [ "$(uname -m)" != "x86_64" ] || ! grep -iq "ubuntu" /etc/os-release 2>/dev/null; then
        log_info "当前系统非 x86_64 Ubuntu (架构: $(uname -m))，跳过仿真验证。"
        return 0
    fi

    # 后台启动仿真服务端 (绑定一个测试用的非默认端口避免冲突)
    sim_server 127.0.0.1:50052 >/dev/null 2>&1 &
    local server_pid=$!
    TEST_SIM_SERVER_PID=$server_pid
    
    # 给服务端 2 秒启动时间
    sleep 2

    # 通过管道向 control_client 发送动作指令 '1' 和退出指令 'ESC(\x1b)'
    # 模拟真实用户的键盘操控
    local output
    output=$(echo -e "1\n\x1b" | control_client 127.0.0.1:50052 2>&1)
    local exit_code=$?

    # 验证：1. 客户端是否能正常连接并由于 ESC 正常退出(0)
    assert_exit_code 0 "$exit_code" "客户端收到 ESC 后应该正常退出(返回码0)"
    
    # 验证：2. 日志中包含连接成功的标志
    assert_contains "$output" "Connecting to" "客户端日志应显示尝试连接 gRPC 服务端"
    
    # 测试结束后杀死后台的服务端
    kill -9 $server_pid 2>/dev/null
}

function test_scenario_3_bad_config() {
    ((TOTAL_TESTS++))
    log_info "--- 开始执行方案3: 配置文件解析异常验证 ---"

    # 生成一个缺失必填字段、语法残缺的坏配置
    cat <<EOF > /tmp/test_bad_config.yaml
llm:
  url: 
motor:
  port: /dev/ttyACM0
invalid_yaml_field: [
EOF

    local output
    output=$(reachy_voice_bot --config /tmp/test_bad_config.yaml 2>&1)
    local exit_code=$?

    # 验证：使用错误配置启动，程序应该拒绝执行
    assert_exit_code 1 "$exit_code" "解析损坏配置时，程序应返回非 0"
}

function test_scenario_4_status_query() {
    ((TOTAL_TESTS++))
    log_info "--- 开始执行方案4: 状态查询接口验证 ---"

    local check_status
    check_status=$(reachy_voice_bot status 2>&1)
    if echo "$check_status" | grep -q "运行中"; then
        log_info "已有 reachy_voice_bot 在运行，跳过状态(未知)查询测试，以免干扰已有业务。"
        return 0
    fi
    
    local output
    output=$(reachy_voice_bot status 2>&1)
    local exit_code=$?

    # 这里的退出码取决于您的 status 实现，如果没有运行可能返回 0 也可能返回 1
    # 重点验证输出中是否包含预期的“未知”或“未运行”信息
    assert_contains "$output" "未知|已停止" "未启动进程时查询状态，应返回'未知'或类似提示"
}

# ==========================================
# 3. 运行主函数
# ==========================================
test_scenario_1_invalid_device
test_scenario_2_simulation_loop
test_scenario_3_bad_config
test_scenario_4_status_query

echo "=========================================="
if [ $FAILED_TESTS -eq 0 ]; then
    log_info "所有的 $TOTAL_TESTS 个功能性测试全部通过！"
    exit 0
else
    log_err "测试失败！ 共计 $TOTAL_TESTS 个用例，其中 $FAILED_TESTS 个未通过。"
    exit 1
fi
