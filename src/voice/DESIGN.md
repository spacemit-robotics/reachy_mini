# 语音控制运动方案设计

## 1. 目标

新增共享库 `libvoice_ctl.so` 及独立可执行程序 `voice_ctl`，通过回调机制接收 ASR 识别文本，匹配关键词后调用 `motion_api.h` 控制电机。

对 `voice_chat.cpp` 做最小改动（约 5 行）：仅暴露一个 ASR 文本回调注册接口。所有电机初始化、关键词匹配、运动控制逻辑全部在 `voice_ctl.c` 中实现，voice_chat 不感知运动控制的存在。

## 2. 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                    voice_chat  (omni_agent)               │
│                                                          │
│  麦克风 → VAD → ASR → text ──┬──→ processText → LLM → TTS│
│                              │                           │
│                              │  if (g_on_asr_text)       │
│                              └──→ g_on_asr_text(text)    │
│                                        │  (回调)         │
└────────────────────────────────────────│─────────────────┘
                                         │
              voice_ctl_init() 时注册回调  │
              voice_chat_set_asr_callback │
                                         │
                              ┌──────────▼─────────────┐
                              │  libvoice_ctl.so        │
                              │  (reachy-mini, 新增)    │
                              │                        │
                              │  voice_ctl_on_text()   │
                              │    ↓                   │
                              │  关键词匹配             │
                              │    ↓                   │
                              │  motion_api.h          │
                              │    ↓                   │
                              │  电机控制               │
                              └────────────────────────┘
```

核心原则：voice_chat 只负责"把文本交出去"，voice_ctl 负责"拿到文本后做什么"。

运行方式：

```bash
# 模式 A：voice_chat 加载 voice_ctl 插件 (推荐)
voice_chat --llm-url http://localhost:8080 --tts matcha:zh --motor-port /dev/ttyACM0

# voice_chat 启动时:
#   dlopen("libvoice_ctl.so") → voice_ctl_init("/dev/ttyACM0", 25.0)
#   voice_ctl_init 内部调用 voice_chat_set_asr_callback() 注册自己的回调
# ASR 识别到 "向左转":
#   voice_chat 调用 g_on_asr_text("向左转")
#   → voice_ctl 内部: 关键词匹配 → head_turn_left(25°) ✓
#   processText() → LLM 对话照常进行
# 退出时:
#   voice_ctl_cleanup()

# 模式 B：独立调试 (手动输入文本)
voice_ctl --port /dev/ttyACM0
> 向左转
[voice_ctl] 执行: head_turn_left(25.0°)
> 抬头
[voice_ctl] 执行: head_look_up(25.0°)
```

## 3. 接口衔接分析

### 3.1 语音侧（最小改动 — 仅暴露回调钩子）

`voice_chat.cpp` 新增一个全局函数指针和注册函数（约 5 行），ASR 识别完成处加一行调用。voice_chat 完全不知道运动控制的存在，只是"有文本就通知一下"。

voice_chat.cpp 新增的全部代码：

```cpp
// --- 文件顶部 (全局区域) ---
// ASR 文本回调钩子 (供外部插件注册)
static void (*g_on_asr_text)(const char *text) = nullptr;
extern "C" void voice_chat_set_asr_callback(void (*cb)(const char *)) {
    g_on_asr_text = cb;
}

// --- ASR 识别完成处 (第 606 行之后) ---
if (g_on_asr_text) g_on_asr_text(text.c_str());
```

就这些。voice_chat 不 `#include` 任何 voice_ctl 头文件，不调用任何电机函数，不解析 `--motor-port`。

### 3.2 运动侧（已有）

`motion_api.h` 提供完整的语义化 C 接口：

| 函数 | 功能 | 限位 |
|------|------|------|
| `head_turn_left(devs, angle)` | 头部左转 | ±45° |
| `head_turn_right(devs, angle)` | 头部右转 | ±45° |
| `head_look_up(devs, angle)` | 抬头 | ±35° |
| `head_look_down(devs, angle)` | 低头 | ±35° |
| `head_tilt_left(devs, angle)` | 头部左歪 | ±25° |
| `head_tilt_right(devs, angle)` | 头部右歪 | ±25° |
| `body_turn_left(devs, angle)` | 身体左转 | ±180° |
| `body_turn_right(devs, angle)` | 身体右转 | ±180° |
| `center_all(devs)` | 全部回中 | — |

## 4. 详细设计

### 4.1 新增文件

```
reachy-mini/src/voice/
├── DESIGN.md        # 本文档
├── voice_ctl.c      # 库实现 (初始化 + 指令匹配 + 电机控制)
├── voice_ctl.h      # 导出 C API (dlopen 接口)
└── voice_ctl_main.c # 独立可执行程序入口 (stdin 交互调试)
```

### 4.2 voice_ctl.h — 导出 API

```c
#ifndef VOICE_CTL_H
#define VOICE_CTL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * voice_chat 暴露的回调注册函数 (由 voice_chat.cpp 定义, voice_ctl 调用)
 * voice_ctl 在 init 时通过 dlsym 获取此符号并调用，注册自己的文本处理回调。
 */
typedef void (*asr_text_callback_t)(const char *text);
extern void voice_chat_set_asr_callback(asr_text_callback_t cb);

/**
 * 初始化语音运动控制
 * @param serial_port  电机串口路径 (如 "/dev/ttyACM0")
 * @param default_angle 默认动作角度 (度, 如 25.0)
 * @return 0=成功, <0=失败
 *
 * 内部完成:
 *   1. 电机初始化 (motor_alloc_uart × 9 → motor_init)
 *   2. 通过 dlsym 获取 voice_chat_set_asr_callback 并注册回调
 *      (独立运行模式下跳过此步)
 */
int voice_ctl_init(const char *serial_port, float default_angle);

/**
 * 将 ASR 文本匹配为运动指令并执行
 * @param text  ASR 识别文本
 * @return 1=匹配并执行, 0=未匹配 (非运动指令)
 *
 * 作为回调被 voice_chat 间接调用，也可在独立模式下直接调用。
 */
int voice_ctl_dispatch(const char *text);

/**
 * 清理资源 (回中 + 释放电机)
 */
void voice_ctl_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* VOICE_CTL_H */
```

### 4.3 指令匹配策略

采用关键词表驱动，支持多个同义词映射到同一动作：

```c
typedef int (*MotionFunc)(struct motor_dev **devs, float angle);

typedef struct {
    const char *keywords[8];   // 同义词列表 (NULL 结尾)
    MotionFunc action;         // 对应的 motion_api 函数
    float angle_override;      // >0 时覆盖默认角度
    const char *description;   // 日志描述
} VoiceCommand;

static const VoiceCommand CMD_TABLE[] = {
    {{"向左转", "左转", "往左看", "看左边", NULL},
     head_turn_left, 0, "head_turn_left"},

    {{"向右转", "右转", "往右看", "看右边", NULL},
     head_turn_right, 0, "head_turn_right"},

    {{"抬头", "向上看", "往上看", "看上面", NULL},
     head_look_up, 0, "head_look_up"},

    {{"低头", "向下看", "往下看", "看下面", NULL},
     head_look_down, 0, "head_look_down"},

    {{"左歪头", "歪头", NULL},
     head_tilt_left, 15, "head_tilt_left"},

    {{"右歪头", NULL},
     head_tilt_right, 15, "head_tilt_right"},

    {{"身体左转", "转身向左", NULL},
     body_turn_left, 45, "body_turn_left"},

    {{"身体右转", "转身向右", NULL},
     body_turn_right, 45, "body_turn_right"},

    {{"回正", "回中", "复位", "正前方", NULL},
     center_all_wrap, 0, "center_all"},
};
```

匹配逻辑：遍历 `CMD_TABLE`，对每条记录的 `keywords` 做 `strstr` 子串匹配，首个命中即执行。

### 4.4 voice_ctl.c — 库实现

```c
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
#include "voice_ctl.h"
#include "motion_api.h"

// 模块内部状态 (单例)
static struct motor_dev *g_devs[9];
static float g_default_angle = 25.0f;
static bool g_initialized = false;

// --- 回调包装 (适配 void(*)(const char*) 签名) ---
static void on_asr_text(const char *text) {
    voice_ctl_dispatch(text);
}

int voice_ctl_init(const char *serial_port, float default_angle) {
    if (g_initialized) return 0;

    // 1. 电机初始化
    // motor_alloc_uart × 9 → motor_init ...
    g_default_angle = default_angle;
    g_initialized = true;

    // 2. 尝试注册 ASR 回调 (仅在被 voice_chat 加载时生效)
    //    voice_chat_set_asr_callback 是 voice_chat 进程的导出符号
    //    独立运行时 dlsym 返回 NULL，跳过即可
    typedef void (*set_cb_fn)(void (*)(const char *));
    set_cb_fn set_cb = (set_cb_fn)dlsym(RTLD_DEFAULT,
                                         "voice_chat_set_asr_callback");
    if (set_cb) {
        set_cb(on_asr_text);
        printf("[voice_ctl] ASR 回调已注册\n");
    }

    printf("[voice_ctl] 初始化完成, 串口=%s, 默认角度=%.1f°\n",
           serial_port, default_angle);
    return 0;
}

int voice_ctl_dispatch(const char *text) {
    if (!g_initialized || !text) return 0;
    // 遍历 CMD_TABLE，strstr 匹配 → 调用 motion_api
    // ...
    return 0; // or 1
}

void voice_ctl_cleanup(void) {
    if (!g_initialized) return;
    // 注销回调
    typedef void (*set_cb_fn)(void (*)(const char *));
    set_cb_fn set_cb = (set_cb_fn)dlsym(RTLD_DEFAULT,
                                         "voice_chat_set_asr_callback");
    if (set_cb) set_cb(NULL);

    center_all(g_devs);
    // motor_free × 9
    g_initialized = false;
    printf("[voice_ctl] 已清理\n");
}
```

关键点：`voice_ctl_init()` 内部通过 `dlsym(RTLD_DEFAULT, "voice_chat_set_asr_callback")` 反向查找宿主进程的符号。voice_chat 用 `-rdynamic` 链接即可导出该符号。独立运行时 dlsym 返回 NULL，自然跳过。

### 4.5 voice_ctl_main.c — 独立可执行入口

```c
int main(int argc, char *argv[]) {
    // 1. 解析参数 (--port, --angle, --help)
    // 2. voice_ctl_init(port, angle)
    // 3. 主循环：逐行读取 stdin
    //    a. 调用 voice_ctl_dispatch(line)
    //    b. 匹配成功 → 打印执行日志
    //    c. 匹配失败 → 打印忽略
    // 4. voice_ctl_cleanup()
}
```

命令行参数：

```
Usage: voice_ctl [options]
  --port <path>     电机串口 (默认: /dev/ttyACM0)
  --angle <deg>     默认动作角度 (默认: 25.0)
  --help            显示帮助
```

## 5. voice_chat.cpp 改动（最小侵入）

### 5.1 全部改动（共 5 行新增）

```diff
--- a/application/native/omni_agent/examples/voice_chat.cpp
+++ b/application/native/omni_agent/examples/voice_chat.cpp
@@ (文件顶部, #include 之后, Config 之前)
+// ASR 文本回调钩子 (供外部插件注册, 如 voice_ctl)
+static void (*g_on_asr_text)(const char *text) = nullptr;
+extern "C" void voice_chat_set_asr_callback(void (*cb)(const char *)) {
+    g_on_asr_text = cb;
+}

@@ (ASR 识别完成, "识别完成" 打印之后)
 std::cout << getTimestamp() << " [ASR] 识别完成: \"" << text << "\"\n";
+if (g_on_asr_text) g_on_asr_text(text.c_str());
```

### 5.2 改动说明

| 位置 | 改动 | 行数 |
|------|------|------|
| 文件顶部 | 函数指针 + 注册函数 | 4 行 |
| ASR 识别完成处 | 回调调用 | 1 行 |

总计 5 行新增，0 行修改，0 行删除。

voice_chat 不 `#include` 任何 voice_ctl 头文件，不调用任何电机函数，不解析 `--motor-port`，不 `dlopen`。它只是暴露了一个通用的"ASR 文本通知"钩子，谁注册谁收到。

### 5.3 omni_agent 链接选项

voice_chat 需要用 `-rdynamic` 链接，使 `voice_chat_set_asr_callback` 符号对 dlopen 加载的 .so 可见：

```cmake
# omni_agent/CMakeLists.txt (voice_chat target)
target_link_options(voice_chat PRIVATE -rdynamic)
```

如果 omni_agent 已经链接了 `-ldl`（用于其他 dlopen），则无需额外改动。

### 5.4 加载流程（由 voice_ctl 主导）

voice_chat 本身不负责加载 voice_ctl。加载方式有两种：

方式 A — 启动脚本 `LD_PRELOAD`：
```bash
LD_PRELOAD=libvoice_ctl.so voice_chat --llm-url http://localhost:8080
# libvoice_ctl.so 的 __attribute__((constructor)) 自动调用 voice_ctl_init
```

方式 B — voice_chat 新增 `--motor-port` 参数（可选，约 10 行额外改动）：
```cpp
// parseArgs 新增
} else if (strcmp(argv[i], "--motor-port") == 0 && i + 1 < argc) {
    cfg.motor_port = argv[++i];
}
// main 初始化后
if (!cfg.motor_port.empty()) {
    void *h = dlopen("libvoice_ctl.so", RTLD_NOW);
    if (h) {
        auto init = (int(*)(const char*,float))dlsym(h, "voice_ctl_init");
        if (init) init(cfg.motor_port.c_str(), 25.0f);
    }
}
```

推荐方式 B，用户体验更好。但即使选方式 B，dlopen/init 逻辑也只是"加载并调用 init"，所有运动控制逻辑仍在 voice_ctl 内部。

## 6. 构建集成

### 6.1 reachy-mini/CMakeLists.txt 新增

```cmake
# ============================================================================
# 共享库: libvoice_ctl.so — 语音运动控制 (供 voice_chat dlopen 加载)
# ============================================================================
add_library(voice_ctl SHARED
    src/voice/voice_ctl.c
    ${KINEMATICS_SOURCES}
)
add_dependencies(voice_ctl trigger_motor_build)
set_target_properties(voice_ctl PROPERTIES
    LINKER_LANGUAGE C
    OUTPUT_NAME voice_ctl
    VERSION 1.0.0
    SOVERSION 1
)
target_link_libraries(voice_ctl ${MOTOR_LIBS})

# ============================================================================
# 可执行文件 5: voice_ctl_cli — 语音指令独立调试工具
# ============================================================================
add_executable(voice_ctl_cli
    src/voice/voice_ctl_main.c
    src/voice/voice_ctl.c
    ${KINEMATICS_SOURCES}
)
add_dependencies(voice_ctl_cli trigger_motor_build)
set_target_properties(voice_ctl_cli PROPERTIES
    LINKER_LANGUAGE C
    OUTPUT_NAME voice_ctl
)
target_link_libraries(voice_ctl_cli ${MOTOR_LIBS})

# 更新安装规则
install(TARGETS test_api test_motion face_tracker gesture_tracker voice_ctl voice_ctl_cli
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
```

### 6.2 omni_agent 侧

仅需确保 voice_chat 链接时加 `-rdynamic`，使 `voice_chat_set_asr_callback` 符号可被 dlopen 的 .so 通过 `dlsym(RTLD_DEFAULT, ...)` 找到。

如果选择方式 B（`--motor-port` 参数），还需在 voice_chat 链接 `-ldl`。

### 6.3 依赖关系

```
libvoice_ctl.so (C, 共享库)          voice_ctl (C, 独立可执行)
  ├── voice_ctl.c                      ├── voice_ctl_main.c
  ├── motion_api.c                     ├── voice_ctl.c
  ├── reachy_kinematics.c              ├── motion_api.c
  └── libmotor.a                       ├── reachy_kinematics.c
                                       └── libmotor.a
voice_chat (C++, omni_agent)
  ├── 新增 5 行: g_on_asr_text 回调钩子
  └── -rdynamic 导出 voice_chat_set_asr_callback

libvoice_ctl.so 加载时:
  dlsym(RTLD_DEFAULT, "voice_chat_set_asr_callback")
  → 注册 on_asr_text 回调
  → voice_chat ASR 识别后自动调用

不依赖: vision, OpenCV, LLM, TTS, VAD
```

## 7. 使用场景

### 7.1 voice_chat 内嵌运动控制（推荐）

```bash
# 一条命令启动语音对话 + 运动控制
voice_chat --llm-url http://localhost:8080 --tts matcha:zh \
           --motor-port /dev/ttyACM0

# ASR 识别到 "向左转" →
#   [voice_ctl] 执行: head_turn_left(25.0°) ✓
#   (同时 LLM 对话照常进行)
# ASR 识别到 "今天天气怎么样" →
#   voice_ctl_dispatch 返回 0 (未匹配)
#   (仅 LLM 对话处理)
```

### 7.2 独立调试

```bash
# 手动输入文本测试电机
voice_ctl --port /dev/ttyACM0
> 向左转
[voice_ctl] 匹配: head_turn_left(25.0°) ✓
> 抬头
[voice_ctl] 匹配: head_look_up(25.0°) ✓
> 你好
[voice_ctl] 未匹配运动指令，忽略
> 回正
[voice_ctl] 匹配: center_all() ✓
```

### 7.3 脚本批量控制

```bash
echo -e "向左转\n抬头\n回正" | voice_ctl --port /dev/ttyACM0
```

## 8. 方案优势

| 对比项 | 管道方案 | 前版 dlopen 方案 | 本方案 (回调钩子) |
|--------|---------|-----------------|------------------|
| voice_chat 改动 | 0 行 | ~34 行 | 5 行 (纯通用钩子) |
| voice_chat 感知运动控制 | 否 | 是 (dlopen/init/dispatch/cleanup) | 否 (只暴露回调) |
| 进程数 | 2 个 | 1 个 | 1 个 |
| 延迟 | 管道缓冲 | 函数调用 | 函数调用 |
| 部署 | shell 脚本编排 | `--motor-port` | `--motor-port` 或 `LD_PRELOAD` |
| 运动逻辑位置 | voice_ctl 进程 | voice_chat 调用 dispatch | voice_ctl.so 内部 |
| 可复用性 | 低 | 低 | 高 (回调钩子可供任意插件使用) |

## 9. 扩展性

| 扩展 | 说明 |
|------|------|
| 角度参数提取 | 从文本中解析数字，如"向左转30度" → `head_turn_left(30)` |
| 组合动作 | "点头" → `look_down(15)` + `sleep` + `look_up(15)` + `center_all` |
| 动作速度控制 | "慢慢转头" → 降低 motion_api 内部步进速度 |
| 返回值反馈 | `voice_ctl_dispatch` 返回描述字符串，voice_chat 可注入 LLM 上下文 |
| 连续控制 | "继续" → 重复上一个动作 |
| 热加载 | 运行时 dlclose + dlopen 更新 .so，无需重启 voice_chat |

## 10. 实现优先级

1. `voice_ctl.h` + `voice_ctl.c` — 库 API + 指令匹配 + 回调注册（核心，约 200 行）
2. `voice_chat.cpp` — 新增 5 行回调钩子
3. `voice_ctl_main.c` — 独立调试入口（约 60 行）
4. CMakeLists.txt 新增 libvoice_ctl.so + voice_ctl targets
5. omni_agent CMakeLists.txt 加 `-rdynamic`
6. 独立模式调试验证
7. voice_chat 内嵌模式验证
8. 角度提取、组合动作等扩展
