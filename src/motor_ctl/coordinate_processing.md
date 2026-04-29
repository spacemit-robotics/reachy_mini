# 人脸跟随坐标处理流程说明

## 概述

本文档详细说明视觉检测获取的人脸坐标如何进入运动控制线程，并经过多级处理最终转换为电机控制指令的完整流程。

---

## 整体架构

- **视觉检测线程**（主线程）：~20Hz，检测人脸并计算归一化坐标
- **运动控制线程**：50Hz，读取坐标并执行多级控制算法
- **线程通信**：通过 `FaceCoordinate` 结构体实现线程安全的坐标共享

---

## 1. 视觉检测阶段（主线程）

### 1.1 人脸检测与选择
```cpp
// 选择最佳人脸（面积最大 + 置信度最高的加权组合）
int best_idx = select_best_face(results);
```

### 1.2 归一化坐标计算
```cpp
// 计算人脸中心点
float cx = (results[best_idx].x1 + results[best_idx].x2) / 2.0f;
float cy = (results[best_idx].y1 + results[best_idx].y2) / 2.0f;

// 归一化到 [-1, 1] 范围
float u = (cx / frame.cols) * 2.0f - 1.0f;  // 水平偏差
float v = (cy / frame.rows) * 2.0f - 1.0f;  // 垂直偏差
```

**坐标系定义**：
- `u = 0, v = 0`：人脸在画面中心
- `u = -1`：人脸在画面最左侧
- `u = +1`：人脸在画面最右侧
- `v = -1`：人脸在画面最上方
- `v = +1`：人脸在画面最下方

### 1.3 线程安全更新
```cpp
// 更新共享坐标（运动控制线程会读取）
face_coord->update(u, v);
```

---

## 2. 坐标共享机制

### 2.1 FaceCoordinate 结构体
```cpp
struct FaceCoordinate {
    float u, v;           // 归一化偏差 [-1, 1]
    bool valid;           // 坐标有效性
    uint64_t seq;         // 序列号（防止重复消费）
    std::chrono::steady_clock::time_point last_update;  // 时间戳
    std::chrono::steady_clock::time_point prev_update;  // 上次更新时间
    
    std::mutex mutex;                    // 线程安全
    std::condition_variable cv;          // 条件通知
};
```

### 2.2 关键方法
- **`update(u, v)`**：视觉线程更新坐标，自动递增序列号
- **`peek(u, v, seq)`**：运动控制线程非阻塞读取坐标
- **`get_coord_dt()`**：获取两次坐标更新的时间间隔（用于时间域滤波）
- **`invalidate()`**：标记坐标无效（无人脸时）

---

## 3. 运动控制线程处理流程（50Hz）

### 3.1 坐标读取与消重

**目的**：避免重复处理同一帧坐标

```cpp
float u, v;
uint64_t current_seq;
bool has_coord = face_coord->peek(u, v, current_seq);
bool is_new_coord = has_coord && (current_seq > last_consumed_seq);
```

**原理**：
- 视觉线程 ~20Hz，运动控制线程 50Hz，存在频率差
- 使用序列号 `seq` 判断是否为新坐标
- 只有新坐标才更新 EMA 滤波器，避免用旧数据重复滤波导致响应变慢

**效果**：
- 确保每个视觉坐标只被处理一次
- 运动控制线程在没有新坐标时，继续使用上次滤波结果执行控制

---

### 3.2 死区平滑处理

**目的**：消除中心区域的微小抖动，同时避免死区边界突变

```cpp
auto sigmoid_smooth = [](float error, float zone) {
    float normalized = error / zone;
    float sigmoid = 1.0f / (1.0f + std::exp(-4.0f * (std::abs(normalized) - 1.0f)));
    return error * sigmoid;
};

float u_eff = sigmoid_smooth(u, DEAD_ZONE);  // DEAD_ZONE = 0.055
float v_eff = sigmoid_smooth(v, DEAD_ZONE);
```

**死区作用**：
- 人脸在画面中心 ±5.5% 范围内时，认为已对齐，不再调整
- 避免在目标附近来回微调，造成电机抖动

**Sigmoid 平滑**：
- 传统硬死区在边界处产生阶跃响应，会导致抖动
- Sigmoid 函数实现平滑过渡：
  - 死区内：输出接近 0
  - 死区外：平滑过渡到原始值
  - 边界处：无突变

**参数说明**：
- `DEAD_ZONE = 0.055`：归一化死区半径（5.5%）
- `sigmoid` 函数中的 `4.0`：控制过渡区宽度（值越大过渡越陡）

---

### 3.3 角度转换

**目的**：将归一化像素偏差转换为真实世界的角度偏差

```cpp
float target_delta_yaw = -std::atan(u_eff / FOCAL_X) * 180.0f / M_PI;
float target_delta_pitch = std::atan(v_eff / FOCAL_Y) * 180.0f / M_PI;
```

**相机模型**：
- 基于相机视场角（HFOV=102°, VFOV=67°）
- 归一化焦距：
  - `FOCAL_X = 1/tan(HFOV/2) = 1/tan(51°) ≈ 0.8098`
  - `FOCAL_Y = 1/tan(VFOV/2) = 1/tan(33.5°) ≈ 1.5109`

**透视投影原理**：
```
tan(θ) = 像素偏差 / 焦距
θ = atan(像素偏差 / 焦距)
```

**分辨率无关性**：
- 归一化焦距只与视场角有关，与分辨率无关
- 只要宽高比保持 16:9，适用于任意分辨率（640x480 或 1920x1080）

**符号约定**：
- Yaw（偏航角）：负号表示人脸在右侧时，机器人向右转（正角度）
- Pitch（俯仰角）：正号表示人脸在下方时，机器人向下看（正角度）

---

### 3.4 EMA 时间域滤波

**目的**：平滑噪声，同时保持对真实运动的快速响应

```cpp
// 根据坐标更新间隔动态计算 alpha
float coord_dt = face_coord->get_coord_dt();  // 两次坐标更新的时间间隔
float ema_alpha = 1.0f - std::exp(-coord_dt / EMA_TIME_CONSTANT);
// EMA_TIME_CONSTANT = 0.26s

// 指数移动平均滤波
filtered_error_yaw = ema_alpha * target_delta_yaw + 
                     (1.0f - ema_alpha) * filtered_error_yaw;
filtered_error_pitch = ema_alpha * target_delta_pitch + 
                       (1.0f - ema_alpha) * filtered_error_pitch;
```

**噪声来源**：
- 视觉检测存在像素级抖动（±1-2 像素）
- 直接使用原始坐标会导致电机震荡

**EMA 滤波器**：
- 指数移动平均（Exponential Moving Average）
- 对新数据和历史数据进行加权平均
- `alpha` 控制响应速度：
  - `alpha = 1`：完全跟随新数据（无滤波）
  - `alpha = 0`：完全保持旧值（无响应）

**时间域自适应**：
- 根据实际坐标更新间隔动态调整滤波强度
- 坐标更新快（20ms）→ alpha 大 → 响应快
- 坐标更新慢（100ms）→ alpha 小 → 更平滑
- 公式：`alpha = 1 - exp(-dt / τ)`，其中 `τ = 0.26s` 为时间常数

**时间常数意义**：
- `τ = 0.26s`：约 63% 的阶跃响应时间
- 平衡平滑性和响应速度的折中选择

**坐标更新间隔限制**：
```cpp
// 限制范围：至少 10ms，最多 200ms
if (dt < 0.01f) dt = 0.01f;
if (dt > 0.2f) dt = 0.2f;
```

---

### 3.5 自适应增益策略

**目的**：大误差快速接近，小误差精细调整，避免过冲和震荡

```cpp
// 计算误差幅值
float error_magnitude = std::sqrt(filtered_error_yaw * filtered_error_yaw +
                                 filtered_error_pitch * filtered_error_pitch);

// 基于误差的增益调整（分段线性）
float weight_per_sec;
if (error_magnitude > ERROR_LARGE_THRESHOLD) {
    // 大误差：使用最大增益，快速接近
    weight_per_sec = WEIGHT_MAX;  // 6.0
} else if (error_magnitude > ERROR_SMALL_THRESHOLD) {
    // 中等误差：线性插值
    float ratio = (error_magnitude - ERROR_SMALL_THRESHOLD) /
                  (ERROR_LARGE_THRESHOLD - ERROR_SMALL_THRESHOLD);
    weight_per_sec = WEIGHT_BASE + ratio * (WEIGHT_MAX - WEIGHT_BASE);
} else {
    // 小误差：使用基础增益，避免过冲
    weight_per_sec = WEIGHT_BASE;  // 2.5
}

// 前几帧略微提升响应
if (frame_count_since_face < 3) {
    weight_per_sec *= 1.15f;
}

// 时间域化：转换为单个控制周期的权重
float adaptive_weight = weight_per_sec * control_dt;
```

**分段控制律**：

| 误差范围 | 增益值 | 物理意义 | 应用场景 |
|---------|--------|---------|---------|
| > 15% | 6.0 | 每秒消除 600% 的当前误差 | 人脸在边缘，快速追上 |
| 5%-15% | 2.5-6.0 | 线性插值，平滑过渡 | 接近目标过程 |
| < 5% | 2.5 | 每秒消除 250% 的当前误差 | 目标附近，稳定跟踪 |

**参数说明**：
- `ERROR_LARGE_THRESHOLD = 0.15`：大误差阈值（归一化，15%）
- `ERROR_SMALL_THRESHOLD = 0.05`：小误差阈值（归一化，5%）
- `WEIGHT_BASE = 2.5`：基础权重（保守）
- `WEIGHT_MAX = 6.0`：最大权重（激进）

**物理意义**：
- `weight_per_sec` 表示每秒消除误差的倍数
- 例如：误差 10°，增益 6.0，则每秒修正 60°
- 实际修正量 = `adaptive_weight * filtered_error`

**初始响应加速**：
- 前 3 帧（检测到人脸后）增益提升 15%
- 快速建立初始跟踪，减少启动延迟

---

### 3.6 速度阻尼（防过冲）

**目的**：检测到正在快速接近目标时，主动减速，防止冲过头

```cpp
// 估算当前速度（°/s）
if (control_dt > 0.001f) {
    velocity_yaw = (actual_yaw - prev_yaw) / control_dt;
    velocity_pitch = (actual_pitch - prev_pitch) / control_dt;
}

// 计算目标位置
float new_yaw = actual_yaw + adaptive_weight * filtered_error_yaw;
float new_pitch = actual_pitch + adaptive_weight * filtered_error_pitch;

// 速度阻尼：如果正在快速接近目标，减少修正量（防止过冲）
float error_sign_yaw = (filtered_error_yaw > 0) ? 1.0f : -1.0f;
float error_sign_pitch = (filtered_error_pitch > 0) ? 1.0f : -1.0f;

// 如果速度方向与误差方向相同（正在接近），且速度较大，则施加阻尼
if (velocity_yaw * error_sign_yaw > 20.0f) {  // 速度 > 20°/s 且正在接近
    float damping = 1.0f - VELOCITY_DAMPING * (std::abs(velocity_yaw) / MAX_ANGLE_SPEED);
    damping = std::max(0.3f, damping);  // 至少保留 30% 修正
    new_yaw = actual_yaw + (new_yaw - actual_yaw) * damping;
}
if (velocity_pitch * error_sign_pitch > 20.0f) {
    float damping = 1.0f - VELOCITY_DAMPING * (std::abs(velocity_pitch) / MAX_ANGLE_SPEED);
    damping = std::max(0.3f, damping);
    new_pitch = actual_pitch + (new_pitch - actual_pitch) * damping;
}
```

**过冲问题**：
- 由于系统惯性和延迟，高速接近目标时容易冲过头
- 导致来回摆动（震荡）

**速度检测**：
- 通过 `velocity * error_sign` 判断是否正在接近目标
- 同号（正值）：正在接近
- 异号（负值）：正在远离（不施加阻尼）

**阻尼系数计算**：
```
damping = 1.0 - VELOCITY_DAMPING * (|velocity| / MAX_ANGLE_SPEED)
damping = max(0.3, damping)
```

**阻尼效果**：

| 速度 | 阻尼系数 | 保留修正量 | 效果 |
|------|---------|-----------|------|
| 20°/s | 0.925 | 92.5% | 轻微减速 |
| 40°/s | 0.85 | 85% | 中等减速 |
| 60°/s | 0.775 | 77.5% | 明显减速 |
| 80°/s | 0.7 → 0.3 | 30% | 强力刹车 |

**参数说明**：
- `VELOCITY_DAMPING = 0.3`：速度阻尼系数
- `MAX_ANGLE_SPEED = 80.0`：最大角速度（°/s）
- 阻尼触发阈值：20°/s
- 最小保留修正：30%

**物理类比**：
- 类似汽车刹车系统
- 接近目标时自动减速
- 速度越快，刹车越强

---

### 3.7 速度限制与硬限位

**目的**：保护电机和机械结构，确保安全运行

```cpp
// 速度限制
float max_change = MAX_ANGLE_SPEED * control_dt;  // 80°/s * 0.02s = 1.6°
float yaw_change = new_yaw - current_yaw_deg;
float pitch_change = new_pitch - current_pitch_deg;

if (std::abs(yaw_change) > max_change) {
    yaw_change = (yaw_change > 0) ? max_change : -max_change;
}
if (std::abs(pitch_change) > max_change) {
    pitch_change = (pitch_change > 0) ? max_change : -max_change;
}

current_yaw_deg = actual_yaw + yaw_change;
current_pitch_deg = actual_pitch + pitch_change;

// 硬限位
if (current_yaw_deg > 170.0f) current_yaw_deg = 170.0f;
if (current_yaw_deg < -170.0f) current_yaw_deg = -170.0f;
if (current_pitch_deg > 35.0f) current_pitch_deg = 35.0f;
if (current_pitch_deg < -35.0f) current_pitch_deg = -35.0f;
```

**速度限制（80°/s）**：
- **防止电机过载**：超速运行会导致电机发热、损坏
- **避免机械冲击**：突然加速会对机械结构产生冲击
- **确保运动平滑**：80°/s 是人眼可接受的平滑运动速度
- **单周期限制**：50Hz 控制频率下，单次最大变化 1.6°

**硬限位**：

| 轴 | 范围 | 原因 |
|----|------|------|
| Yaw（偏航） | ±170° | 防止线缆缠绕，保留 20° 安全余量 |
| Pitch（俯仰） | ±35° | 机械结构限制，防止碰撞 |

**安全设计**：
- 速度限制是软约束（可以逐步加速）
- 硬限位是硬约束（绝对不能超过）
- 双重保护确保系统安全

---

### 3.8 电机指令发送

**目的**：将计算好的目标角度发送给电机控制器执行

```cpp
if (motor_controller) {
    motor_controller->set_target(0, current_pitch_deg, current_yaw_deg, 0, 0, 0);
}
```

**参数说明**：
- `set_target(x, y, z, roll, pitch, yaw)`
- 本应用只控制 `pitch` 和 `yaw`
- 其他自由度保持为 0

**异步执行**：
- `motor_controller` 是异步控制器
- `set_target` 不阻塞，立即返回
- 电机控制器内部会进行插值和 PID 控制

**控制频率**：
- 运动控制线程：50Hz
- 电机控制器内部：可能更高频率（如 100Hz）
- 通过插值实现平滑运动

---

## 4. 整体控制策略总结

### 4.1 多级级联控制系统

这是一个**多级级联控制系统**，每一级都有明确的职责：

```
视觉坐标 (u, v)
    ↓
[3.1] 坐标消重 → 确保数据新鲜度
    ↓
[3.2] 死区平滑 → 消除中心抖动
    ↓
[3.3] 角度转换 → 物理建模（像素 → 角度）
    ↓
[3.4] EMA 滤波 → 噪声抑制
    ↓
[3.5] 自适应增益 → 快速收敛 + 稳定跟踪
    ↓
[3.6] 速度阻尼 → 防止过冲
    ↓
[3.7] 限幅保护 → 安全约束
    ↓
[3.8] 电机执行 → 发送指令
```

### 4.2 设计目标

| 目标 | 实现方式 |
|------|---------|
| **快速响应** | 自适应增益（大误差用高增益） |
| **平滑运动** | EMA 滤波 + 速度限制 |
| **稳定跟踪** | 死区 + 小误差低增益 |
| **防止过冲** | 速度阻尼 |
| **系统安全** | 硬限位 + 速度限制 |

### 4.3 关键参数汇总

| 参数 | 值 | 说明 |
|------|-----|------|
| 控制频率 | 50Hz | 运动控制线程循环频率 |
| 死区 | 5.5% | 归一化死区半径 |
| EMA 时间常数 | 0.26s | 滤波器响应时间 |
| 基础增益 | 2.5 | 小误差时的增益 |
| 最大增益 | 6.0 | 大误差时的增益 |
| 速度阻尼系数 | 0.3 | 防过冲阻尼强度 |
| 最大角速度 | 80°/s | 速度限制 |
| Yaw 限位 | ±170° | 偏航角范围 |
| Pitch 限位 | ±35° | 俯仰角范围 |

### 4.4 性能特点

- **响应时间**：约 0.3-0.5 秒（从检测到人脸到基本对齐）
- **稳态精度**：±2-3°（受视觉检测精度限制）
- **运动平滑度**：无明显抖动或震荡
- **鲁棒性**：能处理视觉检测的噪声和延迟

---

## 5. 调试与优化建议

### 5.1 参数调优指南

**如果响应太慢**：
- 增大 `WEIGHT_MAX`（如 6.0 → 8.0）
- 减小 `EMA_TIME_CONSTANT`（如 0.26 → 0.20）
- 增大 `MAX_ANGLE_SPEED`（如 80 → 100）

**如果出现震荡**：
- 减小 `WEIGHT_BASE`（如 2.5 → 2.0）
- 增大 `EMA_TIME_CONSTANT`（如 0.26 → 0.30）
- 增大 `VELOCITY_DAMPING`（如 0.3 → 0.4）
- 增大 `DEAD_ZONE`（如 0.055 → 0.08）

**如果过冲严重**：
- 增大 `VELOCITY_DAMPING`（如 0.3 → 0.5）
- 降低阻尼触发阈值（如 20°/s → 15°/s）
- 减小 `WEIGHT_MAX`（如 6.0 → 5.0）

### 5.2 调试输出建议

在运动控制循环中添加调试输出：

```cpp
if (frame_count % 25 == 0) {  // 每 0.5 秒输出一次
    std::printf("u=%.3f v=%.3f | err_yaw=%.2f err_pitch=%.2f | "
                "vel_yaw=%.1f vel_pitch=%.1f | weight=%.2f\n",
                u, v, filtered_error_yaw, filtered_error_pitch,
                velocity_yaw, velocity_pitch, weight_per_sec);
}
```

### 5.3 常见问题排查

**问题：电机不动**
- 检查 `face_coord->valid` 是否为 true
- 检查误差是否在死区内
- 检查电机控制器是否正常初始化

**问题：运动不平滑**
- 检查视觉检测帧率是否稳定
- 增大 EMA 时间常数
- 检查电机控制器内部 PID 参数

**问题：跟踪延迟大**
- 减小 EMA 时间常数
- 增大自适应增益
- 检查视觉检测延迟

---

## 6. 相关代码位置

- **坐标共享结构**：第 82-130 行
- **运动控制线程**：第 289-424 行
- **视觉检测与坐标更新**：第 638-656 行
- **参数定义**：第 25-46 行

---

## 附录：数学公式推导

### A.1 归一化焦距计算

给定相机水平视场角 HFOV：

```
tan(HFOV/2) = (width/2) / focal_length
focal_length = (width/2) / tan(HFOV/2)
```

归一化焦距（相对于半宽）：

```
focal_x_normalized = focal_length / (width/2) = 1 / tan(HFOV/2)
```

对于 HFOV = 102°：

```
focal_x = 1 / tan(51°) ≈ 0.8098
```

### A.2 EMA 滤波器时间域化

标准 EMA 公式（基于样本数）：

```
y[n] = α * x[n] + (1 - α) * y[n-1]
```

时间域化（基于时间间隔）：

```
α = 1 - exp(-Δt / τ)
```

其中：
- `Δt`：采样间隔
- `τ`：时间常数（63% 响应时间）

### A.3 速度阻尼公式

阻尼系数：

```
damping = 1 - k_d * (|v| / v_max)
damping = max(damping_min, damping)
```

修正量：

```
Δθ_damped = Δθ * damping
```

其中：
- `k_d = 0.3`：阻尼系数
- `v_max = 80°/s`：最大速度
- `damping_min = 0.3`：最小阻尼（保留 30% 修正）

---

**文档版本**：v1.0  
**最后更新**：2026-04-09  
**作者**：SpacemiT Robotics Team
