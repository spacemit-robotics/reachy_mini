/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tracker_manager.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>

typedef struct {
    TrackerType type;
    const char *process_name;
    const char *config_file;
    const char *display_name;
    pid_t pid;
    bool launched;
    bool paused;
} TrackerInstance;

static TrackerInstance g_trackers[TRACKER_TYPE_MAX] = {
    [TRACKER_TYPE_FACE] = {
        .type = TRACKER_TYPE_FACE,
        .process_name = "face_tracker",
        .config_file = "yolov5-face.yaml",
        .display_name = "人脸跟随",
        .pid = -1,
        .launched = false,
        .paused = false
    },
    [TRACKER_TYPE_GESTURE] = {
        .type = TRACKER_TYPE_GESTURE,
        .process_name = "gesture_tracker",
        .config_file = "yolov5_gesture.yaml",
        .display_name = "手势跟随",
        .pid = -1,
        .launched = false,
        .paused = false
    }
};

static int g_camera_id = 0;
static AsyncMotorController *g_motor_ctrl = NULL;

int tracker_manager_init(int camera_id, AsyncMotorController *motor_ctrl) {
    g_camera_id = camera_id;
    g_motor_ctrl = motor_ctrl;
    printf("[TrackerManager] 模块初始化完成 (Camera: %d)\n", camera_id);
    return 0;
}

int tracker_start(TrackerType type) {
    if (type >= TRACKER_TYPE_MAX) return -1;
    TrackerInstance *inst = &g_trackers[type];

    if (inst->launched && tracker_is_running(type)) {
        printf("[TrackerManager] %s 已经在运行中\n", inst->display_name);
        return 0;
    }

    // 1. 互斥逻辑：停止其他正在运行的 tracker
    for (int i = 0; i < TRACKER_TYPE_MAX; i++) {
        if (i != type && g_trackers[i].launched) {
            tracker_stop(i);
        }
    }

    // 2. 释放电机控制权
    if (g_motor_ctrl) {
        printf("[TrackerManager] 暂停 voice_bot 电机控制线程...\n");
        async_motor_controller_stop(g_motor_ctrl);
        usleep(200000);
    }

    // 3. 启动子进程
    printf("[TrackerManager] 正在启动 %s (%s)...\n", inst->display_name, inst->process_name);
    inst->pid = fork();
    if (inst->pid == 0) {
        // 子进程：关闭从父进程继承的文件描述符 (串口、音频设备等)
        // 防止子进程持有 /dev/ttyACM0 等设备 FD 导致资源冲突
        for (int fd = 3; fd < 1024; ++fd) close(fd);

        // 子进程：执行可执行文件
        char camera_arg[16];
        snprintf(camera_arg, sizeof(camera_arg), "%d", g_camera_id);

        // 使用 --config 选项传递配置文件，避免位置参数在 POSIXLY_CORRECT 下
        // 导致 getopt_long 提前停止解析后续选项
        execlp(inst->process_name, inst->process_name,
            "--config", inst->config_file,
            "--control", "--camera-id", camera_arg,
            "--release-flag", "-1", (char *)NULL);

        // 如果 exec 失败
        fprintf(stderr, "[TrackerManager] Error: 无法执行 %s: %s\n", inst->process_name, strerror(errno));
        _exit(1);
    } else if (inst->pid < 0) {
        fprintf(stderr, "[TrackerManager] Error: fork 失败\n");
        return -1;
    }

    inst->launched = true;
    inst->paused = false;
    printf("[TrackerManager] %s 已启动，PID: %d\n", inst->display_name, inst->pid);

    return 0;
}

int tracker_stop(TrackerType type) {
    if (type >= TRACKER_TYPE_MAX) return -1;
    TrackerInstance *inst = &g_trackers[type];

    if (!inst->launched) return 0;

    printf("[TrackerManager] 正在停止 %s (PID: %d)...\n", inst->display_name, inst->pid);

    // 1. 发送 SIGTERM
    kill(inst->pid, SIGTERM);

    // 2. 等待退出并回收资源 (避免僵尸进程)
    int status;
    bool exited = false;
    for (int i = 0; i < 10; i++) {
        pid_t result = waitpid(inst->pid, &status, WNOHANG);
        if (result == inst->pid || (result == -1 && errno == ECHILD)) {
            exited = true;
            break;
        }
        usleep(200000);
    }

    // 3. 如果还没退，强制杀死
    if (!exited) {
        printf("[TrackerManager] %s 未响应 SIGTERM，发送 SIGKILL...\n", inst->display_name);
        kill(inst->pid, SIGKILL);
        waitpid(inst->pid, &status, 0);
    }

    inst->pid = -1;
    inst->launched = false;
    inst->paused = false;

    // 4. 恢复电机控制权
    if (g_motor_ctrl) {
        printf("[TrackerManager] 恢复 voice_bot 电机控制线程...\n");
        async_motor_controller_start(g_motor_ctrl);
        // 回正
        async_motor_controller_set_target(g_motor_ctrl, 0, 0, 0, 0, 0, 0);
    }

    return 0;
}

void tracker_stop_all(void) {
    for (int i = 0; i < TRACKER_TYPE_MAX; i++) {
        if (g_trackers[i].launched) {
            tracker_stop(i);
        }
    }
}

void tracker_pause_all(void) {
    for (int i = 0; i < TRACKER_TYPE_MAX; i++) {
        if (g_trackers[i].launched && !g_trackers[i].paused) {
            if (g_trackers[i].pid > 0 && kill(g_trackers[i].pid, 0) == 0) {
                kill(g_trackers[i].pid, SIGUSR1);
                g_trackers[i].paused = true;
                printf("[TrackerManager] 已暂停 %s (释放 NPU)\n", g_trackers[i].display_name);
            }
        }
    }
}

void tracker_resume_all(void) {
    for (int i = 0; i < TRACKER_TYPE_MAX; i++) {
        if (g_trackers[i].launched && g_trackers[i].paused) {
            if (g_trackers[i].pid > 0 && kill(g_trackers[i].pid, 0) == 0) {
                kill(g_trackers[i].pid, SIGUSR2);
                g_trackers[i].paused = false;
                printf("[TrackerManager] 已恢复 %s\n", g_trackers[i].display_name);
            }
        }
    }
}

bool tracker_is_running(TrackerType type) {
    if (type >= TRACKER_TYPE_MAX) return false;
    TrackerInstance *inst = &g_trackers[type];

    if (!inst->launched) return false;

    if (inst->pid > 0) {
        // 尝试非阻塞回收僵尸进程，防止 kill(pid,0) 对 zombie 返回 0
        // 导致 NPU 逻辑围栏永久拦截语音指令
        int status;
        pid_t result = waitpid(inst->pid, &status, WNOHANG);
        if (result == inst->pid || (result == -1 && errno == ECHILD)) {
            // 子进程已退出 (正常退出或崩溃)，清理状态
            printf("[TrackerManager] %s (PID: %d) 已退出，清理状态\n",
                inst->display_name, inst->pid);
            inst->launched = false;
            inst->pid = -1;
            inst->paused = false;
            // 恢复电机控制权
            if (g_motor_ctrl) {
                async_motor_controller_start(g_motor_ctrl);
                async_motor_controller_set_target(g_motor_ctrl, 0, 0, 0, 0, 0, 0);
            }
            return false;
        }
        // 进程仍在运行
        if (kill(inst->pid, 0) == 0) {
            return true;
        }
    }

    // 进程已不在，清理状态
    inst->launched = false;
    inst->pid = -1;
    return false;
}
