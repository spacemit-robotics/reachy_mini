#include <stdio.h>
#include <unistd.h>

#include "motion_api.h"
#include "motor.h"

#define MOTOR_COUNT 9
#define BAUDRATE 1000000
#define DEFAULT_PORT "/dev/ttyACM0"

int main(int argc, char *argv[]) {
    const char *port = DEFAULT_PORT;
    if (argc > 1) {
    port = argv[1];
    }

    struct motor_dev *devs[MOTOR_COUNT];
    uint8_t motor_ids[MOTOR_COUNT] = {10, 11, 12, 13, 14, 15, 16, 17, 18};

    printf("[示例] 初始化电机 (端口: %s)...\n", port);

    /* 1. 分配并初始化电机 (由于 API 不再包含封装，需在此手动执行) */
    for (int i = 0; i < MOTOR_COUNT; i++) {
    devs[i] =
        motor_alloc_uart("drv_uart_rm", port, BAUDRATE, motor_ids[i], NULL);
    if (!devs[i]) {
        fprintf(stderr, "错误: 无法分配电机 ID %d\n", motor_ids[i]);
        motor_free(devs, i);
        return -1;
    }
    }

    if (motor_init(devs, MOTOR_COUNT) != 0) {
    fprintf(stderr, "错误: 初始化电机失败\n");
    motor_free(devs, MOTOR_COUNT);
    return -1;
    }

    printf("[示例] 初始化成功。开始执行动作序列...\n");

    // 1. 头部动作示范
    printf("- 头部向左看 30 度\n");
    head_turn_left(devs, 30.0f);
    sleep(1);

    printf("- 头部向右看 30 度\n");
    head_turn_right(devs, 30.0f);
    sleep(1);

    printf("- 抬头 20 度\n");
    head_look_up(devs, 20.0f);
    sleep(1);

    printf("- 返回中心位姿\n");
    center_all(devs);
    sleep(1);

    // 2. 身体动作示范
    printf("- 身体向左转 30 度\n");
    body_turn_left(devs, 30.0f);
    sleep(1);

    printf("- 身体向右转 30 度\n");
    body_turn_right(devs, 30.0f);
    sleep(1);

    // 3. 天线动作示范
    printf("- 天线向前转动\n");
    antenna_turn_forward(devs, 45.0f, 45.0f);
    sleep(1);

    printf("- 所有关节归位\n");
    center_all(devs);
    sleep(1);

    /* 2. 清理资源 */
    printf("[示例] 动作结束，释放资源。\n");
    motor_free(devs, MOTOR_COUNT);

    return 0;
}
