#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <motor.h>

#include "../motor_ctl/motor_controller.h"
#include "audio/audio_player.h"
#include "dance_interface.h"
#include "dance_player.h"

#define MOTOR_COUNT 9
#define BAUDRATE 1000000
#define DEFAULT_PORT "/dev/ttyACM0"

static volatile int keep_running = 1;

typedef struct {
    int input_device;
    int output_device;
    int capture_rate;
    int capture_channels;
    int playback_rate;
    int playback_channels;
} AudioConfig;

static AudioConfig g_audio_config = {-1, -1, 16000, 1, 48000, 1};

void int_handler(int dummy) {
    keep_running = 0;
    dance_request_stop();
}

void print_usage(const char *prog_name) {
    printf("Usage: %s [port] [options]\n", prog_name);
    printf("Options:\n");
    printf("  -i, --input-device <id>     Input device index (default: system default)\n");
    printf("  -o, --output-device <id>    Output device index (default: system default)\n");
    printf("  -l, --list-devices          List available audio devices and exit\n");
    printf("  --capture-rate <hz>         Capture sampling rate (default: 16000)\n");
    printf("  --capture-channels <n>      Capture channels (default: 1)\n");
    printf("  --playback-rate <hz>        Playback sampling rate (default: 48000)\n");
    printf("  --playback-channels <n>     Playback channels (default: 1)\n");
    printf("\n");
}

void print_menu() {
    printf("\n========= Reachy Mini Dance Test =========\n");
    for (int i = 0; i < g_dance_routine_count; i++) {
        printf("%d. %s\n", i + 1, g_dance_routines[i].name);
    }
    printf("r. Reset to Zero    (回到零位)\n");
    printf("q. Quit             (退出)\n");
    printf("==========================================\n");
    printf("Select an action: ");
}

int main(int argc, char *argv[]) {
    const char *port = DEFAULT_PORT;

    static struct option long_options[] = {{"input-device", required_argument, 0, 'i'},
                                            {"output-device", required_argument, 0, 'o'},
                                            {"list-devices", no_argument, 0, 'l'},
                                            {"capture-rate", required_argument, 0, 1001},
                                            {"capture-channels", required_argument, 0, 1002},
                                            {"playback-rate", required_argument, 0, 1003},
                                            {"playback-channels", required_argument, 0, 1004},
                                            {0, 0, 0, 0}};

    int opt;
    int option_index = 0;
    while ((opt = getopt_long(argc, argv, "i:o:l", long_options, &option_index)) != -1) {
        switch (opt) {
        case 'i':
            g_audio_config.input_device = atoi(optarg);
            break;
        case 'o':
            g_audio_config.output_device = atoi(optarg);
            break;
        case 'l':
            audio_list_devices();
            return 0;
        case 1001:
            g_audio_config.capture_rate = atoi(optarg);
            break;
        case 1002:
            g_audio_config.capture_channels = atoi(optarg);
            break;
        case 1003:
            g_audio_config.playback_rate = atoi(optarg);
            break;
        case 1004:
            g_audio_config.playback_channels = atoi(optarg);
            break;
        case '?':
            print_usage(argv[0]);
            return -1;
        default:
            break;
        }
    }

    if (optind < argc) {
        port = argv[optind];
    }

    signal(SIGINT, int_handler);

    printf("[Test] Initializing %d motors on %s...\n", MOTOR_COUNT, port);
    printf("[Test] Audio Config: Capture %dHz/%dch, Playback %dHz/%dch (Device: %d)\n",
            g_audio_config.capture_rate, g_audio_config.capture_channels,
            g_audio_config.playback_rate, g_audio_config.playback_channels,
            g_audio_config.output_device);

    struct motor_dev *devs[MOTOR_COUNT];
    uint8_t motor_ids[MOTOR_COUNT] = {10, 11, 12, 13, 14, 15, 16, 17, 18};

    for (int i = 0; i < MOTOR_COUNT; i++) {
        devs[i] = motor_alloc_uart("drv_uart_xl330", port, BAUDRATE, motor_ids[i], NULL);
        if (!devs[i]) {
            fprintf(stderr, "Failed to alloc motor ID %d\n", motor_ids[i]);
            return -1;
        }
    }

    if (motor_init(devs, MOTOR_COUNT) != 0) {
        fprintf(stderr, "Failed to init motors\n");
        return -1;
    }

    AsyncMotorController *ctrl = async_motor_controller_create(devs, MOTOR_COUNT);
    if (!ctrl) {
        fprintf(stderr, "Failed to create AsyncMotorController\n");
        return -1;
    }

    if (!async_motor_controller_start(ctrl)) {
        fprintf(stderr, "Failed to start AsyncMotorController\n");
        return -1;
    }

    // 设置初始速度限制比较保守
    async_motor_controller_set_speed_limit(ctrl, 30.0f);
    printf("[Test] System ready. Using 30 deg/s for safety.\n");

    char cmd[16];
    DanceAudioConfig dance_audio = {
        g_audio_config.input_device,  g_audio_config.output_device,
        g_audio_config.capture_rate,  g_audio_config.capture_channels,
        g_audio_config.playback_rate, g_audio_config.playback_channels,
    };

    while (keep_running) {
        print_menu();
        if (scanf("%s", cmd) != 1)
            break;

        if (cmd[0] == 'q')
            break;

        if (cmd[0] == 'r') {
            printf("[Test] Resetting to zero...\n");
            async_motor_controller_set_speed_limit(ctrl, 30.0f);
            async_motor_controller_set_target(ctrl, 0, 0, 0, 0, 0, 0);
            sleep(2);
            continue;
        }

        int idx = cmd[0] - '1';
        if (idx >= 0 && idx < g_dance_routine_count) {
            dance_player_execute(&g_dance_routines[idx], ctrl, &dance_audio);
        } else {
            printf("Unknown command: %s\n", cmd);
        }
    }

    printf("[Test] Shutting down...\n");
    audio_stop();
    async_motor_controller_stop(ctrl);
    async_motor_controller_destroy(ctrl);
    motor_free(devs, MOTOR_COUNT);

    return 0;
}
