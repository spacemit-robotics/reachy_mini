/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dance_player.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "audio/audio_player.h"

/* ================================================================
 * Built-in routine table
 * ================================================================ */

const DanceRoutine g_dance_routines[] = {
    {"headbanger", "1_rock.wav", dance_move_headbanger, 120.0f, 4.0f, 100.0f, 2, 4},
    {"jackson", "2_jackson.wav", dance_move_jackson_square, 120.0f, 10.0f, 100.0f, 1, 4},
    {"chicken", "3_checken.wav", dance_move_chicken_peck, 120.0f, 4.0f, 120.0f, 4, 4},
    {"uh_huh_tilt", "4_Slow_Tilting_Light.wav", dance_move_uh_huh_tilt, 100.0f, 4.0f, 80.0f, 2, 4},
};

const int g_dance_routine_count = (int)(sizeof(g_dance_routines) / sizeof(g_dance_routines[0]));

/* ================================================================
 * Core execute
 * ================================================================ */

int dance_player_execute(const DanceRoutine *routine, AsyncMotorController *ctrl,
                            const DanceAudioConfig *audio_cfg) {
    if (!routine || !routine->move_func || !ctrl)
        return -1;

    /* --- compute cycles from audio duration --- */
    int cycles = routine->default_cycles;
    float duration = -1.0f;

    /* --- resolve full audio path --- */
    char full_path[512] = {0};
    const char *audio_path = routine->wav_path;

#ifdef DANCE_AUDIO_DIR
    if (routine->wav_path && routine->wav_path[0] != '/') {
        int n = snprintf(full_path, sizeof(full_path), "%s/%s", DANCE_AUDIO_DIR, routine->wav_path);
        if (n >= (int)sizeof(full_path)) {
            fprintf(stderr, "[DancePlayer] Error: Audio path too long\n");
            return -1;
        }
        audio_path = full_path;
    }
#endif

    if (audio_path) {
        duration = audio_get_duration(audio_path);
        if (duration > 0.0f) {
            cycles = (int)(duration * routine->bpm / (60.0f * routine->beats_per_cycle) + 0.5f);
            if (cycles < 1)
                cycles = 1;
            if (cycles > routine->max_cycles)
                cycles = routine->max_cycles;
        }
    }

    printf("[DancePlayer] %s — %.2fs, %d cycles, %.0f BPM, speed %.0f\n", routine->name, duration,
            cycles, routine->bpm, routine->speed_limit);
    if (full_path[0] != '\0') {
        printf("[DancePlayer] Full path: %s\n", full_path);
    }

    /* --- start audio (async, returns immediately) --- */
    if (audio_path && audio_cfg) {
        audio_play_async_full(audio_path, audio_cfg->capture_rate, audio_cfg->capture_channels,
                            audio_cfg->playback_rate, audio_cfg->playback_channels,
                            audio_cfg->output_device);
    }

    /* --- run dance (blocking) --- */
    async_motor_controller_set_speed_limit(ctrl, routine->speed_limit);
    routine->move_func(ctrl, cycles, routine->bpm);

    /* --- stop audio --- */
    audio_stop();

    return 0;
}

/* ================================================================
 * Lookup by name
 * ================================================================ */

const DanceRoutine *dance_player_find(const DanceRoutine *routines, int count, const char *name) {
    if (!routines || !name)
        return NULL;
    for (int i = 0; i < count; i++) {
        if (strcasecmp(routines[i].name, name) == 0)
            return &routines[i];
    }
    return NULL;
}
