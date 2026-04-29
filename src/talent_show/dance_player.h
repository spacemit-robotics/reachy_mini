/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef DANCE_PLAYER_H
#define DANCE_PLAYER_H

#include "dance_interface.h"
#include "motor_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*DanceMoveFunc)(AsyncMotorController *ctrl, int cycles, float bpm);

typedef struct {
    const char *name;          /* display name */
    const char *wav_path;      /* audio file path */
    DanceMoveFunc move_func;   /* dance motion function */
    float bpm;                 /* beats per minute */
    float beats_per_cycle;     /* beats consumed per cycle */
    float speed_limit;         /* motor speed limit (deg/s) */
    int default_cycles;        /* fallback when audio duration unknown */
    int max_cycles;            /* upper clamp */
} DanceRoutine;

typedef struct {
    int input_device;
    int output_device;
    int capture_rate;
    int capture_channels;
    int playback_rate;
    int playback_channels;
} DanceAudioConfig;

/**
 * @brief Execute a dance routine with synchronized audio playback.
 *
 * Computes cycle count from audio duration, starts async playback,
 * runs the dance motion (blocking), then stops audio.
 *
 * @return 0 on success, -1 on error
 */
int dance_player_execute(const DanceRoutine *routine,
                        AsyncMotorController *ctrl,
                        const DanceAudioConfig *audio_cfg);

/**
 * @brief Look up a routine by name (case-insensitive).
 *
 * @param routines  Array of routines
 * @param count     Number of entries
 * @param name      Name to search
 * @return Pointer to matching entry, or NULL
 */
const DanceRoutine *dance_player_find(const DanceRoutine *routines,
                                        int count,
                                        const char *name);

/**
 * @brief Built-in routine table and its size.
 */
extern const DanceRoutine g_dance_routines[];
extern const int g_dance_routine_count;

#ifdef __cplusplus
}
#endif

#endif /* DANCE_PLAYER_H */
