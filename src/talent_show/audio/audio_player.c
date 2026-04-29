#include "audio_player.h"

#include <alsa/asoundlib.h>
#include <pthread.h>
#include <samplerate.h>
#include <sndfile.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    float *data;
    sf_count_t total_frames;
    sf_count_t current_frame;
    int channels;
} AudioData;

static snd_pcm_t *g_alsa_handle = NULL;
static AudioData g_audio_data = {NULL, 0, 0, 0};
static pthread_mutex_t g_audio_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_audio_thread;
static volatile int g_audio_running = 0;

static void *alsa_playback_thread(void *arg) {
    int channels = g_audio_data.channels;
    int period_size = 1024;
    int16_t *buffer = (int16_t *)malloc(sizeof(int16_t) * period_size * channels);
    if (!buffer)
        return NULL;

    while (g_audio_running) {
        pthread_mutex_lock(&g_audio_mutex);
        if (g_audio_data.current_frame >= g_audio_data.total_frames) {
            pthread_mutex_unlock(&g_audio_mutex);
            break;
        }

        int frames_to_read = period_size;
        if (g_audio_data.current_frame + frames_to_read > g_audio_data.total_frames) {
            frames_to_read = g_audio_data.total_frames - g_audio_data.current_frame;
        }

        for (int i = 0; i < frames_to_read; i++) {
            for (int c = 0; c < channels; c++) {
                float sample = g_audio_data.data[g_audio_data.current_frame * channels + c];
                int val = (int)(sample * 32767.0f);
                if (val > 32767)
                    val = 32767;
                if (val < -32768)
                    val = -32768;
                buffer[i * channels + c] = (int16_t)val;
            }
            g_audio_data.current_frame++;
        }
        pthread_mutex_unlock(&g_audio_mutex);

        snd_pcm_sframes_t frames = snd_pcm_writei(g_alsa_handle, buffer, frames_to_read);
        if (frames < 0) {
            frames = snd_pcm_recover(g_alsa_handle, frames, 0);
            if (frames < 0) {
                fprintf(stderr, "[AudioPlayer] ALSA write/recover error: %s\n",
                        snd_strerror(frames));
                break;
            }
        }
    }

    free(buffer);
    g_audio_running = 0;
    return NULL;
}

float audio_get_duration(const char *path) {
    SF_INFO sfinfo;
    memset(&sfinfo, 0, sizeof(sfinfo));
    SNDFILE *file = sf_open(path, SFM_READ, &sfinfo);
    if (!file) {
        fprintf(stderr, "[AudioPlayer] Error: Could not open file %s\n", path);
        return -1.0f;
    }
    float duration = (float)sfinfo.frames / sfinfo.samplerate;
    sf_close(file);
    return duration;
}

void audio_list_devices(void) {
    printf("ALSA Backend: Use 'aplay -l' to list hardware devices.\n");
}

int audio_play_async(const char *path) {
    return audio_play_async_full(path, 0, 0, 48000, 1, 1);
}

int audio_play_async_ext(const char *path, int target_rate, int target_channels) {
    return audio_play_async_full(path, 0, 0, target_rate, target_channels, 1);
}

int audio_play_async_full(const char *path, int src_rate, int src_channels,
        int target_rate, int target_channels, int output_device_id) {
    audio_stop();

    pthread_mutex_lock(&g_audio_mutex);

    SF_INFO sfinfo;
    memset(&sfinfo, 0, sizeof(sfinfo));
    SNDFILE *file = sf_open(path, SFM_READ, &sfinfo);
    if (!file) {
        fprintf(stderr, "[AudioPlayer] Error: Could not open file %s\n", path);
        pthread_mutex_unlock(&g_audio_mutex);
        return -1;
    }

    int effective_src_rate = (src_rate > 0) ? src_rate : sfinfo.samplerate;
    float *original_data = (float *)malloc(sizeof(float) * sfinfo.frames * sfinfo.channels);
    if (!original_data) {
        sf_close(file);
        pthread_mutex_unlock(&g_audio_mutex);
        return -1;
    }
    sf_readf_float(file, original_data, sfinfo.frames);
    sf_close(file);

    double ratio = (double)target_rate / effective_src_rate;
    int64_t target_frames = (int64_t)(sfinfo.frames * ratio + 1);

    float *converted_src_data = (float *)malloc(sizeof(float) * sfinfo.frames * target_channels);
    if (!converted_src_data) {
        free(original_data);
        pthread_mutex_unlock(&g_audio_mutex);
        return -1;
    }

    if (sfinfo.channels == target_channels) {
        memcpy(converted_src_data, original_data, sizeof(float) * sfinfo.frames * target_channels);
    } else {
        for (sf_count_t i = 0; i < sfinfo.frames; i++) {
            if (sfinfo.channels > 1 && target_channels == 1) {
                converted_src_data[i] = (original_data[i * 2] + original_data[i * 2 + 1]) / 2.0f;
            } else if (sfinfo.channels == 1 && target_channels == 2) {
                converted_src_data[i * 2] = original_data[i];
                converted_src_data[i * 2 + 1] = original_data[i];
            } else {
                converted_src_data[i] = original_data[i * sfinfo.channels];
            }
        }
    }
    free(original_data);

    g_audio_data.data = (float *)malloc(sizeof(float) * target_frames * target_channels);
    if (!g_audio_data.data) {
        free(converted_src_data);
        pthread_mutex_unlock(&g_audio_mutex);
        return -1;
    }

    SRC_DATA src_data = {
        converted_src_data, g_audio_data.data, sfinfo.frames, target_frames, 0, 0, 0, ratio};
    src_simple(&src_data, SRC_SINC_FASTEST, target_channels);
    free(converted_src_data);

    g_audio_data.total_frames = src_data.output_frames_gen;
    g_audio_data.channels = target_channels;
    g_audio_data.current_frame = 0;

    char device_name[32];
    int card = (output_device_id >= 0) ? output_device_id : 1;
    snprintf(device_name, sizeof(device_name), "hw:%d,0", card);

    int err = snd_pcm_open(&g_alsa_handle, device_name, SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        snprintf(device_name, sizeof(device_name), "plughw:%d,0", card);
        err = snd_pcm_open(&g_alsa_handle, device_name, SND_PCM_STREAM_PLAYBACK, 0);
    }

    if (err < 0) {
        fprintf(stderr, "[AudioPlayer] ALSA open error: %s\n", snd_strerror(err));
        free(g_audio_data.data);
        g_audio_data.data = NULL;
        pthread_mutex_unlock(&g_audio_mutex);
        return -1;
    }

    snd_pcm_set_params(g_alsa_handle, SND_PCM_FORMAT_S16_LE,
            SND_PCM_ACCESS_RW_INTERLEAVED, target_channels, target_rate, 1, 500000);

    g_audio_running = 1;
    pthread_create(&g_audio_thread, NULL, alsa_playback_thread, NULL);

    pthread_mutex_unlock(&g_audio_mutex);
    return 0;
}

void audio_stop(void) {
    g_audio_running = 0;
    if (g_alsa_handle) {
        pthread_join(g_audio_thread, NULL);
        snd_pcm_drain(g_alsa_handle);
        snd_pcm_close(g_alsa_handle);
        g_alsa_handle = NULL;
    }
    pthread_mutex_lock(&g_audio_mutex);
    if (g_audio_data.data) {
        free(g_audio_data.data);
        g_audio_data.data = NULL;
    }
    pthread_mutex_unlock(&g_audio_mutex);
}
