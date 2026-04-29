/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef VOICE_COMMON_HPP
#define VOICE_COMMON_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "tts_service.h"

// ============================================================================
// Global state
// ============================================================================

extern std::atomic<bool> g_running;
extern std::atomic<bool> g_processing;
extern std::atomic<bool> g_barge_in;
extern std::mutex g_process_thread_mutex;
extern std::unique_ptr<std::thread> g_process_thread;

void voiceSignalHandler(int sig);

// ============================================================================
// Timestamp
// ============================================================================

std::string getTimestamp();

// ============================================================================
// TTS engine selection
// ============================================================================

struct EngineSelection {
    SpacemiT::BackendType backend;
    std::string voice;
};

extern const std::vector<std::pair<std::string, std::string>> kKokoroVoices;

std::string resolveVoiceName(const std::string& input);
void printVoiceList();
EngineSelection parseEngine(const std::string& spec);

// ============================================================================
// Audio conversion utilities
// ============================================================================

std::vector<float> pcm16BytesToFloat(const std::vector<uint8_t>& bytes);
void saveWav(const std::string& filename, const std::vector<int16_t>& data, int sample_rate);

#endif  // VOICE_COMMON_HPP
