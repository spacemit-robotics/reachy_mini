/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TEXT_BUFFER_HPP
#define TEXT_BUFFER_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>

class TextBuffer {
public:
    TextBuffer();
    ~TextBuffer();

    void addText(const std::string& text);
    std::string getNextSentence();
    bool hasSentence() const;
    void clear();
    void stop();

private:
    void processBuffer();
    bool isEndOfSentence(char c) const;

    std::string buffer_;
    std::queue<std::string> sentences_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> stop_flag_;

    static const char CHINESE_PUNCTUATION[];
};

#endif  // TEXT_BUFFER_HPP
