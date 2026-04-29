/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TRACKER_UTILS_H
#define TRACKER_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * 获取当前单调时间（微秒）
     */
    uint64_t get_time_us(void);

#ifdef __cplusplus
}
#endif





#endif  // TRACKER_UTILS_H
