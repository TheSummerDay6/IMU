/**
 * @file bsp_dwt.h (test stub)
 * @brief Stub for DWT timer BSP to enable host-side unit testing.
 */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include <stdint.h>

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/* Configurable stub dt value for tests */
static float stub_dwt_dt = 0.001f;

static inline float DWT_GetDeltaT(uint32_t *cnt_last)
{
    (void)cnt_last;
    return stub_dwt_dt;
}

static inline double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    (void)cnt_last;
    return (double)stub_dwt_dt;
}

static inline void DWT_Init(uint32_t CPU_Freq_mHz) { (void)CPU_Freq_mHz; }
static inline float DWT_GetTimeline_s(void) { return 0; }
static inline float DWT_GetTimeline_ms(void) { return 0; }
static inline uint64_t DWT_GetTimeline_us(void) { return 0; }
static inline void DWT_Delay(float Delay) { (void)Delay; }
static inline void DWT_SysTimeUpdate(void) {}

#endif /* _BSP_DWT_H */
