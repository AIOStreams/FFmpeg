/*
 * HEVC Intra Prediction NEON initialization
 *
 * Copyright (c) 2026 Jun Zhao <barryjzhao@tencent.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <string.h>

#include "libavutil/attributes.h"
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/aarch64/cpu.h"
#include "libavcodec/hevc/pred.h"

// DC prediction
void ff_hevc_pred_dc_4x4_8_neon(uint8_t *src, const uint8_t *top,
                               const uint8_t *left, ptrdiff_t stride,
                               int c_idx);
void ff_hevc_pred_dc_8x8_8_neon(uint8_t *src, const uint8_t *top,
                               const uint8_t *left, ptrdiff_t stride,
                               int c_idx);
void ff_hevc_pred_dc_16x16_8_neon(uint8_t *src, const uint8_t *top,
                                const uint8_t *left, ptrdiff_t stride,
                                int c_idx);
void ff_hevc_pred_dc_32x32_8_neon(uint8_t *src, const uint8_t *top,
                                const uint8_t *left, ptrdiff_t stride,
                                int c_idx);

// Planar prediction
void ff_hevc_pred_planar_4x4_8_neon(uint8_t *src, const uint8_t *top,
                                   const uint8_t *left, ptrdiff_t stride);
void ff_hevc_pred_planar_8x8_8_neon(uint8_t *src, const uint8_t *top,
                                   const uint8_t *left, ptrdiff_t stride);
void ff_hevc_pred_planar_16x16_8_neon(uint8_t *src, const uint8_t *top,
                                    const uint8_t *left, ptrdiff_t stride);
void ff_hevc_pred_planar_32x32_8_neon(uint8_t *src, const uint8_t *top,
                                    const uint8_t *left, ptrdiff_t stride);

// 3-tap reference sample filter
void ff_hevc_ref_filter_3tap_8x8_8_neon(uint8_t *filtered_left,
                                        uint8_t *filtered_top,
                                        const uint8_t *left,
                                        const uint8_t *top, int size);
void ff_hevc_ref_filter_3tap_16x16_8_neon(uint8_t *filtered_left,
                                          uint8_t *filtered_top,
                                          const uint8_t *left,
                                          const uint8_t *top, int size);
void ff_hevc_ref_filter_3tap_32x32_8_neon(uint8_t *filtered_left,
                                          uint8_t *filtered_top,
                                          const uint8_t *left,
                                          const uint8_t *top, int size);

// Strong intra smoothing
void ff_hevc_ref_filter_strong_8_neon(uint8_t *filtered_top, uint8_t *left,
                                      const uint8_t *top);

// Mode 10 and 26
void ff_hevc_pred_angular_mode_10_8_neon(uint8_t *src, const uint8_t *top,
                                        const uint8_t *left, ptrdiff_t stride,
                                        int c_idx, int log2_size);
void ff_hevc_pred_angular_mode_26_8_neon(uint8_t *src, const uint8_t *top,
                                        const uint8_t *left, ptrdiff_t stride,
                                        int c_idx, int log2_size);

void ff_hevc_pred_planar_4x4_10_neon(uint8_t *src, const uint8_t *top,
                                     const uint8_t *left, ptrdiff_t stride);
void ff_hevc_pred_planar_8x8_10_neon(uint8_t *src, const uint8_t *top,
                                     const uint8_t *left, ptrdiff_t stride);
void ff_hevc_pred_planar_16x16_10_neon(uint8_t *src, const uint8_t *top,
                                       const uint8_t *left, ptrdiff_t stride);
void ff_hevc_pred_planar_32x32_10_neon(uint8_t *src, const uint8_t *top,
                                       const uint8_t *left, ptrdiff_t stride);

void ff_hevc_pred_angular_v_10_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint16_t *ref, int size, int angle);
void ff_hevc_pred_angular_h_10_neon(uint8_t *dst, ptrdiff_t stride,
                                    const uint16_t *ref, int size, int angle);

static void pred_dc_neon(uint8_t *src, const uint8_t *top,
                         const uint8_t *left, ptrdiff_t stride,
                         int log2_size, int c_idx)
{
    switch (log2_size) {
    case 2:
        ff_hevc_pred_dc_4x4_8_neon(src, top, left, stride, c_idx);
        break;
    case 3:
        ff_hevc_pred_dc_8x8_8_neon(src, top, left, stride, c_idx);
        break;
    case 4:
        ff_hevc_pred_dc_16x16_8_neon(src, top, left, stride, c_idx);
        break;
    case 5:
        ff_hevc_pred_dc_32x32_8_neon(src, top, left, stride, c_idx);
        break;
    default:
        av_unreachable("log2_size must be 2, 3, 4 or 5");
    }
}

#define PRED_ANGULAR_NEON(IDX, LOG2)                                          \
static void pred_angular_##IDX##_neon(uint8_t *src, const uint8_t *top,       \
                                      const uint8_t *left, ptrdiff_t stride,  \
                                      int c_idx, int mode)                    \
{                                                                             \
    if (mode == 10)                                                           \
        ff_hevc_pred_angular_mode_10_8_neon(src, top, left, stride,           \
                                           c_idx, LOG2);                      \
    else if (mode == 26)                                                      \
        ff_hevc_pred_angular_mode_26_8_neon(src, top, left, stride,           \
                                           c_idx, LOG2);                      \
    else                                                                      \
        ff_hevc_pred_angular_##IDX##_8(src, top, left, stride, c_idx, mode);  \
}

PRED_ANGULAR_NEON(0, 2)
PRED_ANGULAR_NEON(1, 3)
PRED_ANGULAR_NEON(2, 4)
PRED_ANGULAR_NEON(3, 5)

#undef PRED_ANGULAR_NEON

static av_always_inline void pred_angular_10_neon(uint8_t *_src,
                                                  const uint8_t *_top,
                                                  const uint8_t *_left,
                                                  ptrdiff_t stride, int c_idx,
                                                  int mode, int size)
{
    static const int8_t intra_pred_angle[] = {
         32,  26,  21,  17, 13,  9,  5, 2, 0, -2, -5, -9, -13, -17, -21, -26, -32,
        -26, -21, -17, -13, -9, -5, -2, 0, 2,  5,  9, 13,  17,  21,  26,  32
    };
    static const int16_t inv_angle[] = {
        -4096, -1638, -910, -630, -482, -390, -315, -256, -315, -390, -482,
        -630, -910, -1638, -4096
    };
    uint16_t *src        = (uint16_t *)_src;
    const uint16_t *top  = (const uint16_t *)_top;
    const uint16_t *left = (const uint16_t *)_left;
    const uint16_t *edge = mode >= 18 ? top  : left;
    const uint16_t *side = mode >= 18 ? left : top;
    const uint16_t *ref  = edge - 1;
    uint16_t ref_array[3 * 32 + 4];
    int angle = intra_pred_angle[mode - 2];
    int last  = (size * angle) >> 5;

    if (angle < 0 && last < -1) {
        uint16_t *ref_tmp = ref_array + size;
        memcpy(ref_tmp, edge - 1, (size + 1) * sizeof(*ref_tmp));
        for (int x = last; x <= -1; x++)
            ref_tmp[x] = side[-1 + ((x * inv_angle[mode - 11] + 128) >> 8)];
        ref = ref_tmp;
    }

    if (mode >= 18) {
        ff_hevc_pred_angular_v_10_neon(_src, stride, ref, size, angle);
        if (mode == 26 && c_idx == 0 && size < 32)
            for (int y = 0; y < size; y++)
                src[y * stride] =
                    av_clip_uintp2(top[0] + ((left[y] - left[-1]) >> 1), 10);
    } else {
        ff_hevc_pred_angular_h_10_neon(_src, stride, ref, size, angle);
        if (mode == 10 && c_idx == 0 && size < 32)
            for (int x = 0; x < size; x++)
                src[x] = av_clip_uintp2(left[0] + ((top[x] - top[-1]) >> 1), 10);
    }
}

#define PRED_ANGULAR_10_NEON(IDX, SIZE)                                       \
static void pred_angular_##IDX##_10_neon(uint8_t *src, const uint8_t *top,    \
                                         const uint8_t *left,                 \
                                         ptrdiff_t stride, int c_idx,         \
                                         int mode)                            \
{                                                                             \
    pred_angular_10_neon(src, top, left, stride, c_idx, mode, SIZE);          \
}

PRED_ANGULAR_10_NEON(0, 4)
PRED_ANGULAR_10_NEON(1, 8)
PRED_ANGULAR_10_NEON(2, 16)
PRED_ANGULAR_10_NEON(3, 32)

#undef PRED_ANGULAR_10_NEON

av_cold void ff_hevc_pred_init_aarch64(HEVCPredContext *hpc, int bit_depth)
{
    int cpu_flags = av_get_cpu_flags();

    if (!have_neon(cpu_flags))
        return;

    if (bit_depth == 8) {
        hpc->pred_dc        = pred_dc_neon;
        hpc->pred_planar[0] = ff_hevc_pred_planar_4x4_8_neon;
        hpc->pred_planar[1] = ff_hevc_pred_planar_8x8_8_neon;
        hpc->pred_planar[2] = ff_hevc_pred_planar_16x16_8_neon;
        hpc->pred_planar[3] = ff_hevc_pred_planar_32x32_8_neon;

        hpc->ref_filter_3tap[0] = ff_hevc_ref_filter_3tap_8x8_8_neon;
        hpc->ref_filter_3tap[1] = ff_hevc_ref_filter_3tap_16x16_8_neon;
        hpc->ref_filter_3tap[2] = ff_hevc_ref_filter_3tap_32x32_8_neon;
        hpc->ref_filter_strong  = ff_hevc_ref_filter_strong_8_neon;

        hpc->pred_angular[0] = pred_angular_0_neon;
        hpc->pred_angular[1] = pred_angular_1_neon;
        hpc->pred_angular[2] = pred_angular_2_neon;
        hpc->pred_angular[3] = pred_angular_3_neon;
    } else if (bit_depth == 10) {
        hpc->pred_planar[0] = ff_hevc_pred_planar_4x4_10_neon;
        hpc->pred_planar[1] = ff_hevc_pred_planar_8x8_10_neon;
        hpc->pred_planar[2] = ff_hevc_pred_planar_16x16_10_neon;
        hpc->pred_planar[3] = ff_hevc_pred_planar_32x32_10_neon;

        hpc->pred_angular[0] = pred_angular_0_10_neon;
        hpc->pred_angular[1] = pred_angular_1_10_neon;
        hpc->pred_angular[2] = pred_angular_2_10_neon;
        hpc->pred_angular[3] = pred_angular_3_10_neon;
    }
}
