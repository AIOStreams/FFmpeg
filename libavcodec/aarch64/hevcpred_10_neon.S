/*
 * HEVC 10-bit intra prediction NEON optimizations
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

#include "libavutil/aarch64/asm.S"

// Every intermediate fits in 16 bits with 10-bit samples.

// Modes 18-34: row y blends ref[i + x + 1] and ref[i + x + 2], where i and the
// weight are the integer and fractional parts of (y + 1) * angle / 32.
function ff_hevc_pred_angular_v_10_neon, export=1
        lsl             x1, x1, #1
        add             x2, x2, #2
        mov             w5, w4
        movi            v31.8h, #32
        cmp             w3, #8
        b.lt            4f
        b.eq            8f
        cmp             w3, #16
        b.eq            16f

32:     asr             w6, w5, #5
        and             w7, w5, #31
        add             x8, x2, w6, sxtw #1
        add             w5, w5, w4
        ld1             {v0.8h, v1.8h, v2.8h, v3.8h}, [x8]
        cbz             w7, 1f
        add             x9, x8, #2
        dup             v30.8h, w7
        ld1             {v4.8h, v5.8h, v6.8h, v7.8h}, [x9]
        sub             v29.8h, v31.8h, v30.8h
        mul             v0.8h, v0.8h, v29.8h
        mul             v1.8h, v1.8h, v29.8h
        mul             v2.8h, v2.8h, v29.8h
        mul             v3.8h, v3.8h, v29.8h
        mla             v0.8h, v4.8h, v30.8h
        mla             v1.8h, v5.8h, v30.8h
        mla             v2.8h, v6.8h, v30.8h
        mla             v3.8h, v7.8h, v30.8h
        urshr           v0.8h, v0.8h, #5
        urshr           v1.8h, v1.8h, #5
        urshr           v2.8h, v2.8h, #5
        urshr           v3.8h, v3.8h, #5
1:      subs            w3, w3, #1
        st1             {v0.8h, v1.8h, v2.8h, v3.8h}, [x0], x1
        b.ne            32b
        ret

16:     asr             w6, w5, #5
        and             w7, w5, #31
        add             x8, x2, w6, sxtw #1
        add             w5, w5, w4
        ld1             {v0.8h, v1.8h}, [x8]
        cbz             w7, 1f
        add             x9, x8, #2
        dup             v30.8h, w7
        ld1             {v4.8h, v5.8h}, [x9]
        sub             v29.8h, v31.8h, v30.8h
        mul             v0.8h, v0.8h, v29.8h
        mul             v1.8h, v1.8h, v29.8h
        mla             v0.8h, v4.8h, v30.8h
        mla             v1.8h, v5.8h, v30.8h
        urshr           v0.8h, v0.8h, #5
        urshr           v1.8h, v1.8h, #5
1:      subs            w3, w3, #1
        st1             {v0.8h, v1.8h}, [x0], x1
        b.ne            16b
        ret

8:      asr             w6, w5, #5
        and             w7, w5, #31
        add             x8, x2, w6, sxtw #1
        add             w5, w5, w4
        ldr             q0, [x8]
        cbz             w7, 1f
        ldur            q4, [x8, #2]
        dup             v30.8h, w7
        sub             v29.8h, v31.8h, v30.8h
        mul             v0.8h, v0.8h, v29.8h
        mla             v0.8h, v4.8h, v30.8h
        urshr           v0.8h, v0.8h, #5
1:      subs            w3, w3, #1
        st1             {v0.8h}, [x0], x1
        b.ne            8b
        ret

4:      asr             w6, w5, #5
        and             w7, w5, #31
        add             x8, x2, w6, sxtw #1
        add             w5, w5, w4
        ldr             d0, [x8]
        cbz             w7, 1f
        ldur            d4, [x8, #2]
        dup             v30.4h, w7
        sub             v29.4h, v31.4h, v30.4h
        mul             v0.4h, v0.4h, v29.4h
        mla             v0.4h, v4.4h, v30.4h
        urshr           v0.4h, v0.4h, #5
1:      subs            w3, w3, #1
        st1             {v0.4h}, [x0], x1
        b.ne            4b
        ret
endfunc

.macro angular_h_col c, t, n=8h
        asr             w6, w5, #5
        and             w7, w5, #31
        add             x8, x11, w6, sxtw #1
        add             w5, w5, w4
  .ifc \n, 8h
        ldr             q\c, [x8]
  .else
        ldr             d\c, [x8]
  .endif
        cbz             w7, 1f
  .ifc \n, 8h
        ldur            q\t, [x8, #2]
  .else
        ldur            d\t, [x8, #2]
  .endif
        dup             v30.\n, w7
        sub             v29.\n, v31.\n, v30.\n
        mul             v\c\().\n, v\c\().\n, v29.\n
        mla             v\c\().\n, v\t\().\n, v30.\n
        urshr           v\c\().\n, v\c\().\n, #5
1:
.endm

// Modes 2-17: the same along columns, computed as 8x8 tiles and transposed.
function ff_hevc_pred_angular_h_10_neon, export=1
        lsl             x1, x1, #1
        add             x2, x2, #2
        movi            v31.8h, #32
        cmp             w3, #4
        b.eq            4f

        mov             w12, w4                 // angle * (x0 + 1)
        mov             x14, x0                 // dst column tile
        mov             w15, w3                 // columns left
2:      mov             x11, x2                 // ref + y0
        mov             x13, x14                // dst tile
        mov             w10, w3                 // rows left
3:      mov             w5, w12
        angular_h_col   0, 16
        angular_h_col   1, 17
        angular_h_col   2, 18
        angular_h_col   3, 19
        angular_h_col   4, 20
        angular_h_col   5, 21
        angular_h_col   6, 22
        angular_h_col   7, 23
        trn1            v16.8h, v0.8h, v1.8h
        trn2            v17.8h, v0.8h, v1.8h
        trn1            v18.8h, v2.8h, v3.8h
        trn2            v19.8h, v2.8h, v3.8h
        trn1            v20.8h, v4.8h, v5.8h
        trn2            v21.8h, v4.8h, v5.8h
        trn1            v22.8h, v6.8h, v7.8h
        trn2            v23.8h, v6.8h, v7.8h
        trn1            v0.4s, v16.4s, v18.4s
        trn2            v2.4s, v16.4s, v18.4s
        trn1            v1.4s, v17.4s, v19.4s
        trn2            v3.4s, v17.4s, v19.4s
        trn1            v4.4s, v20.4s, v22.4s
        trn2            v6.4s, v20.4s, v22.4s
        trn1            v5.4s, v21.4s, v23.4s
        trn2            v7.4s, v21.4s, v23.4s
        trn1            v16.2d, v0.2d, v4.2d
        trn2            v20.2d, v0.2d, v4.2d
        trn1            v17.2d, v1.2d, v5.2d
        trn2            v21.2d, v1.2d, v5.2d
        trn1            v18.2d, v2.2d, v6.2d
        trn2            v22.2d, v2.2d, v6.2d
        trn1            v19.2d, v3.2d, v7.2d
        trn2            v23.2d, v3.2d, v7.2d
        st1             {v16.8h}, [x13], x1
        st1             {v17.8h}, [x13], x1
        st1             {v18.8h}, [x13], x1
        st1             {v19.8h}, [x13], x1
        st1             {v20.8h}, [x13], x1
        st1             {v21.8h}, [x13], x1
        st1             {v22.8h}, [x13], x1
        st1             {v23.8h}, [x13], x1
        add             x11, x11, #16
        subs            w10, w10, #8
        b.ne            3b
        mov             w12, w5
        add             x14, x14, #16
        subs            w15, w15, #8
        b.ne            2b
        ret

4:      mov             w5, w4
        mov             x11, x2
        angular_h_col   0, 16, 4h
        angular_h_col   1, 17, 4h
        angular_h_col   2, 18, 4h
        angular_h_col   3, 19, 4h
        trn1            v16.4h, v0.4h, v1.4h
        trn2            v17.4h, v0.4h, v1.4h
        trn1            v18.4h, v2.4h, v3.4h
        trn2            v19.4h, v2.4h, v3.4h
        trn1            v0.2s, v16.2s, v18.2s
        trn2            v2.2s, v16.2s, v18.2s
        trn1            v1.2s, v17.2s, v19.2s
        trn2            v3.2s, v17.2s, v19.2s
        st1             {v0.4h}, [x0], x1
        st1             {v1.4h}, [x0], x1
        st1             {v2.4h}, [x0], x1
        st1             {v3.4h}, [x0], x1
        ret
endfunc

// Planar row y is (acc + (N-1-x) * left[y]) >> (log2(N) + 1), where
// acc = (x+1) * top[N] + (N-1-y) * top[x] + (y+1) * left[N] + N.

.macro planar10_row vl, lane, shift, regs
  .if \regs == 0
        mul             v28.4h, v16.4h, \vl\().h[\lane]
        add             v28.4h, v28.4h, v24.4h
        add             v24.4h, v24.4h, v20.4h
        ushr            v28.4h, v28.4h, #\shift
        st1             {v28.4h}, [x0], x3
  .else
        mul             v28.8h, v16.8h, \vl\().h[\lane]
    .if \regs > 1
        mul             v29.8h, v17.8h, \vl\().h[\lane]
    .endif
    .if \regs > 2
        mul             v30.8h, v18.8h, \vl\().h[\lane]
        mul             v31.8h, v19.8h, \vl\().h[\lane]
    .endif
        add             v28.8h, v28.8h, v24.8h
    .if \regs > 1
        add             v29.8h, v29.8h, v25.8h
    .endif
    .if \regs > 2
        add             v30.8h, v30.8h, v26.8h
        add             v31.8h, v31.8h, v27.8h
    .endif
        add             v24.8h, v24.8h, v20.8h
    .if \regs > 1
        add             v25.8h, v25.8h, v21.8h
    .endif
    .if \regs > 2
        add             v26.8h, v26.8h, v22.8h
        add             v27.8h, v27.8h, v23.8h
    .endif
        ushr            v28.8h, v28.8h, #\shift
    .if \regs > 1
        ushr            v29.8h, v29.8h, #\shift
    .endif
    .if \regs > 2
        ushr            v30.8h, v30.8h, #\shift
        ushr            v31.8h, v31.8h, #\shift
    .endif
    .if \regs == 1
        st1             {v28.8h}, [x0], x3
    .elseif \regs == 2
        st1             {v28.8h, v29.8h}, [x0], x3
    .else
        st1             {v28.8h, v29.8h, v30.8h, v31.8h}, [x0], x3
    .endif
  .endif
.endm

.macro planar10_rows8 vl, shift, regs
        planar10_row    \vl, 0, \shift, \regs
        planar10_row    \vl, 1, \shift, \regs
        planar10_row    \vl, 2, \shift, \regs
        planar10_row    \vl, 3, \shift, \regs
        planar10_row    \vl, 4, \shift, \regs
        planar10_row    \vl, 5, \shift, \regs
        planar10_row    \vl, 6, \shift, \regs
        planar10_row    \vl, 7, \shift, \regs
.endm

function ff_hevc_pred_planar_4x4_10_neon, export=1
        lsl             x3, x3, #1
        movrel          x5, planar_x_10
        ldr             d16, [x5]               // x
        ldr             d20, [x1]               // top[x]
        ldr             d0, [x2]                // left[y]
        add             x6, x1, #8
        add             x7, x2, #8
        ld1r            {v4.4h}, [x6]           // top[N]
        ld1r            {v5.4h}, [x7]           // left[N]
        movi            v6.4h, #3
        movi            v7.4h, #4
        add             v7.4h, v7.4h, v4.4h
        add             v7.4h, v7.4h, v5.4h
        mla             v7.4h, v16.4h, v4.4h
        mla             v7.4h, v20.4h, v6.4h
        mov             v24.8b, v7.8b
        sub             v20.4h, v5.4h, v20.4h
        sub             v16.4h, v6.4h, v16.4h
        planar10_row    v0, 0, 3, 0
        planar10_row    v0, 1, 3, 0
        planar10_row    v0, 2, 3, 0
        planar10_row    v0, 3, 3, 0
        ret
endfunc

function ff_hevc_pred_planar_8x8_10_neon, export=1
        lsl             x3, x3, #1
        movrel          x5, planar_x_10
        ldr             q16, [x5]
        ldr             q20, [x1]
        ldr             q0, [x2]
        add             x6, x1, #16
        add             x7, x2, #16
        ld1r            {v4.8h}, [x6]
        ld1r            {v5.8h}, [x7]
        movi            v6.8h, #7
        movi            v7.8h, #8
        add             v7.8h, v7.8h, v4.8h
        add             v24.8h, v7.8h, v5.8h
        mla             v24.8h, v16.8h, v4.8h
        mla             v24.8h, v20.8h, v6.8h
        sub             v20.8h, v5.8h, v20.8h
        sub             v16.8h, v6.8h, v16.8h
        planar10_rows8  v0, 4, 1
        ret
endfunc

function ff_hevc_pred_planar_16x16_10_neon, export=1
        lsl             x3, x3, #1
        movrel          x5, planar_x_10
        ld1             {v16.8h, v17.8h}, [x5]
        ld1             {v20.8h, v21.8h}, [x1]
        ld1             {v0.8h, v1.8h}, [x2]
        add             x6, x1, #32
        add             x7, x2, #32
        ld1r            {v4.8h}, [x6]
        ld1r            {v5.8h}, [x7]
        movi            v6.8h, #15
        movi            v7.8h, #16
        add             v7.8h, v7.8h, v4.8h
        add             v7.8h, v7.8h, v5.8h
        mov             v24.16b, v7.16b
        mov             v25.16b, v7.16b
        mla             v24.8h, v16.8h, v4.8h
        mla             v25.8h, v17.8h, v4.8h
        mla             v24.8h, v20.8h, v6.8h
        mla             v25.8h, v21.8h, v6.8h
        sub             v20.8h, v5.8h, v20.8h
        sub             v21.8h, v5.8h, v21.8h
        sub             v16.8h, v6.8h, v16.8h
        sub             v17.8h, v6.8h, v17.8h
        planar10_rows8  v0, 5, 2
        planar10_rows8  v1, 5, 2
        ret
endfunc

function ff_hevc_pred_planar_32x32_10_neon, export=1
        lsl             x3, x3, #1
        movrel          x5, planar_x_10
        ld1             {v16.8h, v17.8h, v18.8h, v19.8h}, [x5]
        ld1             {v20.8h, v21.8h, v22.8h, v23.8h}, [x1]
        ld1             {v0.8h, v1.8h, v2.8h, v3.8h}, [x2]
        add             x6, x1, #64
        add             x7, x2, #64
        ld1r            {v4.8h}, [x6]
        ld1r            {v5.8h}, [x7]
        movi            v6.8h, #31
        movi            v7.8h, #32
        add             v7.8h, v7.8h, v4.8h
        add             v7.8h, v7.8h, v5.8h
        mov             v24.16b, v7.16b
        mov             v25.16b, v7.16b
        mov             v26.16b, v7.16b
        mov             v27.16b, v7.16b
        mla             v24.8h, v16.8h, v4.8h
        mla             v25.8h, v17.8h, v4.8h
        mla             v26.8h, v18.8h, v4.8h
        mla             v27.8h, v19.8h, v4.8h
        mla             v24.8h, v20.8h, v6.8h
        mla             v25.8h, v21.8h, v6.8h
        mla             v26.8h, v22.8h, v6.8h
        mla             v27.8h, v23.8h, v6.8h
        sub             v20.8h, v5.8h, v20.8h
        sub             v21.8h, v5.8h, v21.8h
        sub             v22.8h, v5.8h, v22.8h
        sub             v23.8h, v5.8h, v23.8h
        sub             v16.8h, v6.8h, v16.8h
        sub             v17.8h, v6.8h, v17.8h
        sub             v18.8h, v6.8h, v18.8h
        sub             v19.8h, v6.8h, v19.8h
        planar10_rows8  v0, 6, 4
        planar10_rows8  v1, 6, 4
        planar10_rows8  v2, 6, 4
        planar10_rows8  v3, 6, 4
        ret
endfunc

const planar_x_10, align=4
        .hword          0,  1,  2,  3,  4,  5,  6,  7
        .hword          8,  9, 10, 11, 12, 13, 14, 15
        .hword          16, 17, 18, 19, 20, 21, 22, 23
        .hword          24, 25, 26, 27, 28, 29, 30, 31
endconst
