/**
 * RFX codec decompose
 *
 * Copyright 2024-2025 Jay Sorg <jay.sorg@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if defined(HAVE_CONFIG_H)
#include <config_ac.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rfxcodec_decode.h>

#include "rfxcommon.h"
#include "rfxdecode.h"
#include "rfxdecode_decompose.h"
#include "rfxconstants.h"
#include "rfxdecode_tile.h"
#include "rfxdecode_rlgr1.h"
#include "rfxdecode_rlgr3.h"
#include "rfxdecode_rlgr1_diff.h"
#include "rfxdecode_rlgr3_diff.h"
#include "rfxdecode_differential.h"
#include "rfxdecode_quantization.h"
#include "rfxdecode_dwt.h"
#include "rfxdecode_alpha.h"

#ifdef RFX_USE_ACCEL_X86
#include "x86/funcs_x86.h"
#endif

#ifdef RFX_USE_ACCEL_AMD64
#include "amd64/funcs_amd64.h"
#endif

#define LLOG_LEVEL 1
#define LLOGLN(_level, _args) \
    do { if (_level < LLOG_LEVEL) { printf _args ; printf("\n"); } } while (0)

/******************************************************************************/
/* https://msdn.microsoft.com/en-us/library/ff635267.aspx
 * 1.0      1.0         1.0
 * 0.0     -3.43730     1.769905    note, the -3.43730 looks like doc error
 * 1.40525 -0.71440     0.0                       should be -0.343730
   r = y *  1.0 + u *  0.0      + v *  1.40525
   g = y *  1.0 + u * -0.343730 + v * -0.71440
   b = y *  1.0 + u *  1.769905 + v *  0.0 */
/* 65536       0     92094
   65536  -22527    -46819
   65536  115992         0 */
static int
rfxcodec_decode_yuva2argb(uint8 *y_buf, uint8 *u_buf,
                          uint8 *v_buf, uint8 *a_buf,
                          uint8* rgb_data, int stride_bytes)
{
    int index;
    int jndex;
    sint32 a, r, g, b;
    sint32 y, u, v;
    uint32 *dst32;

    LLOGLN(10, ("rfxcodec_decode_yuva2argb:"));
    for (index = 0; index < 64; index++)
    {
        dst32 = (uint32*)(rgb_data + index * stride_bytes);
        for (jndex = 0; jndex < 64; jndex++)
        {
            y = *(y_buf++);
            u = *(u_buf++);
            u -= 128;
            v = *(v_buf++);
            v -= 128;
            r = (y *  65536 + u *      0 + v *  92094 + 32 * 1024) >> 16;
            g = (y *  65536 + u * -22527 + v * -46819 + 32 * 1024) >> 16;
            b = (y *  65536 + u * 115992 + v *      0 + 32 * 1024) >> 16;
            a = *(a_buf++);
            r = MINMAX(r, 0, 255);
            g = MINMAX(g, 0, 255);
            b = MINMAX(b, 0, 255);
            *(dst32++) = (a << 24) | (r << 16) | (g << 8) | b;
        }
    }
    return 0;
}

/******************************************************************************/
int
rfx_decode_yuva2argb(struct rfxdecode *dec, int tile_x, int tile_y)
{
    uint8 *y_buf;
    uint8 *u_buf;
    uint8 *v_buf;
    uint8 *a_buf;
    uint8 *rgb_data;
    int stride_bytes;

    LLOGLN(10, ("rfx_decode_yuva2argb:"));
    y_buf = dec->y_buffer;
    u_buf = dec->u_buffer;
    v_buf = dec->v_buffer;
    a_buf = dec->a_buffer;
    rgb_data = (uint8*)(dec->dst_data + tile_y * dec->dst_stride_bytes +
                        tile_x * (dec->bits_per_pixel / 8));
    stride_bytes = dec->dst_stride_bytes;
    return rfxcodec_decode_yuva2argb(y_buf, u_buf, v_buf, a_buf,
                                     rgb_data, stride_bytes);
}

/******************************************************************************/
static int
rfxcodec_decode_yuva2abgr(uint8 *y_buf, uint8 *u_buf,
                          uint8 *v_buf, uint8 *a_buf,
                          uint8* rgb_data, int stride_bytes)
{
    int index;
    int jndex;
    sint32 a, r, g, b;
    sint32 y, u, v;
    uint32 *dst32;

    for (index = 0; index < 64; index++)
    {
        dst32 = (uint32*)(rgb_data + index * stride_bytes);
        for (jndex = 0; jndex < 64; jndex++)
        {
            y = *(y_buf++);
            u = *(u_buf++);
            u -= 128;
            v = *(v_buf++);
            v -= 128;
            r = (y *  65536 + u *      0 + v *  92094 + 32 * 1024) >> 16;
            g = (y *  65536 + u * -22527 + v * -46819 + 32 * 1024) >> 16;
            b = (y *  65536 + u * 115992 + v *      0 + 32 * 1024) >> 16;
            a = *(a_buf++);
            r = MINMAX(r, 0, 255);
            g = MINMAX(g, 0, 255);
            b = MINMAX(b, 0, 255);
            *(dst32++) = (a << 24) | (b << 16) | (g << 8) | r;
        }
    }
    return 0;
}

/******************************************************************************/
int
rfx_decode_yuva2abgr(struct rfxdecode *dec, int tile_x, int tile_y)
{
    uint8 *y_buf;
    uint8 *u_buf;
    uint8 *v_buf;
    uint8 *a_buf;
    uint8 *rgb_data;
    int stride_bytes;

    LLOGLN(10, ("rfx_decode_yuva2abgr:"));
    y_buf = dec->y_buffer;
    u_buf = dec->u_buffer;
    v_buf = dec->v_buffer;
    a_buf = dec->a_buffer;
    rgb_data = (uint8*)(dec->dst_data + tile_y * dec->dst_stride_bytes +
                        tile_x * (dec->bits_per_pixel / 8));
    stride_bytes = dec->dst_stride_bytes;
    return rfxcodec_decode_yuva2abgr(y_buf, u_buf, v_buf, a_buf,
                                     rgb_data, stride_bytes);
}

/******************************************************************************/
static int
rfxcodec_decode_yuva2rgb(uint8 *y_buf, uint8 *u_buf,
                         uint8 *v_buf, uint8 *a_buf,
                         uint8* rgb_data, int stride_bytes)
{
    int index;
    int jndex;
    sint32 r, g, b;
    sint32 y, u, v;
    uint8 *dst8;

    for (index = 0; index < 64; index++)
    {
        dst8 = rgb_data + index * stride_bytes;
        for (jndex = 0; jndex < 64; jndex++)
        {
            y = *(y_buf++);
            u = *(u_buf++);
            u -= 128;
            v = *(v_buf++);
            v -= 128;
            r = (y *  65536 + u *      0 + v *  92094 + 32 * 1024) >> 16;
            g = (y *  65536 + u * -22527 + v * -46819 + 32 * 1024) >> 16;
            b = (y *  65536 + u * 115992 + v *      0 + 32 * 1024) >> 16;
            r = MINMAX(r, 0, 255);
            g = MINMAX(g, 0, 255);
            b = MINMAX(b, 0, 255);
            *(dst8++) = b;
            *(dst8++) = g;
            *(dst8++) = r;
        }
    }
    return 0;
}

/******************************************************************************/
int
rfx_decode_yuva2rgb(struct rfxdecode *dec, int tile_x, int tile_y)
{
    uint8 *y_buf;
    uint8 *u_buf;
    uint8 *v_buf;
    uint8 *rgb_data;
    int stride_bytes;

    y_buf = dec->y_buffer;
    u_buf = dec->u_buffer;
    v_buf = dec->v_buffer;
    rgb_data = (uint8*)(dec->dst_data + tile_y * dec->dst_stride_bytes +
                        tile_x * (dec->bits_per_pixel / 8));
    stride_bytes = dec->dst_stride_bytes;
    return rfxcodec_decode_yuva2rgb(y_buf, u_buf, v_buf, NULL,
                                    rgb_data, stride_bytes);
}

/******************************************************************************/
static int
rfxcodec_decode_yuva2bgr(uint8 *y_buf, uint8 *u_buf,
                         uint8 *v_buf, uint8 *a_buf,
                         uint8* rgb_data, int stride_bytes)
{
    int index;
    int jndex;
    sint32 r, g, b;
    sint32 y, u, v;
    uint8 *dst8;

    for (index = 0; index < 64; index++)
    {
        dst8 = rgb_data + index * stride_bytes;
        for (jndex = 0; jndex < 64; jndex++)
        {
            y = *(y_buf++);
            u = *(u_buf++);
            u -= 128;
            v = *(v_buf++);
            v -= 128;
            r = (y *  65536 + u *      0 + v *  92094 + 32 * 1024) >> 16;
            g = (y *  65536 + u * -22527 + v * -46819 + 32 * 1024) >> 16;
            b = (y *  65536 + u * 115992 + v *      0 + 32 * 1024) >> 16;
            r = MINMAX(r, 0, 255);
            g = MINMAX(g, 0, 255);
            b = MINMAX(b, 0, 255);
            *(dst8++) = r;
            *(dst8++) = g;
            *(dst8++) = b;
        }
    }
    return 0;
}

/******************************************************************************/
int
rfx_decode_yuva2bgr(struct rfxdecode *dec, int tile_x, int tile_y)
{
    uint8 *y_buf;
    uint8 *u_buf;
    uint8 *v_buf;
    uint8 *rgb_data;
    int stride_bytes;

    y_buf = dec->y_buffer;
    u_buf = dec->u_buffer;
    v_buf = dec->v_buffer;
    rgb_data = (uint8*)(dec->dst_data + tile_y * dec->dst_stride_bytes +
                        tile_x * (dec->bits_per_pixel / 8));
    stride_bytes = dec->dst_stride_bytes;
    return rfxcodec_decode_yuva2bgr(y_buf, u_buf, v_buf, NULL,
                                    rgb_data, stride_bytes);
}

/******************************************************************************/
int
rfx_decode_yuva2argb_x86_sse2(struct rfxdecode *dec, int tile_x, int tile_y)
{
#if defined(RFX_USE_ACCEL_X86)
    uint8 *y_buf;
    uint8 *u_buf;
    uint8 *v_buf;
    uint8 *a_buf;
    uint8 *rgb_data;
    int stride_bytes;

    LLOGLN(10, ("rfx_decode_yuva2argb_x86_sse2:"));
    y_buf = dec->y_buffer;
    u_buf = dec->u_buffer;
    v_buf = dec->v_buffer;
    a_buf = dec->a_buffer;
    rgb_data = (uint8*)(dec->dst_data + tile_y * dec->dst_stride_bytes +
                        tile_x * (dec->bits_per_pixel / 8));
    stride_bytes = dec->dst_stride_bytes;
    rfxcodec_decode_yuva2argb_x86_sse2(y_buf, u_buf, v_buf, a_buf,
                                       rgb_data, stride_bytes);
#endif
    return 0;
}

/******************************************************************************/
int
rfx_decode_yuva2argb_x86_sse41(struct rfxdecode *dec, int tile_x, int tile_y)
{
#if defined(RFX_USE_ACCEL_X86)
    uint8 *y_buf;
    uint8 *u_buf;
    uint8 *v_buf;
    uint8 *a_buf;
    uint8 *rgb_data;
    int stride_bytes;

    LLOGLN(10, ("rfx_decode_yuva2argb_x86_sse41:"));
    y_buf = dec->y_buffer;
    u_buf = dec->u_buffer;
    v_buf = dec->v_buffer;
    a_buf = dec->a_buffer;
    rgb_data = (uint8*)(dec->dst_data + tile_y * dec->dst_stride_bytes +
                        tile_x * (dec->bits_per_pixel / 8));
    stride_bytes = dec->dst_stride_bytes;
    rfxcodec_decode_yuva2argb_x86_sse41(y_buf, u_buf, v_buf, a_buf,
                                        rgb_data, stride_bytes);
#endif
    return 0;
}

/******************************************************************************/
int
rfx_decode_yuva2argb_amd64_sse2(struct rfxdecode *dec, int tile_x, int tile_y)
{
#if defined(RFX_USE_ACCEL_AMD64)
    uint8 *y_buf;
    uint8 *u_buf;
    uint8 *v_buf;
    uint8 *a_buf;
    uint8 *rgb_data;
    int stride_bytes;

    LLOGLN(10, ("rfx_decode_yuva2argb_amd64_sse2:"));
    y_buf = dec->y_buffer;
    u_buf = dec->u_buffer;
    v_buf = dec->v_buffer;
    a_buf = dec->a_buffer;
    rgb_data = (uint8*)(dec->dst_data + tile_y * dec->dst_stride_bytes +
                        tile_x * (dec->bits_per_pixel / 8));
    stride_bytes = dec->dst_stride_bytes;
    rfxcodec_decode_yuva2argb_amd64_sse2(y_buf, u_buf, v_buf, a_buf,
                                         rgb_data, stride_bytes);
#endif
    return 0;
}

/******************************************************************************/
int
rfx_decode_yuva2argb_amd64_sse41(struct rfxdecode *dec, int tile_x, int tile_y)
{
#if defined(RFX_USE_ACCEL_AMD64)
    uint8 *y_buf;
    uint8 *u_buf;
    uint8 *v_buf;
    uint8 *a_buf;
    uint8 *rgb_data;
    int stride_bytes;

    LLOGLN(10, ("rfx_decode_yuva2argb_amd64_sse41:"));
    y_buf = dec->y_buffer;
    u_buf = dec->u_buffer;
    v_buf = dec->v_buffer;
    a_buf = dec->a_buffer;
    rgb_data = (uint8*)(dec->dst_data + tile_y * dec->dst_stride_bytes +
                        tile_x * (dec->bits_per_pixel / 8));
    stride_bytes = dec->dst_stride_bytes;
    rfxcodec_decode_yuva2argb_amd64_sse41(y_buf, u_buf, v_buf, a_buf,
                                          rgb_data, stride_bytes);
#endif
    return 0;
}

/******************************************************************************/
int
rfx_decode_component_rlgr1(struct rfxdecode *dec,
                           uint8* cdata, int cdata_bytes,
                           uint8* dst_data, int quant)
{
    int num_coef;

    num_coef = rfx_rlgr1_decode(cdata, cdata_bytes, dec->dwt_buffer, 4096);
    if (num_coef != 4096)
    {
        return 1;
    }
    if (rfx_differential_decode(dec->dwt_buffer + 4032, 64) != 0)
    {
        return 2;
    }
    if (rfx_quantization_decode(dec->dwt_buffer,
                                dec->quants[quant].quants) != 0)
    {
        return 3;
    }
    if (rfx_dwt_2d_decode(dec->dwt_buffer, dec->dwt_buffer1, dst_data) != 0)
    {
        return 4;
    }
    return 0;
}

/******************************************************************************/
int
rfx_decode_component_rlgr3(struct rfxdecode *dec,
                           uint8* cdata, int cdata_bytes,
                           uint8* dst_data, int quant)
{
    int num_coef;

    num_coef = rfx_rlgr3_decode(cdata, cdata_bytes, dec->dwt_buffer, 4096);
    if (num_coef != 4096)
    {
        return 1;
    }
    if (rfx_differential_decode(dec->dwt_buffer + 4032, 64) != 0)
    {
        return 2;
    }
    if (rfx_quantization_decode(dec->dwt_buffer,
                                dec->quants[quant].quants) != 0)
    {
        return 3;
    }
    if (rfx_dwt_2d_decode(dec->dwt_buffer, dec->dwt_buffer1, dst_data) != 0)
    {
        return 4;
    }
    return 0;
}

/******************************************************************************/
int
rfx_decode_component_rlgr1_x86_sse2(struct rfxdecode *dec,
                                    uint8* cdata, int cdata_bytes,
                                    uint8* dst_data, int quant)
{
#if defined(RFX_USE_ACCEL_X86)
    int num_coef;

    num_coef = rfx_rlgr1_decode_diff(cdata, cdata_bytes, dec->dwt_buffer);
    if (num_coef != 4096)
    {
        return 1;
    }
    if (rfxcodec_decode_shift_idwt_x86_sse2(dec->quants[quant].quants,
                                            dec->dwt_buffer,
                                            dec->dwt_buffer1,
                                            dst_data) != 0)
    {
        return 2;
    }
#endif
    return 0;
}

/******************************************************************************/
int
rfx_decode_component_rlgr3_x86_sse2(struct rfxdecode *dec,
                                    uint8* cdata, int cdata_bytes,
                                    uint8* dst_data, int quant)
{
#if defined(RFX_USE_ACCEL_X86)
    int num_coef;

    num_coef = rfx_rlgr3_decode_diff(cdata, cdata_bytes, dec->dwt_buffer);
    if (num_coef != 4096)
    {
        return 1;
    }
    if (rfxcodec_decode_shift_idwt_x86_sse2(dec->quants[quant].quants,
                                            dec->dwt_buffer,
                                            dec->dwt_buffer1,
                                            dst_data) != 0)
    {
        return 2;
    }
#endif
    return 0;
}

/******************************************************************************/
int
rfx_decode_component_rlgr1_amd64_sse2(struct rfxdecode *dec,
                                      uint8* cdata, int cdata_bytes,
                                      uint8* dst_data, int quant)
{
#if defined(RFX_USE_ACCEL_AMD64)
    int num_coef;

    num_coef = rfx_rlgr1_decode_diff(cdata, cdata_bytes, dec->dwt_buffer);
    if (num_coef != 4096)
    {
        return 1;
    }
    if (rfxcodec_decode_shift_idwt_amd64_sse2(dec->quants[quant].quants,
                                              dec->dwt_buffer,
                                              dec->dwt_buffer1,
                                              dst_data) != 0)
    {
        return 2;
    }
#endif
    return 0;
}

/******************************************************************************/
int
rfx_decode_component_rlgr3_amd64_sse2(struct rfxdecode *dec,
                                      uint8* cdata, int cdata_bytes,
                                      uint8* dst_data, int quant)
{
#if defined(RFX_USE_ACCEL_AMD64)
    int num_coef;

    num_coef = rfx_rlgr3_decode_diff(cdata, cdata_bytes, dec->dwt_buffer);
    if (num_coef != 4096)
    {
        return 1;
    }
    if (rfxcodec_decode_shift_idwt_amd64_sse2(dec->quants[quant].quants,
                                              dec->dwt_buffer,
                                              dec->dwt_buffer1,
                                              dst_data) != 0)
    {
        return 2;
    }
#endif
    return 0;
}

/******************************************************************************/
int
rfx_decode_rgb(struct rfxdecode *dec, STREAM *s, struct rfx_tile* tile,
               int YLen, int CbLen, int CrLen)
{
    uint8* y_buffer;
    uint8* u_buffer;
    uint8* v_buffer;

    y_buffer = dec->y_buffer;
    u_buffer = dec->u_buffer;
    v_buffer = dec->v_buffer;
    LLOGLN(10, ("rfx_decode_rgb: quant_y %d", tile->quant_y));
    if (dec->rfx_decode(dec, s->p, YLen, y_buffer, tile->quant_y) != 0)
    {
        return 1;
    }
    stream_seek(s, YLen);
    LLOGLN(10, ("rfx_decode_rgb: quant_cb %d", tile->quant_cb));
    if (dec->rfx_decode(dec, s->p, CbLen, u_buffer, tile->quant_cb) != 0)
    {
        return 2;
    }
    stream_seek(s, CbLen);
    LLOGLN(10, ("rfx_decode_rgb: quant_cr %d", tile->quant_cr));
    if (dec->rfx_decode(dec, s->p, CrLen, v_buffer, tile->quant_cr) != 0)
    {
        return 3;
    }
    stream_seek(s, CrLen);
    if (dec->rfx_decode_cc(dec, tile->x, tile->y) != 0)
    {
        return 4;
    }

    return 0;
}

/******************************************************************************/
int
rfx_decode_argb(struct rfxdecode *dec, STREAM *s, struct rfx_tile* tile,
                int YLen, int CbLen, int CrLen, int ALen)
{
    uint8* y_buffer;
    uint8* u_buffer;
    uint8* v_buffer;
    uint8* a_buffer;
    int code;
    int alen_rv;

    y_buffer = dec->y_buffer;
    u_buffer = dec->u_buffer;
    v_buffer = dec->v_buffer;
    a_buffer = dec->a_buffer;
    if (dec->rfx_decode(dec, s->p, YLen, y_buffer, tile->quant_y) != 0)
    {
        return 1;
    }
    stream_seek(s, YLen);
    if (dec->rfx_decode(dec, s->p, CbLen, u_buffer, tile->quant_cb) != 0)
    {
        return 2;
    }
    stream_seek(s, CbLen);
    if (dec->rfx_decode(dec, s->p, CrLen, v_buffer, tile->quant_cr) != 0)
    {
        return 3;
    }
    stream_seek(s, CrLen);
    stream_read_uint8(s, code);
    ALen--;
    if (code & 0x10)
    {
        alen_rv = rfx_decode_plane(s->p, 64, 64, a_buffer, ALen);
        LLOGLN(10, ("rfx_decode_argb: alen_rv %d", alen_rv));
        if (alen_rv != ALen)
        {
            return 4;
        }
    }
    else
    {
        memcpy(a_buffer, s->p, ALen);
    }
    stream_seek(s, ALen);
    if (dec->rfx_decode_cc(dec, tile->x, tile->y) != 0)
    {
        return 5;
    }

    return 0;
}

