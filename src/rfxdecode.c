/**
 * RFX codec encoder
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
int
rfxcodec_decode_get_version(int *major, int *minor, int *micro)
{
    *major = LIBRFXCODEC_VERSION_MAJOR;
    *minor = LIBRFXCODEC_VERSION_MINOR;
    *micro = LIBRFXCODEC_VERSION_MICRO;
    return 0;
}

/******************************************************************************/
int
rfxcodec_decode_create_ex(int max_width, int max_height, int format, int flags,
                          void **handle)
{
    struct rfxdecode *dec;
    int ax;
    int bx;
    int cx;
    int dx;

    dec = (struct rfxdecode *)malloc(sizeof(struct rfxdecode));
    if (dec == NULL)
    {
        return 1;
    }
    memset(dec, 0, sizeof(struct rfxdecode));

    dec->dwt_buffer = (sint16*)(((uintptr)(dec->dwt_buffer_a)) & ~15);
    dec->dwt_buffer1 = (sint16*)(((uintptr)(dec->dwt_buffer1_a)) & ~15);
    dec->dwt_buffer2 = (sint16*)(((uintptr)(dec->dwt_buffer2_a)) & ~15);

    dec->a_buffer = (uint8*)(((uintptr)(dec->a_buffer_a)) & ~15);
    dec->y_buffer = (uint8*)(((uintptr)(dec->y_buffer_a)) & ~15);
    dec->u_buffer = (uint8*)(((uintptr)(dec->u_buffer_a)) & ~15);
    dec->v_buffer = (uint8*)(((uintptr)(dec->v_buffer_a)) & ~15);

#if defined(RFX_USE_ACCEL_X86)
    cpuid_x86(1, 0, &ax, &bx, &cx, &dx);
#elif defined(RFX_USE_ACCEL_AMD64)
    cpuid_amd64(1, 0, &ax, &bx, &cx, &dx);
#else
    ax = 0;
    bx = 0;
    cx = 0;
    dx = 0;
#endif
    if (dx & (1 << 26)) /* SSE 2 */
    {
        LLOGLN(10, ("rfxcodec_decode_create: got sse2"));
        dec->got_sse2 = 1;
    }
    if (cx & (1 << 0)) /* SSE 3 */
    {
        LLOGLN(10, ("rfxcodec_decode_create: got sse3"));
        dec->got_sse3 = 1;
    }
    if (cx & (1 << 19)) /* SSE 4.1 */
    {
        LLOGLN(10, ("rfxcodec_decode_create: got sse4.1"));
        dec->got_sse41 = 1;
    }
    if (cx & (1 << 20)) /* SSE 4.2 */
    {
        LLOGLN(10, ("rfxcodec_decode_create: got sse4.2"));
        dec->got_sse42 = 1;
    }
    if (cx & (1 << 23)) /* popcnt */
    {
        LLOGLN(10, ("rfxcodec_decode_create: got popcnt"));
        dec->got_popcnt = 1;
    }
#if defined(RFX_USE_ACCEL_X86)
    cpuid_x86(0x80000001, 0, &ax, &bx, &cx, &dx);
#elif defined(RFX_USE_ACCEL_AMD64)
    cpuid_amd64(0x80000001, 0, &ax, &bx, &cx, &dx);
#else
    ax = 0;
    bx = 0;
    cx = 0;
    dx = 0;
#endif
    if (cx & (1 << 5)) /* lzcnt */
    {
        LLOGLN(10, ("rfxcodec_decode_create: got lzcnt"));
        dec->got_lzcnt = 1;
    }
    if (cx & (1 << 6)) /* SSE 4.a */
    {
        LLOGLN(10, ("rfxcodec_decode_create: got sse4.a"));
        dec->got_sse4a = 1;
    }

    dec->max_width = max_width;
    dec->max_height = max_height;
    dec->format = format;
    switch (format)
    {
        case RFX_FORMAT_BGRA:
            dec->rfx_decode_cc = rfx_decode_yuva2argb;
            dec->bits_per_pixel = 32;
            break;
        case RFX_FORMAT_RGBA:
            dec->rfx_decode_cc = rfx_decode_yuva2abgr;
            dec->bits_per_pixel = 32;
            break;
        case RFX_FORMAT_BGR:
            dec->rfx_decode_cc = rfx_decode_yuva2bgr;
            dec->bits_per_pixel = 24;
            break;
        case RFX_FORMAT_RGB:
            dec->rfx_decode_cc = rfx_decode_yuva2rgb;
            dec->bits_per_pixel = 24;
            break;
        case RFX_FORMAT_YUV:
            dec->bits_per_pixel = 32;
            break;
        default:
            free(dec);
            return 2;
    }

    dec->rfx_decode_rlgr1 = rfx_decode_component_rlgr1;
    dec->rfx_decode_rlgr3 = rfx_decode_component_rlgr3;

    /* assign decoding functions */
    if (flags & RFX_FLAGS_NOACCEL)
    {
    }
    else
    {
#if defined(RFX_USE_ACCEL_X86)
        if (dec->got_sse2)
        {
            dec->rfx_decode_rlgr1 = rfx_decode_component_rlgr1_x86_sse2;
            dec->rfx_decode_rlgr3 = rfx_decode_component_rlgr3_x86_sse2;
            if (format == RFX_FORMAT_BGRA)
            {
                dec->rfx_decode_cc = rfx_decode_yuva2argb_x86_sse2;
            }
        }
        if (dec->got_sse41)
        {
            if (format == RFX_FORMAT_BGRA)
            {
                dec->rfx_decode_cc = rfx_decode_yuva2argb_x86_sse41;
            }
        }
#elif defined(RFX_USE_ACCEL_AMD64)
        if (dec->got_sse2)
        {
            dec->rfx_decode_rlgr1 = rfx_decode_component_rlgr1_amd64_sse2;
            dec->rfx_decode_rlgr3 = rfx_decode_component_rlgr3_amd64_sse2;
            if (format == RFX_FORMAT_BGRA)
            {
                dec->rfx_decode_cc = rfx_decode_yuva2argb_amd64_sse2;
            }
        }
        if (dec->got_sse41)
        {
            if (format == RFX_FORMAT_BGRA)
            {
                dec->rfx_decode_cc = rfx_decode_yuva2argb_amd64_sse41;
            }
        }
#else
#endif
    }
    if (ax == 0)
    {
    }
    if (bx == 0)
    {
    }

    /* rlgr1 */
    if (dec->rfx_decode_rlgr1 == rfx_decode_component_rlgr1)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_rlgr1 set to rfx_decode_component_rlgr1"));
    }
    else if (dec->rfx_decode_rlgr1 == rfx_decode_component_rlgr1_x86_sse2)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_rlgr1 set to rfx_decode_component_rlgr1_x86_sse2"));
    }
    else if (dec->rfx_decode_rlgr1 == rfx_decode_component_rlgr1_amd64_sse2)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_rlgr1 set to rfx_decode_component_rlgr1_amd64_sse2"));
    }

    /* rlgr3 */
    if (dec->rfx_decode_rlgr3 == rfx_decode_component_rlgr3)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_rlgr3 set to rfx_decode_component_rlgr3"));
    }
    else if (dec->rfx_decode_rlgr3 == rfx_decode_component_rlgr3_x86_sse2)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_rlgr3 set to rfx_decode_component_rlgr3_x86_sse2"));
    }
    else if (dec->rfx_decode_rlgr3 == rfx_decode_component_rlgr3_amd64_sse2)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_rlgr3 set to rfx_decode_component_rlgr3_amd64_sse2"));
    }

    /* color conversion */
    if (dec->rfx_decode_cc == rfx_decode_yuva2argb)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_cc set to rfx_decode_yuva2argb"));
    }
    else if (dec->rfx_decode_cc == rfx_decode_yuva2abgr)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_cc set to rfx_decode_yuva2abgr"));
    }
    else if (dec->rfx_decode_cc == rfx_decode_yuva2bgr)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_cc set to rfx_decode_yuva2bgr"));
    }
    else if (dec->rfx_decode_cc == rfx_decode_yuva2rgb)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_cc set to rfx_decode_yuva2rgb"));
    }
    else if (dec->rfx_decode_cc == rfx_decode_yuva2argb_x86_sse2)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_cc set to rfx_decode_yuva2argb_x86_sse2"));
    }
    else if (dec->rfx_decode_cc == rfx_decode_yuva2argb_amd64_sse2)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_cc set to rfx_decode_yuva2argb_amd64_sse2"));
    }
    else if (dec->rfx_decode_cc == rfx_decode_yuva2argb_x86_sse41)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_cc set to rfx_decode_yuva2argb_x86_sse41"));
    }
    else if (dec->rfx_decode_cc == rfx_decode_yuva2argb_amd64_sse41)
    {
        LLOGLN(10, ("rfxcodec_decode_create: rfx_decode_cc set to rfx_decode_yuva2argb_amd64_sse41"));
    }

    *handle = dec;
    return 0;
}

/******************************************************************************/
void *
rfxcodec_decode_create(int max_width, int max_height, int format, int flags)
{
    int error;
    void *handle;

    error = rfxcodec_decode_create_ex(max_width, max_height, format,
                                      flags, &handle);
    if (error == 0)
    {
        return handle; 
    }
    return NULL;
}

/******************************************************************************/
int
rfxcodec_decode_destroy(void *handle)
{
    struct rfxdecode *dec;

    dec = (struct rfxdecode *) handle;
    if (dec == NULL)
    {
        return 0;
    }
    if (dec->rects != NULL)
    {
        free(dec->rects);
    }
    if (dec->quants != NULL)
    {
        free(dec->quants);
    }
    if (dec->tiles != NULL)
    {
        free(dec->tiles);
    }
    free(dec);
    return 0;
}

/******************************************************************************/
int
rfxcodec_decode_ex(void *handle, char *cdata, int cdata_bytes,
                   char *ddata, int dwidth, int dheight, int dstride_bytes,
                   struct rfx_rect **rects, int *num_rects,
                   struct rfx_tile **tiles, int *num_tiles, int flags)
{
    struct rfxdecode *dec;
    STREAM ls;
    int error;

    LLOGLN(10, ("rfxcodec_decode_ex:"));
    dec = (struct rfxdecode *) handle;
    dec->dst_data = ddata;
    dec->dst_width = dwidth;
    dec->dst_height = dheight;
    dec->dst_stride_bytes = dstride_bytes;
    ls.data = (uint8 *) cdata;
    ls.p = ls.data;
    ls.size = cdata_bytes;
    error = rfx_decompose_message(dec, &ls);
    if (error != 0)
    {
        return error;
    }
    if (rects != NULL)
    {
        *rects = dec->rects;
        *num_rects = dec->num_rects;
    }
    if (tiles != NULL)
    {
        *tiles = dec->tiles;
        *num_tiles = dec->num_tiles;
    }
    return 0;
}

/******************************************************************************/
int
rfxcodec_decode(void *handle, char *cdata, int cdata_bytes,
                char *ddata, int dwidth, int dheight, int dstride_bytes)
{
    LLOGLN(10, ("rfxcodec_decode:"));
    return rfxcodec_decode_ex(handle, cdata, cdata_bytes,
                              ddata, dwidth, dheight, dstride_bytes,
                              0, 0, 0, 0, 0);
}

