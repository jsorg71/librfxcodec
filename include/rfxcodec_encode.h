/**
 * RFX codec encoder
 *
 * Copyright 2014-2015 Jay Sorg <jay.sorg@gmail.com>
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

#ifndef __RFXCODEC_ENCODE_H
#define __RFXCODEC_ENCODE_H

#include <rfxcodec_common.h>

struct rfx_rect
{
    int x;
    int y;
    int cx;
    int cy;
};

struct rfx_tile
{
    int x; /* multiple of 64 */
    int y; /* multiple of 64 */
    int cx; /* must be 64 or less */
    int cy; /* must be 64 or less */
    int quant_y;
    int quant_cb;
    int quant_cr;
};

void *
rfxcodec_encode_create(int width, int height, int format, int flags);
int
rfxcodec_encode_create_ex(int width, int height, int format, int flags,
                          void **handle);
int
rfxcodec_encode_destroy(void *handle);
/* quants, 5 ints per set, should be num_quants * 5 chars in quants)
 * each char is 2 quant values
 * quantizer order is
 * 0 - LL3
 * 1 - LH3
 * 2 - HL3
 * 3 - HH3
 * 4 - LH2
 * 5 - HL2
 * 6 - HH2
 * 7 - LH1
 * 8 - HL1
 * 9 - HH1 */
int
rfxcodec_encode(void *handle, char *cdata, int *cdata_bytes,
                const char *buf, int width, int height, int stride_bytes,
                const struct rfx_rect *region, int num_region,
                const struct rfx_tile *tiles, int num_tiles,
                const char *quants, int num_quants);
int
rfxcodec_encode_ex(void *handle, char *cdata, int *cdata_bytes,
                   const char *buf, int width, int height, int stride_bytes,
                   const struct rfx_rect *region, int num_region,
                   const struct rfx_tile *tiles, int num_tiles,
                   const char *quants, int num_quants, int flags);

/* access to low level functions, use load library since some function might
   not get compiled in */
/* use simple types here, no sint16_t, uint8_t... */

typedef int (*rfx_rlgr1_encode_proc)(const short *data, unsigned char *buffer, int buffer_size);
typedef int (*rfx_rlgr3_encode_proc)(const short *data, unsigned char *buffer, int buffer_size);
typedef int (*rfx_differential_encode_proc)(short *buffer, int buffer_size);
typedef int (*rfx_quantization_encode_proc)(short *buffer, const char *quantization_values);
typedef int (*rfx_dwt_2d_encode_proc)(const unsigned char *in_buffer, short *buffer, short *dwt_buffer);

typedef int (*rfx_encode_diff_rlgr1_proc)(short *coef, unsigned char *cdata, int cdata_size);
typedef int (*rfx_encode_diff_rlgr3_proc)(short *coef, unsigned char *cdata, int cdata_size);

typedef int (*cpuid_x86_proc)(int eax_in, int ecx_in, int *eax, int *ebx, int *ecx, int *edx);
typedef int (*rfxcodec_encode_dwt_shift_x86_sse2_proc)(const char *qtable, const unsigned char *data, short *dwt_buffer1, short *dwt_buffer);
typedef int (*rfxcodec_encode_dwt_shift_x86_sse41_proc)(const char *qtable, const unsigned char *data, short *dwt_buffer1, short *dwt_buffer);

typedef int (*cpuid_amd64_proc)(int eax_in, int ecx_in, int *eax, int *ebx, int *ecx, int *edx);
typedef int (*rfxcodec_encode_dwt_shift_amd64_sse2_proc)(const char *qtable, const unsigned char *data, short *dwt_buffer1, short *dwt_buffer);
typedef int (*rfxcodec_encode_dwt_shift_amd64_sse41_proc)(const char *qtable, const unsigned char *data, short *dwt_buffer1, short *dwt_buffer);

#endif
