/**
 * RFX codec decoder
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

#ifndef __RFXDECODE_H
#define __RFXDECODE_H

struct rfxdecode;

typedef int (*rfx_decode_proc)(struct rfxdecode *dec,
                               uint8* cdata, int cdata_bytes,
                               uint8* dst_data, int quant);

typedef int (*rfx_decode_cc_proc)(struct rfxdecode *dec, int x, int y);

struct rfxdecode_quants
{
    uint8 quants[8]; /* 5 bytes or 10 nibbles are used for quants */
};

struct rfxdecode
{
    int version;
    int codec_id;
    int codec_version;
    int channel_id;
    int properties;
    int flags;
    int ctx_id;
    int tile_size;
    int frame_idx;
    int num_regions;

    int num_quants;
    int pad0;
    struct rfxdecode_quants* quants;

    int region_flags;
    int num_rects;
    struct rfx_rect* rects;

    int num_tiles;
    int pad1;
    struct rfx_tile* tiles;

    /* dst info */
    char *dst_data;
    int dst_width;
    int dst_height;
    int dst_stride_bytes;

    int max_width;
    int max_height;

    int channel_width;
    int channel_height;

    int header_processed;
    int mode;
    int bits_per_pixel;
    int format;
    int pad2[5];

    uint8 pad3[16];
    uint8 a_buffer_a[4096];
    uint8 y_buffer_a[4096];
    uint8 u_buffer_a[4096];
    uint8 v_buffer_a[4096];
    uint8 pad4[16];
    sint16 dwt_buffer_a[4096];
    sint16 dwt_buffer1_a[4096];
    sint16 dwt_buffer2_a[4096];
    uint8 pad5[16];
    sint16* dwt_buffer;
    sint16* dwt_buffer1;
    sint16* dwt_buffer2;
    uint8* a_buffer;
    uint8* y_buffer;
    uint8* u_buffer;
    uint8* v_buffer;

    rfx_decode_proc rfx_decode_rlgr1;
    rfx_decode_proc rfx_decode_rlgr3;
    rfx_decode_proc rfx_decode;

    rfx_decode_cc_proc rfx_decode_cc;

    int got_sse2;
    int got_sse3;
    int got_sse41;
    int got_sse42;
    int got_sse4a;
    int got_popcnt;
    int got_lzcnt;
    int got_neon;
};

#endif

