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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rfxcodec_decode.h>

#include "rfxcommon.h"
#include "rfxdecode.h"
#include "rfxdecode_decompose.h"
#include "rfxconstants.h"
#include "rfxdecode_tile.h"

#define LLOG_LEVEL 1
#define LLOGLN(_level, _args) \
    do { if (_level < LLOG_LEVEL) { printf _args ; printf("\n"); } } while (0)

/******************************************************************************/
static int
rfx_decompose_message_sync(struct rfxdecode *dec, STREAM *s)
{
    uint32 magic;

    /* RFX_SYNC */
    /* magic (4 bytes), 0xCACCACCA */
    stream_read_uint32(s, magic);
    if (magic != WF_MAGIC)
    {
        return 1;
    }
    /* version (2 bytes), WF_VERSION_1_0 (0x0100) */
    stream_read_uint16(s, dec->version);
    if (dec->version != WF_VERSION_1_0)
    {
        return 2;
    }
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_codec_versions(struct rfxdecode *dec, STREAM *s)
{
    int numCodecs;

    /* numCodecs (1 byte), must be set to 0x01 */
    stream_read_uint8(s, numCodecs);
    if (numCodecs != 1)
    {
        return 1;
    }
    /* RFX_CODEC_VERSIONT */
    /* codecId (1 byte) */
    stream_read_uint8(s, dec->codec_id);
    /* version (2 bytes) */
    stream_read_uint16(s, dec->codec_version);
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_channels(struct rfxdecode *dec, STREAM *s)
{
    uint8 numChannels;

    /* numChannels (1 byte), must bet set to 0x01 */
    stream_read_uint8(s, numChannels);
    if (numChannels != 1)
    {
        return 1;
    }
    /* RFX_CHANNELT */
    /* channelId (1 byte) */
    stream_read_uint8(s, dec->channel_id);
    /* width (2 bytes) */
    stream_read_uint16(s, dec->channel_width);
    /* height (2 bytes) */
    stream_read_uint16(s, dec->channel_height);
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_context(struct rfxdecode *dec, STREAM *s)
{
    uint16 properties;

    /* ctxId (1 byte), must be set to 0x00 */
    stream_read_uint8(s, dec->ctx_id);
    /* tileSize (2 bytes), must be set to CT_TILE_64x64 (0x0040) */
    stream_read_uint16(s, dec->tile_size);
    /* properties (2 bytes) */
    stream_read_uint16(s, properties);
    dec->properties = properties;
    dec->flags = (properties & 0x0007);
    switch ((properties & 0x1E00) >> 9)
    {
        case CLW_ENTROPY_RLGR1:
            dec->mode = RLGR1;
            dec->rfx_decode = dec->rfx_decode_rlgr1;
            break;
        case CLW_ENTROPY_RLGR3:
            dec->mode = RLGR3;
            dec->rfx_decode = dec->rfx_decode_rlgr3;
            break;
        default:
            return 1;
    }
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_frame_begin(struct rfxdecode *dec, STREAM *s)
{
    /* frameIdx (4 bytes), if codec is in video mode, must be ignored */
    stream_read_uint32(s, dec->frame_idx);
    /* numRegions (2 bytes) */
    stream_read_uint16(s, dec->num_regions);
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_frame_end(struct rfxdecode *dec, STREAM *s)
{
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_region(struct rfxdecode *dec, STREAM *s)
{
    int index;

    stream_read_uint8(s, dec->region_flags);
    stream_read_uint16(s, dec->num_rects);
    if (dec->num_rects < 1)
    {
        return 1;
    }
    if (dec->rects != NULL)
    {
        free(dec->rects);
    }
    dec->rects = (struct rfx_rect*)malloc(sizeof(struct rfx_rect) * dec->num_rects);
    if (dec->rects == NULL)
    {
        return 2;
    }
    /* rects */
    for (index = 0; index < dec->num_rects; index++)
    {
        /* RFX_RECT */
        stream_read_uint16(s, dec->rects[index].x);
        stream_read_uint16(s, dec->rects[index].y);
        stream_read_uint16(s, dec->rects[index].cx);
        stream_read_uint16(s, dec->rects[index].cy);
    }
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_tile_rgb(struct rfxdecode *dec, STREAM *s,
                               struct rfx_tile* tile)
{
    uint16 YLen, CbLen, CrLen;

    /* RFX_TILE */
    stream_read_uint8(s, tile->quant_y);
    stream_read_uint8(s, tile->quant_cb);
    stream_read_uint8(s, tile->quant_cr);
    stream_read_uint16(s, tile->x);
    stream_read_uint16(s, tile->y);
    LLOGLN(10, ("rfx_decompose_message_tile_rgb: x %d y %d", tile->x, tile->y));
    stream_read_uint16(s, YLen);
    stream_read_uint16(s, CbLen);
    stream_read_uint16(s, CrLen);
    LLOGLN(10, ("rfx_decompose_message_tile_rgb: YLen %d CbLen %d CrLen %d", YLen, CbLen, CrLen));
    tile->x *= 64;
    tile->y *= 64;
    tile->cx = 64;
    tile->cy = 64;
    if (rfx_decode_rgb(dec, s, tile, YLen, CbLen, CrLen) != 0)
    {
        return 1;
    }
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_tileset_rgb(struct rfxdecode *dec, STREAM *s)
{
    int index;
    uint16 subtype;
    uint32 blockLen;
    uint32 blockType;
    uint32 tilesDataSize;
    STREAM ls;

    if (dec->rfx_decode == NULL)
    {
        /* no decode function assigned */
        return 1;
    }
    /* subtype (2 bytes) must be set to CBT_TILESET (0xCAC2) */
    stream_read_uint16(s, subtype);
    if (subtype != CBT_TILESET)
    {
        return 2;
    }
    /* idx (2 bytes), must be set to 0x0000 */
    stream_seek(s, 2);
    /* properties (2 bytes) */
    stream_seek(s, 2);
    /* numQuant (1 byte) */
    stream_read_uint8(s, dec->num_quants);
    /* tileSize (1 byte), must be set to 0x40 */
    stream_seek(s, 1);
    if (dec->num_quants < 1)
    {
        return 3;
    }
    /* numTiles (2 bytes) */
    stream_read_uint16(s, dec->num_tiles);
    if (dec->num_tiles < 1)
    {
        return 4;
    }
    if (dec->tiles != NULL)
    {
        free(dec->tiles);
    }
    dec->tiles = (struct rfx_tile*)malloc(sizeof(struct rfx_tile) * dec->num_tiles);
    if (dec->tiles == NULL)
    {
        return 5;
    }
    memset(dec->tiles, 0, sizeof(struct rfx_tile) * dec->num_tiles);
    /* tilesDataSize (4 bytes) */
    stream_read_uint32(s, tilesDataSize);
    if (stream_get_left(s) < (int) (tilesDataSize + dec->num_quants * 5))
    {
        return 6;
    }
    if (dec->quants != NULL)
    {
        free(dec->quants);
    }
    dec->quants = (struct rfxdecode_quants*)
                  malloc(sizeof(struct rfxdecode_quants) * dec->num_quants);
    if (dec->quants == NULL)
    {
        return 7;
    }
    memset(dec->quants, 0, sizeof(struct rfxdecode_quants) * dec->num_quants);
    /* quantVals */
    for (index = 0; index < dec->num_quants; index++)
    {
        /* RFX_CODEC_QUANT */
        memcpy(dec->quants + index, s->p, 5);
        stream_seek(s, 5);
    }
    /* tiles */
    for (index = 0; index < dec->num_tiles; index++)
    {
        /* RFX_TILE */
        /* blockType (2 bytes), must be set to CBT_TILE (0xCAC3) */
        stream_read_uint16(s, blockType);
        /* blockLen (4 bytes) */
        stream_read_uint32(s, blockLen);
        if ((blockType != CBT_TILE) || (blockLen < 6))
        {
            return 8;
        }
        ls.data = s->p;
        ls.p = ls.data;
        ls.size = blockLen - 6;
        if (rfx_decompose_message_tile_rgb(dec, &ls, dec->tiles + index) != 0)
        {
            return 9;
        }
        stream_seek(s, blockLen - 6);
    }
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_tile_argb(struct rfxdecode *dec, STREAM *s,
                                struct rfx_tile* tile)
{
    uint16 YLen, CbLen, CrLen, ALen;

    /* RFX_TILE */
    stream_read_uint8(s, tile->quant_y);
    stream_read_uint8(s, tile->quant_cb);
    stream_read_uint8(s, tile->quant_cr);
    stream_read_uint16(s, tile->x);
    stream_read_uint16(s, tile->y);
    LLOGLN(10, ("rfx_decompose_message_tile_argb: x %d y %d", tile->x, tile->y));
    stream_read_uint16(s, YLen);
    stream_read_uint16(s, CbLen);
    stream_read_uint16(s, CrLen);
    stream_read_uint16(s, ALen);
    LLOGLN(10, ("rfx_decompose_message_tile_argb: YLen %d CbLen %d CrLen %d ALen %d", YLen, CbLen, CrLen, ALen));
    tile->x *= 64;
    tile->y *= 64;
    tile->cx = 64;
    tile->cy = 64;
    if (rfx_decode_argb(dec, s, tile, YLen, CbLen, CrLen, ALen) != 0)
    {
        return 1;
    }
    return 0;
}

/******************************************************************************/
static int
rfx_decompose_message_tileset_argb(struct rfxdecode *dec, STREAM *s)
{
    int index;
    uint16 subtype;
    uint32 blockLen;
    uint32 blockType;
    uint32 tilesDataSize;
    STREAM ls;

    if (dec->rfx_decode == NULL)
    {
        /* no decode function assigned */
        return 1;
    }
    /* subtype (2 bytes) must be set to CBT_TILESET (0xCAC2) */
    stream_read_uint16(s, subtype);
    if (subtype != CBT_TILESET)
    {
        return 2;
    }
    /* idx (2 bytes), must be set to 0x0000 */
    stream_seek(s, 2);
    /* properties (2 bytes) */
    stream_seek(s, 2);
    /* numQuant (1 byte) */
    stream_read_uint8(s, dec->num_quants);
    /* tileSize (1 byte), must be set to 0x40 */
    stream_seek(s, 1);
    if (dec->num_quants < 1)
    {
        return 3;
    }
    /* numTiles (2 bytes) */
    stream_read_uint16(s, dec->num_tiles);
    if (dec->num_tiles < 1)
    {
        return 4;
    }
    if (dec->tiles != NULL)
    {
        free(dec->tiles);
    }
    dec->tiles = (struct rfx_tile*)malloc(sizeof(struct rfx_tile) * dec->num_tiles);
    if (dec->tiles == NULL)
    {
        return 5;
    }
    memset(dec->tiles, 0, sizeof(struct rfx_tile) * dec->num_tiles);
    /* tilesDataSize (4 bytes) */
    stream_read_uint32(s, tilesDataSize);
    if (stream_get_left(s) < (int) (tilesDataSize + dec->num_quants * 5))
    {
        return 6;
    }
    if (dec->quants != NULL)
    {
        free(dec->quants);
    }
    dec->quants = (struct rfxdecode_quants*)
                  malloc(sizeof(struct rfxdecode_quants) * dec->num_quants);
    if (dec->quants == NULL)
    {
        return 7;
    }
    memset(dec->quants, 0, sizeof(struct rfxdecode_quants) * dec->num_quants);
    /* quantVals */
    for (index = 0; index < dec->num_quants; index++)
    {
        /* RFX_CODEC_QUANT */
        memcpy(dec->quants + index, s->p, 5);
        stream_seek(s, 5);
    }
    /* tiles */
    for (index = 0; index < dec->num_tiles; index++)
    {
        /* RFX_TILE */
        /* blockType (2 bytes), must be set to CBT_TILE (0xCAC3) */
        stream_read_uint16(s, blockType);
        /* blockLen (4 bytes) */
        stream_read_uint32(s, blockLen);
        if ((blockType != CBT_TILE) || (blockLen < 6))
        {
            return 8;
        }
        ls.data = s->p;
        ls.p = ls.data;
        ls.size = blockLen - 6;
        if (rfx_decompose_message_tile_argb(dec, &ls, dec->tiles + index) != 0)
        {
            return 9;
        }
        stream_seek(s, blockLen - 6);
    }
    return 0;
}

/******************************************************************************/
int
rfx_decompose_message(struct rfxdecode *dec, STREAM *s)
{
    int blockType;
    int blockLen;
    uint8* holdp;
    STREAM ls;

    while (stream_get_left(s) > 5)
    {
        /* RFX_BLOCKT */
        holdp = s->p;
        stream_read_uint16(s, blockType);
        stream_read_uint32(s, blockLen);
        LLOGLN(10, ("rfx_decompose_message: blockType 0x%4.4x blockLen %d",
               blockType, blockLen));
        if ((blockLen < 6) || (stream_get_left(s) < (blockLen - 6)))
        {
            return 1;
        }
        ls.data = s->p;
        ls.p = ls.data;
        ls.size = blockLen - 6;
        switch (blockType)
        {
            case WBT_SYNC:
                if (rfx_decompose_message_sync(dec, &ls) != 0)
                {
                    return 2;
                }
                break;
            case WBT_CODEC_VERSIONS:
                if (rfx_decompose_message_codec_versions(dec, &ls) != 0)
                {
                    return 3;
                }
                break;
            case WBT_CHANNELS:
                if (rfx_decompose_message_channels(dec, &ls) != 0)
                {
                    return 4;
                }
                break;
            case WBT_CONTEXT:
                ls.p += 2; /* codecId (1 byte) channelId (1 byte) */
                if (rfx_decompose_message_context(dec, &ls) != 0)
                {
                    return 5;
                }
                break;
            case WBT_FRAME_BEGIN:
                ls.p += 2; /* codecId (1 byte) channelId (1 byte) */
                if (rfx_decompose_message_frame_begin(dec, &ls) != 0)
                {
                    return 6;
                }
                break;
            case WBT_FRAME_END:
                ls.p += 2; /* codecId (1 byte) channelId (1 byte) */
                if (rfx_decompose_message_frame_end(dec, &ls) != 0)
                {
                    return 7;
                }
                break;
            case WBT_REGION:
                ls.p += 2; /* codecId (1 byte) channelId (1 byte) */
                if (rfx_decompose_message_region(dec, &ls) != 0)
                {
                    return 8;
                }
                break;
            case WBT_EXTENSION:
                ls.p += 2; /* codecId (1 byte) channelId (1 byte) */
                if (rfx_decompose_message_tileset_rgb(dec, &ls) != 0)
                {
                    return 9;
                }
                break;
            case WBT_EXTENSION_PLUS:
                ls.p += 2; /* codecId (1 byte) channelId (1 byte) */
                if (rfx_decompose_message_tileset_argb(dec, &ls) != 0)
                {
                    return 10;
                }
                break;
            default:
                LLOGLN(0, ("rfx_decompose_message: unknown blockType 0x%X", blockType));
                break;
        }
        s->p = holdp + blockLen;
    }
    return 0;
}

