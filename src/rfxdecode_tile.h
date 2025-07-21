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

#ifndef __RFXDECODE_TILE_H
#define __RFXDECODE_TILE_H

#include "rfxcommon.h"

int
rfx_decode_rgb(struct rfxdecode *dec, STREAM *s, struct rfx_tile* tile,
               int YLen, int CbLen, int CrLen);
int
rfx_decode_argb(struct rfxdecode *dec, STREAM *s, struct rfx_tile* tile,
                int YLen, int CbLen, int CrLen, int ALen);

int
rfx_decode_yuva2argb(struct rfxdecode *dec, int x, int y);
int
rfx_decode_yuva2abgr(struct rfxdecode *dec, int x, int y);
int
rfx_decode_yuva2rgb(struct rfxdecode *dec, int x, int y);
int
rfx_decode_yuva2bgr(struct rfxdecode *dec, int x, int y);
int
rfx_decode_yuva2argb_x86_sse2(struct rfxdecode *dec, int x, int y);
int
rfx_decode_yuva2argb_x86_sse41(struct rfxdecode *dec, int x, int y);
int
rfx_decode_yuva2argb_amd64_sse2(struct rfxdecode *dec, int x, int y);
int
rfx_decode_yuva2argb_amd64_sse41(struct rfxdecode *dec, int x, int y);

int
rfx_decode_component_rlgr1(struct rfxdecode *dec,
                           uint8* cdata, int cdata_bytes,
                           uint8* dst_data, int quant);
int
rfx_decode_component_rlgr3(struct rfxdecode *dec,
                           uint8* cdata, int cdata_bytes,
                           uint8* dst_data, int quant);
int
rfx_decode_component_rlgr1_x86_sse2(struct rfxdecode *dec,
                                    uint8* cdata, int cdata_bytes,
                                    uint8* dst_data, int quant);
int
rfx_decode_component_rlgr3_x86_sse2(struct rfxdecode *dec,
                                    uint8* cdata, int cdata_bytes,
                                    uint8* dst_data, int quant);
int
rfx_decode_component_rlgr1_amd64_sse2(struct rfxdecode *dec,
                                      uint8* cdata, int cdata_bytes,
                                      uint8* dst_data, int quant);
int
rfx_decode_component_rlgr3_amd64_sse2(struct rfxdecode *dec,
                                      uint8* cdata, int cdata_bytes,
                                      uint8* dst_data, int quant);

#endif /* __RFXDECODE_TILE_H */

