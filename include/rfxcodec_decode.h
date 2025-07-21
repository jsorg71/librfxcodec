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

#ifndef __RFXCODEC_DECODE_H
#define __RFXCODEC_DECODE_H

#include <rfxcodec_common.h>

int
rfxcodec_decode_get_version(int *major, int *minor, int *micro);
void *
rfxcodec_decode_create(int max_width, int max_height, int format, int flags);
int
rfxcodec_decode_create_ex(int max_width, int max_height, int format,
                          int flags, void **handle);
int
rfxcodec_decode_destroy(void *handle);
int
rfxcodec_decode(void *handle, char *cdata, int cdata_bytes,
                char *ddata, int dwidth, int dheight, int dstride_bytes);
int
rfxcodec_decode_ex(void *handle, char *cdata, int cdata_bytes,
                   char *ddata, int dwidth, int dheight, int dstride_bytes,
                   struct rfx_rect **rects, int *num_rects,
                   struct rfx_tile **tiles, int *num_tiles, int flags);

#endif
