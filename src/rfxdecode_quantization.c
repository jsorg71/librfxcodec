/**
 * FreeRDP: A Remote Desktop Protocol client.
 * RemoteFX Codec Library - Quantization
 *
 * Copyright 2011 Vic Lee
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

#include "rfxcommon.h"
#include "rfxdecode_quantization.h"

/******************************************************************************/
static void
rfx_quantization_decode_block(sint16 * buffer, int buffer_size, sint32 factor)
{
    sint16 * dst;

    factor += DWT_FACTOR;
    if (factor <= 0)
    {
        return;
    }
    for (dst = buffer; buffer_size > 0; dst++, buffer_size--)
    {
        *dst <<= factor;
    }
}

/******************************************************************************/
int
rfx_quantization_decode(sint16 * buffer, const uint8* qtable)
{
    sint32 factor;

    factor = ((qtable[4] >> 0) & 0xf) - 6;
    rfx_quantization_decode_block(buffer, 1024, factor); /* HL1 */
    factor = ((qtable[3] >> 4) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 1024, 1024, factor); /* LH1 */
    factor = ((qtable[4] >> 4) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 2048, 1024, factor); /* HH1 */
    factor = ((qtable[2] >> 4) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 3072, 256, factor); /* HL2 */
    factor = ((qtable[2] >> 0) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 3328, 256, factor); /* LH2 */
    factor = ((qtable[3] >> 0) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 3584, 256, factor); /* HH2 */
    factor = ((qtable[1] >> 0) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 3840, 64, factor); /* HL3 */
    factor = ((qtable[0] >> 4) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 3904, 64, factor); /* LH3 */
    factor = ((qtable[1] >> 4) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 3968, 64, factor); /* HH3 */
    factor = ((qtable[0] >> 0) & 0xf) - 6;
    rfx_quantization_decode_block(buffer + 4032, 64, factor); /* LL3 */
    return 0; 
}

