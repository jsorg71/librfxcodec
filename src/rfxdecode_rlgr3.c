/**
 * FreeRDP: A Remote Desktop Protocol client.
 * RemoteFX Codec Library - RLGR
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

/**
 * This implementation of RLGR refers to
 * [MS-RDPRFX] 3.1.8.1.7.3 RLGR1/RLGR3 Pseudocode
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rfx_bitstream.h"
#include "rfxdecode_rlgr3.h"

/* Constants used within the RLGR1/RLGR3 algorithm */
#define KPMAX   (80)  /* max value for kp or krp */
#define LSGR    (3)   /* shift count to convert kp to k */
#define UP_GR   (4)   /* increase in kp after a zero run in RL mode */
#define DN_GR   (6)   /* decrease in kp after a nonzero symbol in RL mode */
#define UQ_GR   (3)   /* increase in kp after nonzero symbol in GR mode */
#define DQ_GR   (3)   /* decrease in kp after zero symbol in GR mode */

/* Gets (returns) the next nBits from the bitstream */
#define GetBits(nBits, _r) rfx_bitstream_get_bits(bs, nBits, _r)

/* From current output pointer, write "value", check and update buffer_size */
#define WriteValue(value)                                   \
do {                                                        \
    if (buffer_size > 0)                                    \
        *dst++ = (value);                                   \
    buffer_size--;                                          \
} while (0)

/* From current output pointer, write next nZeroes terms with value 0, check and update buffer_size */
#define WriteZeroes(nZeroes)                                \
do {                                                        \
    int nZeroesWritten = (nZeroes);                         \
    if (nZeroesWritten > buffer_size)                       \
        nZeroesWritten = buffer_size;                       \
    if (nZeroesWritten > 0)                                 \
    {                                                       \
        memset(dst, 0, nZeroesWritten * sizeof(sint16));    \
        dst += nZeroesWritten;                              \
    }                                                       \
    buffer_size -= (nZeroes);                               \
} while (0)

/* Returns the least number of bits required to represent a given value */
#define GetMinBits(_val, _nbits)                            \
do {                                                        \
    uint32 _v = _val;                                       \
    _nbits = 0;                                             \
    while (_v)                                              \
    {                                                       \
        _v >>= 1;                                           \
        _nbits++;                                           \
    }                                                       \
} while (0)

/* Converts from (2 * magnitude - sign) to integer */
#define GetIntFrom2MagSign(twoMs) (((twoMs) & 1) ? -1 * (sint16)(((twoMs) + 1) >> 1) : (sint16)((twoMs) >> 1))

/*
 * Update the passed parameter and clamp it to the range [0, KPMAX]
 * Return the value of parameter right-shifted by LSGR
 */
#define UpdateParam(_param, _deltaP, _k)                    \
do {                                                        \
    _param += _deltaP;                                      \
    if (_param > KPMAX)                                     \
        _param = KPMAX;                                     \
    if (_param < 0)                                         \
        _param = 0;                                         \
    _k = (_param >> LSGR);                                  \
} while (0)

/* Outputs the Golomb/Rice encoding of a non-negative integer */
#define GetGRCode(_krp, _kr, _r) do {                       \
    int vk;                                                 \
    int bits;                                               \
    uint16 lmag;                                            \
                                                            \
    /* chew up/count leading 1s and escape 0 */             \
    GetBits(1, bits);                                       \
    for (vk = 0; bits == 1;)                                \
    {                                                       \
        vk++;                                               \
        GetBits(1, bits);                                   \
    }                                                       \
    /* get next *kr bits, and combine with leading 1s */    \
    GetBits(_kr, bits);                                     \
    lmag = (vk << _kr) | bits;                              \
    /* adjust krp and kr based on vk */                     \
    if (!vk)                                                \
    {                                                       \
        UpdateParam(_krp, -2, _kr);                         \
    }                                                       \
    else if (vk != 1)                                       \
    {                                                       \
        /* at 1, no change! */                              \
        UpdateParam(_krp, vk, _kr);                         \
    }                                                       \
    _r = lmag;                                              \
} while (0)

/******************************************************************************/
int
rfx_rlgr3_decode(const uint8 * data, int data_size,
                 sint16 * buffer, int buffer_size)
{
    int k;
    int kp;
    int kr;
    int krp;
    int bits;
    sint16 * dst;
    RFX_BITSTREAM bs;

    rfx_bitstream_attach(bs, data, data_size);
    dst = buffer;

    /* initialize the parameters */
    k = 1;
    kp = k << LSGR;
    kr = 1;
    krp = kr << LSGR;

    while (!rfx_bitstream_eos(bs))
    {
        int run;
        if (k)
        {
            int mag;
            uint32 sign;

            /* RL MODE */
            GetBits(1, bits);
            while (!rfx_bitstream_eos(bs) && bits == 0)
            {
                /* we have an RL escape "0", which translates to a run (1<<k) of zeros */
                WriteZeroes(1 << k);
                UpdateParam(kp, UP_GR, k); /* raise k and kp up because of zero run */
                GetBits(1, bits);
            }

            /* next k bits will contain remaining run or zeros */
            GetBits(k, run);
            WriteZeroes(run);

            /* get nonzero value, starting with sign bit and then GRCode for magnitude -1 */
            GetBits(1, sign);

            /* magnitude - 1 was coded (because it was nonzero) */
            GetGRCode(krp, kr, mag);
            mag++;

            WriteValue(sign ? -mag : mag);
            UpdateParam(kp, -DN_GR, k); /* lower k and kp because of nonzero term */
        }
        else
        {
            uint32 mag;
            uint32 nIdx;
            uint32 val1;
            uint32 val2;

            /* GR (GOLOMB-RICE) MODE */
            GetGRCode(krp, kr, mag); /* values coded are 2 * magnitude - sign */

            /*
             * In GR mode FOR RLGR3, we have encoded the
             * sum of two (2 * mag - sign) values
             */

            /* maximum possible bits for first term */
            GetMinBits(mag, nIdx);

            /* decode val1 is first term's (2 * mag - sign) value */
            GetBits(nIdx, val1);

            /* val2 is second term's (2 * mag - sign) value */
            val2 = mag - val1;

            if (val1 && val2)
            {
                    /* raise k and kp if both terms nonzero */
                    UpdateParam(kp, -2 * DQ_GR, k);
            }
            else if (!val1 && !val2)
            {
                    /* lower k and kp if both terms zero */
                    UpdateParam(kp, 2 * UQ_GR, k);
            }

            WriteValue(GetIntFrom2MagSign(val1));
            WriteValue(GetIntFrom2MagSign(val2));
        }
    }

    return (int) (dst - buffer);
}

