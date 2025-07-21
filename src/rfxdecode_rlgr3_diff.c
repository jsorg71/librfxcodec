/**
 * FreeRDP: A Remote Desktop Protocol client.
 * RemoteFX Codec Library - RLGR
 *
 * Copyright 2011 Vic Lee
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

/**
 * This implementation of RLGR refers to
 * [MS-RDPRFX] 3.1.8.1.7.3 RLGR1/RLGR3 Pseudocode
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rfxcommon.h"
#include "rfxdecode_rlgr3_diff.h"

#define CheckRead do { \
    while (aval_bit_count < 24) \
    { \
        if (cdata >= cdata_end) \
        { \
            break; \
        } \
        aval_bits <<= 8; \
        aval_bits |= *(cdata++); \
        aval_bit_count += 8; \
    } \
} while (0)

#define ReadBits(_bits, _r)  do { \
    _r = (aval_bits >> (aval_bit_count - (_bits))) & \
                       ((1 << (_bits)) - 1); \
    aval_bit_count -= _bits; \
} while (0)

/* Outputs the Golomb/Rice encoding of a non-negative integer */
#define GetGRCode(_krp, _kr, _r) do { \
    int vk; \
    int bits; \
    uint16 lmag; \
    uint32 laval_bits_n; \
    uint32 bits_needed; \
    vk = 0; \
    CheckRead; \
    laval_bits_n = ~(aval_bits | (-1 << aval_bit_count)); \
    while (laval_bits_n == 0) \
    { \
        vk += aval_bit_count; \
        aval_bit_count = 0; \
        CheckRead; \
        if (aval_bit_count == 0) break; \
        laval_bits_n = ~(aval_bits | (-1 << aval_bit_count)); \
    } \
    if (laval_bits_n != 0) \
    { \
        GBSR(laval_bits_n, bits_needed); \
        bits_needed++; \
        vk += aval_bit_count - bits_needed; \
        aval_bit_count -= aval_bit_count - bits_needed + 1; \
    } \
    else \
    { \
        aval_bit_count -= 1; \
    } \
    CheckRead; \
    /* get next *kr bits, and combine with leading 1s */ \
    /*GetBits(_kr, bits);*/ \
    ReadBits(_kr, bits); \
    CheckRead; \
    lmag = (vk << _kr) | bits; \
    /* adjust krp and kr based on vk */ \
    if (!vk) \
    { \
        /*UpdateParam(_krp, -2, _kr);*/ \
        _krp += -2; \
        if (_krp < 0) \
        { \
            _krp = 0; \
        } \
        _kr = _krp >> 3; \
    } \
    else if (vk != 1) \
    { \
        /* at 1, no change! */ \
        /*UpdateParam(_krp, vk, _kr);*/ \
        _krp += vk; \
        if (_krp > 80) \
        { \
            _krp = 80; \
        } \
        _kr = _krp >> 3; \
    } \
    _r = lmag; \
} while (0)

/******************************************************************************/
int
rfx_rlgr3_decode_diff(const uint8 *cdata, int cdata_size, sint16 *coef)
{
    int k;
    int kp;
    int kr;
    int krp;
    //RFX_BITSTREAM bs;
    sint16 *coef_end;
    sint16 *coef_org;

    int aval_bit_count;
    unsigned int aval_bits;

    const uint8 * cdata_end;

    //rfx_bitstream_attach(bs, data, data_size);
    coef_end = coef + 4096;
    coef_org = coef;

    /* initialize the parameters */
    k = 1;
    kp = k << 3;
    kr = 1;
    krp = kr << 3;

    cdata_end = cdata + cdata_size;
    aval_bits = 0;
    aval_bit_count = 0;
    CheckRead;

    memset(coef, 0, 8192);

    //while (!rfx_bitstream_eos(bs))
    while (aval_bit_count > 0)
    {
        int run;
        if (k)
        {
            int mag;
            int index;
            uint32 sign;

            /* RL MODE */

            uint32 laval_bits;
            uint32 bits_needed;
            int lz;

            lz = 0;
            CheckRead;
            laval_bits = aval_bits & ((1 << aval_bit_count) - 1);
            while (laval_bits == 0)
            {
                lz += aval_bit_count;
                aval_bit_count = 0;
                CheckRead;
                laval_bits = aval_bits & ((1 << aval_bit_count) - 1);
            }
            if (laval_bits != 0)
            {
                GBSR(laval_bits, bits_needed);
                bits_needed++;
                lz += aval_bit_count - bits_needed;
                aval_bit_count -= aval_bit_count - bits_needed + 1;
            }
            else
            {
                aval_bit_count -= 1;
            }
            if (lz > 0)
            {
                int nskip;
                CheckRead;
                for (index = 0; index < lz; index++)
                {
                    nskip = 1 << k;
                    coef += nskip;
                    kp += 4;
                    if (kp > 80)
                    {
                        kp = 80;
                    }
                    k = kp >> 3;
                }

                if (coef >= coef_end)
                {
                    coef = coef_end;
                    break;
                }
            }

            /* next k bits will contain remaining run of zeros */
            //GetBits(k, run);
            ReadBits(k, run);

            CheckRead;

            //WriteZeroes(run);
            coef += run;

            /* get nonzero value, starting with sign bit and then GRCode for magnitude -1 */
            //GetBits(1, sign);
            ReadBits(1, sign);

            /* magnitude - 1 was coded (because it was nonzero) */
            GetGRCode(krp, kr, mag);
            mag++;

            //WriteValue(sign ? -mag : mag);
            if (sign)
            {
                *coef = -mag;
            }
            else
            {
                *coef = mag;
            }
            coef += 1;

            //UpdateParam(kp, -DN_GR, k); /* lower k and kp because of nonzero term */
            kp -= 6;
            if (kp < 0)
            {
                kp = 0;
            }
            k = kp >> 3;
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
            //GetMinBits(mag, nIdx);
            if (mag != 0)
            {
                GBSR(mag, nIdx);
                nIdx++;
            }
            else
            {
                nIdx = 0;
            }

            /* decode val1 is first term's (2 * mag - sign) value */
            //GetBits(nIdx, val1);
            ReadBits(nIdx, val1);

            CheckRead;

            /* val2 is second term's (2 * mag - sign) value */
            val2 = mag - val1;

            if (val1 != 0)
            {
                if (val2 != 0)
                {
                    /* raise k and kp if both terms nonzero */
                    //UpdateParam(kp, -2 * DQ_GR, k);
                    kp -= 6;
                    if (kp < 0)
                    {
                        kp = 0;
                    }
                    k = kp >> 3;
                }
            }
            else
            {
                if (val2 == 0)
                {
                    /* lower k and kp if both terms zero */
                    //UpdateParam(kp, 2 * UQ_GR, k);
                    kp += 6;
                    if (kp > 80)
                    {
                        kp = 80;
                    }
                    k = kp >> 3;
                }
            }

            //WriteValue(GetIntFrom2MagSign(val1));
            if (val1 & 1)
            {
                *coef = -1 * (sint16)(((val1) + 1) >> 1);
            }
            else
            {
                *coef = (sint16)((val1) >> 1);
            }
            coef += 1;

            //WriteValue(GetIntFrom2MagSign(val2));
            if (val2 & 1)
            {
                *coef = -1 * (sint16)(((val2) + 1) >> 1);
            }
            else
            {
                *coef = (sint16)((val2) >> 1);
            }
            coef += 1;

        }
        if (coef >= coef_end)
        {
            coef = coef_end;
            break;
        }
    }
    for (k = 4032; k < 4095; k++)
    {
        coef_org[k + 1] += coef_org[k];
    }
    return (int) (coef - coef_org);
}

