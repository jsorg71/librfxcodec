/**
 * RemoteFX Codec Library
 *
 * Copyright 2020-2024 Jay Sorg <jay.sorg@gmail.com>
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
 *
 * DWT Reduce-Extrapolate Method MS-RDPEGFX 3.2.8.1.2.2
 * also does Quantization and Linearization 3.2.8.1.3
 */

#if defined(HAVE_CONFIG_H)
#include <config_ac.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rfxcommon.h"
#include "rfxencode_dwt_shift_rem.h"

/*
    LL0 -> L0, H0       - level 1 vertical

    L0 -> LL1, HL1      - level 1 horizontal
    H0 -> LH1, HH1

    LL1 -> L1, H1       - level 2 vertical

    L1 -> LL2, HL2      - level 2 horizontal
    H1 -> LH2, HH2

    LL2 -> L2, H2       - level 3 vertical

    L2 -> LL3, HL3      - level 3 horizontal
    H2 -> LH3, HH3
*/

/* level 1      LL0 -> L0, H0 */
#define SETUP_IC_LL0(_y) in_buffer + (_y)
#define SETUP_OC_L0(_y) out_buffer + (_y)
#define SETUP_OC_H0(_y) out_buffer + 64 * 33 + (_y)
#define IC_LL0_U8(_offset) (ic[(_offset) * 64] - 128) << DWT_FACTOR
#define IC_LL0_S16(_offset) ic[(_offset) * 64]
#define OC_L0(_offset) lo[(_offset) * 64]
#define OC_H0(_offset) hi[(_offset) * 64]
/*              L0 -> LL1, HL1 */
#define SETUP_IC_L0(_y) in_buffer + 64 * (_y)
#define SETUP_OC_LL1(_y) out_buffer + 31 * 33 + 33 * 31 + 31 * 31 + 33 * (_y)
#define SETUP_OC_HL1(_y) out_buffer + 31 * (_y);
#define IC_L0(_offset) (ic[(_offset)])
#define OC_LL1(_offset) lo[(_offset)]
#define OC_HL1(_offset) hi[(_offset)]
/*              H0 -> LH1, HH1 */
#define SETUP_IC_H0(_y) in_buffer + 64 * (33 + (_y))
#define SETUP_OC_LH1(_y) out_buffer + 31 * 33 + 33 * (_y)
#define SETUP_OC_HH1(_y) out_buffer + 31 * 33 + 33 * 31 + 31 * (_y)
#define IC_H0(_offset) (ic[(_offset)])
#define OC_LH1(_offset) lo[(_offset)]
#define OC_HH1(_offset) hi[(_offset)]

/* level 2      LL1 -> L1, H1 */
#define SETUP_IC_LL1(_y) in_buffer + (_y)
#define SETUP_OC_L1(_y) out_buffer + (_y)
#define SETUP_OC_H1(_y) out_buffer + 33 * 17 + (_y)
#define IC_LL1(_offset) ic[(_offset) * 33]
#define OC_L1(_offset) lo[(_offset) * 33]
#define OC_H1(_offset) hi[(_offset) * 33]
/*              L1 -> LL2, HL2 */
#define SETUP_IC_L1(_y) in_buffer + 33 * (_y)
#define SETUP_OC_LL2(_y) out_buffer + 16 * 17 + 17 * 16 + 16 * 16 + 17 * (_y)
#define SETUP_OC_HL2(_y) out_buffer + 16 * (_y)
#define IC_L1(_offset) (ic[(_offset)])
#define OC_LL2(_offset) lo[(_offset)]
#define OC_HL2(_offset) hi[(_offset)]
/*              H1 -> LH2, HH2 */
#define SETUP_IC_H1(_y) in_buffer + 33 * (17 + (_y))
#define SETUP_OC_LH2(_y) out_buffer + 16 * 17 + 17 * (_y)
#define SETUP_OC_HH2(_y) out_buffer + 16 * 17 + 17 * 16 + 16 * (_y)
#define IC_H1(_offset) (ic[(_offset)])
#define OC_LH2(_offset) lo[(_offset)]
#define OC_HH2(_offset) hi[(_offset)]

/* level 3      LL2 -> L2, H2 */
#define SETUP_IC_LL2(_y) in_buffer + (_y)
#define SETUP_OC_L2(_y) out_buffer + (_y)
#define SETUP_OC_H2(_y) out_buffer + 17 * 9 + (_y)
#define IC_LL2(_offset) ic[(_offset) * 17]
#define OC_L2(_offset) lo[(_offset) * 17]
#define OC_H2(_offset) hi[(_offset) * 17]
/*              L2 -> LL3, HL3 */
#define SETUP_IC_L2(_y) in_buffer + 17 * (_y)
#define SETUP_OC_LL3(_y) out_buffer + 8 * 9 + 9 * 8 + 8 * 8 + 9 * (_y)
#define SETUP_OC_HL3(_y) out_buffer + 8 * (_y)
#define IC_L2(_offset) (ic[(_offset)])
#define OC_LL3(_offset) lo[(_offset)]
#define OC_HL3(_offset) hi[(_offset)]
/*              H2 -> LH3, HH3 */
#define SETUP_IC_H2(_y) in_buffer + 17 * (9 + (_y))
#define SETUP_OC_LH3(_y) out_buffer + 8 * 9 + 9 * (_y)
#define SETUP_OC_HH3(_y) out_buffer + 8 * 9 + 9 * 8 + 8 * (_y)
#define IC_H2(_offset) (ic[(_offset)])
#define OC_LH3(_offset) lo[(_offset)]
#define OC_HH3(_offset) hi[(_offset)]

#define SETUPLOQ(_index, _shift) do { \
    lo_fact = (((quants[_index] >> (_shift)) & 0xf) - 6) + DWT_FACTOR; \
    lo_half = 1 << (lo_fact - 1); } while (0)
#define SETUPHIQ(_index, _shift) do { \
    hi_fact = (((quants[_index] >> (_shift)) & 0xf) - 6) + DWT_FACTOR; \
    hi_half = 1 << (hi_fact - 1); } while (0)
#define LOQ(_val) ((_val) + lo_half) >> lo_fact
#define HIQ(_val) ((_val) + hi_half) >> hi_fact
#define NOQ(_val) _val

/******************************************************************************/
static void
rfx_encode_dwt_shift_rem_vert_lv1_u8(const uint8 *in_buffer,
                                     sint16 *out_buffer)
{
    const uint8 *ic; /* input coefficients */
    sint16 *lo;
    sint16 *hi;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    sint16 ic62;
    int n;
    int y;

    for (y = 0; y < 64; y++)
    {
        /* setup */
        ic = SETUP_IC_LL0(y);
        lo = SETUP_OC_L0(y);
        hi = SETUP_OC_H0(y);
        /* pre */
        x2n = IC_LL0_U8(0);
        x2n1 = IC_LL0_U8(1);
        x2n2 = IC_LL0_U8(2);
        OC_H0(0) = NOQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_L0(0) = NOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 31; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_LL0_U8(2 * n + 1);
            x2n2 = IC_LL0_U8(2 * n + 2);
            OC_H0(n) = NOQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_L0(n) = NOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic62 = x2n = x2n2;
        x2n1 = IC_LL0_U8(63);
        x2n2 = 2 * x2n1 - x2n; /* ic[64] = 2 * ic[63] - ic[62] */
        OC_L0(31) = NOQ(x2n + (hn1 >> 1));
        /* post ex */
        x2n = x2n2;
        /* x2n1 already set, mirror 65 -> 63 */
        x2n2 = ic62;      /* mirror 66 -> 62 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_L0(32) = NOQ(x2n + (hn >> 1));
    }
}

/******************************************************************************/
static void
rfx_encode_dwt_shift_rem_horz_lv1(const sint16 *in_buffer, sint16 *out_buffer,
                                  const char *quants)
{
    const sint16 *ic; /* input coefficients */
    sint16 *lo;
    sint16 *hi;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    sint16 ic62;
    int n;
    int y;
    int lo_fact;
    int hi_fact;
    int lo_half;
    int hi_half;

    /* LL1 no Q */
    SETUPHIQ(4, 0); /* HL1 */
    for (y = 0; y < 33; y++) /* lo */
    {
        /* setup */
        ic = SETUP_IC_L0(y);
        lo = SETUP_OC_LL1(y);
        hi = SETUP_OC_HL1(y);
        /* pre */
        x2n = IC_L0(0);
        x2n1 = IC_L0(1);
        x2n2 = IC_L0(2);
        OC_HL1(0) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LL1(0) = NOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 31; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_L0(2 * n + 1);
            x2n2 = IC_L0(2 * n + 2);
            OC_HL1(n) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_LL1(n) = NOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic62 = x2n = x2n2;
        x2n1 = IC_L0(63);
        x2n2 = 2 * x2n1 - x2n; /* ic[64] = 2 * ic[63] - ic[62] */
        OC_LL1(31) = NOQ(x2n + (hn1 >> 1));
        /* post ex */
        x2n = x2n2;
        /* x2n1 already set, mirror 65 -> 63 */
        x2n2 = ic62;      /* mirror 66 -> 62 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_LL1(32) = NOQ(x2n + (hn >> 1));
    }
    SETUPLOQ(3, 4); /* LH1 */
    SETUPHIQ(4, 4); /* HH1 */
    for (y = 0; y < 31; y++) /* hi */
    {
        /* setup */
        ic = SETUP_IC_H0(y);
        lo = SETUP_OC_LH1(y);
        hi = SETUP_OC_HH1(y);
        /* pre */
        x2n = IC_H0(0);
        x2n1 = IC_H0(1);
        x2n2 = IC_H0(2);
        OC_HH1(0) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LH1(0) = LOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 31; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_H0(2 * n + 1);
            x2n2 = IC_H0(2 * n + 2);
            OC_HH1(n) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_LH1(n) = LOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic62 = x2n = x2n2;
        x2n1 = IC_H0(63);
        x2n2 = 2 * x2n1 - x2n; /* ic[64] = 2 * ic[63] - ic[62] */
        OC_LH1(31) = LOQ(x2n + (hn1 >> 1));
        /* post ex */
        x2n = x2n2;
        /* x2n1 already set, mirror 65 -> 63 */
        x2n2 = ic62;      /* mirror 66 -> 62 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_LH1(32) = LOQ(x2n + (hn >> 1));
    }
}

/******************************************************************************/
static void
rfx_encode_dwt_shift_rem_vert_lv2(const sint16 *in_buffer, sint16 *out_buffer)
{
    const sint16 *ic; /* input coefficients */
    sint16 *lo;
    sint16 *hi;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    sint16 ic30;
    int n;
    int y;

    for (y = 0; y < 33; y++)
    {
        /* setup */
        ic = SETUP_IC_LL1(y);
        lo = SETUP_OC_L1(y);
        hi = SETUP_OC_H1(y);
        /* pre */
        x2n = IC_LL1(0);
        x2n1 = IC_LL1(1);
        x2n2 = IC_LL1(2);
        OC_H1(0) = NOQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_L1(0) = NOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 15; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_LL1(2 * n + 1);
            x2n2 = IC_LL1(2 * n + 2);
            OC_H1(n) = NOQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_L1(n) = NOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic30 = x2n = x2n2;
        x2n1 = IC_LL1(31);
        x2n2 = IC_LL1(32);
        OC_H1(15) = NOQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_L1(15) = NOQ(x2n + ((hn1 + hn) >> 1));
        /* post ex */
        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 33 -> 31 */
        x2n2 = ic30;      /* mirror 34 -> 30 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_L1(16) = NOQ(x2n + ((hn1 + hn) >> 1));
    }
}

/******************************************************************************/
static void
rfx_encode_dwt_shift_rem_horz_lv2(const sint16 *in_buffer, sint16 *out_buffer,
                                  const char *quants)
{
    const sint16 *ic; /* input coefficients */
    sint16 *lo;
    sint16 *hi;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    sint16 ic30;
    int n;
    int y;
    int lo_fact;
    int hi_fact;
    int lo_half;
    int hi_half;

    /* LL2 no Q */
    SETUPHIQ(2, 4); /* HL2 */
    for (y = 0; y < 17; y++) /* lo */
    {
        /* setup */
        ic = SETUP_IC_L1(y);
        lo = SETUP_OC_LL2(y);
        hi = SETUP_OC_HL2(y);
        /* pre */
        x2n = IC_L1(0);
        x2n1 = IC_L1(1);
        x2n2 = IC_L1(2);
        OC_HL2(0) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LL2(0) = NOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 15; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_L1(2 * n + 1);
            x2n2 = IC_L1(2 * n + 2);
            OC_HL2(n) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_LL2(n) = NOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic30 = x2n = x2n2;
        x2n1 = IC_L1(31);
        x2n2 = IC_L1(32);
        OC_HL2(15) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LL2(15) = NOQ(x2n + ((hn1 + hn) >> 1));
        /* post ex */
        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 33 -> 31 */
        x2n2 = ic30;      /* mirror 34 -> 30 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_LL2(16) = NOQ(x2n + ((hn1 + hn) >> 1));
    }
    SETUPLOQ(2, 0); /* LH2 */
    SETUPHIQ(3, 0); /* HH2 */
    for (y = 0; y < 16; y++) /* hi */
    {
        /* setup */
        ic = SETUP_IC_H1(y);
        lo = SETUP_OC_LH2(y);
        hi = SETUP_OC_HH2(y);
        /* pre */
        x2n = IC_H1(0);
        x2n1 = IC_H1(1);
        x2n2 = IC_H1(2);
        OC_HH2(0) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LH2(0) = LOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 15; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_H1(2 * n + 1);
            x2n2 = IC_H1(2 * n + 2);
            OC_HH2(n) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_LH2(n) = LOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic30 = x2n = x2n2;
        x2n1 = IC_H1(31);
        x2n2 = IC_H1(32);
        OC_HH2(15) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LH2(15) = LOQ(x2n + ((hn1 + hn) >> 1));
        /* post ex */
        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 33 -> 31 */
        x2n2 = ic30;      /* mirror 34 -> 30 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_LH2(16) = LOQ(x2n + ((hn1 + hn) >> 1));
    }
}

/******************************************************************************/
static void
rfx_encode_dwt_shift_rem_vert_lv3(const sint16 *in_buffer, sint16 *out_buffer)
{
    const sint16 *ic; /* input coefficients */
    sint16 *lo;
    sint16 *hi;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    sint16 ic14;
    int n;
    int y;

    for (y = 0; y < 17; y++)
    {
        /* setup */
        ic = SETUP_IC_LL2(y);
        lo = SETUP_OC_L2(y);
        hi = SETUP_OC_H2(y);
        /* pre */
        x2n = IC_LL2(0);
        x2n1 = IC_LL2(1);
        x2n2 = IC_LL2(2);
        OC_H2(0) = NOQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_L2(0) = NOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_LL2(2 * n + 1);
            x2n2 = IC_LL2(2 * n + 2);
            OC_H2(n) = NOQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_L2(n) = NOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic14 = x2n = x2n2;
        x2n1 = IC_LL2(15);
        x2n2 = IC_LL2(16);
        OC_H2(7) = NOQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_L2(7) = NOQ(x2n + ((hn1 + hn) >> 1));
        /* post ex */
        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 17 -> 15 */
        x2n2 = ic14;      /* mirror 18 -> 14 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_L2(8) = NOQ(x2n + ((hn1 + hn) >> 1));
    }
}

/******************************************************************************/
static void
rfx_encode_dwt_shift_rem_horz_lv3(const sint16 *in_buffer, sint16 *out_buffer,
                                  const char *quants)
{
    const sint16 *ic; /* input coefficients */
    sint16 *lo;
    sint16 *hi;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    sint16 ic14;
    int n;
    int y;
    int lo_fact;
    int hi_fact;
    int lo_half;
    int hi_half;

    SETUPLOQ(0, 0); /* LL3 */
    SETUPHIQ(1, 0); /* HL3 */
    for (y = 0; y < 9; y++) /* lo */
    {
        /* setup */
        ic = SETUP_IC_L2(y);
        lo = SETUP_OC_LL3(y);
        hi = SETUP_OC_HL3(y);
        /* pre */
        x2n = IC_L2(0);
        x2n1 = IC_L2(1);
        x2n2 = IC_L2(2);
        OC_HL3(0) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LL3(0) = LOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_L2(2 * n + 1);
            x2n2 = IC_L2(2 * n + 2);
            OC_HL3(n) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_LL3(n) = LOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic14 = x2n = x2n2;
        x2n1 = IC_L2(15);
        x2n2 = IC_L2(16);
        OC_HL3(7) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LL3(7) = LOQ(x2n + ((hn1 + hn) >> 1));
        /* post ex */
        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 17 -> 15 */
        x2n2 = ic14;      /* mirror 18 -> 14 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_LL3(8) = LOQ(x2n + ((hn1 + hn) >> 1));
    }
    SETUPLOQ(0, 4); /* LH3 */
    SETUPHIQ(1, 4); /* HH3 */
    for (y = 0; y < 8; y++) /* hi */
    {
        /* setup */
        ic = SETUP_IC_H2(y);
        lo = SETUP_OC_LH3(y);
        hi = SETUP_OC_HH3(y);
        /* pre */
        x2n = IC_H2(0);
        x2n1 = IC_H2(1);
        x2n2 = IC_H2(2);
        OC_HH3(0) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LH3(0) = LOQ(x2n + hn); /* mirror */
        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = IC_H2(2 * n + 1);
            x2n2 = IC_H2(2 * n + 2);
            OC_HH3(n) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
            OC_LH3(n) = LOQ(x2n + ((hn1 + hn) >> 1));
        }
        /* post */
        hn1 = hn;
        ic14 = x2n = x2n2;
        x2n1 = IC_H2(15);
        x2n2 = IC_H2(16);
        OC_HH3(7) = HIQ(hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1);
        OC_LH3(7) = LOQ(x2n + ((hn1 + hn) >> 1));
        /* post ex */
        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 17 -> 15 */
        x2n2 = ic14;      /* mirror 18 -> 14 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        OC_LH3(8) = LOQ(x2n + ((hn1 + hn) >> 1));
    }
}

/******************************************************************************/
int
rfx_encode_dwt_shift_rem(const uint8 *in_buffer, sint16 *out_buffer,
                         sint16 *tmp_buffer, const char *quants)
{
    rfx_encode_dwt_shift_rem_vert_lv1_u8(in_buffer, tmp_buffer);
    rfx_encode_dwt_shift_rem_horz_lv1(tmp_buffer, out_buffer, quants);
    rfx_encode_dwt_shift_rem_vert_lv2(out_buffer + 3007, tmp_buffer);
    rfx_encode_dwt_shift_rem_horz_lv2(tmp_buffer, out_buffer + 3007, quants);
    rfx_encode_dwt_shift_rem_vert_lv3(out_buffer + 3807, tmp_buffer);
    rfx_encode_dwt_shift_rem_horz_lv3(tmp_buffer, out_buffer + 3807, quants);
    return 0;
}
