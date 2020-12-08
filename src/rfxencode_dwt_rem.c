/**
 * RemoteFX Codec Library - DWT Reduced-Extrapolate Method
 *
 * Copyright 2020 Jay Sorg <jay.sorg@gmail.com>
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
 */

#if defined(HAVE_CONFIG_H)
#include <config_ac.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rfxcommon.h"
#include "rfxencode_dwt_rem.h"

#define ICL1(_offset) (ic[(_offset) * 64] - 128) << DWT_FACTOR
#define ICL2(_offset) ic[(_offset) * 33]
#define ICL3(_offset) ic[(_offset) * 17]

#define HIL1(_offset) hptr[(_offset) * 64]
#define LOL1(_offset) lptr[(_offset) * 64]
#define HIL2(_offset) hptr[(_offset) * 33]
#define LOL2(_offset) lptr[(_offset) * 33]
#define HIL3(_offset) hptr[(_offset) * 17]
#define LOL3(_offset) lptr[(_offset) * 17]

/******************************************************************************/
static void
rfx_rem_dwt_2d_encode_vert_lv1(const uint8 *in_buffer, sint16 *out_buffer)
{
    sint16 *lptr;
    sint16 *hptr;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    int n;
    int y;
    const uint8 *ic;
    sint16 ic62;

    for (y = 0; y < 64; y++)
    {

        /* setup */
        ic = in_buffer + y;

        /* pre */
        lptr = out_buffer + y;
        hptr = lptr + 64 * 33;
        x2n = ICL1(0);
        x2n1 = ICL1(1);
        x2n2 = ICL1(2);
        HIL1(0) = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        LOL1(0) = x2n + hn; /* mirror */

        /* loop */
        for (n = 1; n < 31; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ICL1(2 * n + 1);
            x2n2 = ICL1(2 * n + 2);
            HIL1(n) = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            LOL1(n) = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic62 = x2n = x2n2;
        x2n1 = ICL1(63);
        x2n2 = 2 * x2n1 - x2n; /* ic[64] = 2 * ic[63] - ic[62] */
        LOL1(31) = x2n + (hn1 >> 1);

        x2n = x2n2;
        /* x2n1 already set, mirror 65 -> 63 */
        x2n2 = ic62;      /* mirror 66 -> 62 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        LOL1(32) = x2n + (hn >> 1);

    }
}

/******************************************************************************/
static void
rfx_rem_dwt_2d_encode_horz_lv1(const sint16 *in_buffer, sint16 *out_buffer)
{
    sint16 *hl;
    sint16 *lh;
    sint16 *hh;
    sint16 *ll;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    int n;
    int y;
    const sint16 *ic;
    sint16 ic62;

    hl = out_buffer;
    lh = hl + 31 * 33;
    hh = lh + 33 * 31;
    ll = hh + 31 * 31;

    for (y = 0; y < 33; y++) /* low */
    {

        /* setup */
        ic = in_buffer + 64 * y;

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hl[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[0] = x2n + hn; /* mirror */

        /* loop */
        for (n = 1; n < 31; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hl[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            ll[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic62 = x2n = x2n2;
        x2n1 = ic[63];
        x2n2 = 2 * x2n1 - x2n; /* ic[64] = 2 * ic[63] - ic[62] */
        ll[31] = x2n + (hn1 >> 1);

        x2n = x2n2;
        /* x2n1 already set, mirror 65 -> 63 */
        x2n2 = ic62;      /* mirror 66 -> 62 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[32] = x2n + (hn >> 1);

    }

    for (y = 0; y < 31; y++) /* high */
    {

        /* setup */
        ic = in_buffer + 64 * (33 + y);

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hh[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[0] = x2n + hn;

        /* loop */
        for (n = 1; n < 31; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hh[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            lh[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic62 = x2n = x2n2;
        x2n1 = ic[63];
        x2n2 = 2 * x2n1 - x2n; /* ic[64] = 2 * ic[63] - ic[62] */
        lh[31] = x2n + (hn1 >> 1);

        x2n = x2n2;
        /* x2n1 already set, mirror 65 -> 63 */
        x2n2 = ic62;      /* mirror 66 -> 62 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[32] = x2n + (hn >> 1);

    }

}

/******************************************************************************/
static void
rfx_rem_dwt_2d_encode_vert_lv2(const sint16 *in_buffer, sint16 *out_buffer)
{
    sint16 *lptr;
    sint16 *hptr;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    int n;
    int y;
    const sint16 *ic;
    sint16 ic30;

    for (y = 0; y < 33; y++)
    {

        /* setup */
        ic = in_buffer + y;

        /* pre */
        lptr = out_buffer + y;
        hptr = lptr + 33 * 17;
        x2n = ICL2(0);
        x2n1 = ICL2(1);
        x2n2 = ICL2(2);
        HIL2(0) = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        LOL2(0) = x2n + hn; /* mirror */

        /* loop */
        for (n = 1; n < 15; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ICL2(2 * n + 1);
            x2n2 = ICL2(2 * n + 2);
            HIL2(n) = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            LOL2(n) = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic30 = x2n = x2n2;
        x2n1 = ICL2(31);
        x2n2 = ICL2(32);
        HIL2(15) = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        LOL2(15) = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 33 -> 31 */
        x2n2 = ic30;      /* mirror 34 -> 30 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        LOL2(16) = x2n + ((hn1 + hn) >> 1);

    }
}

/******************************************************************************/
static void
rfx_rem_dwt_2d_encode_horz_lv2(const sint16 *in_buffer, sint16 *out_buffer)
{
    sint16 *hl;
    sint16 *lh;
    sint16 *hh;
    sint16 *ll;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    int n;
    int y;
    const sint16 *ic;
    sint16 ic30;

    hl = out_buffer;
    lh = hl + 16 * 17;
    hh = lh + 17 * 16;
    ll = hh + 16 * 16;

    for (y = 0; y < 17; y++) /* low */
    {

        /* setup */
        ic = in_buffer + 33 * y;

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hl[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[0] = x2n + hn;

        /* loop */
        for (n = 1; n < 15; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hl[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            ll[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic30 = x2n = x2n2;
        x2n1 = ic[31];
        x2n2 = ic[32];
        hl[15] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[15] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 33 -> 31 */
        x2n2 = ic30;      /* mirror 34 -> 30 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[16] = x2n + ((hn1 + hn) >> 1);

    }

    for (y = 0; y < 16; y++) /* high */
    {

        /* setup */
        ic = in_buffer + 33 * (17 + y);

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hh[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[0] = x2n + hn;

        /* loop */
        for (n = 1; n < 15; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hh[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            lh[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic30 = x2n = x2n2;
        x2n1 = ic[31];
        x2n2 = ic[32];
        hh[15] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[15] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 33 -> 31 */
        x2n2 = ic30;      /* mirror 34 -> 30 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[16] = x2n + ((hn1 + hn) >> 1);

    }

}

/******************************************************************************/
static void
rfx_rem_dwt_2d_encode_vert_lv3(const sint16 *in_buffer, sint16 *out_buffer)
{
    sint16 *lptr;
    sint16 *hptr;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    int n;
    int y;
    const sint16 *ic;
    sint16 ic14;

    for (y = 0; y < 17; y++)
    {

        /* setup */
        ic = in_buffer + y;

        /* pre */
        lptr = out_buffer + y;
        hptr = lptr + 17 * 9;
        x2n = ICL3(0);
        x2n1 = ICL3(1);
        x2n2 = ICL3(2);
        HIL3(0) = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        LOL3(0) = x2n + hn; /* mirror */

        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ICL3(2 * n + 1);
            x2n2 = ICL3(2 * n + 2);
            HIL3(n) = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            LOL3(n) = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic14 = x2n = x2n2;
        x2n1 = ICL3(15);
        x2n2 = ICL3(16);
        HIL3(7) = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        LOL3(7) = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 17 -> 15 */
        x2n2 = ic14;      /* mirror 18 -> 14 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        LOL3(8) = x2n + ((hn1 + hn) >> 1);

    }
}

/******************************************************************************/
static void
rfx_rem_dwt_2d_encode_horz_lv3(const sint16 *in_buffer, sint16 *out_buffer)
{
    sint16 *hl;
    sint16 *lh;
    sint16 *hh;
    sint16 *ll;
    sint16 x2n;     /* n[2n]     */
    sint16 x2n1;    /* n[2n + 1] */
    sint16 x2n2;    /* n[2n + 2] */
    sint16 hn1;     /* H[n - 1]  */
    sint16 hn;      /* H[n]      */
    const sint16 *l_src;
    const sint16 *h_src;
    int n;
    int y;
    const sint16 *ic;
    sint16 ic14;

    hl = out_buffer;
    lh = hl + 8 * 9;
    hh = lh + 9 * 8;
    ll = hh + 8 * 8;

    for (y = 0; y < 9; y++) /* low */
    {

        /* setup */
        ic = in_buffer + 17 * y;

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hl[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[0] = x2n + hn; /* mirror */

        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hl[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            ll[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic14 = x2n = x2n2;
        x2n1 = ic[15];
        x2n2 = ic[16];
        hl[7] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[7] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 17 -> 15 */
        x2n2 = ic14;      /* mirror 18 -> 14 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[8] = x2n + ((hn1 + hn) >> 1);

    }

    for (y = 0; y < 8; y++) /* high */
    {

        /* setup */
        ic = in_buffer + 17 * (9 + y);

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hh[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[0] = x2n + hn; /* mirror */

        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            x2n = x2n2;
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hh[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            lh[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        ic14 = x2n = x2n2;
        x2n1 = ic[15];
        x2n2 = ic[16];
        hh[7] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[7] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = x2n2;
        /* x2n1 already set, mirror 17 -> 15 */
        x2n2 = ic14;      /* mirror 18 -> 14 */
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[8] = x2n + ((hn1 + hn) >> 1);

    }

}

/******************************************************************************/
int
rfx_rem_dwt_2d_encode(const uint8 *in_buffer, sint16 *out_buffer,
                      sint16 *tmp_buffer)
{
    rfx_rem_dwt_2d_encode_vert_lv1(in_buffer, tmp_buffer);
    rfx_rem_dwt_2d_encode_horz_lv1(tmp_buffer, out_buffer);
    rfx_rem_dwt_2d_encode_vert_lv2(out_buffer + 3007, tmp_buffer);
    rfx_rem_dwt_2d_encode_horz_lv2(tmp_buffer, out_buffer + 3007);
    rfx_rem_dwt_2d_encode_vert_lv3(out_buffer + 3807, tmp_buffer);
    rfx_rem_dwt_2d_encode_horz_lv3(tmp_buffer, out_buffer + 3807);
    return 0;
}
