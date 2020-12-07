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
    const uint8 *src;
    int n;
    int y;
    sint16 ic[68];

    for (y = 0; y < 64; y++)
    {

        /* setup */
        src = in_buffer + y;
        for (n = 0; n < 64; n++)
        {
            ic[y] = (src[n * 64] - 128) << DWT_FACTOR;
        }
        ic[64] = 2 * ic[63] - ic[62];
        ic[65] = ic[63];
        ic[66] = ic[62];

        /* pre */
        lptr = out_buffer + y;
        hptr = lptr + 64 * 33;
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        *hptr = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + hn;

        /* loop */
        for (n = 1; n < 31; n++)
        {
            hn1 = hn;
            lptr += 64;
            hptr += 64;
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            *hptr = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            *lptr = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        lptr += 64;
        x2n = ic[2 * 31];
        x2n1 = ic[2 * 31 + 1];
        x2n2 = ic[2 * 31 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        lptr += 64;
        x2n = ic[2 * 32];
        x2n1 = ic[2 * 32 + 1];
        x2n2 = ic[2 * 32 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + ((hn1 + hn) >> 1);

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
    const sint16 *l_src;
    const sint16 *h_src;
    int n;
    int y;
    sint16 ic[68];

    hl = out_buffer;
    lh = hl + 31 * 33;
    hh = lh + 33 * 31;
    ll = hh + 31 * 31;

    for (y = 0; y < 33; y++)
    {

        /* setup */
        l_src = in_buffer + y * 64;
        for (n = 0; n < 64; n++)
        {
            ic[n] = l_src[n];
        }
        ic[64] = 2 * ic[63] - ic[62];
        ic[65] = ic[63];
        ic[66] = ic[62];

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hl[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[0] = x2n + hn;

        /* loop */
        for (n = 1; n < 31; n++)
        {
            hn1 = hn;
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hl[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            ll[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        x2n = ic[2 * 31];
        x2n1 = ic[2 * 31 + 1];
        x2n2 = ic[2 * 31 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[31] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = ic[2 * 32];
        x2n1 = ic[2 * 32 + 1];
        x2n2 = ic[2 * 32 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[32] = x2n + ((hn1 + hn) >> 1);

    }

    for (y = 0; y < 31; y++)
    {

        /* setup */
        h_src = l_src + 64 * 33 + y * 64;
        for (n = 0; n < 64; n++)
        {
            ic[n] = h_src[n];
        }
        ic[64] = 2 * ic[63] - ic[62];
        ic[65] = ic[63];
        ic[66] = ic[62];

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
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hh[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            lh[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        x2n = ic[2 * 31];
        x2n1 = ic[2 * 31 + 1];
        x2n2 = ic[2 * 31 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[31] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = ic[2 * 32];
        x2n1 = ic[2 * 32 + 1];
        x2n2 = ic[2 * 32 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[32] = x2n + ((hn1 + hn) >> 1);

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
    const uint16 *src;
    int n;
    int y;
    sint16 ic[36];

    for (y = 0; y < 33; y++)
    {

        /* setup */
        src = in_buffer + y;
        for (n = 0; n < 33; n++)
        {
            ic[y] = src[n * 33];
        }
        ic[33] = ic[31];
        ic[34] = ic[30];

        /* pre */
        lptr = out_buffer + y;
        hptr = lptr + 33 * 17;
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        *hptr = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + hn;

        /* loop */
        for (n = 1; n < 15; n++)
        {
            hn1 = hn;
            lptr += 33;
            hptr += 33;
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            *hptr = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            *lptr = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        lptr += 33;
        hptr += 33;
        x2n = ic[2 * 15];
        x2n1 = ic[2 * 15 + 1];
        x2n2 = ic[2 * 15 + 2];
        *hptr = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        lptr += 33;
        x2n = ic[2 * 16];
        x2n1 = ic[2 * 16 + 1];
        x2n2 = ic[2 * 16 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + ((hn1 + hn) >> 1);

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
    const sint16 *l_src;
    const sint16 *h_src;
    int n;
    int y;
    sint16 ic[36];

    hl = out_buffer;
    lh = hl + 16 * 17;
    hh = lh + 17 * 16;
    ll = hh + 16 * 16;

    for (y = 0; y < 17; y++)
    {

        /* setup */
        l_src = in_buffer + y * 33;
        for (n = 0; n < 33; n++)
        {
            ic[n] = l_src[n];
        }
        ic[33] = ic[31];
        ic[34] = ic[30];

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
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hl[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            ll[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        x2n = ic[2 * 15];
        x2n1 = ic[2 * 15 + 1];
        x2n2 = ic[2 * 15 + 2];
        hl[15] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[15] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = ic[2 * 16];
        x2n1 = ic[2 * 16 + 1];
        x2n2 = ic[2 * 16 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[16] = x2n + ((hn1 + hn) >> 1);

    }

    for (y = 0; y < 16; y++)
    {

        /* setup */
        h_src = l_src + 33 * 17 + y * 33;
        for (n = 0; n < 33; n++)
        {
            ic[n] = h_src[n];
        }
        ic[33] = ic[31];
        ic[34] = ic[30];

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
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hh[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            lh[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        x2n = ic[2 * 15];
        x2n1 = ic[2 * 15 + 1];
        x2n2 = ic[2 * 15 + 2];
        hh[15] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[15] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = ic[2 * 16];
        x2n1 = ic[2 * 16 + 1];
        x2n2 = ic[2 * 16 + 2];
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
    const uint16 *src;
    int n;
    int y;
    sint16 ic[20];

    for (y = 0; y < 17; y++)
    {

        /* setup */
        src = in_buffer + y;
        for (n = 0; n < 17; n++)
        {
            ic[y] = src[n * 17];
        }
        ic[17] = ic[15];
        ic[18] = ic[14];

        /* pre */
        lptr = out_buffer + y;
        hptr = lptr + 17 * 9;
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        *hptr = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + hn;

        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            lptr += 17;
            hptr += 17;
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            *hptr = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            *lptr = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        lptr += 17;
        hptr += 17;
        x2n = ic[2 * 7];
        x2n1 = ic[2 * 7 + 1];
        x2n2 = ic[2 * 7 + 2];
        *hptr = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        lptr += 17;
        x2n = ic[2 * 8];
        x2n1 = ic[2 * 8 + 1];
        x2n2 = ic[2 * 8 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        *lptr = x2n + ((hn1 + hn) >> 1);

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
    sint16 ic[20];

    hl = out_buffer;
    lh = hl + 8 * 9;
    hh = lh + 9 * 8;
    ll = hh + 8 * 8;

    for (y = 0; y < 9; y++)
    {

        /* setup */
        l_src = in_buffer + y * 17;
        for (n = 0; n < 17; n++)
        {
            ic[n] = l_src[n];
        }
        ic[17] = ic[15];
        ic[18] = ic[14];

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hl[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[0] = x2n + hn;

        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hl[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            ll[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        x2n = ic[2 * 7];
        x2n1 = ic[2 * 7 + 1];
        x2n2 = ic[2 * 7 + 2];
        hl[7] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[7] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = ic[2 * 8];
        x2n1 = ic[2 * 8 + 1];
        x2n2 = ic[2 * 8 + 2];
        hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        ll[8] = x2n + ((hn1 + hn) >> 1);

    }

    for (y = 0; y < 8; y++)
    {

        /* setup */
        h_src = l_src + 17 * 9 + y * 17;
        for (n = 0; n < 17; n++)
        {
            ic[n] = h_src[n];
        }
        ic[17] = ic[15];
        ic[18] = ic[14];

        /* pre */
        x2n = ic[0];
        x2n1 = ic[1];
        x2n2 = ic[2];
        hh[0] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[0] = x2n + hn;

        /* loop */
        for (n = 1; n < 7; n++)
        {
            hn1 = hn;
            x2n = ic[2 * n];
            x2n1 = ic[2 * n + 1];
            x2n2 = ic[2 * n + 2];
            hh[n] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
            lh[n] = x2n + ((hn1 + hn) >> 1);
        }

        /* post */
        hn1 = hn;
        x2n = ic[2 * 7];
        x2n1 = ic[2 * 7 + 1];
        x2n2 = ic[2 * 7 + 2];
        hh[7] = hn = (x2n1 - ((x2n + x2n2) >> 1)) >> 1;
        lh[7] = x2n + ((hn1 + hn) >> 1);

        hn1 = hn;
        x2n = ic[2 * 8];
        x2n1 = ic[2 * 8 + 1];
        x2n2 = ic[2 * 8 + 2];
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
