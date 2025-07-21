/**
 * librfxcodec: A Remote Desktop Protocol client.
 * RemoteFX Codec Library
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

#include "rfxcommon.h"
#include "rfxdecode_alpha.h"

#define IN_UINT8_MV(_p) (*((_p)++))

/*****************************************************************************/
int
rfx_decode_plane(uint8* ain, int width, int height, uint8* aout, int size)
{
    int indexw;
    int indexh;
    int code;
    int collen;
    int replen;
    int color;
    int x;
    int revcode;
    unsigned char* last_line;
    unsigned char* this_line;
    unsigned char* org_in;
    unsigned char* org_out;
    unsigned char* in;
    unsigned char* out;

    in = (unsigned char *) ain;
    out = (unsigned char *) aout;
    org_in = in;
    org_out = out;
    last_line = 0;
    indexh = 0;
    while (indexh < height)
    {
        out = org_out + indexh * width;
        color = 0;
        this_line = out;
        indexw = 0;
        if (last_line == 0)
        {
            while (indexw < width)
            {
                code = IN_UINT8_MV(in);
                replen = code & 0xf;
                collen = (code >> 4) & 0xf;
                revcode = (replen << 4) | collen;
                if ((revcode <= 47) && (revcode >= 16))
                {
                    replen = revcode;
                    collen = 0;
                }
                while (collen > 0)
                {
                    color = IN_UINT8_MV(in);
                    *out = color;
                    out += 1;
                    indexw++;
                    collen--;
                }
                while (replen > 0)
                {
                    *out = color;
                    out += 1;
                    indexw++;
                    replen--;
                }
            }
        }
        else
        {
            while (indexw < width)
            {
                code = IN_UINT8_MV(in);
                replen = code & 0xf;
                collen = (code >> 4) & 0xf;
                revcode = (replen << 4) | collen;
                if ((revcode <= 47) && (revcode >= 16))
                {
                    replen = revcode;
                    collen = 0;
                }
                while (collen > 0)
                {
                    x = IN_UINT8_MV(in);
                    if (x & 1)
                    {
                        x = x >> 1;
                        x = x + 1;
                        color = -x;
                    }
                    else
                    {
                        x = x >> 1;
                        color = x;
                    }
                    x = last_line[indexw] + color;
                    *out = x;
                    out += 1;
                    indexw++;
                    collen--;
                }
                while (replen > 0)
                {
                    x = last_line[indexw] + color;
                    *out = x;
                    out += 1;
                    indexw++;
                    replen--;
                }
            }
        }
        indexh++;
        last_line = this_line;
    }
    return (int) (in - org_in);
}

