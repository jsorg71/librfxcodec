
#if defined(HAVE_CONFIG_H)
#include <config_ac.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "../include/rfxcodec_encode.h"

int
main(int argc, char **argv)
{
    struct rfxcodec_encode_internals internals;
    int error;
    int index;
    short diff_buffer1[4096];
    short diff_buffer2[4096];
    short dwt_buffer[4096];
    short hist_buffer[4096];
    int diff_zeros1;
    int dwt_zeros1;
    int diff_zeros2;
    int dwt_zeros2;

    unsigned char in_buffer[4096];
    short out_buffer1[4096];
    short out_buffer2[4096];
    short tmp_buffer1[4096];
    short tmp_buffer2[4096];
    //char quans[5] = { 0x66, 0x66, 0x66, 0x66, 0x66 };
    char quans[5] = { 0x99, 0x99, 0x99, 0x99, 0x99 };
    int fd;

    fd = open("/dev/urandom", O_RDONLY);

    error = rfxcodec_encode_get_internals(&internals);
    if (error == 0)
    {
#if 0
        read(fd, dwt_buffer, 4096 * 2);
        read(fd, hist_buffer, 4096 * 2);
        internals.rfx_encode_diff_count(diff_buffer1, dwt_buffer, hist_buffer, &diff_zeros1, &dwt_zeros1);
        internals.rfx_encode_diff_count_amd64(diff_buffer2, dwt_buffer, hist_buffer, &diff_zeros2, &dwt_zeros2);
        if (memcmp(diff_buffer1, diff_buffer2, 4096 * 2) == 0 && diff_zeros1 == diff_zeros2 && dwt_zeros1 == dwt_zeros2)
        {
            printf("match\n");
        }
        else
        {
            printf("no match\n");
        }
#endif
#if 1
        read(fd, in_buffer, 4096);
        internals.rfx_encode_dwt_shift_rem(in_buffer, out_buffer1, tmp_buffer1, quans);
        //internals.rfx_encode_dwt_shift_rem(in_buffer, out_buffer2, tmp_buffer2, quans);
        internals.rfx_encode_dwt_shift_rem_amd64(in_buffer, out_buffer2, tmp_buffer2, quans);
        if (memcmp(out_buffer1, out_buffer2, 4096 * 2) == 0)
        //if (memcmp(tmp_buffer1, tmp_buffer2, 4096 * 2) == 0)
        {
            printf("match\n");
        }
        else
        {
            printf("no match\n");
        }
#endif
#if 0
        for (index = 0; index < 1024 * 1024; index++)
        {
            internals.rfx_encode_diff_count_amd64(diff_buffer, dwt_buffer, hist_buffer, &diff_zeros, &dwt_zeros);
            //internals.rfx_encode_diff_count(diff_buffer, dwt_buffer, hist_buffer, &diff_zeros, &dwt_zeros);
            //internals.rfx_encode_dwt_shift_rem_amd64(in_buffer, out_buffer1, tmp_buffer1, quans);
            //internals.rfx_encode_dwt_shift_rem(in_buffer, out_buffer, tmp_buffer, quans);
        }
#endif
    }
    return 0;
}
