
#if defined(HAVE_CONFIG_H)
#include <config_ac.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <emmintrin.h>

#include "funcs_amd64.h"

/******************************************************************************/
int
rfx_encode_diff_count_amd64(short *diff_buffer,
                            const short *dwt_buffer,
                            const short *hist_buffer,
                            int *diff_zeros, int *dwt_zeros)
{
    int index;
    int ldiff_zeros = 0;
    int ldwt_zeros = 0;
    int mask;
    __m128i dwt_vec;
    __m128i hist_vec;
    __m128i diff_vec;
    __m128i cmp_vec;
    __m128i zero_vec = _mm_setzero_si128();

    /* diff and count for most of tile */
    for (index = 0; index < 4096 - 88; index += 8)
    {
        /* diff */
        dwt_vec = _mm_load_si128((const __m128i *)(dwt_buffer + index));
        hist_vec = _mm_load_si128((__m128i *)(hist_buffer + index));
        diff_vec = _mm_sub_epi16(dwt_vec, hist_vec);
        _mm_store_si128((__m128i *)(diff_buffer + index), diff_vec);
        /* count */
        cmp_vec = _mm_cmpeq_epi16(diff_vec, zero_vec);
        mask = _mm_movemask_epi8(cmp_vec);
        ldiff_zeros += __builtin_popcount(mask) / 2;
        cmp_vec = _mm_cmpeq_epi16(dwt_vec, zero_vec);
        mask = _mm_movemask_epi8(cmp_vec);
        ldwt_zeros += __builtin_popcount(mask) / 2;
    }
    /* diff for the rest of tile */
    while (index < 4096)
    {
        dwt_vec = _mm_load_si128((const __m128i *)(dwt_buffer + index));
        hist_vec = _mm_load_si128((__m128i *)(hist_buffer + index));
        diff_vec = _mm_sub_epi16(dwt_vec, hist_vec);
        _mm_store_si128((__m128i *)(diff_buffer + index), diff_vec);
        index += 8;
    }
    /* count for the missing part */
    for (index = 4096 - 88; index < 4096 - 81; index++)
    {
        if (diff_buffer[index] == 0)
        {
            ldiff_zeros++;
        }
        if (dwt_buffer[index] == 0)
        {
            ldwt_zeros++;
        }
    }
    *diff_zeros = ldiff_zeros;
    *dwt_zeros = ldwt_zeros;
    return 0;
}
