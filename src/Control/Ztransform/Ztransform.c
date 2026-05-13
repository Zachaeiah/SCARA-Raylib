#include "Ztransform.h"
#include <stdlib.h>
#include <string.h>

#include "utils/Exceptions_Assertions/assert.h"
#include "utils/MemAllocator/mem.h"

const Except_t Zfilter_Failed = {"Z-Filter faild"};

struct ZFilter {
    float *b;     // numerator coefficients
    float *a;     // denominator coefficients (a[0] assumed = 1)

    uint32_t nb;  // number of b coeffs
    uint32_t na;  // number of a coeffs
    uint32_t ny;  // histories (size = nb for x_hist, na for y_hist)

    float *x_hist; // input history
    float *y_hist; // output history
};

ZFilter* ZFilter_ctor(const float *b_in, uint32_t nb,
                      const float *a_in, uint32_t na)
{
    ZFilter *filter = NULL;
    float a0 = a_in[0];

    if (!b_in || !a_in)
        RAISE(NullptrError);

    if (nb == 0 || na == 0)
        RAISE(ValueError);

    if (a0 == 0.0f)
        RAISE(ZeroDivisionError);

    TRY {
         NEW0(filter);

        filter->nb = nb;
        filter->na = na;
        filter->ny = na - 1;

        filter->b = ALLOC(nb * sizeof(float));
        filter->a = ALLOC(na * sizeof(float));
        filter->x_hist = CALLOC(nb, sizeof(float));

        if (filter->ny > 0) {
            filter->y_hist = CALLOC(filter->ny, sizeof(float));
        }

        for (uint32_t i = 0; i < nb; i++) {
            filter->b[i] = b_in[i] / a0;
        }

        for (uint32_t i = 0; i < na; i++) {
            filter->a[i] = a_in[i] / a0;
        }

    } EXCEPT(MemroyError) {
        printf("ZFilter allocation failed: %s\n", Except_frame.exception->reason);

        if (filter) ZFilter_dtor(filter);
        RAISE(Zfilter_Failed);
    } END_TRY;

    return filter;
}

float ZFilter_update(ZFilter *f, float x)
{
    if (!f)
        RAISE(NullptrError);

    assert_debug(f->b);
    assert_debug(f->a);
    assert_debug(f->x_hist);

    if (f->nb > 1) {
        memmove(&f->x_hist[1],
                &f->x_hist[0],
                (f->nb - 1) * sizeof(float));
    }

    f->x_hist[0] = x;

    float y = 0.0f;

    for (uint32_t i = 0; i < f->nb; i++) {
        y += f->b[i] * f->x_hist[i];
    }

    for (uint32_t i = 1; i < f->na; i++) {
        y -= f->a[i] * f->y_hist[i - 1];
    }

    if (f->ny > 1) {
        memmove(&f->y_hist[1],
                &f->y_hist[0],
                (f->ny - 1) * sizeof(float));
    }

    if (f->ny > 0) {
        f->y_hist[0] = y;
    }

    return y;
}

void ZFilter_reset(ZFilter *f)
{
    if (!f)
        RAISE(NullptrError);

    assert_debug(f->x_hist);

    memset(f->x_hist, 0, f->nb * sizeof(float));

    if (f->ny > 0) {
        assert_debug(f->y_hist);
        memset(f->y_hist, 0, f->ny * sizeof(float));
    }
}

void ZFilter_dtor(ZFilter *f)
{
    if (!f)
        return;

    FREE(f->x_hist);
    FREE(f->y_hist);
    FREE(f->b);
    FREE(f->a);
    FREE(f);
}