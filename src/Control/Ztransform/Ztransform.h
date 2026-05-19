#ifndef CONTROL_ZTRANSFORM_H_
#define CONTROL_ZTRANSFORM_H_

#include <stdint.h>
#include "utils/Exceptions_Assertions/except.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Exception raised when Zfilter fails.
 */
extern const Except_t Zfilter_Failed;

typedef struct ZFilter ZFilter;   // Opaque struct for ZFilter implementation

/**
 * @brief Create a discrete Z-domain filter.
 *
 *
 * @note Coefficients are normalized internally by a[0].
 *
 * @throw NullptrError       if b_in or a_in are NULL
 * @throw ValueError         if nb or na are 0
 * @throw ZeroDivisionError  if a_in[0] is 0
 * @throw Zfilter_Failed     if allocation fails
 *
 * @param b_in Numerator coefficients
 * @param nb   Number of numerator coefficients
 * @param a_in Denominator coefficients
 * @param na   Number of denominator coefficients
 *
 * @return Pointer to created filter
 */
extern ZFilter* ZFilter_ctor(
    const float *b_in, uint32_t nb,
    const float *a_in, uint32_t na
);

/**
 * @brief Update the filter with a new input sample.
 *
 * @throw NullptrError if f is NULL
 *
 * @param f Filter instance
 * @param x Input sample
 *
 * @return Output sample
 */
extern float ZFilter_update(ZFilter *f, float x);

/**
 * @brief Reset filter history (internal state) to zero; filter coefficients remain unchanged.
 *
 * @throw NullptrError if f is NULL
 */
extern void ZFilter_reset(ZFilter *f);

/**
 * @brief Destroy filter and release all internal resources.
 *
 * Frees all internal resources, including coefficient arrays and history.
 * Safe to call with NULL.
 */
extern void ZFilter_dtor(ZFilter *f);


#ifdef __cplusplus
}
#endif

#endif