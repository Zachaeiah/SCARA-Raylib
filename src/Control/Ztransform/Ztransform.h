#ifndef CONTROL_ZTRANSFORM_H_
#define CONTROL_ZTRANSFORM_H_

#include <stdint.h>
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/Logger/logger.h"

#ifdef __cplusplus
extern "C" {
#endif


extern const Except_t Zfilter_Failed;
extern const ErrorType Zfilter_Failed_ErrorCode; /**< Represents Z-Filter failure error code. */

#define ZF ZFilter
typedef struct ZF *ZF;   // Opaque struct for ZFilter implementation

/**
 * @brief Create a discrete Z-domain filter.
 *
 *
 * @note Coefficients are normalized internally by a[0].
 *
 * @param b_in Numerator coefficients
 * @param nb   Number of numerator coefficients
 * @param a_in Denominator coefficients
 * @param na   Number of denominator coefficients
 * 
 * @throw NullptrError if b_in or a_in are NULL
 * @throw ValueError if nb or na are 0
 * @throw ZeroDivisionError if a_in[0] is 0
 * @throw Zfilter_Failed if allocation fails
 *
 * @return Pointer to created filter
 */
extern ZF ZFilter_ctor(
    const float *b_in, uint32_t nb,
    const float *a_in, uint32_t na
);

/**
 * @brief Update the filter with a new input sample.
 *
 *
 * @param f Filter instance
 * @param x Input sample
 * 
 * @throw NullptrError if f is NULL
 *
 * @return Output sample
 */
extern float ZFilter_update(ZF f, float x);

/**
 * @brief Reset filter history (internal state) to zero; filter coefficients remain unchanged.
 *
 * @throw NullptrError if f is NULL
 */
extern void ZFilter_reset(ZF f);

/**
 * @brief Destroy filter and release all internal resources.
 *
 * Frees all internal resources, including coefficient arrays and history.
 * Safe to call with NULL.
 */
extern void ZFilter_dtor(ZF f);

#undef ZF

#ifdef __cplusplus
}
#endif

#endif