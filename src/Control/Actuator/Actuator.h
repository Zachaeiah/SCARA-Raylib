#ifndef CONTROL_ACTUATOR_H_
#define CONTROL_ACTUATOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "Control/Ztransform/Ztransform.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/Logger/logger.h"


extern const Except_t Actuator_Failed; // Represents Actuator failure
extern const ErrorType Actuator_Failed_ErrorCode; /**< Represents Actuator failure error code. */

/**
 * @brief The Actuator structure, which applies a ZFilter to the input and includes dead zone and saturation limits.
 */
#define AT Actuator
typedef struct AT *AT;

/**
 * @brief Constructs a new Actuator object with the given filter coefficients and limits.
 * 
 * @param filter pointer to the ZFilter
 * @param Dead_Zone the dead zone limit
 * @param Saturation the saturation limit
 * @return Actuator* 
 */
extern AT Actuator_ctor(ZFilter filter,
                        float dead_zone, float saturation);

/**
 * @brief Updates the actuator with a new input value
 * 
 * @param self pointer to the actuator instance
 * @param x_in input value to update the actuator with
 * @return the output value after applying the ZFilter, dead zone, and saturation
 */
extern float Actuator_update(AT self, float x_in);

/**
 * @brief Resets the actuator to its initial state
 * @param self pointer to the actuator instance
 */
extern void Actuator_reset(AT self);


/**
 * @brief Destroys the Actuator object and frees resources
 * 
 * @param self pointer to the actuator instance
 */
extern void Actuator_dtor(AT self);

#undef AT

#ifdef __cplusplus
}
#endif // __cplusplus


#endif
