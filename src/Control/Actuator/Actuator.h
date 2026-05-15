#ifndef CONTROL_ACTUATOR_H_
#define CONTROL_ACTUATOR_H_

#include <stdint.h>
#include "Control/Ztransform/Ztransform.h"
#include "utils/Exceptions_Assertions/except.h"

extern const Except_t Actuator_Failed;

/**
 * @brief The Actuator structure, which applies a ZFilter to the input and includes dead zone and saturation limits.
 * 
 */
typedef struct Actuator Actuator;


/**
 * @brief Constructs a new Actuator object with the given filter coefficients and limits.
 * 
 * @param filter pointer to the ZFilter
 * @param Dead_Zone the dead zone limit
 * @param Saturation the saturation limit
 * @return Actuator* 
 */
extern Actuator* Actuator_ctor(ZFilter* filter,
                        float Dead_Zone, float Saturation);

/**
 * @brief Updates the actuator with a new input value
 * 
 * @param self pointer to the actuator instance
 * @param x_in input value to update the actuator with
 * @return the output value after applying the ZFilter, dead zone, and saturation
 */
extern float Actuator_update(Actuator* self, float x_in);

/**
 * @brief Resets the actuator to its initial state
 * 
 * @param self pointer to the actuator instance
 */

extern void Actuator_reset(Actuator* self);


/**
 * @brief Construct a new Actuator_dtor object
 * 
 * @param self pointer to the actuator instance
 */
extern void Actuator_dtor(Actuator* self);

#endif