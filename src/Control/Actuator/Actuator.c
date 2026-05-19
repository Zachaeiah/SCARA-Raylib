#include "Actuator.h"
#include <math.h>
#include "utils/Logger/logger.h"
#include "Control/Ztransform/Ztransform.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/MemAllocator/mem.h"

const Except_t Actuator_Failed = {"Actuator faild"};

/**
 * @brief clams float between a min and max value
 * 
 * @param value The value to be clamped
 * @param min The min value to cmap to
 * @param max The max value to camp to
 * @return clamped float 
 */
static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief Actuator struct definition
 * 
 * @param filter pointer to the ZFilter used by the actuator
 * @param Dead_Zone the dead zone threshold for the actuator
 * @param Saturation the saturation limit for the actuator output
 * @note The actuator applies a dead zone to the input, 
 * processes it through the ZFilter, and then applies s
 * aturation to the output.
 * 
 */
typedef struct Actuator{
    ZFilter* filter;
    float Dead_Zone;
    float Saturation;

}Actuator;

/**
 * @brief Constructs a new Actuator object with the given filter coefficients and limits.
 * 
 * @param filter pointer to the ZFilter
 * @param Dead_Zone the dead zone limit
 * @param Saturation the saturation limit
 * @return Actuator* 
 */
extern Actuator* Actuator_ctor(ZFilter* filter, 
                        float Dead_Zone, float Saturation)
{
    Actuator* actuator = NULL;

    
    TRY{
        
        // Allocate memory for the actuator
        NEW(actuator);

        // Create ZFilter for the actuator
        actuator->filter = filter;
        actuator->Dead_Zone = Dead_Zone;
        actuator->Saturation = Saturation;

    }EXCEPT(MemroyError) {
        printf("actuator allocation failed: %s\n", Except_frame.exception->reason);

        if (actuator) Actuator_dtor(actuator);

        // RAISE the Actuator_Failed exception to indicate 
        // that the actuator construction failed
        RAISE(Actuator_Failed);
    }EXCEPT(NullptrError) {
        
        printf("inputs *a || *b is NULL: %s\n", Except_frame.exception->reason);
        if (actuator) Actuator_dtor(actuator);

        // RAISE the Actuator_Failed exception to indicate 
        // that the actuator construction failed
        RAISE(Actuator_Failed);
    } END_TRY;
    
    return actuator;
}

/**
 * @brief Updates the actuator with a new input value
 * 
 * @param self pointer to the actuator instance
 * @param x input value to update the actuator with
 * @return the output value after applying the ZFilter, dead zone, and saturation
 */
float Actuator_update(Actuator* self, float x_in){

    if (!self) return 0.0;

    // apply dead zone
    if (fabsf(x_in) < self->Dead_Zone) {
        x_in = 0.0f;
    }

    // update filter
    float y = ZFilter_update(self->filter, x_in);

    // apply saturation
    y = clampf(y, -self->Saturation, self->Saturation);

    return y;

}

/**
 * @brief Resets the actuator to its initial state
 * 
 * @param self pointer to the actuator instance
 */

void Actuator_reset(Actuator* self){
    if (!self) return;

    ZFilter_reset(self->filter);
}


/**
 * @brief Construct a new Actuator_dtor object
 * 
 * @param self 
 */
void Actuator_dtor(Actuator* self){
    if (!self) return;

    LOG_DEBUG_MSG(NO_ERROR, "freeing Actuator");

    FREE(self);
}