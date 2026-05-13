#include "Actuator.h"
#include "Control/Ztransform/Ztransform.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
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
 * @brief Constructs a new Actuator object with the given filter coefficients and limits
 * 
 * @param filter pointer to the ZFilter coefficients
 * @param limts the limits for the actuator (dead zone and saturation)
 * @return Actuator* pointer to the newly created Actuator object
 * @note This function allocates memory for the Actuator and its ZFilter.
 */
extern Actuator* Actuator_ctor(
                        const float *b, uint32_t nb,
                        const float *a, uint32_t na, 
                        float Dead_Zone, float Saturation)
{
    Actuator* actuator = NULL;

    
    TRY{
        
        // Allocate memory for the actuator
        NEW(actuator);

        // Create ZFilter for the actuator
        actuator->filter = ZFilter_ctor(b, nb, a, na);
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
}

/**
 * @brief Updates the actuator with a new input value
 * 
 * @param self pointer to the actuator instance
 * @param x input value to update the actuator with
 */
void Actuator_update(Actuator* self, float x){

    if (!self) return;

    // apply dead zone
    if (fabsf(x) < self->Dead_Zone) {
        x = 0.0f;
    }

    // update filter
    float y = ZFilter_update(self->filter, x);

    // apply saturation
    y = clampf(y, -self->Saturation, self->Saturation);

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
 Actuator_dtor(Actuator* self){
    if (!self) return;

    if (self->filter) ZFilter_dtor(self->filter);
    FREE(self);
}