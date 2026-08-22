#include "GainController.h"
#include "utils/Logger/logger.h"
#include "Control/Controller/Controller_private.h"

/**
 * @brief A simple gain controller that multiplies the input 
 * by a constant factor k.
 * 
 */
typedef struct GainController {
    Controller base;
    float k;
} GainController;

/**
 * @brief Constructor for the GainController. Initializes the
 * gain factor k to a default value.
 * 
 * @param self A pointer to the controller instance to construct.
 * @param args A pointer to the variable arguments list.
 */
static void GainController_ctor(Controller self, va_list* args)
{
    
    GainController* gain = (GainController*)self;

    gain->k = (float)va_arg(*args, double);

    LOG_DEBUG_MSG(NO_ERROR, "Crateing Gain controller with gain %f", gain->k);
}

/**
 * @brief Update the gain controller with a new input value.
 * 
 * @param self A pointer to the controller instance.
 * @param x The input value.
 * @return float The output value.
 */
static float GainController_update(Controller self, float x)
{
    GainController* gain = (GainController*)self;

    // compute the output of the controller
    float output =  gain->k * x;

    LOG_DEBUG_MSG(NO_ERROR,"Updateing GainController with Gain: %f, X: %f, Out: %f", gain->k , x, output);

    return output;
}

/**
 * @brief Reset the gain controller to its initial state.
 * 
 * @param self A pointer to the controller instance to reset.
 */
static void GainController_reset(Controller self)
{
    LOG_INFO_MSG(NO_ERROR, "Resting Gain controller");
    (void)self;
}

/**
 * @brief Destructor for the GainController. Currently, there are
 * no dynamic resources to free, so this function does nothing.
 * 
 * @param self A pointer to the controller instance to destroy.
 */
static void GainController_dtor(Controller self)
{
    GainController* gain = (GainController*)self;
    LOG_DEBUG_MSG(NO_ERROR, "Destroying Gain Controller with gain: %f", gain->k);
    (void)self;

    // Nothing dynamic to free for this controller.
}

/**
 * @brief The virtual function table for the GainController.
 */
static const ControllerVTable GainController_VTable = {
    .ctor = GainController_ctor,
    .update = GainController_update,
    .reset = GainController_reset,
    .dtor = GainController_dtor
};

/**
 * @brief The type information for the GainController, including its size
 * and virtual function table.
 */
const ControllerType GainController_Type = {
    .size = sizeof(GainController),
    .vtable = &GainController_VTable
};