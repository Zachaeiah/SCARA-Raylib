#include "GainController.h"
#include "Controller_private.h"

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
 */
static void GainController_ctor(Controller* self)
{
    GainController* gain = (GainController*)self;

    gain->k = 2.0f;
}

/**
 * @brief Update the gain controller with a new input value.
 * 
 * @param self A pointer to the controller instance.
 * @param x The input value.
 * @return float The output value.
 */
static float GainController_update(Controller* self, float x)
{
    GainController* gain = (GainController*)self;

    return gain->k * x;
}

/**
 * @brief Reset the gain controller to its initial state.
 * 
 * @param self A pointer to the controller instance to reset.
 */
static void GainController_reset(Controller* self)
{
    GainController* gain = (GainController*)self;

    gain->k = 2.0f;
}

/**
 * @brief Destructor for the GainController. Currently, there are
 * no dynamic resources to free, so this function does nothing.
 * 
 * @param self A pointer to the controller instance to destroy.
 */
static void GainController_dtor(Controller* self)
{
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