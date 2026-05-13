#include "PID_CONT.h"
#include "Control/Controller/Controller_private.h"
#include "Control/Ztransform/Ztransform.h"

#include "utils/Exceptions_Assertions/except.h"

#include <stdint.h>
#include <stdarg.h>

/**
 * @brief The PIDController structure, which extends the base Controller structure.
 * 
 * @param base The base Controller structure, which must be the first member of the PIDController structure.
 * @param PID_filter A pointer to the ZFilter used for PID control.
 */
struct PIDController {
    Controller base;
    ZFilter* PID_filter;
};

/**
 * @brief Constructor for the PIDController structure.
 * 
 * @param self A pointer to the Controller structure.
 * @param args A pointer to the variable arguments list.
 */
static void PIDController_ctor(Controller* self, va_list* args)
{
    PIDController* pid = (PIDController*)self;

    const float* b = va_arg(*args, const float*);
    uint32_t nb = va_arg(*args, uint32_t);

    const float* a = va_arg(*args, const float*);
    uint32_t na = va_arg(*args, uint32_t);

    /**
     * @brief Constructor for the PIDController structure.
     * 
     * @param self A pointer to the Controller structure.
     * @param args A pointer to the variable arguments list.
     */
    pid->PID_filter = ZFilter_ctor(b, nb, a, na);
}

/**
 * @brief Updates the PIDController with a new input value.
 * 
 * @param self A pointer to the Controller structure.
 * @param x The new input value.
 * @return The updated output value.
 */
static float PIDController_update(Controller* self, float x)
{
    PIDController* pid = (PIDController*)self;

    if (!pid->PID_filter) {
        RAISE(NullptrError);
    }

    return ZFilter_update(pid->PID_filter, x);
}

/**
 * @brief Resets the PIDController.
 * 
 * @param self A pointer to the Controller structure.
 */
static void PIDController_reset(Controller* self)
{
    PIDController* pid = (PIDController*)self;

    if (!pid->PID_filter) {
        RAISE(NullptrError);
    }

    ZFilter_reset(pid->PID_filter);
}

/**
 * @brief Destructor for the PIDController structure.
 * 
 * @param self A pointer to the Controller structure.
 */
static void PIDController_dtor(Controller* self)
{
    PIDController* pid = (PIDController*)self;

    /*
        Do not FREE(pid) here.
        Controller_destroy() owns the base object memory.
    */
    ZFilter_dtor(pid->PID_filter);
    pid->PID_filter = NULL;
}

/**
 * @brief The virtual table for the PIDController structure.
 */
static const ControllerVTable PIDController_VTable = {
    .ctor = PIDController_ctor,
    .update = PIDController_update,
    .reset = PIDController_reset,
    .dtor = PIDController_dtor
};

/**
 * @brief The type information for the PIDController structure.
 */
const ControllerType PIDController_Type = {
    .size = sizeof(PIDController),
    .vtable = &PIDController_VTable
};