#include "PID_CONT.h"
#include "Controller_private.h"
#include "Control/Ztransform/Ztransform.h"

#include "utils/Exceptions_Assertions/except.h"

#include <stdint.h>
#include <stdarg.h>

struct PIDController {
    Controller base;
    ZFilter* PID_filter;
};

static void PIDController_ctor(Controller* self, va_list* args)
{
    PIDController* pid = (PIDController*)self;

    const float* b = va_arg(*args, const float*);
    uint32_t nb = va_arg(*args, uint32_t);

    const float* a = va_arg(*args, const float*);
    uint32_t na = va_arg(*args, uint32_t);

    /*
        Do not catch here.

        If ZFilter_ctor fails, let the exception propagate back to
        Controller_create(), which should destroy the partially-created
        controller.
    */
    pid->PID_filter = ZFilter_ctor(b, nb, a, na);
}

static float PIDController_update(Controller* self, float x)
{
    PIDController* pid = (PIDController*)self;

    if (!pid->PID_filter) {
        RAISE(NullptrError);
    }

    return ZFilter_update(pid->PID_filter, x);
}

static void PIDController_reset(Controller* self)
{
    PIDController* pid = (PIDController*)self;

    if (!pid->PID_filter) {
        RAISE(NullptrError);
    }

    ZFilter_reset(pid->PID_filter);
}

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

static const ControllerVTable PIDController_VTable = {
    .ctor = PIDController_ctor,
    .update = PIDController_update,
    .reset = PIDController_reset,
    .dtor = PIDController_dtor
};

const ControllerType PIDController_Type = {
    .size = sizeof(PIDController),
    .vtable = &PIDController_VTable
};