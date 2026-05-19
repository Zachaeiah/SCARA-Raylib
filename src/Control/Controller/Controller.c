#include "Controller_private.h"
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include <stdarg.h>

Controller* Controller_create(const ControllerType* type, ...)
{
    Controller* self = NULL;

    if (!type) {
        RAISE(NullptrError);
    }

    if (!type->vtable) {
        RAISE(NullptrError);
    }

    if (type->size < sizeof(Controller)) {
        RAISE(ValueError);
    }

    if (!type->vtable->update) {
        RAISE(NotImplementedError);
    }

    /*
        Allocate the full derived object, not just the base Controller.
        CALLOC makes partial-construction cleanup safe.
    */
    self = CALLOC(1, type->size);
    self->type = type;

    if (type->vtable->ctor) {
        va_list args;
        int ctor_failed = 0;
        const Except_t* caught = NULL;

        va_start(args, type);

        TRY {
            LOG_DEBUG_MSG(NO_ERROR, "createing controller with Vtable->ctor");
            self->type->vtable->ctor(self, &args);
        }
        ELSE {
            ctor_failed = 1;
            caught = Except_frame.exception;
        }
        FINALLY {
            va_end(args);
        }
        END_TRY;

        if (ctor_failed) {
            LOG_ERROR_MSG(1, "faild to crate controller");
            Controller_destroy(self);

            if (caught) {
                RAISE(*caught);
            }

            RAISE(NotImplementedError);
        }
    }

    return self;
}

float Controller_update(Controller* self, float x)
{
    if (!self) {
        return 0.0f;
    }

    assert_debug(self->type);
    assert_debug(self->type->vtable);

    if (!self->type->vtable->update) {
        RAISE(NotImplementedError);
    }

    return self->type->vtable->update(self, x);
}

void Controller_reset(Controller* self)
{
    if (!self) {
        RAISE(NullptrError);
    }

    assert_debug(self->type);
    assert_debug(self->type->vtable);

    /*
        Reset can be optional.
        If a controller has no state to reset, no-op is fine.
    */
    if (self->type->vtable->reset) {
        self->type->vtable->reset(self);
    }

    LOG_INFO_MSG(NO_ERROR, "Reseting Controller");
}

void Controller_destroy(Controller* self)
{
    if (!self) {
        return;
    }

    LOG_DEBUG_MSG(NO_ERROR, "Destroying Contoller");

    if (self->type && self->type->vtable && self->type->vtable->dtor) {
        self->type->vtable->dtor(self);
    }

    FREE(self);
}