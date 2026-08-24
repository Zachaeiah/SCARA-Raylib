#include "Controller_private.h"
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"
#include <stdarg.h>

const Except_t Controller_failed = {"Controller error"};

Controller Controller_create(const ControllerType* type, ...)
{
    Controller self = NULL;
    va_list args;

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
    TRY {
        self = CALLOC(1, type->size);
        self->type = type;

        va_start(args, type);

        if (type->vtable->ctor) {
            self->type->vtable->ctor(self, &args);
        }
    }
    ELSE {
        LOG_ERROR_MSG(Controller_ErrorCode, "failed to create controller");
        Controller_destroy(self);
        RAISE(Controller_failed);
    }
    FINALLY {
        va_end(args);
    }
    END_TRY;

    return self;
}

float Controller_update(Controller self, float x)
{
    if (!self) RAISE(NullptrError);

    assert_debug(self->type->vtable->update);

    return self->type->vtable->update(self, x);
}

void Controller_reset(Controller self)
{
    if (!self) RAISE(NullptrError);

    assert_debug(self->type->vtable->reset);

    self->type->vtable->reset(self);
}

void Controller_destroy(Controller self)
{
    if (!self) RAISE(NullptrError);

    assert_debug(self->type->vtable->dtor);

    self->type->vtable->dtor(self);

    FREE(self);
}