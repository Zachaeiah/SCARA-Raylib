#include "Link.h"
#include "link_protected.h"
#include "Control/Actuator/Actuator.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include "raylib.h"
#include "raymath.h"

//---------------------------- Structure Definitions ------------------------------------------------

static Link_protected* Link_protected_ctor(Actuator* actuator)
{
    Link_protected* protected;

    NEW0(protected);

    protected->Start = Vector3Zero();
    protected->End = Vector3Zero();
    protected->angle = 0.0f;
    protected->global_angle = 0.0f;
    protected->actuator = actuator;

    return protected;
}


Link* LINK_ctor(Vector3 dim, Color color, Actuator* actuator)
{
    Link* self = NULL;

    if (!actuator) {
        RAISE(NullptrError);
    }

    if (FloatEquals(Vector3Length(dim), 0.001f)) {
        RAISE(ValueError);
    }
    
    NEW(self);
    self->color = color;
    self->dim = dim;
    self->protected = Link_protected_ctor(actuator);
    
    return self;
}

float LINK_update(Link* self, const float x_in)
{
    if (!self) {
        RAISE(NullptrError);
    }

    return Actuator_update(self->protected->actuator, x_in);
}

void LINK_dtor(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
    }

    if (self->protected) {
        FREE(self->protected);
    }
    FREE(self);
    return;
}



