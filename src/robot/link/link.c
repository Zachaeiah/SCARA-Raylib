#include "Link.h"
#include "link_protected.h"
#include "Control/Actuator/Actuator.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <math.h>

//---------------------------- Structure Definitions ------------------------------------------------

static Link_protected* Link_protected_ctor(Actuator* actuator, Link_type type)
{
    Link_protected* p;

    NEW0(p);

    p->Start = Vector3Zero();
    p->End = Vector3Zero();
    p->JP = 0.0f;
    p->type = type;
    p->actuator = actuator;

    switch (type)
    {
        case REVOLUTE_LINK:
            p->draw = LINK_Draw_revolute;
            break;

        case PRISMATIC_LINK:
            p->draw = LINK_Draw_prismatic;
            break;

        default:
            FREE(p);
            RAISE(ValueError);
            break;
    }

    return p;
}


Link* LINK_ctor(Vector3 dim, Color color, Link_type type, Actuator* actuator)
{
    Link* self = NULL;

    if (!actuator) {
        RAISE(NullptrError);
    }

    if (Vector3Length(dim) <= 0.001f) {
        RAISE(ValueError);
    }
    
    NEW(self);

    self->color = color;
    self->dim = dim;
    self->protected = Link_protected_ctor(actuator, type);
    
    return self;
}

float LINK_update(Link* self, const float x_in)
{
    if (!self) {
        RAISE(NullptrError);
    }

    return Actuator_update(self->protected->actuator, x_in);
}


void LINK_Set_Pose( Link* self, const Vector3* start, const Vector3* end, const float* jp)
{
    if (!self) {
        RAISE(NullptrError);
    }

    if (start) {
        self->protected->Start = *start;
    }

    if (end) {
        self->protected->End = *end;
    }

    if (jp) {
        self->protected->JP = *jp;
    }
}

void LINK_Set_Start(Link* self, Vector3 start)
{
    LINK_Set_Pose(self, &start, NULL, NULL);
}


void LINK_Set_End(Link* self, Vector3 end)
{
    LINK_Set_Pose(self, NULL, &end, NULL);
}


void LINK_Set_JP(Link* self, float jp)
{
    LINK_Set_Pose(self, NULL, NULL, &jp);
}


Vector3 LINK_Draw(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
    }

    // call the draw function pointer for the link type
    self->protected->draw(self);

    // Debug visuals
    return self->protected->End;
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



