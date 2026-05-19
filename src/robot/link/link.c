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
    p->Heading = 0.0f;
    p->type = type;
    p->actuator = actuator;

    switch (type)
{
        case BASE_LINK:
            p->draw = LINK_Draw_base;
            break;

        case REVOLUTE_LINK:
            p->draw = LINK_Draw_revolute;
            break;

        case PRISMATIC_LINK:
            p->draw = LINK_Draw_prismatic;
            break;

        case TCP_LINK:
            p->draw = LINK_Draw_tcp;
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

    Link_protected* p = self->protected;

    if (!p) {
        RAISE(NullptrError);
    }

    // Passive link: directly use the input as JP
    if (!p->actuator) {
        p->JP = x_in;
        return p->JP;
    }

    // Actuated link: actuator output becomes the joint position
    p->JP = Actuator_update(p->actuator, x_in);

    return p->JP;
}


void LINK_Set_Pose(Link* self,
                   const Vector3* start,
                   const Vector3* end,
                   const float* jp,
                   const float* heading)
{
    if (!self || !self->protected) {
        RAISE(NullptrError);
    }

    Link_protected* p = self->protected;

    if (start) {
        p->Start = *start;
    }

    if (end) {
        p->End = *end;
    }

    if (jp) {
        p->JP = *jp;
    }

    if (heading) {
        p->Heading = *heading;
    }
}

void LINK_Set_Start(Link* self, Vector3 start)
{
    LINK_Set_Pose(self, &start, NULL, NULL, NULL);
}

void LINK_Set_End(Link* self, Vector3 end)
{
    LINK_Set_Pose(self, NULL, &end, NULL, NULL);
}

void LINK_Set_JP(Link* self, float jp)
{
    LINK_Set_Pose(self, NULL, NULL, &jp, NULL);
}

void LINK_Set_Heading(Link* self, float headingRad)
{
    LINK_Set_Pose(self, NULL, NULL, NULL, &headingRad);
}

float LINK_Get_JP(const Link* self)
{
    if (!self || !self->protected) {
        RAISE(NullptrError);
    }

    return self->protected->JP;
}


Vector3 LINK_Draw(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
    }

    return self->protected->draw(self);
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



