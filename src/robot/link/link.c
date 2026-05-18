#include "Link.h"
#include "link_protected.h"
#include "Control/Actuator/Actuator.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

//---------------------------- Structure Definitions ------------------------------------------------

static Vector3 RotateAroundY(Vector3 v, float angleRad)
{
    float c = cosf(angleRad);
    float s = sinf(angleRad);

    return (Vector3){
        .x = v.x*c + v.z*s,
        .y = v.y,
        .z = -v.x*s + v.z*c
    };
}

static Link_protected* Link_protected_ctor(Actuator* actuator)
{
    Link_protected* protected;

    NEW0(protected);

    protected->Start = Vector3Zero();
    protected->End = Vector3Zero();
    protected->JP = 0.0f;
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

    /**
     * @brief just for testing, will implement the actual control logic later
     * will just return the input angle for now
     * 
     */

    // update the joint angle using the actuator
    self->protected->JP = x_in;

    return 0.0f;

    //return Actuator_update(self->protected->actuator, x_in);
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


void LINK_Set_Start(Link* self, Vector3 start)
{
    if (!self) {
        RAISE(NullptrError);
    }

    self->protected->Start = start;
}


Vector3 LINK_Draw(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
    }

    Link_protected *protected = self->protected;

    Vector3 localCenter = {
        self->dim.x / 2.0f,
        self->dim.y / 2.0f,
        0.0f
    };

    Vector3 localEnd = {
        self->dim.x,
        self->dim.y,
        0.0f
    };

    Vector3 end = Vector3Add(protected->Start, RotateAroundY(localEnd, protected->JP));

    protected->End = end;

    rlPushMatrix();

        rlTranslatef(protected->Start.x, protected->Start.y, protected->Start.z);

        // Rotate around vertical Y axis
        rlRotatef(protected->JP * RAD2DEG, 0.0f, 1.0f, 0.0f);

        // Move cube center relative to pivot
        rlTranslatef(localCenter.x, localCenter.y, localCenter.z);

        //DrawCubeV(Vector3Zero(), self->dim, self->color);
        DrawCubeWiresV(Vector3Zero(), self->dim, BLACK);

    rlPopMatrix();

    // Debug visuals
    DrawSphere(protected->Start, 0.08f, RED);       // pivot/start joint
    DrawSphere(end, 0.08f, BLUE);        // next joint/end
    DrawLine3D(protected->Start, end, PURPLE);      // centerline

    return end;

}



