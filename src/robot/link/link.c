#include "Link.h"
#include "link_internal.h"

#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include "raylib.h"
#include "raymath.h"

#include <math.h>


Link* LINK_Create(Vector3 dimensions, Color color, LinkType type, Actuator* actuator)
{
    if (Vector3Length(dimensions) <= 0.001f) {
        RAISE(ValueError);
        return NULL;
    }

    Link* self = NULL;
    NEW0(self);

    self->color = color;
    self->dimensions = dimensions;

    self->start_world = Vector3Zero();
    self->end_world = Vector3Zero();

    self->joint_position = 0.0f;
    self->heading_world_rad = 0.0f;

    self->type = type;
    self->actuator = actuator;

    switch (type) {
        case LINK_BASE:
            self->draw = LINK_RenderBase;
            break;

        case LINK_REVOLUTE:
            self->draw = LINK_RenderRevolute;
            break;

        case LINK_PRISMATIC:
            self->draw = LINK_RenderPrismatic;
            break;

        case LINK_TCP:
            self->draw = LINK_RenderTCP;
            break;

        default:
            FREE(self);
            RAISE(ValueError);
            return NULL;
    }

    return self;
}


float LINK_UpdateActuator(Link* self, float command)
{
    if (!self) {
        RAISE(NullptrError);
        return 0.0f;
    }

    if (!self->actuator) {
        return 0.0f;
    }

    return Actuator_update(self->actuator, command);
}

void LINK_IntegrateJointPosition(Link* self, float joint_velocity, float dt)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    if (dt <= 0.0f) {
        RAISE(ValueError);
        return;
    }

    switch (self->type) {
        case LINK_REVOLUTE:
        case LINK_PRISMATIC:
            self->joint_position += joint_velocity * dt;
            break;

        case LINK_BASE:
        case LINK_TCP:
            break;

        default:
            RAISE(ValueError);
            return;
    }
}

void LINK_SetJointPosition(Link* self, float joint_position)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    self->joint_position = joint_position;
}

float LINK_GetJointPosition(const Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return 0.0f;
    }

    return self->joint_position;
}

void LINK_SetStart(Link* self, Vector3 start_world)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    self->start_world = start_world;
}

void LINK_SetEnd(Link* self, Vector3 end_world)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    self->end_world = end_world;
}

void LINK_SetHeadingWorld(Link* self, float heading_world_rad)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    self->heading_world_rad = heading_world_rad;
}

Vector3 LINK_GetStart(const Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    return self->start_world;
}

Vector3 LINK_GetEnd(const Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    return self->end_world;
}

float LINK_GetHeadingWorld(const Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return 0.0f;
    }

    return self->heading_world_rad;
}

Vector3 LINK_GetDimensions(const Link* self){
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    return self->dimensions;

}

Vector3 LINK_Draw(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    if (!self->draw) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    return self->draw(self);
}

void LINK_Destroy(Link* self)
{
    if (!self) {
        return;
    }

    FREE(self);
}



