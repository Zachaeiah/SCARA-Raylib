#ifndef LINK_INTERNAL_H_
#define LINK_INTERNAL_H_

#include "link.h"

struct Link {
    Color color;
    Vector3 dimensions;

    Vector3 start_world;
    Vector3 end_world;

    float joint_position;
    float heading_world_rad;

    LinkType type;
    Actuator* actuator;

    Vector3 (*draw)(Link* self);
};

Vector3 LINK_RenderBase(Link* self);
Vector3 LINK_RenderRevolute(Link* self);
Vector3 LINK_RenderPrismatic(Link* self);
Vector3 LINK_RenderTCP(Link* self);

#endif