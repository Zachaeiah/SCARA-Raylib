#include "link_internal.h"

#include "utils/Exceptions_Assertions/except.h"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <math.h>

static Vector3 RotateAroundY(Vector3 v, float angle_rad)
{
    float c = cosf(angle_rad);
    float s = sinf(angle_rad);

    return (Vector3){
        .x = v.x * c + v.z * s,
        .y = v.y,
        .z = -v.x * s + v.z * c
    };
}

Vector3 LINK_RenderRevolute(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    float length = self->dimensions.x;

    Vector3 local_center = {
        .x = length / 2.0f,
        .y = self->dimensions.y / 2.0f,
        .z = 0.0f
    };

    Vector3 local_end = {
        .x = length,
        .y = self->dimensions.y,
        .z = 0.0f
    };

    self->end_world = Vector3Add(
        self->start_world,
        RotateAroundY(local_end, self->heading_world_rad)
    );

    rlPushMatrix();

        rlTranslatef(
            self->start_world.x,
            self->start_world.y,
            self->start_world.z
        );

        rlRotatef(
            self->heading_world_rad * RAD2DEG,
            0.0f,
            1.0f,
            0.0f
        );

        rlTranslatef(
            local_center.x,
            local_center.y,
            local_center.z
        );

        DrawCubeWiresV(Vector3Zero(), self->dimensions, BLACK);

    rlPopMatrix();

    DrawSphere(self->start_world, 0.08f, RED);
    DrawSphere(self->end_world, 0.08f, BLUE);
    DrawLine3D(self->start_world, self->end_world, PURPLE);

    return self->end_world;
}

Vector3 LINK_RenderPrismatic(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    /*
        Convention:
            start_world     = fixed parent attachment point
            joint_position  = slide distance
            visual_start    = translated top of prismatic link
            end_world       = translated end of prismatic link

        The link does not stretch.
        The whole link translates.
    */

    float slide = self->joint_position;

    Vector3 slide_offset = {
        .x = 0.0f,
        .y = slide,
        .z = 0.0f
    };

    Vector3 visual_start = Vector3Add(self->start_world, slide_offset);

    Vector3 local_center = {
        .x = 0.0f,
        .y = -self->dimensions.y / 2.0f,
        .z = 0.0f
    };

    Vector3 local_end = {
        .x = 0.0f,
        .y = -self->dimensions.y,
        .z = 0.0f
    };

    self->end_world = Vector3Add(visual_start, local_end);

    rlPushMatrix();

        rlTranslatef(
            self->start_world.x,
            self->start_world.y,
            self->start_world.z
        );

        rlRotatef(
            self->heading_world_rad * RAD2DEG,
            0.0f,
            1.0f,
            0.0f
        );

        rlTranslatef(
            slide_offset.x,
            slide_offset.y,
            slide_offset.z
        );

        rlTranslatef(
            local_center.x,
            local_center.y,
            local_center.z
        );

        DrawCubeWiresV(Vector3Zero(), self->dimensions, BLACK);

    rlPopMatrix();

    DrawSphere(self->start_world, 0.08f, RED);
    DrawSphere(visual_start, 0.08f, ORANGE);
    DrawSphere(self->end_world, 0.08f, BLUE);

    DrawLine3D(self->start_world, visual_start, PURPLE);
    DrawLine3D(visual_start, self->end_world, PURPLE);

    return self->end_world;
}

Vector3 LINK_RenderBase(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    Vector3 center = {
        .x = self->start_world.x,
        .y = self->start_world.y + self->dimensions.y / 2.0f,
        .z = self->start_world.z
    };

    self->end_world = (Vector3){
        .x = self->start_world.x,
        .y = self->start_world.y + self->dimensions.y,
        .z = self->start_world.z
    };

    DrawCubeWiresV(center, self->dimensions, BLACK);

    DrawSphere(self->start_world, 0.08f, RED);
    DrawSphere(self->end_world, 0.08f, BLUE);
    DrawLine3D(self->start_world, self->end_world, PURPLE);

    return self->end_world;
}

Vector3 LINK_RenderTCP(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    /*
        TCP link:
            start_world       = attachment point from link3
            cube hangs down from start_world
            heading_world_rad = inherited world heading
            end_world         = TCP point
    */

    Vector3 local_center = {
        .x = 0.0f,
        .y = -self->dimensions.y / 2.0f,
        .z = 0.0f
    };

    Vector3 local_end = {
        .x = 0.0f,
        .y = -self->dimensions.y,
        .z = 0.0f
    };

    self->end_world = Vector3Add(self->start_world, local_end);

    rlPushMatrix();

        rlTranslatef(
            self->start_world.x,
            self->start_world.y,
            self->start_world.z
        );

        rlRotatef(
            self->heading_world_rad * RAD2DEG,
            0.0f,
            1.0f,
            0.0f
        );

        DrawCubeWiresV(local_center, self->dimensions, BLACK);

    rlPopMatrix();

    DrawSphere(self->start_world, 0.08f, ORANGE);
    DrawSphere(self->end_world, 0.08f, BLUE);
    DrawLine3D(self->start_world, self->end_world, PURPLE);

    return self->end_world;
}