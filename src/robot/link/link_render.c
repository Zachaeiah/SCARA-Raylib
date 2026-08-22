#include "link_internal.h"

#include "utils/Exceptions_Assertions/except.h"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <math.h>

/**
 * @brief 
 * 
 * @param v 
 * @param angle_rad 
 * @return Vector3 
 */
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

static void DrawCubeTexture(
    Texture2D texture,
    Vector3 position,
    float width,
    float height,
    float length,
    Color tint
)
{
    float x = position.x;
    float y = position.y;
    float z = position.z;

    float w = width / 2.0f;
    float h = height / 2.0f;
    float l = length / 2.0f;

    rlSetTexture(texture.id);

    rlBegin(RL_QUADS);

        rlColor4ub(tint.r, tint.g, tint.b, tint.a);

        // Front face
        rlNormal3f(0.0f, 0.0f, 1.0f);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(x - w, y - h, z + l);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(x + w, y - h, z + l);
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(x + w, y + h, z + l);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(x - w, y + h, z + l);

        // Back face
        rlNormal3f(0.0f, 0.0f, -1.0f);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(x - w, y - h, z - l);
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(x - w, y + h, z - l);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(x + w, y + h, z - l);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(x + w, y - h, z - l);

        // Top face
        rlNormal3f(0.0f, 1.0f, 0.0f);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(x - w, y + h, z - l);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(x - w, y + h, z + l);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(x + w, y + h, z + l);
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(x + w, y + h, z - l);

        // Bottom face
        rlNormal3f(0.0f, -1.0f, 0.0f);
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(x - w, y - h, z - l);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(x + w, y - h, z - l);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(x + w, y - h, z + l);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(x - w, y - h, z + l);

        // Right face
        rlNormal3f(1.0f, 0.0f, 0.0f);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(x + w, y - h, z - l);
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(x + w, y + h, z - l);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(x + w, y + h, z + l);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(x + w, y - h, z + l);

        // Left face
        rlNormal3f(-1.0f, 0.0f, 0.0f);
        rlTexCoord2f(0.0f, 0.0f); rlVertex3f(x - w, y - h, z - l);
        rlTexCoord2f(1.0f, 0.0f); rlVertex3f(x - w, y - h, z + l);
        rlTexCoord2f(1.0f, 1.0f); rlVertex3f(x - w, y + h, z + l);
        rlTexCoord2f(0.0f, 1.0f); rlVertex3f(x - w, y + h, z - l);

    rlEnd();

    rlSetTexture(0);
}

/**
 * @brief 
 * 
 * @param self 
 * @param local_center 
 */
static void LINK_DrawCubeBody(Link self, Vector3 local_center)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    switch (self->render_mode) {
        case LINK_RENDER_TEXTURED:
            if (self->has_texture && self->texture.id != 0) {
                DrawCubeTexture(
                    self->texture,
                    local_center,
                    self->dimensions.x,
                    self->dimensions.y,
                    self->dimensions.z,
                    WHITE
                );
            } else {
                DrawCubeV(local_center, self->dimensions, self->color);
            }
            break;

        case LINK_RENDER_WIREFRAME:
        default:
            DrawCubeWiresV(local_center, self->dimensions, BLACK);
            break;
    }
}

Vector3 LINK_RenderRevolute(Link self)
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

        LINK_DrawCubeBody(self, Vector3Zero());

    rlPopMatrix();

    DrawSphere(self->start_world, 0.08f, RED);
    DrawSphere(self->end_world, 0.08f, BLUE);
    DrawLine3D(self->start_world, self->end_world, PURPLE);

    return self->end_world;
}

Vector3 LINK_RenderPrismatic(Link self)
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

        LINK_DrawCubeBody(self, Vector3Zero());

    rlPopMatrix();

    DrawSphere(self->start_world, 0.08f, RED);
    DrawSphere(visual_start, 0.08f, ORANGE);
    DrawSphere(self->end_world, 0.08f, BLUE);

    DrawLine3D(self->start_world, visual_start, PURPLE);
    DrawLine3D(visual_start, self->end_world, PURPLE);

    return self->end_world;
}

Vector3 LINK_RenderBase(Link self)
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

    LINK_DrawCubeBody(self, center);

    DrawSphere(self->start_world, 0.08f, RED);
    DrawSphere(self->end_world, 0.08f, BLUE);
    DrawLine3D(self->start_world, self->end_world, PURPLE);

    return self->end_world;
}

Vector3 LINK_RenderTCP(Link self)
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

        LINK_DrawCubeBody(self, local_center);

    rlPopMatrix();

    DrawSphere(self->start_world, 0.08f, ORANGE);
    DrawSphere(self->end_world, 0.08f, BLUE);
    DrawLine3D(self->start_world, self->end_world, PURPLE);

    return self->end_world;
}