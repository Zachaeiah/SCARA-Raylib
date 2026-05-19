#ifndef LINK_LINK_PROTECTED_H_
#define LINK_LINK_PROTECTED_H_

#include "Control/Actuator/Actuator.h"
#include "link.h"
#include "raymath.h"
#include "rlgl.h"


/**
 * @brief The protected data structure for the Link. 
 * This contains the internal state and implementation details 
 * of the Link that are not exposed in the public interface.
 * 
 * @param Start the starting position of the link, where 
 * the axis of rotation is located
 * 
 * @param End the position of the end of the link, where 
 * the next link will be attached
 * 
 * @param JP the joint position of the link, which is the 
 * angle for a revolute link and the extension distance for a prismatic link
 * 
 * @param type the type of the link, which determines how 
 * it is drawn and updated
 * 
 * @param actuator a pointer to the Actuator that controls 
 * the link's movement
 * 
 * @param draw a function pointer to the draw function for 
 * the link, which is set based on the link type
 * 
 */
typedef struct Link_protected{
    Vector3 Start; // where the axis of rotation is located
    Vector3 End; // where the next link will be attached
    float JP;  // revolute: angle in radians, prismatic: extension distance
    Link_type type; // type of the link
    Actuator* actuator; // actuator for the link V
    Vector3 (*draw)(Link* self);
}Link_protected;

/**
 * @brief 
 * 
 * @param v 
 * @param angleRad 
 * @return Vector3 
 */
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

/**
 * @brief will draw the link using raylib
 * 
 * @param self pointer to the link instance
 * @return Vector3 the position of the end of the link, 
 * which is used to draw the next link in the chain
 */
static Vector3 LINK_Draw_revolute(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
    }

    Link_protected* p = self->protected;

    float length = self->dim.x;

    Vector3 localCenter = {
        .x = length / 2.0f,
        .y = self->dim.y / 2.0f,
        .z = 0.0f
    };

    Vector3 localEnd = {
        .x = length,
        .y = 0.0f,
        .z = 0.0f
    };

    Vector3 end = Vector3Add(p->Start, RotateAroundY(localEnd, p->JP));
    p->End = end;

    rlPushMatrix();

        rlTranslatef(p->Start.x, p->Start.y, p->Start.z);

        // Revolute joint: rotate around vertical Y axis
        rlRotatef(p->JP * RAD2DEG, 0.0f, 1.0f, 0.0f);

        // Move cube center away from joint pivot
        rlTranslatef(localCenter.x, localCenter.y, localCenter.z);

        DrawCubeV(Vector3Zero(), self->dim, self->color);
        DrawCubeWiresV(Vector3Zero(), self->dim, BLACK);

    rlPopMatrix();

    DrawSphere(p->Start, 0.08f, RED);
    DrawSphere(end, 0.08f, BLUE);
    DrawLine3D(p->Start, end, PURPLE);

    return end;
}

/**
 * @brief will draw the link using raylib
 * 
 * @param self pointer to the link instance
 * @return Vector3 the position of the end of the link,
 * which is used to draw the next link in the chain
 */
static Vector3 LINK_Draw_prismatic(Link* self)
{
    if (!self) {
        RAISE(NullptrError);
    }

    Link_protected* p = self->protected;

    float drawLength = self->dim.x + p->JP;

    if (drawLength < 0.001f) {
        drawLength = 0.001f;
    }

    Vector3 drawDim = {
        .x = drawLength,
        .y = self->dim.y,
        .z = self->dim.z
    };

    Vector3 localCenter = {
        .x = drawLength / 2.0f,
        .y = self->dim.y / 2.0f,
        .z = 0.0f
    };

    Vector3 localEnd = {
        .x = drawLength,
        .y = 0.0f,
        .z = 0.0f
    };

    Vector3 end = Vector3Add(p->Start, localEnd);
    p->End = end;

    rlPushMatrix();

        rlTranslatef(p->Start.x, p->Start.y, p->Start.z);

        // Prismatic joint: no rotation from JP.
        // JP changes the link length.
        rlTranslatef(localCenter.x, localCenter.y, localCenter.z);

        DrawCubeV(Vector3Zero(), drawDim, self->color);
        DrawCubeWiresV(Vector3Zero(), drawDim, BLACK);

    rlPopMatrix();

    DrawSphere(p->Start, 0.08f, RED);
    DrawSphere(end, 0.08f, BLUE);
    DrawLine3D(p->Start, end, PURPLE);

    return end;
}


#endif // LINK_LINK_PROTECTED_H_