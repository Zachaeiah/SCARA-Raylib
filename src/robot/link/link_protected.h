#ifndef LINK_LINK_PROTECTED_H_
#define LINK_LINK_PROTECTED_H_

#include "Control/Actuator/Actuator.h"
#include "utils/Exceptions_Assertions/except.h"
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
    float Heading;
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
        .y = self->dim.y,
        .z = 0.0f
    };

    Vector3 end = Vector3Add(p->Start, RotateAroundY(localEnd, p->Heading));
    p->End = end;

    rlPushMatrix();

        rlTranslatef(p->Start.x, p->Start.y, p->Start.z);

        // Revolute joint: rotate around vertical Y axis
        rlRotatef(p->Heading * RAD2DEG, 0.0f, 1.0f, 0.0f);

        // Move cube center away from joint pivot
        rlTranslatef(localCenter.x, localCenter.y, localCenter.z);

        //DrawCubeV(Vector3Zero(), self->dim, self->color);
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
    if (!self || !self->protected) {
        RAISE(NullptrError);
    }

    Link_protected* p = self->protected;

    /*
        Convention for this version:

        p->Start = fixed parent attachment point
        p->JP    = slide distance
        visualStart = translated position of this link
        p->End   = translated end of this link

        The link does NOT stretch.
        The whole link translates.
    */

    float slide = p->JP;

    Vector3 slideOffset = {
        .x = 0.0f,
        .y = -slide,   // use +slide if you want upward motion
        .z = 0.0f
    };

    Vector3 visualStart = Vector3Add(p->Start, slideOffset);

    Vector3 localCenter = {
        .x = 0.0f,
        .y = -self->dim.y / 2.0f,
        .z = 0.0f
    };

    Vector3 localEnd = {
        .x = 0.0f,
        .y = -self->dim.y,
        .z = 0.0f
    };

    Vector3 end = Vector3Add(visualStart, localEnd);
    p->End = end;

    rlPushMatrix();

        // Move to fixed parent point
        rlTranslatef(p->Start.x, p->Start.y, p->Start.z);

        // Keep inherited heading
        rlRotatef(p->Heading * RAD2DEG, 0.0f, 1.0f, 0.0f);

        // Apply prismatic translation
        rlTranslatef(slideOffset.x, slideOffset.y, slideOffset.z);

        // Move cube center relative to translated start
        rlTranslatef(localCenter.x, localCenter.y, localCenter.z);

        DrawCubeWiresV(Vector3Zero(), self->dim, BLACK);

    rlPopMatrix();

    DrawSphere(p->Start, 0.08f, RED);          // fixed parent point
    DrawSphere(visualStart, 0.08f, ORANGE);    // translated top of link
    DrawSphere(p->End, 0.08f, BLUE);           // translated end
    DrawLine3D(p->Start, visualStart, PURPLE);
    DrawLine3D(visualStart, p->End, PURPLE);

    return p->End;
}
#endif // LINK_LINK_PROTECTED_H_