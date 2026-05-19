//--------------------------------------------------------------------------------------------------
// File: LINK.h
// Description: Module description
// Created on: 03
//--------------------------------------------------------------------------------------------------

#ifndef LINK_LINK_H_
#define LINK_LINK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Control/Actuator/Actuator.h"
#include "raylib.h"
#include "raymath.h"

//---------------------------- Structure Definitions ------------------------------------------------
typedef struct Link_protected Link_protected;

typedef enum Link_type {
    BASE_LINK,
    REVOLUTE_LINK,
    PRISMATIC_LINK,   // optional, but do not use for link4 here
    TCP_LINK
} Link_type;

typedef struct Link
{
    Color color; // color of the link
    Vector3 dim; // x: length, y: height, z: width/depth
    Link_protected* protected; // pointer to the protected data for the link

} Link;

//----------------------------- Function Prototypes --------------------------------------------------

/**
 * @brief will contructa link in the scara arm
 * 
 * @param dim the X:lenght Y: width Z:Hight
 * @param color the color of the link
 * @param type the type of the link
 * @param actuator the actuator for the link
 * @return Link* pointer to the constructed link instance
 */
extern Link* LINK_ctor(Vector3 dim, Color color, Link_type type, Actuator* actuator);

/**
 * @brief will update the link with a new angle input and return the speed output from the actuator
 * 
 * @param self pointer to the link instance
 * @param x_in the input angle to update the link with
 * @return float the output speed from the actuator after applying the input angle
 */
extern float LINK_update(Link* self, const float x_in);

/**
 * @brief will draw the link using raylib
 * 
 * @param self pointer to the link instance
 * @return Vector3 the position of the end of the link, which is used to draw the next link in the chain
 */
extern Vector3 LINK_Draw(Link* self);

/**
 * @brief set the link pose paramaters 
 * 
 * @param self pointer to the link instance
 * @param start set link start vector
 * @param end set link end vector
 * @param jp set link JP
 * @param heading set link Heading
 */
extern void LINK_Set_Pose( Link* self, const Vector3* start, const Vector3* end, const float* jp, const float* heading);

/**
 * @brief Set the link start vector
 * 
 * @param self pointer to the link instance
 * @param start the start vector
 */
extern void LINK_Set_Start(Link* self, Vector3 start);

/**
 * @brief  Set the link end vector
 * 
 * @param self pointer to the link instance
 * @param end the end vector
 */
extern void LINK_Set_End(Link* self, Vector3 end);

/**
 * @brief Set the link JP
 * 
 * @param self pointer to the link instance
 * @param jp the JP
 */
extern void LINK_Set_JP(Link* self, float jp);

/**
 * @brief Get the joint pos of the link
 * 
 * @param self  pointer to the link instance
 * @return float joint pos
 */
extern float LINK_Get_JP(const Link* self);

/**
 * @brief Set the world-space heading angle of the link.
 * 
 * @param self pointer to the link instance
 * @param headingRad heading angle in radians
 */
extern void LINK_Set_Heading(Link* self, float headingRad);

/**
 * @brief will destruct the link and free all allocated memory
 * 
 * @param self pointer to the link instance
 */
extern void LINK_dtor(Link* self);


#ifdef __cplusplus
}
#endif

#endif // LINK_H