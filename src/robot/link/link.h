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



typedef struct Link
{
    Color color; // color of the link
    Vector3 dim; // x: length, y: width, z: height
    Link_protected* protected; // pointer to the protected data for the link

} Link;

//----------------------------- Function Prototypes --------------------------------------------------

/**
 * @brief will contructa link in the scara arm
 * 
 * @param dim the X:lenght Y: width Z:Hight
 * @param color the color of the link
 * @param actuator the actuator for the link
 * @return Link* pointer to the constructed link instance
 */
extern Link* LINK_ctor(Vector3 dim, Color color, Actuator* actuator);

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
 * @brief will set the start position of the link, which is used to draw the link in the correct position
 * 
 * @param self pointer to the link instance
 * @param start the start position of the link, which is used to draw the link in the correct position
 */
extern void LINK_Set_Start(Link* self, Vector3 start);

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