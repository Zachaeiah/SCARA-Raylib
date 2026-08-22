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

//---------------------------- Structure Definitions ------------------------------------------------

typedef enum LinkType {
    LINK_BASE = 0,
    LINK_REVOLUTE,
    LINK_PRISMATIC,
    LINK_TCP
} LinkType;

typedef enum LinkRenderMode {
    LINK_RENDER_WIREFRAME = 0,
    LINK_RENDER_TEXTURED
} LinkRenderMode;

typedef struct Link Link;

//----------------------------- Function Prototypes --------------------------------------------------

/**
 * @brief will contructa link in the scara arm
 * 
 * @param dimensions the X:lenght Y: width Z:Hight
 * @param color the color of the link
 * @param type the type of the link
 * @param actuator the actuator for the link
 * @return Link* pointer to the constructed link instance
 */
Link* LINK_Create(Vector3 dimensions, Color color, LinkType type, Actuator actuator);

/**
 * @brief will update the link with a new angle input and return the speed output from the actuator
 * 
 * @param self pointer to the link instance
 * @param x_in the input angle to update the link with
 * @return float the output speed from the actuator after applying the input angle
 */
extern float LINK_UpdateActuator(Link* self, const float x_in);

/**
 * @brief 
 * 
 * @param self pointer to the link instance
 * @param joint_velocity 
 * @param dt 
 */
extern void LINK_IntegrateJointPosition(Link* self, float joint_velocity, float dt);

/**
 * @brief 
 * 
 * @param self 
 * @param joint_position 
 */
extern void  LINK_SetJointPosition(Link* self, float joint_position);

/**
 * @brief 
 * 
 * @param self 
 * @return float 
 */
extern float LINK_GetJointPosition(const Link* self);

/**
 * @brief 
 * 
 * @param self 
 * @param start_world 
 */
extern void LINK_SetStart(Link* self, Vector3 start_world);

/**
 * @brief 
 * 
 * @param self 
 * @param end_world 
 */
extern void LINK_SetEnd(Link* self, Vector3 end_world);

/**
 * @brief 
 * 
 * @param self 
 * @param heading_world_rad 
 */
extern void LINK_SetHeadingWorld(Link* self, float heading_world_rad);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
extern Vector3 LINK_GetStart(const Link* self);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
extern Vector3 LINK_GetEnd(const Link* self);

/**
 * @brief 
 * 
 * @param self 
 * @return float 
 */
extern float LINK_GetHeadingWorld(const Link* self);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
extern Vector3 LINK_GetDimensions(const Link* self);

/**
 * @brief will draw the link using raylib
 * 
 * @param self pointer to the link instance
 * @return Vector3 the position of the end of the link, which is used to draw the next link in the chain
 */
extern Vector3 LINK_Draw(Link* self);

/**
 * @brief 
 * 
 * @param self 
 * @param mode 
 */
extern void LINK_SetRenderMode(Link* self, LinkRenderMode mode);

/**
 * @brief 
 * 
 * @param self 
 * @param texture 
 */
extern void LINK_SetTexture(Link* self, Texture2D texture);

/** 
 * @brief 
 * 
 * @param self 
 * @param mode 
 */
extern void LINK_SetRenderMode(Link* self, LinkRenderMode mode);

/**
 * @brief 
 * 
 * @param self 
 * @param texture 
 */
extern void LINK_SetTexture(Link* self, Texture2D texture);


/**
 * @brief will destruct the link and free all allocated memory
 * 
 * @param self pointer to the link instance
 */
extern void LINK_Destroy(Link* self);


#ifdef __cplusplus
}
#endif

#endif // LINK_H