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

#define L Link
typedef struct L *L;

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
L LINK_Create(Vector3 dimensions, Color color, LinkType type, Actuator actuator);

/**
 * @brief will update the link with a new angle input and return the speed output from the actuator
 * 
 * @param self pointer to the link instance
 * @param x_in the input angle to update the link with
 * @return float the output speed from the actuator after applying the input angle
 */
extern float LINK_UpdateActuator(L self, const float x_in);

/**
 * @brief 
 * 
 * @param self pointer to the link instance
 * @param joint_velocity 
 * @param dt 
 */
extern void LINK_IntegrateJointPosition(L self, float joint_velocity, float dt);

/**
 * @brief 
 * 
 * @param self 
 * @param joint_position 
 */
extern void  LINK_SetJointPosition(L self, float joint_position);

/**
 * @brief 
 * 
 * @param self 
 * @return float 
 */
extern float LINK_GetJointPosition(const L self);

/**
 * @brief 
 * 
 * @param self 
 * @param start_world 
 */
extern void LINK_SetStart(L self, Vector3 start_world);

/**
 * @brief 
 * 
 * @param self 
 * @param end_world 
 */
extern void LINK_SetEnd(L self, Vector3 end_world);

/**
 * @brief 
 * 
 * @param self 
 * @param heading_world_rad 
 */
extern void LINK_SetHeadingWorld(L self, float heading_world_rad);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
extern Vector3 LINK_GetStart(const L self);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
extern Vector3 LINK_GetEnd(const L self);

/**
 * @brief 
 * 
 * @param self 
 * @return float 
 */
extern float LINK_GetHeadingWorld(const L self);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
extern Vector3 LINK_GetDimensions(const L self);

/**
 * @brief will draw the link using raylib
 * 
 * @param self pointer to the link instance
 * @return Vector3 the position of the end of the link, which is used to draw the next link in the chain
 */
extern Vector3 LINK_Draw(L self);

/**
 * @brief 
 * 
 * @param self 
 * @param mode 
 */
extern void LINK_SetRenderMode(L self, LinkRenderMode mode);

/**
 * @brief 
 * 
 * @param self 
 * @param texture 
 */
extern void LINK_SetTexture(L self, Texture2D texture);

/** 
 * @brief 
 * 
 * @param self 
 * @param mode 
 */
extern void LINK_SetRenderMode(L self, LinkRenderMode mode);

/**
 * @brief 
 * 
 * @param self 
 * @param texture 
 */
extern void LINK_SetTexture(L self, Texture2D texture);


/**
 * @brief will destruct the link and free all allocated memory
 * 
 * @param self pointer to the link instance
 */
extern void LINK_Destroy(L self);

#undef L

#ifdef __cplusplus
}
#endif

#endif // LINK_H