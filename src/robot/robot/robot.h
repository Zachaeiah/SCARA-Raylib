#ifndef ROBOT_ROBOT_H_
#define ROBOT_ROBOT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Control/Controller/Controller.h"
#include "Robot/link/link.h"
#include "raylib.h"

typedef struct Robot Robot;

typedef enum RobotControllerIndex {
    ROBOT_CTRL_J1_POS = 0,
    ROBOT_CTRL_J2_POS,
    ROBOT_CTRL_J3_POS,

    ROBOT_CTRL_J1_VEL,
    ROBOT_CTRL_J2_VEL,
    ROBOT_CTRL_J3_VEL,

    ROBOT_NUM_CTRLS
} RobotControllerIndex;

typedef enum RobotLinkIndex {
    ROBOT_LINK_1 = 0,
    ROBOT_LINK_2,
    ROBOT_LINK_3,
    ROBOT_LINK_4,

    ROBOT_NUM_LINKS
} RobotLinkIndex;

typedef enum RobotJointIndex {
    ROBOT_JOINT_1 = 0,
    ROBOT_JOINT_2,
    ROBOT_JOINT_3,

    ROBOT_NUM_JOINTS
} RobotJointIndex;

typedef enum RobotLimitIndex {
    ROBOT_LIMIT_MIN = 0,
    ROBOT_LIMIT_MAX,

    ROBOT_NUM_LIMITS
} RobotLimitIndex;

/**
 * @brief 
 * 
 * @param controllers 
 * @param links 
 * @return Robot* 
 */
Robot* ROBOT_Create(Controller* controllers[ROBOT_NUM_CTRLS],
                    Link* links[ROBOT_NUM_LINKS]);


/**
 * @brief 
 * 
 * @param self 
 */
void ROBOT_Destroy(Robot* self);

/**
 * @brief 
 * 
 * @param self 
 * @param joint_position_limits 
 * @param joint_velocity_limits 
 */
void ROBOT_SetJointLimits(
    Robot* self,
    const float joint_position_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS],
    const float joint_velocity_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS]
);

/**
 * @brief 
 * 
 * @param self 
 * @param joint_position_target 
 */
void ROBOT_SetJointPositionTarget(Robot* self, Vector3 joint_position_target);

/**
 * @brief 
 * 
 * @param self 
 * @param joint_velocity_target 
 */
void ROBOT_SetJointVelocityTarget(Robot* self, Vector3 joint_velocity_target);

/**
 * @brief 
 * 
 * @param self 
 * @param tcp_position_target 
 */
void ROBOT_SetTCPPositionTarget(Robot* self, Vector3 tcp_position_target);

/**
 * @brief 
 * 
 * @param self 
 * @param tcp_velocity_target 
 */
void ROBOT_SetTCPVelocityTarget(Robot* self, Vector3 tcp_velocity_target);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
Vector3 ROBOT_GetJointPosition(const Robot* self);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
Vector3 ROBOT_GetJointVelocity(const Robot* self);

/**
 * @brief 
 * 
 * @param self 
 * @param joint 
 * @return float 
 */
float ROBOT_GetJointPositionAt(const Robot* self, RobotJointIndex joint);

/**
 * @brief 
 * 
 * @param self 
 * @param joint 
 * @return float 
 */
float ROBOT_GetJointVelocityAt(const Robot* self, RobotJointIndex joint);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
Vector3 ROBOT_GetTCPPosition(const Robot* self);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
Vector3 ROBOT_GetTCPVelocity(const Robot* self);

/**
 * @brief 
 * 
 * @param self 
 * @param dt 
 */
void ROBOT_Update(Robot* self, float dt);

/**
 * @brief 
 * 
 * @param self 
 */
void ROBOT_Draw(Robot* self);

static RobotLinkIndex LinkIndexFromJoint(RobotJointIndex joint)
{
    switch (joint) {
        case ROBOT_JOINT_1:
            return ROBOT_LINK_2;   // link2 is J1 arm

        case ROBOT_JOINT_2:
            return ROBOT_LINK_3;   // link3 is J2 arm

        case ROBOT_JOINT_3:
            return ROBOT_LINK_4;   // link4 is J3 prismatic

        default:
            RAISE(ValueError);
            return ROBOT_LINK_2;
    }
}


#ifdef __cplusplus
}
#endif

#endif