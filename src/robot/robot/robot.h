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
 * @brief Represents the state of the robot.
 */
typedef struct RobotState {
    Vector3 tcp_position; // current position of the robot's TCP
    Vector3 tcp_velocity; // current velocity of the robot's TCP

    Vector3 joint_position; // current position of each joint
    Vector3 joint_velocity; // current velocity of each joint
} RobotState;

/**
 * @brief Creates a new Robot instance.
 * 
 * @param controllers An array of pointers to Controller instances, one for each robot joint and control mode.
 * @param links An array of pointers to Link instances, one for each robot link.
 * @return Robot* A pointer to the newly created Robot instance, or NULL if creation failed due to invalid parameters.
 */
extern Robot* ROBOT_Create(Controller* controllers[ROBOT_NUM_CTRLS],
                    Link* links[ROBOT_NUM_LINKS]);


/**
 * @brief Destroys a Robot instance and frees associated resources.
 * 
 * @param self A pointer to the Robot instance to be destroyed. If NULL, the function does nothing.
 */
extern void ROBOT_Destroy(Robot* self);

/**
 * @brief Sets the position and velocity limits for each robot joint.
 * 
 * @param self A pointer to the Robot instance.
 * @param joint_position_limits An array of position limits for each robot joint.
 * @param joint_velocity_limits An array of velocity limits for each robot joint.
 */
extern void ROBOT_SetJointLimits(
    Robot* self,
    const float joint_position_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS],
    const float joint_velocity_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS]
);

/**
 * @brief Applies velocity limits to the robot's joint velocities and returns the resulting state.
 * 
 * @param self A pointer to the Robot instance.
 * @return Vector3 A Vector3 struct containing the joint velocities after applying velocity limits.
 */
extern Vector3 ROBOT_ApplyJointVelocityLimits(Robot* self);

/**
 * @brief Applies position limits to the robot's joint positions and returns the resulting state.
 * 
 * @param self A pointer to the Robot instance.
 * @return Vector3 A Vector3 struct containing the joint positions after applying position limits.
 */
extern Vector3 ROBOT_ApplyJointPositionLimits(Robot* self);

/**
 * @brief Applies joint position and velocity limits to the robot's current state and returns the resulting state.
 * 
 * @param self A pointer to the Robot instance.
 * @return RobotState A RobotState struct containing the joint positions and velocities after applying limits.
 */
extern RobotState ROBOT_ApplyJointLimits(Robot* self);

/**
 * @brief Sets the target position for a specific joint.
 * 
 * @param self A pointer to the Robot instance.
 * @param joint_position_target The target position for the joint.
 */
extern void ROBOT_SetJointPositionTarget(Robot* self, Vector3 joint_position_target);

/**
 * @brief Sets the target velocity for a specific joint.
 * 
 * @param self A pointer to the Robot instance.
 * @param joint_velocity_target The target velocity for the joint.
 */
extern void ROBOT_SetJointVelocityTarget(Robot* self, Vector3 joint_velocity_target);

/**
 * @brief Sets the target position for the robot's TCP.
 * 
 * @param self A pointer to the Robot instance.
 * @param tcp_position_target The target position for the TCP.
 */
extern void ROBOT_SetTCPPositionTarget(Robot* self, Vector3 tcp_position_target);

/**
 * @brief Sets the target velocity for the robot's TCP.
 * 
 * @param self A pointer to the Robot instance.
 * @param tcp_velocity_target The target velocity for the TCP.
 */
extern void ROBOT_SetTCPVelocityTarget(Robot* self, Vector3 tcp_velocity_target);

/**
 * @brief Gets the current position of the robot's joints.
 * 
 * @param self A pointer to the Robot instance.
 * @return Vector3 The positions of the robot's joints.
 */
extern Vector3 ROBOT_GetJointPosition(const Robot* self);

/**
 * @brief Gets the velocity of the robot's joints.
 * 
 * @param self A pointer to the Robot instance.
 * @return Vector3 The velocities of the robot's joints.
 */
extern Vector3 ROBOT_GetJointVelocity(const Robot* self);

/**
 * @brief Gets the position of a specific joint.
 * 
 * @param self A pointer to the Robot instance.
 * @param joint The index of the joint for which to get the position.
 * @return float The position of the specified joint.
 */
extern float ROBOT_GetJointPositionAt(const Robot* self, RobotJointIndex joint);

/**
 * @brief Gets the velocity of a specific joint.
 * 
 * @param self A pointer to the Robot instance.
 * @param joint The index of the joint for which to get the velocity.
 * @return float The velocity of the specified joint.
 */
extern float ROBOT_GetJointVelocityAt(const Robot* self, RobotJointIndex joint);

/**
 * @brief Gets the current position of the robot's TCP.
 * 
 * @param self A pointer to the Robot instance.
 * @return Vector3 The position of the TCP.
 */
extern Vector3 ROBOT_GetTCPPosition(const Robot* self);

/**
 * @brief Gets the current velocity of the robot's TCP.
 * 
 * @param self A pointer to the Robot instance.
 * @return Vector3 The velocity of the TCP.
 */
extern Vector3 ROBOT_GetTCPVelocity(const Robot* self);

/**
 * @brief gets the current state of the robot, including joint positions, joint velocities, TCP position, and TCP velocity.
 * 
 * @param self A pointer to the Robot instance.
 * @param out_state A pointer to a RobotState struct where the current state of the robot will be stored. Must not be NULL.
 */
extern void ROBOT_GetState(const Robot* self, RobotState* out_state);

/**
 * @brief Updates the robot's state based on the current control mode and target values. This function should be called in a loop with a consistent time step to ensure proper control behavior.
 * 
 * @param self A pointer to the Robot instance.
 * @param dt 
 */
extern void ROBOT_Update(Robot* self, float dt);

/**
 * @brief Draws the robot using raylib's drawing functions. This function should visualize the robot's current state, including joint angles and TCP position, in a way that reflects the robot's configuration.
 * 
 * @param self A pointer to the Robot instance.
 */
extern void ROBOT_Draw(Robot* self);

/**
 * @brief Handles changes in the robot's control mode, such as switching between position and velocity control. This function should reset relevant controllers and update internal state as needed when the control mode changes.
 * 
 * @param self A pointer to the Robot instance.
 */
extern void ROBOT_HandleModeChange(Robot* self);

#ifdef __cplusplus
}
#endif

#endif