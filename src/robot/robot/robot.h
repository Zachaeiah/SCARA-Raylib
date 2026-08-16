#ifndef ROBOT_ROBOT_H_
#define ROBOT_ROBOT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Control/Controller/Controller.h"
#include "Robot/link/link.h"
#include "raylib.h"

extern const ErrorType JP_CMD_CLAMPED_ErrorCode; 
extern const ErrorType JP_CMD_REJECTED_ErrorCode; 
extern const ErrorType JP_RANGE_ErrorCode;

extern const ErrorType TCP_CMD_CLAMPED_ErrorCode;
extern const ErrorType TCP_CMD_REJECTED_ErrorCode;
    
// Forward declarations
typedef struct Robot Robot;

typedef enum RobotControllerIndex {
    ROBOT_CTRL_J1_POS = 0,
    ROBOT_CTRL_J2_POS,
    ROBOT_CTRL_J3_POS,

    ROBOT_CTRL_J1_VEL,
    ROBOT_CTRL_J2_VEL,
    ROBOT_CTRL_J3_VEL,

    ROBOT_NUM_CTRLS // this should always be equal to the number of controllers in the robot, and should be updated if the robot's design changes
} RobotControllerIndex;

typedef enum RobotLinkIndex {
    ROBOT_LINK_1 = 0,
    ROBOT_LINK_2,
    ROBOT_LINK_3,
    ROBOT_LINK_4,

    ROBOT_NUM_LINKS // this should always be equal to the number of links in the robot, and should be updated if the robot's design changes
} RobotLinkIndex;

typedef enum RobotJointIndex {
    ROBOT_JOINT_1 = 0,
    ROBOT_JOINT_2,
    ROBOT_JOINT_3,

    ROBOT_NUM_JOINTS // this should always be equal to the number of joints in the robot, and should be updated if the robot's design changes
} RobotJointIndex;

typedef enum RobotLimitIndex {
    ROBOT_LIMIT_MIN = 0,
    ROBOT_LIMIT_MAX,

    ROBOT_NUM_LIMITS // this should always be equal to 2, representing the minimum and maximum limits for joint positions and velocities
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
 * @brief Applies velocity limits to the robot's joint velocities and returns the resulting state.
 * 
 * @param self A pointer to the Robot instance.
 * @return Vector3 A Vector3 struct containing the joint velocities after applying velocity limits.
 */
extern Vector3 ROBOT_ApplyJointVelocityLimits(Robot* self);

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


/** 
 * @brief Gets the value of a Vector3 component based on the joint index.
 * @param v The Vector3 struct.
 * @param joint The joint index.
 * @return The value of the specified component.
 */
extern float Vector3GetByJoint(Vector3 v, RobotJointIndex joint);

/** 
 * @brief Sets the value of a Vector3 component based on the joint index.
 * @param v A pointer to the Vector3 struct to modify.
 * @param joint The joint index.
 * @param value The value to set for the specified component.
 */
extern void Vector3SetByJoint(Vector3* v, RobotJointIndex joint, float value);

#ifdef __cplusplus
}
#endif

#endif