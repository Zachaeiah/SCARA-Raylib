#include "robot.h"

typedef enum RobotCommandStatus {
    ROBOT_COMMAND_OK = 0,
    ROBOT_COMMAND_CLAMPED,
    ROBOT_COMMAND_REJECTED
} RobotCommandStatus;


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
 * @brief Sets the target position for a specific joint.
 * 
 * @param self A pointer to the Robot instance.
 * @param joint_position_target The target position for the joint.
 * 
 * @return RobotCommandStatus indicating whether the command was accepted, clamped, or rejected based on the robot's limits and current state.
 */
extern RobotCommandStatus ROBOT_SetJointPositionTarget(Robot* self, Vector3 joint_position_target);

/**
 * @brief Sets the target velocity for a specific joint.
 * 
 * @param self A pointer to the Robot instance.
 * @param joint_velocity_target The target velocity for the joint.
 * 
 * @return RobotCommandStatus indicating whether the command was accepted, clamped, or rejected based on the robot's limits and current state.
 */
extern RobotCommandStatus ROBOT_SetJointVelocityTarget(Robot* self, Vector3 joint_velocity_target);

/**
 * @brief Sets the target position for the robot's TCP.
 * 
 * @param self A pointer to the Robot instance.
 * @param tcp_position_target The target position for the TCP.
 * 
 * @return RobotCommandStatus indicating whether the command was accepted, clamped, or rejected based on the robot's limits and current state.
 */
extern RobotCommandStatus ROBOT_SetTCPPositionTarget(Robot* self, Vector3 tcp_position_target);

/**
 * @brief Sets the target velocity for the robot's TCP.
 * 
 * @param self A pointer to the Robot instance.
 * @param tcp_velocity_target The target velocity for the TCP.
 * 
 * @return RobotCommandStatus indicating whether the command was accepted, clamped, or rejected based on the robot's limits and current state.
 */
extern RobotCommandStatus ROBOT_SetTCPVelocityTarget(Robot* self, Vector3 tcp_velocity_target);


