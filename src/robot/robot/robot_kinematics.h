#ifndef ROBOT_ROBOT_KINEMATICS_H_
#define ROBOT_ROBOT_KINEMATICS_H_

#include "raylib.h"
#include "robot_protected.h"
#define MAX_SOLUTIONS 2

typedef struct FK_result{
    Vector3 position; // position of the end effector
    float orientation; // orientation of the end effector (in radians)
    char reachable; // flag indicating if the target position is reachable
} FK_result;

typedef struct IK_result{
    Vector4 angles[MAX_SOLUTIONS]; // joint angles to reach the target position (in radians)
    char reachable[MAX_SOLUTIONS]; // flag indicating if the target position is reachable
} IK_result;

/**
 * @brief Computes the forward kinematics for the robot given the joint angles.
 * 
 * @param robot pointer to the Robot instance
 * @param joint_angles the joint angles as a Vector4 (in radians)
 * @return FK_result containing the position, orientation, and reachability of the end effector
 */
extern FK_result ROBOT_forward_kinematics(Robot* robot, Vector4 joint_angles);

/**
 * @brief Computes the inverse kinematics for the robot given a target position and orientation.
 * 
 * @param robot pointer to the Robot instance
 * @param target_position the target position as a Vector3
 * @param target_orientation the target orientation as a float (in radians)
 * @return IK_result containing the joint angles to reach the target position and their reachability
 */
extern IK_result ROBOT_inverse_kinematics(Robot* robot, Vector3 target_position, float target_orientation);

#endif // ROBOT_ROBOT_KINEMATICS_H_