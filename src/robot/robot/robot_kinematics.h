#ifndef ROBOT_ROBOT_KINEMATICS_H_
#define ROBOT_ROBOT_KINEMATICS_H_

#include "raylib.h"
#include "robot.h"


#define LEFT_SOLUTION 0
#define RIGHT_SOLUTION 1
#define MAX_SOLUTIONS 2

typedef struct FK_result{
    Vector3 TCP; // position of the end effector
    char reachable; // flag indicating if the target position is reachable
} FK_result;

typedef struct IK_result{
    Vector3 JP[MAX_SOLUTIONS]; // joint angles to reach the target position (in radians)
    char reachable[MAX_SOLUTIONS]; // flag indicating if the target position is reachable
} IK_result;

/**
 * @brief Computes the forward kinematics for the robot given the joint angles.
 * 
 * @param robot pointer to the Robot instance
 * @param target_JP the target joint angles as a Vector3 (in radians)
 * @return FK_result containing the position and reachability of the end effector
 */
extern FK_result ROBOT_forward_kinematics(Robot* robot, Vector3 target_JP);

/**
 * @brief Computes the inverse kinematics for the robot given a target position and orientation.
 * 
 * @param robot pointer to the Robot instance
 * @param target_TCP the target position as a Vector3
 */
extern IK_result ROBOT_inverse_kinematics(Robot* robot, Vector3 target_TCP);


#endif // ROBOT_ROBOT_KINEMATICS_H_