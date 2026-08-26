#ifndef ROBOT_ROBOT_KINEMATICS_H_
#define ROBOT_ROBOT_KINEMATICS_H_

#include "raylib.h"
#include "stdbool.h"
#include "robot.h"


#define LEFT_SOLUTION 0
#define RIGHT_SOLUTION 1
#define MAX_SOLUTIONS 2

/**
 * @brief Result of a forward kinematics calculation.
 */
typedef struct FK_result {
    Vector3 TCP;       // Calculated TCP position
    bool reachable;    // True if the position is valid/reachable
} FK_result;

/**
 * @brief Result of an inverse kinematics calculation.
 */
typedef struct IK_result {
    Vector3 JP[MAX_SOLUTIONS];       // Joint positions for each IK solution
    bool reachable[MAX_SOLUTIONS];   // True if the corresponding solution is reachable
} IK_result;

/**
 * @brief Result of a Jacobian velocity calculation.
 */
typedef struct JB_result {
    Vector3 tcp_velocity;     // TCP velocity
    Vector3 joint_velocity;   // Corresponding joint velocity
    bool reachable;           // True if the velocity can be achieved
    bool singularity;         // True if the robot is at or near a singularity
} JB_result;

/**
 * @brief Computes the forward kinematics for the robot given the joint angles.
 * 
 * @param robot pointer to the Robot instance
 * @param target_JP the target joint angles as a Vector3 (in radians)
 * @return FK_result containing the position and reachability of the end effector
 */
extern FK_result ROBOT_forward_kinematics(Robot robot, Vector3 target_JP);

/**
 * @brief Computes the inverse kinematics for the robot given a target position and orientation.
 * 
 * @param robot pointer to the Robot instance
 * @param target_TCP the target position as a Vector3
 */
extern IK_result ROBOT_inverse_kinematics(Robot robot, Vector3 target_TCP);

/**
 * @brief 
 * 
 * @param robot 
 * @return JB_result 
 */
extern JB_result ROBOT_jacobian_velcitys(Robot robot, Vector3 target_TCP_vel);

/**
 * @brief Prints the result of a forward kinematics calculation.
 *
 * @param result Pointer to the FK_result to print.
 */
extern void ROBOT_PRINT_FK(const FK_result *result);

/**
 * @brief Prints the result of an inverse kinematics calculation.
 *
 * Prints all available IK solutions and their reachability.
 *
 * @param result Pointer to the IK_result to print.
 */
extern void ROBOT_PRINT_IK(const IK_result *result);



#endif // ROBOT_ROBOT_KINEMATICS_H_