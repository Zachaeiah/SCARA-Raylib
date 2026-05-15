#ifndef ROBOT_ROBOT_H_
#define ROBOT_ROBOT_H_

#include "Control/Controller/Controller.h"
#include "link/link.h"
#include "raylib.h"

#define NUM_LINKS 3
#define MIN_LIMIT_INDEX 0
#define MAX_LIMIT_INDEX 1

typedef struct Robot_protected Robot_protected;

typedef struct Robot{
    float joint_angle_limits[NUM_LINKS][2]; // joint limits for each link [min, max]
    float link_velocity_limits[NUM_LINKS][2]; // velocity limits for each link [min, max]
    Robot_protected* protected; // pointer to the protected data
} Robot;

/**
 * @brief Constructs a new Robot instance with the given controllers and links.
 * 
 * @param controllers an array of pointers to the controllers for each link
 * @param links an array of pointers to the links in the robot
 * @return Robot* pointer to the constructed Robot instance
 */
extern Robot* ROBOT_ctor(Controller* controllers[NUM_LINKS], Link* links[NUM_LINKS]);

/**
 * @brief Sets the limits for the robot joints and links.
 * 
 * @param self pointer to the Robot instance
 * @param joint_angle_limits the joint angle limits for each link [min, max]
 * @param link_velocity_limits the velocity limits for each link [min, max]
 */
extern void ROBOT_set_limits(Robot* self, float joint_angle_limits[NUM_LINKS][2], 
                                          float link_velocity_limits[NUM_LINKS][2]);

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param angles_setpoint the target joint angles as a Vector4
 */
extern void ROBOT_set_angles(Robot* self, Vector4 angles_setpoint);

/**
 * @brief Sets the velocity for the robot links.
 * 
 * @param self pointer to the Robot instance
 * @param velocity_setpoint the velocity setpoint for the robot link
 */
extern void ROBOT_set_velocity(Robot* self, Vector4 velocity_setpoint);

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param position_setpoint the target position as a Vector3
 * @param orientation_setpoint the target orientation as a float (in radians)
 */
extern void ROBOT_set_target(Robot* self, Vector3 position_setpoint, float orientation_setpoint);

/**
 * @brief Destructs the Robot instance and frees all allocated memory.
 * 
 * @param self pointer to the Robot instance
 */
extern void ROBOT_dtor(Robot* self);    


#endif