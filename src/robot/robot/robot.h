#ifndef ROBOT_ROBOT_H_
#define ROBOT_ROBOT_H_

#include "Control/Controller/Controller.h"
#include "Robot/link/link.h"
#include "raylib.h"

#define NUM_LINKS 4

typedef struct Robot_protected Robot_protected;

typedef struct Robot{
    float jp_limits[NUM_LINKS][3]; // joint limits for each link [min, max]
    float jp_velocity_limits[NUM_LINKS][3]; // velocity limits for each link [min, max]

    Robot_protected* protected; // pointer to the protected data
} Robot;

/**
 * @brief Constructs a new Robot instance with the given controllers and links.
 * 
 * @param controllers an array of pointers to the controllers for each link
 * @param links an array of pointers to the links in the robot
 * @return Robot* pointer to the constructed Robot instance
 */
extern Robot* ROBOT_ctor(Controller** controllers, Link* links[NUM_LINKS]);

/**
 * @brief Sets the limits for the robot joints and links.
 * 
 * @param self pointer to the Robot instance
 * @param jp_limits the joint angle limits for each link [min, max]
 * @param jp_velocity_limits the velocity limits for each link [min, max]
 */
extern void ROBOT_set_limits(Robot* self, float jp_limits[NUM_LINKS][2], 
                                          float jp_velocity_limits[NUM_LINKS][2]);

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param jp_setpoint the target joint angles as a Vector3
 */
extern void ROBOT_set_JP(Robot* self, Vector3 jp_setpoint);

/**
 * @brief Sets the velocity for the robot links.
 * 
 * @param self pointer to the Robot instance
 * @param jp_velocity_setpoint the velocity setpoint for the robot link
 */
extern void ROBOT_set_JP_velocity(Robot* self, Vector3 jp_velocity_setpoint);

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param tcp_setpoint the target position as a Vector3
 */
extern void ROBOT_set_TCP_target(Robot* self, Vector3 tcp_setpoint);

/**
 * @brief Sets the velocity for the robot links.
 * 
 * @param self pointer to the Robot instance
 * @param tcp_velocity_setpoint the velocity setpoint for the robot link
 */
extern void ROBOT_set_TCP_velocity(Robot* self, Vector3 tcp_velocity_setpoint);

/**
 * @brief Updates the robot state based on the current control mode and target state.
 * 
 * @param self pointer to the Robot instance
 */
extern void ROBOT_update(Robot* self);


/**
 * @brief Draws the robot using raylib.
 * 
 * @param self  pointer to the Robot instance
 */
extern void ROBOT_Draw(Robot* self);

/**
 * @brief Destructs the Robot instance and frees all allocated memory.
 * 
 * @param self pointer to the Robot instance
 */
extern void ROBOT_dtor(Robot* self);    




#endif