#ifndef ROBOT_ROBOT_H_
#define ROBOT_ROBOT_H_

#include "Control/Controller/Controller.h"
#include "Robot/link/link.h"
#include "raylib.h"

#define LINK_1_INDEX 0
#define LINK_2_INDEX 1
#define LINK_3_INDEX 2
#define LINK_4_INDEX 3

#define J1_INDEX 0
#define J2_INDEX 1
#define J3_INDEX 2

#define MIN_LIMIT_INDEX 0
#define MAX_LIMIT_INDEX 1

typedef struct Robot_protected Robot_protected;

typedef enum RobotControllerIndex {
    CTRL_J1_POS = 0,
    CTRL_J2_POS,
    CTRL_J3_POS,

    CTRL_J1_VEL,
    CTRL_J2_VEL,
    CTRL_J3_VEL,

    NUM_CTRLS
} RobotControllerIndex;

typedef enum RobotLinkIndex {
    LINK1_INDEX = 0,
    LINK2_INDEX,
    LINK3_INDEX,
    LINK4_INDEX,
    NUM_LINKS
} RobotLinkIndex;

typedef enum RobotJointIndex {
    JOINT1_INDEX = 0,
    JOINT2_INDEX,
    JOINT3_INDEX,

    NUM_JOINTS
} RobotJointIndex;

typedef struct Robot{
    float jp_limits[NUM_LINKS][2]; // joint limits for each link [min, max]
    float jp_velocity_limits[NUM_LINKS][2]; // velocity limits for each link [min, max]

    Robot_protected* protected; // pointer to the protected data
} Robot;

/**
 * @brief Constructs a new Robot instance with the given controllers and links.
 * 
 * @param controllers an array of pointers to the controllers for each link
 * @param links an array of pointers to the links in the robot
 * @return Robot* pointer to the constructed Robot instance
 */
extern Robot* ROBOT_ctor(Controller* controllers[NUM_CTRLS], Link* links[NUM_LINKS]);

/**
 * @brief Sets the limits for the robot joints and links.
 * 
 * @param self pointer to the Robot instance
 * @param jp_limits the joint angle limits for each link [min, max]
 * @param jp_velocity_limits the velocity limits for each link [min, max]
 */
extern void ROBOT_set_limits(Robot* self, const float jp_limits[NUM_LINKS][2], 
                                          const float jp_velocity_limits[NUM_LINKS][2]);

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param jp_setpoint the target joint angles as a Vector3
 */
extern void ROBOT_set_JP_target(Robot* self, Vector3 jp_setpoint);

/**
 * @brief Sets the velocity for the robot links.
 * 
 * @param self pointer to the Robot instance
 * @param jp_velocity_setpoint the velocity setpoint for the robot link
 */
extern void ROBOT_set_JP_velocity_target(Robot* self, Vector3 jp_velocity_setpoint);

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
extern void ROBOT_set_TCP_velocity_target(Robot* self, Vector3 tcp_velocity_setpoint);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
extern Vector3 ROBOT_Get_JP(const Robot* self);

/**
 * @brief 
 * 
 * @param self 
 * @return Vector3 
 */
extern Vector3 ROBOT_Get_JP_velocity(const Robot* self);

/**
 * @brief 
 * 
 * @param self 
 * @param joint_index 
 * @return float 
 */
extern float ROBOT_Get_JointPosition(const Robot* self, int joint_index);

/**
 * @brief 
 * 
 * @param self 
 * @param joint_index 
 * @return float 
 */
extern float ROBOT_Get_JointVelocity(const Robot* self, int joint_index);

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