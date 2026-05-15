
#ifndef ROBOT_PROTECTED_H_
#define ROBOT_PROTECTED_H_

#include "Control/Controller/Controller.h"
#include "link/link.h"
#include "raylib.h"
#include "robot.h"

typedef enum Robot_modes{
    IDLE_MODE, // robot is idle and not moving, stay in current position
    Angle_CONTROL_MODE, // robot is controlling the joint angles to reach a target position
    POSITION_CONTROL_MODE, // robot is controlling the end effector position to reach a target position
    VELOCITY_CONTROL_MODE // robot is controlling the velocity of the end effector to reach a target position
} Robot_mode;

typedef struct Robot_state{
    Vector3 position; // current position of the robot end effector
    Vector4 angles; // current joint angles of the robot (in radians or cm)
    Vector3 velocity; // current velocity of the robot end effector
    float orientation; // current orientation of the robot end effector (in radians)
} Robot_state;

typedef struct Robot_protected{
    Robot_mode mode; // current control mode of the robot
    Link* links[NUM_LINKS]; // array of pointers to the links in the robot
    Controller* controller[NUM_LINKS]; // pointer to the controller for the robot
    Robot_state Current_state; // current state of the robot
    Robot_state Target_state; // target state for the robot to reach
} Robot_protected;


#endif // ROBOT_PROTECTED_H_