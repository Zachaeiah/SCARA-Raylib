#ifndef ROBOT_INTERNAL_H_
#define ROBOT_INTERNAL_H_

#include "robot.h"

typedef enum RobotMode {
    ROBOT_MODE_IDLE = 0,
    ROBOT_MODE_JOINT_POSITION,
    ROBOT_MODE_JOINT_VELOCITY,
} RobotMode;

struct Robot {
    Link links[ROBOT_NUM_LINKS];
    Controller* controllers[ROBOT_NUM_CTRLS];

    RobotMode mode;
    RobotMode previous_mode;

    RobotState current;
    RobotState target;

    float joint_position_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS];
    float joint_velocity_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS];

    unsigned int position_loop_counter;
};

#endif