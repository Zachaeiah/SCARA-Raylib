#ifndef ROBOT_INTERNAL_H_
#define ROBOT_INTERNAL_H_

#include "robot.h"

typedef enum RobotMode {
    ROBOT_MODE_IDLE = 0,
    ROBOT_MODE_JOINT_POSITION,
    ROBOT_MODE_JOINT_VELOCITY,
    ROBOT_MODE_TCP_POSITION,
    ROBOT_MODE_TCP_VELOCITY
} RobotMode;

typedef struct RobotState {
    Vector3 tcp_position;
    Vector3 tcp_velocity;

    Vector3 joint_position;
    Vector3 joint_velocity;
} RobotState;

struct Robot {
    Link* links[ROBOT_NUM_LINKS];
    Controller* controllers[ROBOT_NUM_CTRLS];

    RobotMode mode;

    RobotState current;
    RobotState target;

    float joint_position_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS];
    float joint_velocity_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS];

    unsigned int position_loop_counter;
};

#endif