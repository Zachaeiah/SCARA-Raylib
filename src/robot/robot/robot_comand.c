#include "robot_internal.h"
#include "robot_comand.h"
#include "robot_kinematics.h"
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include "raylib.h"
#include "raymath.h"

ErrorType JP_CMD_CLAMPED = 1; /**< Represents clamped joint position command. */
ErrorType TCP_CMD_CLAMPED = 2; /**< Represents clamped TCP command. */
ErrorType JP_CMD_REJECTED = 3; /**< Represents rejected joint position command. */
ErrorType TCP_CMD_REJECTED = 4; /**< Represents rejected TCP command. */


#define CLAMP_TOLERANCE 0.001f

static bool is_within_limits(float value, float min_limit, float max_limit)
{
    return value >= min_limit - CLAMP_TOLERANCE && value <= max_limit + CLAMP_TOLERANCE;
}


RobotCommandStatus ROBOT_SetJointPositionTarget(Robot* self, Vector3 joint_position_target)
{
    if (!self) {
        RAISE(NullptrError);
        return ROBOT_COMMAND_REJECTED;
    }

    bool command_clamped = false;

    // Check if the target position is within limits
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {

        float pos_min = self->joint_position_limits[i][ROBOT_LIMIT_MIN];
        float pos_max = self->joint_position_limits[i][ROBOT_LIMIT_MAX];

        if (!is_within_limits(Vector3GetByJoint(joint_position_target, (RobotJointIndex)i), pos_min, pos_max)) {
            LOG_ERROR_MSG(JP_CMD_REJECTED, "Joint position target command was rejected. Target may be out of bounds. Joint %d target: %f, limits: [%f, %f]\n",
                i + 1,
                Vector3GetByJoint(joint_position_target, (RobotJointIndex)i),
                pos_min,
                pos_max
            );
            return ROBOT_COMMAND_REJECTED;
        }
        else
        {
            // If the target is slightly outside the limits, clamp it and return CLAMPED status
            if (Vector3GetByJoint(joint_position_target, (RobotJointIndex)i) < pos_min) {
                Vector3SetByJoint(&joint_position_target, (RobotJointIndex)i, pos_min);
                command_clamped = true;
            }
            else if (Vector3GetByJoint(joint_position_target, (RobotJointIndex)i) > pos_max) {
                Vector3SetByJoint(&joint_position_target, (RobotJointIndex)i, pos_max);
                command_clamped = true;
            }
            
        }

        if (command_clamped) {
            LOG_WARN_MSG(JP_CMD_CLAMPED, "Joint position target command was clamped. Target is near bounds. Joint %d target: %f, limits: [%f, %f]\n",
                i + 1,
                Vector3GetByJoint(joint_position_target, (RobotJointIndex)i),
                pos_min,
                pos_max
            );
        }
    }

    self->target.joint_position = joint_position_target;
    self->mode = ROBOT_MODE_JOINT_POSITION;
    return command_clamped ? ROBOT_COMMAND_CLAMPED : ROBOT_COMMAND_OK;
}

RobotCommandStatus ROBOT_SetJointVelocityTarget(Robot* self, Vector3 joint_velocity_target)
{
    if (!self) {
        RAISE(NullptrError);
        return ROBOT_COMMAND_REJECTED;
    }

    RAISE(NotImplementedError);
    return ROBOT_COMMAND_REJECTED;
}

RobotCommandStatus ROBOT_SetTCPPositionTarget(Robot* self, Vector3 tcp_position_target)
{
    if (!self) {
        RAISE(NullptrError);
        return ROBOT_COMMAND_REJECTED;
    }

    RobotCommandStatus command_status[MAX_SOLUTIONS] = {ROBOT_COMMAND_OK, ROBOT_COMMAND_OK};
    float joint_change_delta[MAX_SOLUTIONS] = {0.0f, 0.0f};
    IK_result ik_result_CL;


    IK_result ik_result = ROBOT_inverse_kinematics(self, tcp_position_target);
    ik_result_CL = ik_result;
    

    // If both solutions are rejected, return REJECTED
    if (command_status[LEFT_SOLUTION] == ROBOT_COMMAND_REJECTED && command_status[RIGHT_SOLUTION] == ROBOT_COMMAND_REJECTED) {
        LOG_ERROR_MSG(TCP_CMD_REJECTED, "TCP position target command was rejected. Target is unreachable. Target: (%.3f, %.3f, %.3f) JP: (%.3f, %.3f, %.3f)\n",
            tcp_position_target.x,
            tcp_position_target.y,
            tcp_position_target.z,
            ik_result.JP[LEFT_SOLUTION].x,
            ik_result.JP[LEFT_SOLUTION].y,
            ik_result.JP[LEFT_SOLUTION].z
        );
        return ROBOT_COMMAND_REJECTED;
    }
    else if (command_status[LEFT_SOLUTION] == ROBOT_COMMAND_REJECTED) {
        LOG_WARN_MSG(TCP_CMD_REJECTED, "Left IK solution is rejected due to joint limits. Target TCP: (%.3f, %.3f, %.3f) JP: (%.3f, %.3f, %.3f)\n",
            tcp_position_target.x,
            tcp_position_target.y,
            tcp_position_target.z,
            ik_result.JP[LEFT_SOLUTION].x,
            ik_result.JP[LEFT_SOLUTION].y,
            ik_result.JP[LEFT_SOLUTION].z
        );
    }
    else if (command_status[RIGHT_SOLUTION] == ROBOT_COMMAND_REJECTED) {
        LOG_WARN_MSG(TCP_CMD_REJECTED, "Right IK solution is rejected due to joint limits. Target TCP: (%.3f, %.3f, %.3f) JP: (%.3f, %.3f, %.3f)\n",
            tcp_position_target.x,
            tcp_position_target.y,
            tcp_position_target.z,
            ik_result.JP[RIGHT_SOLUTION].x,
            ik_result.JP[RIGHT_SOLUTION].y,
            ik_result.JP[RIGHT_SOLUTION].z
        );
    }

    // Check if the IK solutions are within joint limits
    for (int i = 0; i < MAX_SOLUTIONS; i++) {
        if (command_status[i] == ROBOT_COMMAND_REJECTED) {
            // log that solution i is rejected due to joint limits
            LOG_WARN_MSG(TCP_CMD_REJECTED, "IK solution %d is rejected due to joint limits. J1:%.3f, J2:%.3f, J3:%.3f\n",   
                i,
                ik_result.JP[i].x,
                ik_result.JP[i].y,
                ik_result.JP[i].z
            );

            continue; // Skip rejected solutions
        }

        Vector3 jp_solution = ik_result.JP[i];

        command_status[i] = ROBOT_SetJointPositionTarget(self, jp_solution);

        if (command_status[i] == ROBOT_COMMAND_REJECTED) {
            continue; // Skip solutions that are rejected due to joint limits
        }

    }

    for (int i = 0; i < MAX_SOLUTIONS; i++) {
        if (command_status[i] == ROBOT_COMMAND_REJECTED) {
            joint_change_delta[i] = INFINITY; // Assign a large value to rejected solutions for comparison
            continue; // Skip rejected solutions
        }

        Vector3 jp_solution = ik_result.JP[i];
        Vector3 current_jp = self->current.joint_position;

        // Calculate the total joint change for this solution
        joint_change_delta[i] = Vector3Distance(jp_solution, current_jp);

    }

    // Select the solution with the smallest joint change
    int selected_solution = LEFT_SOLUTION;


    if (command_status[RIGHT_SOLUTION] != ROBOT_COMMAND_REJECTED) {
        if (joint_change_delta[RIGHT_SOLUTION] < joint_change_delta[LEFT_SOLUTION]) {
            selected_solution = RIGHT_SOLUTION;
        }
    }

    if (command_status[LEFT_SOLUTION] != ROBOT_COMMAND_REJECTED) {
        if (joint_change_delta[LEFT_SOLUTION] < joint_change_delta[RIGHT_SOLUTION]) {
            selected_solution = LEFT_SOLUTION;
        }
    }

    if (command_status[selected_solution] == ROBOT_COMMAND_REJECTED) {
        LOG_ERROR_MSG(TCP_CMD_REJECTED, "TCP position target command was rejected. Target is unreachable due to joint limits. Target: (%f, %f, %f)\n",
            tcp_position_target.x,
            tcp_position_target.y,
            tcp_position_target.z
        );
        return ROBOT_COMMAND_REJECTED; // This should not happen, but just in case
    }

    if (command_status[selected_solution] == ROBOT_COMMAND_CLAMPED) {
        ik_result_CL.JP[selected_solution] = ROBOT_forward_kinematics(self, ik_result.JP[selected_solution]).TCP;
        LOG_WARN_MSG(TCP_CMD_CLAMPED, "TCP position target command was clamped. Target is near bounds. Target: (%f, %f, %f), Clamped TCP: (%f, %f, %f)\n",
            tcp_position_target.x,
            tcp_position_target.y,
            tcp_position_target.z,
            ik_result_CL.JP[selected_solution].x,
            ik_result_CL.JP[selected_solution].y,
            ik_result_CL.JP[selected_solution].z
        );
    }

    // Set the target joint position to the selected IK solution
    self->target.tcp_position = ik_result_CL.JP[selected_solution];
    self->target.joint_position = ik_result.JP[selected_solution];
    self->mode = ROBOT_MODE_JOINT_POSITION;
    return command_status[selected_solution] == ROBOT_COMMAND_CLAMPED ? ROBOT_COMMAND_CLAMPED : ROBOT_COMMAND_OK;
}

RobotCommandStatus ROBOT_SetTCPVelocityTarget(Robot* self, Vector3 tcp_velocity_target)
{
    if (!self) {
        RAISE(NullptrError);
        return ROBOT_COMMAND_REJECTED;
    }

    RAISE(NotImplementedError);


    // self->target.tcp_velocity = tcp_velocity_target;
    // self->mode = ROBOT_MODE_TCP_VELOCITY;
    return ROBOT_COMMAND_REJECTED; // Placeholder until velocity control is implemented
}