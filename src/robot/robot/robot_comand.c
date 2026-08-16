#include "robot_internal.h"
#include "robot_comand.h"
#include "robot_kinematics.h"
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include "raylib.h"
#include "raymath.h"

#include <math.h>
#include <stdbool.h>

#define CLAMP_TOLERANCE 0.000005f

typedef struct JointCommandCheck {
    RobotCommandStatus status;
    Vector3 joint_position;
    char clamped_joint;
} JointCommandCheck;

static bool IsFiniteVector3(Vector3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool IsWithinLimitsTol(float value, float min_limit, float max_limit)
{
    return value >= min_limit - CLAMP_TOLERANCE &&
           value <= max_limit + CLAMP_TOLERANCE;
}

static JointCommandCheck CheckJointPositionTarget(Robot* self, Vector3 requested_jp)
{
    JointCommandCheck result = {
        .status = ROBOT_COMMAND_OK,
        .joint_position = requested_jp,
        .clamped_joint = 0x00
    };

    if (!self) {
        result.status = ROBOT_COMMAND_REJECTED;
        return result;
    }

    if (!IsFiniteVector3(requested_jp)) {
        LOG_ERROR_MSG(
            JP_CMD_REJECTED_ErrorCode,
            "Joint position target rejected. Non-finite target. JP: (%5.f, %.5f, %5.f)\n",
            requested_jp.x,
            requested_jp.y,
            requested_jp.z
        );

        result.status = ROBOT_COMMAND_REJECTED;
        return result;
    }

    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        RobotJointIndex joint = (RobotJointIndex)i;

        float pos_min = self->joint_position_limits[i][ROBOT_LIMIT_MIN];
        float pos_max = self->joint_position_limits[i][ROBOT_LIMIT_MAX];
        float value = Vector3GetByJoint(result.joint_position, joint);

        if (!IsWithinLimitsTol(value, pos_min, pos_max)) {
            LOG_WARN_MSG(
                JP_CMD_REJECTED_ErrorCode,
                "Joint position target rejected. Joint %d target: %.3f, limits: [%.3f, %.3f]\n",
                i + 1,
                value,
                pos_min,
                pos_max
            );

            result.status = ROBOT_COMMAND_REJECTED;
            return result;
        }

        if (value < pos_min) {
            Vector3SetByJoint(&result.joint_position, joint, pos_min);
            result.status = ROBOT_COMMAND_CLAMPED;
            result.clamped_joint = (0x1) << i;
        }
        else if (value > pos_max) {
            Vector3SetByJoint(&result.joint_position, joint, pos_max);
            result.status = ROBOT_COMMAND_CLAMPED;
            result.clamped_joint = (0x1) << i;
        }
    }

    return result;
}

static float JointDelta(Vector3 a, Vector3 b)
{
    return Vector3Distance(a, b);
}

RobotCommandStatus ROBOT_SetJointPositionTarget(Robot* self, Vector3 joint_position_target)
{
    FK_result fk_sol;


    if (!self) {
        RAISE(NullptrError);
        return ROBOT_COMMAND_REJECTED;
    }

    JointCommandCheck check = CheckJointPositionTarget(self, joint_position_target);

    if (check.status == ROBOT_COMMAND_REJECTED) {
        return ROBOT_COMMAND_REJECTED;
    }

    fk_sol = ROBOT_forward_kinematics(self, check.joint_position);


    if (check.status == ROBOT_COMMAND_CLAMPED) {
        LOG_WARN_MSG(
            JP_CMD_CLAMPED_ErrorCode,
            "Joint position target clamped. Requested JP: (%.3f, %.3f, %.3f), clamped JP: (%.3f, %.3f, %.3f)\n",
            joint_position_target.x,
            joint_position_target.y,
            joint_position_target.z,
            check.joint_position.x,
            check.joint_position.y,
            check.joint_position.z
        );
    }

    

    self->target.joint_position = check.joint_position;
    self->target.tcp_position = fk_sol.TCP;
    self->mode = ROBOT_MODE_JOINT_POSITION;

    return check.status;
}

RobotCommandStatus ROBOT_SetTCPPositionTarget(Robot* self, Vector3 tcp_position_target)
{
    if (!self) {
        RAISE(NullptrError);
        return ROBOT_COMMAND_REJECTED;
    }

    if (!IsFiniteVector3(tcp_position_target)) {
        LOG_ERROR_MSG(
            TCP_CMD_REJECTED_ErrorCode,
            "TCP position target rejected. Non-finite target. TCP: (%.3f, %.3f, %.3f)\n",
            tcp_position_target.x,
            tcp_position_target.y,
            tcp_position_target.z
        );

        return ROBOT_COMMAND_REJECTED;
    }

    IK_result ik_result = ROBOT_inverse_kinematics(self, tcp_position_target);

    JointCommandCheck checks[MAX_SOLUTIONS];
    float joint_delta[MAX_SOLUTIONS];

    for (int i = 0; i < MAX_SOLUTIONS; i++) {
        checks[i] = CheckJointPositionTarget(self, ik_result.JP[i]);

        if (checks[i].status == ROBOT_COMMAND_REJECTED) {
            joint_delta[i] = INFINITY;

            LOG_WARN_MSG(
                TCP_CMD_REJECTED_ErrorCode,
                "IK solution %d rejected due to joint limits or invalid value. JP: (%.3f, %.3f, %.3f)\n",
                i,
                ik_result.JP[i].x,
                ik_result.JP[i].y,
                ik_result.JP[i].z
            );

            continue;
        }

        joint_delta[i] = JointDelta(checks[i].joint_position, self->current.joint_position);
    }

    int selected_solution = -1;
    float best_delta = INFINITY;

    for (int i = 0; i < MAX_SOLUTIONS; i++) {
        if (checks[i].status == ROBOT_COMMAND_REJECTED) {
            continue;
        }

        if (joint_delta[i] < best_delta) {
            best_delta = joint_delta[i];
            selected_solution = i;
        }
    }

    if (selected_solution < 0) {
        LOG_ERROR_MSG(
            TCP_CMD_REJECTED_ErrorCode,
            "TCP position target rejected. No valid IK solution. Requested TCP: (%.3f, %.3f, %.3f)\n",
            tcp_position_target.x,
            tcp_position_target.y,
            tcp_position_target.z
        );

        return ROBOT_COMMAND_REJECTED;
    }

    Vector3 selected_jp = checks[selected_solution].joint_position;
    FK_result selected_tcp = ROBOT_forward_kinematics(self, selected_jp);

    if (checks[selected_solution].status == ROBOT_COMMAND_CLAMPED) {
        LOG_WARN_MSG(
            TCP_CMD_CLAMPED_ErrorCode,
            "TCP position target clamped. Requested TCP: (%.3f, %.3f, %.3f), achievable TCP: (%.3f, %.3f, %.3f), selected IK solution: %d\n",
            tcp_position_target.x,
            tcp_position_target.y,
            tcp_position_target.z,
            selected_tcp.TCP.x,
            selected_tcp.TCP.y,
            selected_tcp.TCP.z,
            selected_solution
        );
    }

    self->target.joint_position = selected_jp;
    self->target.tcp_position = selected_tcp.TCP;
    self->mode = ROBOT_MODE_JOINT_POSITION;

    return checks[selected_solution].status;
}

RobotCommandStatus ROBOT_SetTCPVelocityTarget(Robot* self, Vector3 tcp_velocity_target)
{
    if (!self) {
        RAISE(NullptrError);
        return ROBOT_COMMAND_REJECTED;
    }

    JB_result jb = ROBOT_jacobian_velcitys(self, tcp_velocity_target);

    if (!jb.reachable){
        LOG_ERROR_MSG(
            TCP_CMD_REJECTED_ErrorCode,
            "TCP velocity command rejected. Robot is near a Jacobian singularity. TCP vel target: (%.3f, %.3f, %.3f), JP: (%.3f, %.3f, %.3f)\n",
            tcp_velocity_target.x,
            tcp_velocity_target.y,
            tcp_velocity_target.z,
            self->current.joint_position.x,
            self->current.joint_position.y,
            self->current.joint_position.z
        );
        return ROBOT_COMMAND_REJECTED;
    }

    
    if (jb.singularity) {
        LOG_ERROR_MSG(
            TCP_CMD_REJECTED_ErrorCode,
            "TCP velocity command rejected. Robot joint velocity out of range. Joint vel: (%.3f, %.3f, %.3f)\n",
            jb.joint_velocity.x,
            jb.joint_velocity.y,
            jb.joint_velocity.x
        );
        return ROBOT_COMMAND_REJECTED;
    }

    self->target.tcp_velocity = tcp_velocity_target;
    self->target.joint_velocity = jb.joint_velocity;
    self->current.tcp_velocity = jb.tcp_velocity;

    self->mode = ROBOT_MODE_JOINT_VELOCITY;

    return ROBOT_COMMAND_OK;
}