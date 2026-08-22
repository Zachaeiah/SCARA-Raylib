#include "robot_kinematics.h"
#include "robot_internal.h"
#include "robot.h"
#include "raymath.h"
#include "utils/Logger/logger.h"
#include <string.h>

ErrorType JP_RANGE = 5; /**< Represents joint position out of range. */

#define IK_EPSILON 0.000001f
#define JACOBIAN_SINGULARITY_EPSILON IK_EPSILON

/**
 * @brief 
 * 
 * @param robot 
 * @param link_index 
 * @param value 
 * @return int 
 */
static bool joint_pos_in_range(Robot* robot, int link_index, float value)
{
    bool in_range = value >= robot->joint_position_limits[link_index][ROBOT_LIMIT_MIN] &&
                    value <= robot->joint_position_limits[link_index][ROBOT_LIMIT_MAX];

    return in_range;
}

/**
 * @brief 
 * 
 * @param robot 
 * @param jp 
 * @return int 
 */
static bool jp_in_range(Robot* robot, Vector3 jp)
{
    return joint_pos_in_range(robot, ROBOT_JOINT_1, jp.x) &&
           joint_pos_in_range(robot, ROBOT_JOINT_2, jp.y) &&
           joint_pos_in_range(robot, ROBOT_JOINT_3, jp.z);
}

/**
 * @brief 
 * 
 * @param robot 
 * @param link_index 
 * @param value 
 * @return int 
 */
static bool joint_vel_in_range(Robot* robot, int link_index, float value)
{
    bool in_range = value >= robot->joint_velocity_limits[link_index][ROBOT_LIMIT_MIN] &&
                    value <= robot->joint_velocity_limits[link_index][ROBOT_LIMIT_MAX];

    return in_range;
}

/**
 * @brief 
 * 
 * @param robot 
 * @param jp 
 * @return int 
 */
static bool jv_in_range(Robot* robot, Vector3 jv)
{
    return joint_vel_in_range(robot, ROBOT_JOINT_1, jv.x) &&
           joint_vel_in_range(robot, ROBOT_JOINT_2, jv.y) &&
           joint_vel_in_range(robot, ROBOT_JOINT_3, jv.z);
}

/**
 * @brief 
 * 
 * @param v 
 * @return true 
 * @return false 
 */
static bool IsFiniteVector3(Vector3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}




FK_result ROBOT_forward_kinematics(Robot* robot, Vector3 target_JP)
{
    FK_result result = {
        .TCP = Vector3Zero(),
        .reachable = 0
    };

    if (!robot) {
        return result;
    }


    Link* link2 = robot->links[ROBOT_LINK_2];
    Link* link3 = robot->links[ROBOT_LINK_3];
    Link* link4 = robot->links[ROBOT_LINK_4];

    if (!link2 || !link3 || !link4) {
        return result;
    }

    Vector3 Link2_dim = LINK_GetDimensions(link2);
    Vector3 Link3_dim = LINK_GetDimensions(link3);
    Vector3 Link4_dim = LINK_GetDimensions(link4);

    

    const float q1 = target_JP.x;
    const float q2 = target_JP.y;
    const float q3 = target_JP.z;

    const float l1 = Link2_dim.x;
    const float l2 = Link3_dim.x;

    result.TCP.x = l1 * sinf(q1) + l2 * sinf(q1 + q2);
    result.TCP.z = l1 * cosf(q1) + l2 * cosf(q1 + q2);
    result.TCP.y = Link2_dim.y + Link3_dim.y + Link4_dim.y + q3;

    result.reachable = jp_in_range(robot, target_JP);

    return result;
}


IK_result ROBOT_inverse_kinematics(Robot* robot, Vector3 target_TCP)
{
    LOG_DEBUG_MSG(NO_ERROR, "Calculating IK for target TCP: (%.3f, %.3f, %.3f)\n",
        target_TCP.x,
        target_TCP.y,
        target_TCP.z
    );


    IK_result result = {
        .JP = { Vector3Zero(), Vector3Zero() },
        .reachable = { 0, 0 }
    };

    if (!robot ) {
        RAISE(NullptrError);
        return result;
    }

    Link* link2 = robot->links[ROBOT_LINK_2];
    Link* link3 = robot->links[ROBOT_LINK_3];
    Link* link4 = robot->links[ROBOT_LINK_4];

    if (!link2 || !link3 || !link4) {
        RAISE(NullptrError);
        return result;
    }

    Vector3 Link2_dim = LINK_GetDimensions(link2);
    Vector3 Link3_dim = LINK_GetDimensions(link3);

    const float l1 = Link2_dim.x;
    const float l2 = Link3_dim.x;

    /*
        FK uses:
            TCP.x = l1*sin(q1) + l2*sin(q1 + q2)
            TCP.z = l1*cos(q1) + l2*cos(q1 + q2)

        So the IK 2D plane is:
            math_x = TCP.z
            math_y = TCP.x
    */
    Vector2 target_2d = {
        .x = target_TCP.z,
        .y = target_TCP.x
    };

    const float r = Vector2Length(target_2d);
    const float min_reach = fabsf(l1 - l2);
    const float max_reach = l1 + l2;

    if (r < min_reach) {
        LOG_ERROR_MSG(TCP_CMD_REJECTED_ErrorCode, "Target TCP is within minimum reach. Target TCP: (%.3f, %.3f, %.3f), Reach: %.3f, Min Reach: %.3f\n",
            target_TCP.x,
            target_TCP.y,
            target_TCP.z,
            r,
            min_reach
        );

        return result;
    }

    if ( r > max_reach) {
        LOG_ERROR_MSG(TCP_CMD_REJECTED_ErrorCode, "Target TCP is beyond maximum reach. Target TCP: (%.3f, %.3f, %.3f), Reach: %.3f. Max Reach: %.3f\n",
            target_TCP.x,
            target_TCP.y,
            target_TCP.z,
            r,
            max_reach
        );

        return result;
    }

    const float q3 = target_TCP.y;

    if (!joint_pos_in_range(robot, ROBOT_JOINT_3, q3)) {
        LOG_ERROR_MSG(TCP_CMD_REJECTED_ErrorCode, "Target TCP is unreachable. Target TCP: (%.3f, %.3f, %.3f), Joint 3 angle: %.3f\n",
            target_TCP.x,
            target_TCP.y,
            target_TCP.z,
            q3
        );
        return result;
    }

    const float beta = atan2f(target_2d.y, target_2d.x);

    float cos_alpha = ((l1 * l1) + (r * r) - (l2 * l2)) / (2.0f * l1 * r);
    cos_alpha = Clamp(cos_alpha, -1.0f, 1.0f);

    const float alpha = acosf(cos_alpha);

    if (fabsf(alpha) < IK_EPSILON) {
        LOG_DEBUG_MSG(NO_ERROR, "Alpha is near zero, target is near singularity. Target TCP: (%.3f, %.3f, %.3f), Alpha: %.6f\n",
            target_TCP.x,
            target_TCP.y,
            target_TCP.z,
            alpha
        );
    }

    float q1[MAX_SOLUTIONS];
    float q2[MAX_SOLUTIONS];

    q1[LEFT_SOLUTION]  = beta + alpha;
    q1[RIGHT_SOLUTION] = beta - alpha;

    if(!joint_pos_in_range(robot, ROBOT_JOINT_2, q1[LEFT_SOLUTION])) {
        LOG_ERROR_MSG(TCP_CMD_REJECTED_ErrorCode, "Left IK solution is out of joint limits. Target TCP: (%.3f, %.3f, %.3f), Joint 1 angle: %.3f\n",
            target_TCP.x,
            target_TCP.y,
            target_TCP.z,
            q1[LEFT_SOLUTION]
        );
    }

    if(!joint_pos_in_range(robot, ROBOT_JOINT_2, q1[RIGHT_SOLUTION])) {
        LOG_ERROR_MSG(TCP_CMD_REJECTED_ErrorCode, "Right IK solution is out of joint limits. Target TCP: (%.3f, %.3f, %.3f), Joint 1 angle: %.3f\n",
            target_TCP.x,
            target_TCP.y,
            target_TCP.z,
            q1[RIGHT_SOLUTION]
        );
    }

    for (int i = 0; i < MAX_SOLUTIONS; i++) {
        const float dx = target_2d.x - l1 * cosf(q1[i]);
        const float dy = target_2d.y - l1 * sinf(q1[i]);

        q2[i] = atan2f(dy, dx) - q1[i];

        Vector3 jp = {
            .x = q1[i],
            .y = q2[i],
            .z = q3
        };

        result.reachable[i] = jp_in_range(robot, jp);

        if (!result.reachable[i]) {
            LOG_WARN_MSG(TCP_CMD_REJECTED_ErrorCode, "IK solution %d is out of joint limits. Joint angles: J1: %.3f, J2: %.3f, J3: %.3f\n",
                i,
                jp.x,
                jp.y,
                jp.z
            );
        }
        result.JP[i] = jp;
    }

    if (!result.reachable[LEFT_SOLUTION] && !result.reachable[RIGHT_SOLUTION]) {
        LOG_ERROR_MSG(TCP_CMD_REJECTED_ErrorCode, "IK solutions are unreachable due to joint limits. Target TCP: (%.3f, %.3f, %.3f)\n",
            target_TCP.x,
            target_TCP.y,
            target_TCP.z
        );
    }

    return result;
}

JB_result ROBOT_jacobian_velcitys(Robot* robot, Vector3 target_TCP_vel){

    JB_result jb_sol  = {
        .tcp_velocity = Vector3Zero(), 
        .joint_velocity = Vector3Zero(),
        .reachable = false,
        .singularity = false
    };

    if (!robot ) {
        RAISE(NullptrError);
        return jb_sol ;
    }

    if (!IsFiniteVector3(target_TCP_vel)) {
        return jb_sol ;
    }

    Vector3 jp = robot->current.joint_position;
    Vector3 current_jv = robot->current.joint_velocity;

    if (!IsFiniteVector3(jp) || !IsFiniteVector3(current_jv)) {
        return jb_sol ;
    }

    float theta_1 = jp.x;
    float theta_2 = jp.y;
    float theta_12 = theta_1 + theta_2;
    float sin_theta_2 = sinf(theta_2);

     if (fabsf(sin_theta_2) < JACOBIAN_SINGULARITY_EPSILON) {
        jb_sol.singularity = true;
        return jb_sol;
    }

    float vx = target_TCP_vel.x;
    float vy = target_TCP_vel.y;
    float vz = target_TCP_vel.z;

    jb_sol.joint_velocity.x = 0.05f * ((vx * cosf(theta_12)) + (vy * sinf(theta_12))) / sin_theta_2;

    jb_sol.joint_velocity.y =
        -(1.0f / 1340.0f) *
        (
            (80.0f * vx * cosf(theta_1)) +
            (67.0f * vx * cosf(theta_12)) +
            (80.0f * vy * sinf(theta_1)) +
            (67.0f * vy * sinf(theta_12))
        ) / sin_theta_2;

    jb_sol.joint_velocity.z = vz;

    if (!IsFiniteVector3(jb_sol.joint_velocity)) {
        return jb_sol;
    }

    if(jv_in_range(robot, jb_sol.joint_velocity)){
        jb_sol.reachable = true;
    }
    else{
        return jb_sol;
    }

    jb_sol.tcp_velocity.x =
        -0.25f *
        (
            (80.0f * current_jv.x * sinf(theta_1)) +
            (67.0f * current_jv.x * sinf(theta_12)) +
            (67.0f * current_jv.y * sinf(theta_12))
        );

    jb_sol.tcp_velocity.y =
        0.25f *
        (
            (80.0f * current_jv.x * cosf(theta_1)) +
            (67.0f * current_jv.x * cosf(theta_12)) +
            (67.0f * current_jv.y * cosf(theta_12))
        );

    jb_sol.tcp_velocity.z = current_jv.z;

    if (!IsFiniteVector3(jb_sol.tcp_velocity)) {
        return jb_sol;
    }


    return jb_sol ;
}
