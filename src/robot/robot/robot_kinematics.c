#include "robot_kinematics.h"
#include "robot_protected.h"

#include <string.h>

#define IK_EPSILON 0.000001f

/**
 * @brief 
 * 
 * @param robot 
 * @param link_index 
 * @param value 
 * @return int 
 */
static int joint_in_range(Robot* robot, int link_index, float value)
{
    return value >= robot->jp_limits[link_index][MIN_LIMIT_INDEX] &&
           value <= robot->jp_limits[link_index][MAX_LIMIT_INDEX];
}

/**
 * @brief 
 * 
 * @param robot 
 * @param jp 
 * @return int 
 */
static int jp_in_range(Robot* robot, Vector3 jp)
{
    return joint_in_range(robot, LINK_2_INDEX, jp.x) &&
           joint_in_range(robot, LINK_3_INDEX, jp.y) &&
           joint_in_range(robot, LINK_4_INDEX, jp.z);
}

FK_result ROBOT_forward_kinematics(Robot* robot, Vector3 target_JP)
{
    FK_result result = {
        .TCP = Vector3Zero(),
        .reachable = 0
    };

    if (!robot || !robot->protected) {
        return result;
    }

    Link* link2 = robot->protected->links[LINK_2_INDEX];
    Link* link3 = robot->protected->links[LINK_3_INDEX];
    Link* link4 = robot->protected->links[LINK_4_INDEX];

    if (!link2 || !link3 || !link4) {
        return result;
    }

    const float q1 = target_JP.x;
    const float q2 = target_JP.y;
    const float q3 = target_JP.z;

    const float l1 = link2->dim.x;
    const float l2 = link3->dim.x;

    result.TCP.x = l1 * sinf(q1) + l2 * sinf(q1 + q2);
    result.TCP.z = l1 * cosf(q1) + l2 * cosf(q1 + q2);
    result.TCP.y = link2->dim.y + link3->dim.y + link4->dim.y + q3;

    result.reachable = jp_in_range(robot, target_JP);

    return result;
}


IK_result ROBOT_inverse_kinematics(Robot* robot, Vector3 target_TCP)
{
    IK_result result = {0};

    if (!robot || !robot->protected) {
        return result;
    }

    Link* link2 = robot->protected->links[LINK_2_INDEX];
    Link* link3 = robot->protected->links[LINK_3_INDEX];
    Link* link4 = robot->protected->links[LINK_4_INDEX];

    if (!link2 || !link3 || !link4) {
        return result;
    }

    const float l1 = link2->dim.x;
    const float l2 = link3->dim.x;

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

    if (r < IK_EPSILON || r < min_reach || r > max_reach) {
        return result;
    }

    const float base_height = link2->dim.y + link3->dim.y + link4->dim.y;
    const float q3 = target_TCP.y - base_height;

    if (!joint_in_range(robot, LINK_4_INDEX, q3)) {
        return result;
    }

    const float beta = atan2f(target_2d.y, target_2d.x);

    float cos_alpha = ((l1 * l1) + (r * r) - (l2 * l2)) / (2.0f * l1 * r);
    cos_alpha = Clamp(cos_alpha, -1.0f, 1.0f);

    const float alpha = acosf(cos_alpha);

    float q1[MAX_SOLUTIONS];
    float q2[MAX_SOLUTIONS];

    q1[LEFT_SOLUTION]  = beta + alpha;
    q1[RIGHT_SOLUTION] = beta - alpha;

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

        /*
            Rename this field if your IK_result uses a different name.
            Common expected layout:
                Vector3 JP[MAX_SOLUTIONS];
                uint8_t reachable[MAX_SOLUTIONS];
        */
        result.JP[i] = jp;
    }

    return result;
}