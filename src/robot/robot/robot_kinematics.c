#include "robot_kinematics.h"
#include "robot_protected.h"

#define LINK_1_INDEX 0
#define LINK_2_INDEX 1
#define LINK_3_INDEX 2

FK_result ROBOT_forward_kinematics(Robot* robot, Vector3 target_JP){

    FK_result fk_sol = {
        .TCP = Vector3Zero(),
        .reachable = 0
    };

    Vector3 pos = Vector3Zero(); // position of the end effector
    float max_length = 0.0f; // maximum reach of the robot
    float pos_magnitude = 0.0f; // magnitude of the target position vector

    // compute the position of the end effector using the joint angles and link dimensions
    pos.x = robot->protected->links[LINK_1_INDEX]->dim.x * cosf(target_JP.x) + 
            robot->protected->links[LINK_2_INDEX]->dim.x * cosf(target_JP.x + target_JP.y);

    // compute the y and z position of the end effector using the joint angles and link dimensions
    pos.y = robot->protected->links[LINK_1_INDEX]->dim.x * sinf(target_JP.x) +
            robot->protected->links[LINK_2_INDEX]->dim.x * sinf(target_JP.x + target_JP.y);
    
    // compute the z position of the end effector using the link dimensions
    // target_JP.z is the prismatic joint extension for the end effector
    pos.z = robot->protected->links[LINK_1_INDEX]->dim.x + 
            robot->protected->links[LINK_2_INDEX]->dim.x + 
            robot->protected->links[LINK_3_INDEX]->dim.x + target_JP.z;


    // compute the maximum reach of the robot by summing the lengths of the links
    for (int i = 0; i < NUM_LINKS; i++) {
        max_length += robot->protected->links[i]->dim.x;
    }   

    pos_magnitude = Vector3Length(pos);

    // check if the target position is within the maximum reach of the robot
    if (pos_magnitude <= max_length) {
        fk_sol.reachable = 1;
    } else {
        fk_sol.reachable = 0;
    }

// set the fk_sol struct with the computed position, orientation, and reachability
    fk_sol.TCP = pos;

    return fk_sol;

}


IK_result ROBOT_inverse_kinematics(Robot* robot, Vector3 target_TCP)
{

    IK_result ik_sol = {
        .JP = {{0.0f, 0.0f, 0.0f, 0.0f}},
        .reachable = {0, 0}
    };

    return ik_sol;

}