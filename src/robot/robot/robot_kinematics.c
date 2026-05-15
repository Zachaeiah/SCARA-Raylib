#include "robot_kinematics.h"
#include "robot_protected.h"

FK_result ROBOT_forward_kinematics(Robot* robot, Vector4 joint_angles){

    FK_result fk_sol = {
        .position = Vector3Zero(),
        .orientation = 0.0f,
        .reachable = 0
    };

    Vector3 pos = Vector3Zero(); // position of the end effector
    float max_length = 0.0f; // maximum reach of the robot
    float orientation = 0.0f; // orientation of the end effector
    float pos_magnitude = 0.0f; // magnitude of the target position vector

    // compute the position of the end effector using the joint angles and link dimensions
    pos.x = robot->protected->links[0]->dim.x * cosf(joint_angles.x) + 
            robot->protected->links[1]->dim.x * cosf(joint_angles.x + joint_angles.y) + 
            robot->protected->links[2]->dim.x * cosf(joint_angles.x + joint_angles.y + joint_angles.z);

    // compute the y and z position of the end effector using the joint angles and link dimensions
    pos.y = robot->protected->links[0]->dim.x * sinf(joint_angles.x) +
            robot->protected->links[1]->dim.x * sinf(joint_angles.x + joint_angles.y) +
            robot->protected->links[2]->dim.x * sinf(joint_angles.x + joint_angles.y + joint_angles.z);
    
    // compute the z position of the end effector using the link dimensions
    // joint_angles.w is the prismatic joint extension for the end effector
    pos.z = robot->protected->links[0]->dim.z + 
            robot->protected->links[1]->dim.z + 
            robot->protected->links[2]->dim.z + joint_angles.w;

    // compute the orientation of the end effector using the joint angles
    orientation = joint_angles.x + joint_angles.y + joint_angles.z;

    // compute the maximum reach of the robot by summing the lengths of the links
    max_length = robot->protected->links[0]->dim.x + robot->protected->links[1]->dim.x + robot->protected->links[2]->dim.x;

    // set the fk_sol struct with the computed position, orientation, and reachability
    fk_sol.position = pos;
    fk_sol.orientation = orientation;

    pos_magnitude = Vector3Length(pos);

    // check if the target position is within the maximum reach of the robot
    if (pos_magnitude <= max_length) {
        fk_sol.reachable = 1;
    } else {
        fk_sol.reachable = 0;
    }


    return fk_sol;

}


IK_result ROBOT_inverse_kinematics(Robot* robot, Vector3 target_position, float target_orientation)
{

    IK_result ik_sol = {
        .angles = {{0.0f, 0.0f, 0.0f, 0.0f}},
        .reachable = {0, 0}
    };

    return ik_sol;

}