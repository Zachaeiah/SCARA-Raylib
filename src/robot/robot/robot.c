
#include <math.h>
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/MemAllocator/mem.h"
#include "Robot_kinematics.h"
#include "robot_protected.h"
#include "robot.h"


/**
 * @brief Constructs a new Robot instance with the given controllers and links.
 * 
 * @param controllers an array of pointers to the controllers for each link
 * @param links an array of pointers to the links in the robot
 * @return Robot* pointer to the constructed Robot instance
 */
Robot* ROBOT_ctor(Controller* controllers[NUM_LINKS], Link* links[NUM_LINKS]){

    Robot* robot = NULL;
    Robot_protected* protected = NULL;
    Robot_state state;

    if (controllers == NULL || links == NULL) {
        RAISE(NullptrError);
        return NULL;
    }

    for(int i = 0; i < NUM_LINKS; i++) {
        if (controllers[i] == NULL || links[i] == NULL) {
            RAISE(NullptrError);
            return NULL;
        }
    }

    NEW0(robot);
    NEW0(protected);

    if (!robot || !protected) {
        RAISE(NullptrError);
        return NULL;
    }

    // set default limits for the robot joints and links
    for (int i = 0; i < NUM_LINKS; i++) {
        robot->joint_angle_limits[i][MIN_LIMIT_INDEX] = -PI/2; // min angle
        robot->joint_angle_limits[i][MAX_LIMIT_INDEX] = PI/2;  // max angle
        robot->link_velocity_limits[i][MIN_LIMIT_INDEX] = -1.0f; // min velocity
        robot->link_velocity_limits[i][MAX_LIMIT_INDEX] = 1.0f;  // max velocity
    }

    // initialize the robot state
    state.angles = Vector4Zero();
    state.position = Vector3Zero();
    state.velocity = Vector3Zero();
    state.orientation = 0.0f;

    // assign the initial state to the protected data
    protected->Current_state = state;
    protected->Target_state = state;

    protected->mode = IDLE_MODE; // set initial mode to IDLE

    // assign the controllers and links to the robot
    for (int i = 0; i < NUM_LINKS; i++) {
        protected->links[i] = links[i];
        protected->controller[i] = controllers[i];
    }

    robot->protected = protected;

    return robot;

}

/**
 * @brief Sets the limits for the robot joints and links.
 * 
 * @param self pointer to the Robot instance
 * @param joint_angle_limits the joint angle limits for each link [min, max]
 * @param link_velocity_limits the velocity limits for each link [min, max]
 */
void ROBOT_set_limits(Robot* self, float joint_angle_limits[NUM_LINKS][2], 
                                   float link_velocity_limits[NUM_LINKS][2])
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    if (!joint_angle_limits || !link_velocity_limits) {
        RAISE(NullptrError);
        return;
    }

    for (int i = 0; i < NUM_LINKS; i++) {
        if (joint_angle_limits[i][MIN_LIMIT_INDEX] >= joint_angle_limits[i][MAX_LIMIT_INDEX]) {
            RAISE(ValueError);
            return;
        }
        if (link_velocity_limits[i][MIN_LIMIT_INDEX] >= link_velocity_limits[i][MAX_LIMIT_INDEX]) {
            RAISE(ValueError);
            return;
        }
    }

    for (int i = 0; i < NUM_LINKS; i++) {
        self->joint_angle_limits[i][MIN_LIMIT_INDEX] = joint_angle_limits[i][MIN_LIMIT_INDEX];
        self->joint_angle_limits[i][MAX_LIMIT_INDEX] = joint_angle_limits[i][MAX_LIMIT_INDEX];
        self->link_velocity_limits[i][MIN_LIMIT_INDEX] = link_velocity_limits[i][MIN_LIMIT_INDEX];
        self->link_velocity_limits[i][MAX_LIMIT_INDEX] = link_velocity_limits[i][MAX_LIMIT_INDEX];
    }
}

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param angles_setpoint the target joint angles as a Vector4
 */
void ROBOT_set_angles(Robot* self, Vector4 angles_setpoint){

    if (!self) {
        RAISE(NullptrError);
        return;
    }

    // check if the target angles are within the joint limits
    FK_result fk_sol = ROBOT_forward_kinematics(self, angles_setpoint);

    // if the target angles are not reachable, raise a ValueError
    if (!fk_sol.reachable) {
        RAISE(ValueError);
        return;
    }

    // set control mode to angle control
    self->protected->mode = Angle_CONTROL_MODE; 

    // set the target angles in the protected data
    self->protected->Target_state.angles = angles_setpoint;

}

/**
 * @brief Sets the velocity for the robot links.
 * 
 * @param self pointer to the Robot instance
 * @param velocity_setpoint the velocity setpoint for the robot link
 */
void ROBOT_set_velocity(Robot* self, Vector4 velocity_setpoint){

}

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param position_setpoint the target position as a Vector3
 * @param orientation_setpoint the target orientation as a float (in radians)
 */
void ROBOT_set_target(Robot* self, Vector3 position_setpoint, float orientation_setpoint){

}



/**
 * @brief Destructs the Robot instance and frees all allocated memory.
 * 
 * @param self pointer to the Robot instance
 */
void ROBOT_dtor(Robot* self){

}