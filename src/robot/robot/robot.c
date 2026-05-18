
#include <math.h>
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/MemAllocator/mem.h"
#include "Robot_kinematics.h"
#include "robot_protected.h"
#include "robot.h"

#define MIN_LIMIT_INDEX 0
#define MAX_LIMIT_INDEX 1

static int inRangef(float value, float min, float max) {
    return (value >= min) && (value <= max);
}


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
        robot->jp_limits[i][MIN_LIMIT_INDEX] = -PI/2; // min angle
        robot->jp_limits[i][MAX_LIMIT_INDEX] = PI/2;  // max angle
        robot->jp_velocity_limits[i][MIN_LIMIT_INDEX] = -1.0f; // min velocity
        robot->jp_velocity_limits[i][MAX_LIMIT_INDEX] = 1.0f;  // max velocity
    }

    // initialize the robot state
    state.JP = Vector3Zero();
    state.TCP = Vector3Zero();
    state.TCP_velocity = Vector3Zero();
    state.JP_velocity = Vector3Zero();

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
void ROBOT_set_limits(Robot* self, float jp_limits[NUM_LINKS][2], 
                                   float jp_velocity_limits[NUM_LINKS][2])
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    if (!jp_limits || !jp_velocity_limits) {
        RAISE(NullptrError);
        return;
    }

    for (int i = 0; i < NUM_LINKS; i++) {
        if (jp_limits[i][MIN_LIMIT_INDEX] >= jp_limits[i][MAX_LIMIT_INDEX]) {
            RAISE(ValueError);
            return;
        }
        if (jp_velocity_limits[i][MIN_LIMIT_INDEX] >= jp_velocity_limits[i][MAX_LIMIT_INDEX]) {
            RAISE(ValueError);
            return;
        }
    }

    for (int i = 0; i < NUM_LINKS; i++) {
        self->jp_limits[i][MIN_LIMIT_INDEX] = jp_limits[i][MIN_LIMIT_INDEX];
        self->jp_limits[i][MAX_LIMIT_INDEX] = jp_limits[i][MAX_LIMIT_INDEX];
        self->jp_velocity_limits[i][MIN_LIMIT_INDEX] = jp_velocity_limits[i][MIN_LIMIT_INDEX];
        self->jp_velocity_limits[i][MAX_LIMIT_INDEX] = jp_velocity_limits[i][MAX_LIMIT_INDEX];
    }
}

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param jp_setpoint the target joint angles as a Vector3
 */
void ROBOT_set_JP(Robot* self, Vector3 jp_setpoint){

    if (!self) {
        RAISE(NullptrError);
        return;
    }

    // check if the target angles are within the joint limits
    FK_result fk_sol = ROBOT_forward_kinematics(self, jp_setpoint);

    // if the target angles are not reachable, raise a ValueError
    if (!fk_sol.reachable) {
        RAISE(ValueError);
        return;
    }

    // set control mode to angle control
    self->protected->mode = Angle_CONTROL_MODE; 

    // set the target angles in the protected data
    self->protected->Target_state.JP = jp_setpoint;

}

/**
 * @brief Sets the velocity for the robot links.
 * 
 * @param self pointer to the Robot instance
 * @param jp_velocity_setpoint the velocity setpoint for the robot link
 */
void ROBOT_set_JP_velocity(Robot* self, Vector3 jp_velocity_setpoint){
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    // check if the target velocity is within the link limits
    if (inRangef(jp_velocity_setpoint.x, self->jp_velocity_limits[0][MIN_LIMIT_INDEX], self->jp_velocity_limits[0][MAX_LIMIT_INDEX]) && 
        inRangef(jp_velocity_setpoint.y, self->jp_velocity_limits[1][MIN_LIMIT_INDEX], self->jp_velocity_limits[1][MAX_LIMIT_INDEX]) && 
        inRangef(jp_velocity_setpoint.z, self->jp_velocity_limits[2][MIN_LIMIT_INDEX], self->jp_velocity_limits[2][MAX_LIMIT_INDEX])) {
        // Velocity is within limits
    } else {
        RAISE(ValueError);
        return;
    }

    // set control mode to velocity control
    self->protected->mode = VELOCITY_CONTROL_MODE;

    // set the target velocity in the protected data
    self->protected->Target_state.JP_velocity = (Vector3){jp_velocity_setpoint.x, jp_velocity_setpoint.y, jp_velocity_setpoint.z};
}

/**
 * @brief Sets the target position for the robot to reach.
 * 
 * @param self pointer to the Robot instance
 * @param tcp_setpoint the target position as a Vector3
 */
void ROBOT_set_TCP_target(Robot* self, Vector3 tcp_setpoint){

    if (!self) {
        RAISE(NullptrError);
        return;
    }

    // check if the target position is within the robot's reach
    Vector3 jp_zero= Vector3Zero(); // dummy joint angles for fk_sol computation

    FK_result fk_sol = ROBOT_forward_kinematics(self, jp_zero);

    if (Vector3Length(tcp_setpoint) >= Vector3Length(fk_sol.TCP)) {
        RAISE(ValueError);
        return;
    }

    // set control mode to position control
    self->protected->mode = POSITION_CONTROL_MODE;

    // set the target position in the protected data
    self->protected->Target_state.TCP = tcp_setpoint;
    return;

}

/**
 * @brief Sets the velocity for the robot links.
 * 
 * @param self pointer to the Robot instance
 * @param velocity_setpoint the velocity setpoint for the robot link
 */
void ROBOT_set_TCP_velocity(Robot* self, Vector3 velocity_setpoint){

    if (!self) {
        RAISE(NullptrError);
        return;
    }

    // check if the target velocity is within the link limits

    if (inRangef(velocity_setpoint.x, self->jp_velocity_limits[0][MIN_LIMIT_INDEX], self->jp_velocity_limits[0][MAX_LIMIT_INDEX]) && 
        inRangef(velocity_setpoint.y, self->jp_velocity_limits[1][MIN_LIMIT_INDEX], self->jp_velocity_limits[1][MAX_LIMIT_INDEX]) && 
        inRangef(velocity_setpoint.z, self->jp_velocity_limits[2][MIN_LIMIT_INDEX], self->jp_velocity_limits[2][MAX_LIMIT_INDEX])) {
        // Velocity is within limits
    } else {
        RAISE(ValueError);
        return;
    }

    // set control mode to velocity control
    self->protected->mode = VELOCITY_CONTROL_MODE;

    // set the target velocity in the protected data
    self->protected->Target_state.TCP_velocity = (Vector3){velocity_setpoint.x, velocity_setpoint.y, velocity_setpoint.z};
    
}

/**
 * @brief Updates the robot state based on the current control mode and target state.
 * 
 * @param self pointer to the Robot instance
 */
void ROBOT_update(Robot* self){

    // just for testing, will implement the actual control logic later
     if (!self) {
        RAISE(NullptrError);
        return;
    }

    // update the current state to match the target state for testing
    self->protected->Current_state = self->protected->Target_state;

    // update the joint angles of the links to match the target joint angles for testing
    LINK_update(self->protected->links[0], self->protected->Target_state.JP.x);
    LINK_update(self->protected->links[1], self->protected->Target_state.JP.y);
    LINK_update(self->protected->links[2], self->protected->Target_state.JP.z);

}



/**
 * @brief Destructs the Robot instance and frees all allocated memory.
 * 
 * @param self pointer to the Robot instance
 */
void ROBOT_dtor(Robot* self){

    if (!self) {
        RAISE(NullptrError);
        return;
    }

    // free the protected data
    if (self->protected) {
        FREE(self->protected);
    }

    // free the robot instance
    FREE(self);

}

/**
 * @brief Draws the robot using raylib.
 * 
 * @param self  pointer to the Robot instance
 */
void ROBOT_Draw(Robot* self){

    if (!self) {
        RAISE(NullptrError);
        return;
    }

    // bace
    LINK_Set_Start(self->protected->links[0], Vector3Zero()); // set the start position of the first link to the origin
    Vector3 end = LINK_Draw(self->protected->links[0]); // draw the first link and get the end position

    // link 1
    LINK_Set_Start(self->protected->links[1], end); // set the start position of the second link to the end of the first link
    end = LINK_Draw(self->protected->links[1]); // draw the second link and get the end position

    // link 2
    LINK_Set_Start(self->protected->links[2], end); // set the start position of the third link to the end of the second link
    end = LINK_Draw(self->protected->links[2]); // draw the third link and get the end position

    
}