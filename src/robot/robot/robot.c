
#include <math.h>
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/MemAllocator/mem.h"
#include "Robot_kinematics.h"
#include "robot_protected.h"
#include "robot.h"

#define MIN_LIMIT_INDEX 0
#define MAX_LIMIT_INDEX 1

#define ROBOT_UPDATE_HZ        1000.0f
#define ROBOT_UPDATE_DT        (1.0f / ROBOT_UPDATE_HZ)

#define ROBOT_POS_LOOP_HZ      100.0f
#define ROBOT_POS_LOOP_DIVIDER 10

#define J1_INDEX 0
#define J2_INDEX 1
#define J3_INDEX 2

static int inRangef(float value, float min, float max) {
    return (value >= min) && (value <= max);
}

static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static Vector3 Robot_Clamp_JP_Velocity(Robot* self, Vector3 v)
{
    v.x = clampf(v.x, self->jp_velocity_limits[J1_INDEX][MIN_LIMIT_INDEX],
                      self->jp_velocity_limits[J1_INDEX][MAX_LIMIT_INDEX]);

    v.y = clampf(v.y, self->jp_velocity_limits[J2_INDEX][MIN_LIMIT_INDEX],
                      self->jp_velocity_limits[J2_INDEX][MAX_LIMIT_INDEX]);

    v.z = clampf(v.z, self->jp_velocity_limits[J3_INDEX][MIN_LIMIT_INDEX],
                      self->jp_velocity_limits[J3_INDEX][MAX_LIMIT_INDEX]);

    return v;
}

static Vector3 Robot_Clamp_JP_Position(Robot* self, Vector3 jp)
{
    jp.x = clampf(jp.x, self->jp_limits[J1_INDEX][MIN_LIMIT_INDEX],
                        self->jp_limits[J1_INDEX][MAX_LIMIT_INDEX]);

    jp.y = clampf(jp.y, self->jp_limits[J2_INDEX][MIN_LIMIT_INDEX],
                        self->jp_limits[J2_INDEX][MAX_LIMIT_INDEX]);

    jp.z = clampf(jp.z, self->jp_limits[J3_INDEX][MIN_LIMIT_INDEX],
                        self->jp_limits[J3_INDEX][MAX_LIMIT_INDEX]);

    return jp;
}

void Robot_Velocity_Loop(Robot* self);
void Robot_Position_Loop(Robot* self);


/**
 * @brief Constructs a new Robot instance with the given controllers and links.
 * 
 * @param controllers an array of pointers to the controllers for each link
 * @param links an array of pointers to the links in the robot
 * @return Robot* pointer to the constructed Robot instance
 */
Robot* ROBOT_ctor(Controller* controllers[NUM_CTRLS], Link* links[NUM_LINKS]){

    Robot* robot = NULL;
    Robot_protected* protected = NULL;
    Robot_state state;

    if (controllers == NULL || links == NULL) {
        RAISE(NullptrError);
        return NULL;
    }

    for(int i = 0; i < NUM_LINKS; i++) {
        if (links[i] == NULL) {
            RAISE(NullptrError);
            return NULL;
        }
    }

    for(int i = 0; i < NUM_CTRLS; i++) {
        if (controllers[i] == NULL) {
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
    }

    for (int i = 0; i < NUM_CTRLS; i++) {
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

    Robot_Clamp_JP_Position(self, jp_setpoint);

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
    Robot_Clamp_JP_Velocity(self, jp_velocity_setpoint);

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

    RAISE(ValueError);
    
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

    if (self->protected->tick_counter >= ROBOT_POS_LOOP_HZ){
        Robot_Position_Loop(self);
        self->protected->tick_counter = 0;
    }

    Robot_Velocity_Loop(self);

    self->protected->tick_counter++;

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
void ROBOT_Draw(Robot* self)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    Vector3 origin = Vector3Zero();

    Link* link1 = self->protected->links[0];
    Link* link2 = self->protected->links[1];
    Link* link3 = self->protected->links[2];
    Link* link4 = self->protected->links[3];

    // link1: fixed base
    LINK_Set_Start(link1, origin);
    Vector3 end1 = LINK_Draw(link1);

    // link2: child of base, rotated by joint1
    LINK_Set_Start(link2, end1);
    Vector3 end2 = LINK_Draw(link2);

    // link3: child of link2, rotated by joint2
    LINK_Set_Start(link3, end2);
    Vector3 end3 = LINK_Draw(link3);

    // joint3: prismatic transform between link3 and link4
    float slide = LINK_Get_JP(link3);

    Vector3 slideOffset = {
        .x = 0.0f,
        .y = -slide,
        .z = 0.0f
    };

    Vector3 tcpStart = Vector3Add(end3, slideOffset);

    // link4: passive TCP/tool body
    LINK_Set_Start(link4, tcpStart);
    LINK_Draw(link4);
}

void Robot_Velocity_Loop(Robot* self){
    


}


void Robot_Position_Loop(Robot* self){

}