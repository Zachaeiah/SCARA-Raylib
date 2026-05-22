
#include "robot_internal.h"

#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include "raymath.h"

#define ROBOT_POSITION_LOOP_DIVIDER 10u

#define MIN_LIMIT_INDEX 0
#define MAX_LIMIT_INDEX 1

#define ROBOT_VEL_LOOP_HZ        1000.0f
#define ROBOT_VEL_UPDATE_DT          (1.0f / ROBOT_VEL_LOOP_HZ)

#define ROBOT_POS_LOOP_HZ        100.0f
#define ROBOT_POS_LOOP_TICKS     (ROBOT_VEL_LOOP_HZ / ROBOT_POS_LOOP_HZ)

void ROBOT_VelocityLoop(Robot* self, float dt);
void ROBOT_PositionLoop(Robot* self);

static float Vector3GetByJoint(Vector3 v, RobotJointIndex joint)
{
    switch (joint) {
        case ROBOT_JOINT_1: return v.x;
        case ROBOT_JOINT_2: return v.y;
        case ROBOT_JOINT_3: return v.z;

        default:
            RAISE(ValueError);
            return 0.0f;
    }
}

static void Vector3SetByJoint(Vector3* v, RobotJointIndex joint, float value)
{
    if (!v) {
        RAISE(NullptrError);
        return;
    }

    switch (joint) {
        case ROBOT_JOINT_1:
            v->x = value;
            break;

        case ROBOT_JOINT_2:
            v->y = value;
            break;

        case ROBOT_JOINT_3:
            v->z = value;
            break;

        default:
            RAISE(ValueError);
            break;
    }
}

static RobotControllerIndex PositionControllerFromJoint(RobotJointIndex joint)
{
    switch (joint) {
        case ROBOT_JOINT_1: return ROBOT_CTRL_J1_POS;
        case ROBOT_JOINT_2: return ROBOT_CTRL_J2_POS;
        case ROBOT_JOINT_3: return ROBOT_CTRL_J3_POS;

        default:
            RAISE(ValueError);
            return ROBOT_CTRL_J1_POS;
    }
}

static RobotControllerIndex VelocityControllerFromJoint(RobotJointIndex joint)
{
    switch (joint) {
        case ROBOT_JOINT_1: return ROBOT_CTRL_J1_VEL;
        case ROBOT_JOINT_2: return ROBOT_CTRL_J2_VEL;
        case ROBOT_JOINT_3: return ROBOT_CTRL_J3_VEL;

        default:
            RAISE(ValueError);
            return ROBOT_CTRL_J1_VEL;
    }
}

static void ROBOT_ReadJointState(Robot* self)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        RobotJointIndex joint = (RobotJointIndex)i;
        RobotLinkIndex link_index = LinkIndexFromJoint(joint);

        float joint_position = LINK_GetJointPosition(self->links[link_index]);
        Vector3SetByJoint(&self->current.joint_position, joint, joint_position);
    }
}

/**
 * @brief Constructs a new Robot instance with the given controllers and links.
 * 
 * @param controllers an array of pointers to the controllers for each link
 * @param links an array of pointers to the links in the robot
 * @return Robot* pointer to the constructed Robot instance
 */
Robot* ROBOT_Create(Controller* controllers[ROBOT_NUM_CTRLS],
                    Link* links[ROBOT_NUM_LINKS])
{
    if (!controllers || !links) {
        RAISE(NullptrError);
        return NULL;
    }

    Robot* self = NULL;
    NEW0(self);

    for (int i = 0; i < ROBOT_NUM_CTRLS; i++) {
        if (!controllers[i]) {
            FREE(self);
            RAISE(NullptrError);
            return NULL;
        }

        self->controllers[i] = controllers[i];
    }

    for (int i = 0; i < ROBOT_NUM_LINKS; i++) {
        if (!links[i]) {
            FREE(self);
            RAISE(NullptrError);
            return NULL;
        }

        self->links[i] = links[i];
    }

    self->mode = ROBOT_MODE_IDLE;

    self->current = (RobotState){0};
    self->target = (RobotState){0};

    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        self->joint_position_limits[i][ROBOT_LIMIT_MIN] = -1000000.0f;
        self->joint_position_limits[i][ROBOT_LIMIT_MAX] =  1000000.0f;

        self->joint_velocity_limits[i][ROBOT_LIMIT_MIN] = -1000000.0f;
        self->joint_velocity_limits[i][ROBOT_LIMIT_MAX] =  1000000.0f;
    }

    self->position_loop_counter = 0u;

    return self;
}

void ROBOT_SetJointLimits(
    Robot* self,
    const float joint_position_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS],
    const float joint_velocity_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS]
)
{
    if (!self || !joint_position_limits || !joint_velocity_limits) {
        RAISE(NullptrError);
        return;
    }

    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        float jp_min = joint_position_limits[i][ROBOT_LIMIT_MIN];
        float jp_max = joint_position_limits[i][ROBOT_LIMIT_MAX];

        float vel_min = joint_velocity_limits[i][ROBOT_LIMIT_MIN];
        float vel_max = joint_velocity_limits[i][ROBOT_LIMIT_MAX];

        if (jp_min >= jp_max || vel_min >= vel_max) {
            RAISE(ValueError);
            return;
        }

        self->joint_position_limits[i][ROBOT_LIMIT_MIN] = jp_min;
        self->joint_position_limits[i][ROBOT_LIMIT_MAX] = jp_max;

        self->joint_velocity_limits[i][ROBOT_LIMIT_MIN] = vel_min;
        self->joint_velocity_limits[i][ROBOT_LIMIT_MAX] = vel_max;
    }
}

void ROBOT_SetJointPositionTarget(Robot* self, Vector3 joint_position_target)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    self->target.joint_position = joint_position_target;
    self->mode = ROBOT_MODE_JOINT_POSITION;
}

void ROBOT_SetJointVelocityTarget(Robot* self, Vector3 joint_velocity_target)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    self->target.joint_velocity = joint_velocity_target;
    self->mode = ROBOT_MODE_JOINT_VELOCITY;
}

void ROBOT_SetTCPPositionTarget(Robot* self, Vector3 tcp_position_target)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    self->target.tcp_position = tcp_position_target;
    self->mode = ROBOT_MODE_TCP_POSITION;
}

void ROBOT_SetTCPVelocityTarget(Robot* self, Vector3 tcp_velocity_target)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    self->target.tcp_velocity = tcp_velocity_target;
    self->mode = ROBOT_MODE_TCP_VELOCITY;
}

float ROBOT_GetJointPositionAt(const Robot* self, RobotJointIndex joint)
{
    if (!self) {
        RAISE(NullptrError);
        return 0.0f;
    }

    return Vector3GetByJoint(self->current.joint_position, joint);
}

float ROBOT_GetJointVelocityAt(const Robot* self, RobotJointIndex joint)
{
    if (!self) {
        RAISE(NullptrError);
        return 0.0f;
    }

    return Vector3GetByJoint(self->current.joint_velocity, joint);
}

Vector3 ROBOT_GetJointPosition(const Robot* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    return self->current.joint_position;
}


Vector3 ROBOT_GetJointVelocity(const Robot* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    return self->current.joint_velocity;
}

Vector3 ROBOT_GetTCPPosition(const Robot* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    return self->current.tcp_position;
}

Vector3 ROBOT_GetTCPVelocity(const Robot* self)
{
    if (!self) {
        RAISE(NullptrError);
        return Vector3Zero();
    }

    return self->current.tcp_velocity;
}


void ROBOT_Update(Robot* self, float dt)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    if (dt <= 0.0f) {
        RAISE(ValueError);
        return;
    }

    if (self->mode == ROBOT_MODE_IDLE) {
        ROBOT_ReadJointState(self);
        return;
    }

    if (self->position_loop_counter >= ROBOT_POSITION_LOOP_DIVIDER) {
        ROBOT_PositionLoop(self);
        self->position_loop_counter = 0u;
    }

    ROBOT_VelocityLoop(self, dt);

    self->position_loop_counter++;
}

void ROBOT_Destroy(Robot* self)
{
    if (!self) {
        return;
    }

    FREE(self);
}


void ROBOT_Draw(Robot* self)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    float j1 = LINK_GetJointPosition(self->links[ROBOT_LINK_2]);
    float j2 = LINK_GetJointPosition(self->links[ROBOT_LINK_3]);
    float j3 = LINK_GetJointPosition(self->links[ROBOT_LINK_4]);

    Vector3 origin = Vector3Zero();

    // Base
    LINK_SetStart(self->links[ROBOT_LINK_1], origin);
    LINK_SetHeadingWorld(self->links[ROBOT_LINK_1], 0.0f);

    Vector3 base_end = LINK_Draw(self->links[ROBOT_LINK_1]);

    // Arm 1
    LINK_SetStart(self->links[ROBOT_LINK_2], base_end);
    LINK_SetHeadingWorld(self->links[ROBOT_LINK_2], j1);

    Vector3 arm1_end = LINK_Draw(self->links[ROBOT_LINK_2]);

    // Arm 2
    LINK_SetStart(self->links[ROBOT_LINK_3], arm1_end);
    LINK_SetHeadingWorld(self->links[ROBOT_LINK_3], j1 + j2);

    Vector3 arm2_end = LINK_Draw(self->links[ROBOT_LINK_3]);

    // Prismatic tool / TCP slide
    LINK_SetStart(self->links[ROBOT_LINK_4], arm2_end);
    LINK_SetHeadingWorld(self->links[ROBOT_LINK_4], j1 + j2);
    LINK_SetJointPosition(self->links[ROBOT_LINK_4], j3);

    Vector3 tcp = LINK_Draw(self->links[ROBOT_LINK_4]);

    self->current.tcp_position = tcp;
}

void ROBOT_VelocityLoop(Robot* self, float dt)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    if (dt <= 0.0f) {
        RAISE(ValueError);
        return;
    }

    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        RobotJointIndex joint = (RobotJointIndex)i;
        RobotLinkIndex link_index = LinkIndexFromJoint(joint);
        RobotControllerIndex ctrl_index = VelocityControllerFromJoint(joint);

        Link* link = self->links[link_index];

        float target_velocity = Vector3GetByJoint(self->target.joint_velocity, joint);
        float current_velocity = Vector3GetByJoint(self->current.joint_velocity, joint);

        float velocity_error = target_velocity - current_velocity;

        /*
            Expected behavior:
                input  = velocity error
                output = actuator command, usually voltage/current
        */
        float actuator_command = Controller_update(
            self->controllers[ctrl_index],
            velocity_error
        );

        float measured_velocity = LINK_UpdateActuator(link, actuator_command);

        measured_velocity = Clamp(
            measured_velocity,
            self->joint_velocity_limits[i][ROBOT_LIMIT_MIN],
            self->joint_velocity_limits[i][ROBOT_LIMIT_MAX]
        );

        LINK_IntegrateJointPosition(link, measured_velocity, dt);

        float measured_position = LINK_GetJointPosition(link);

        measured_position = Clamp(
            measured_position,
            self->joint_position_limits[i][ROBOT_LIMIT_MIN],
            self->joint_position_limits[i][ROBOT_LIMIT_MAX]
        );

        LINK_SetJointPosition(link, measured_position);

        Vector3SetByJoint(&self->current.joint_velocity, joint, measured_velocity);
        Vector3SetByJoint(&self->current.joint_position, joint, measured_position);
    }
}


void ROBOT_PositionLoop(Robot* self)
{
    if (!self) {
        RAISE(NullptrError);
        return;
    }

    if (self->mode != ROBOT_MODE_JOINT_POSITION) {
        return;
    }

    ROBOT_ReadJointState(self);

    for (int i = 0; i < ROBOT_NUM_JOINTS; i++) {
        RobotJointIndex joint = (RobotJointIndex)i;
        RobotControllerIndex ctrl_index = PositionControllerFromJoint(joint);

        float target_position = Vector3GetByJoint(self->target.joint_position, joint);
        float current_position = Vector3GetByJoint(self->current.joint_position, joint);

        float position_error = target_position - current_position;

        /*
            Replace Controller_Update with your actual controller function name.
            Expected behavior:
                input  = position error
                output = velocity command
        */
        float velocity_command = Controller_update(
            self->controllers[ctrl_index],
            position_error
        );

        velocity_command = Clamp(
            velocity_command,
            self->joint_velocity_limits[i][ROBOT_LIMIT_MIN],
            self->joint_velocity_limits[i][ROBOT_LIMIT_MAX]
        );

        Vector3SetByJoint(&self->target.joint_velocity, joint, velocity_command);
    }
}