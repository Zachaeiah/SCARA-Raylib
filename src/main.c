#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include "utils/Profiler/profiler.h"
#include "Robot/Robot/robot.h"
#include "Control/Controller/Controller.h"
#include "Robot/link/link.h"
#include "Control/Ztransform/Ztransform.h"
#include "Control/Controllers/Simple/GainController.h"
#include "Control/Actuator/Actuator.h"
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/except.h"

// ---------------------------------------------------------
// Timing
// ---------------------------------------------------------
#define SCREEN_WIDTH  1600
#define SCREEN_HEIGHT 900

#define RENDER_HZ   120.0f
#define CTRL_FEQ    1000.0f

#define RENDER_DT (1.0f / RENDER_HZ)
#define CTRL_DT    (1.0f / CTRL_FEQ)

#define MAX_PID_STEPS_PER_LOOP 5

#define TEST_POSE_PERIOD_SEC 5.0

static ProfilerTimer prof_robot  = { "Robot Draw" };
static ProfilerTimer prof_frame  = { "Frame" };
static ProfilerTimer prof_control = { "Control Update" };

// ---------------------------------------------------------
// Globals
// ---------------------------------------------------------
static Camera camera = { 0 };

static Robot* SCARA;

// actuator velocity controller
Controller* G1_vel = NULL;
Controller* G2_vel = NULL;
Controller* G3_vel = NULL;

// actuator posions controller
Controller* G1_pos = NULL;
Controller* G2_pos = NULL;
Controller* G3_pos = NULL;

// motor plands
ZFilter* motor_plant1 = NULL;
ZFilter* motor_plant2 = NULL;
ZFilter* motor_plant3 = NULL;

// actuator to control the plands
Actuator* actuator1 = NULL;
Actuator* actuator2 = NULL;
Actuator* actuator3 = NULL;

Link* link1 = NULL;
Link* link2 = NULL;
Link* link3 = NULL;
Link* link4 = NULL;

// ---------------------------------------------------------
// Functions
// ---------------------------------------------------------
void Pos_Ctrl_Update(void);
void Control_Update(void);
void IdleTasks(void);
void UpdateDrawFrame(void);
void UpdateTestPoseCycle(Robot* robot, double now);

// ---------------------------------------------------------
// Main
// ---------------------------------------------------------
int main(void)
{
    if (Logger_init("Logs/log.txt") != 0) {
       printf("Failed to initialize logger. Logging to stderr.\n");
    }

    const float THETA_MAX = 170.0f * DEG2RAD;
    const float DISPLACEMENT_MAX = 1.70f;
    const float LIN_VEL = 0.5;
    const float OMEGA_MAX = 180.0f * DEG2RAD;

    float Gain_vel = 0.9f;
    float Gain_pos = 3.0f;
    float plant_b[] = { 1.0f };
    float plant_a[] = { 1.0f };
    float Dead_Zone = 0.0f, Saturation = 12.0f;

    float jp_limits[NUM_LINKS][2] = {
        { -THETA_MAX,        THETA_MAX },
        { -THETA_MAX,        THETA_MAX },
        { -DISPLACEMENT_MAX, 0.0f },
        { -DISPLACEMENT_MAX, 0.0f      }
    };

    float jp_velocity_limits[NUM_LINKS][2] = {
        { -OMEGA_MAX, OMEGA_MAX },
        { -OMEGA_MAX, OMEGA_MAX },
        { -LIN_VEL,   LIN_VEL },
        { -LIN_VEL,   LIN_VEL   }
    };

    Vector3 link1Dim = { 3.5f, 10.0f, 3.5f }; // bace REVOLUTE LINK 
    Vector3 link2Dim = { 20.0f, 3.5f, 3.5f }; // link 2 REVOLUTE LINK 
    Vector3 link3Dim = { 15.0f, 3.5f, 3.5f }; // link 2 PRISMATIC_LINK
    Vector3 link4Dim = { 3.5f, 17.0f, 3.5f }; // link 2 none

    // setup simple plands for easy testing
    motor_plant1 = ZFilter_ctor(plant_b, 1, plant_a, 1);
    motor_plant2 = ZFilter_ctor(plant_b, 1, plant_a, 1);
    motor_plant3 = ZFilter_ctor(plant_b, 1, plant_a, 1);

    // setup simple actuator for testing
    actuator1 = Actuator_ctor(motor_plant1, Dead_Zone, Saturation);
    actuator2 = Actuator_ctor(motor_plant2, Dead_Zone, Saturation);
    actuator3 = Actuator_ctor(motor_plant3, Dead_Zone, Saturation);

    // setup simple links for testing
    link1 = LINK_ctor(link1Dim, RED, BASE_LINK, actuator1);
    link2 = LINK_ctor(link2Dim, GREEN, REVOLUTE_LINK, actuator2);

    // link3 owns the prismatic joint
    link3 = LINK_ctor(link3Dim, BLUE, REVOLUTE_LINK, actuator3);

    // link4 is just the moving tool/end link
    link4 = LINK_ctor(link4Dim, ORANGE, TCP_LINK, NULL);

    // P controller for actuator velocity
    G1_vel = Controller_create(&GainController_Type, Gain_vel);
    G2_vel = Controller_create(&GainController_Type, Gain_vel);
    G3_vel = Controller_create(&GainController_Type, Gain_vel);

    // P controller for actuator posion
    G1_pos = Controller_create(&GainController_Type, Gain_pos);
    G2_pos = Controller_create(&GainController_Type, Gain_pos);
    G3_pos = Controller_create(&GainController_Type, Gain_pos);

    Controller* controllers[] = {G1_pos, G2_pos, G3_pos, G1_vel, G2_vel, G3_vel,};
    Link* links[NUM_LINKS] = {link1, link2, link3, link4};
    
    // setup scara robot with simple setup for testing
    SCARA = ROBOT_ctor(controllers, links);

    // set joint limits
    ROBOT_set_limits(SCARA, jp_limits, jp_velocity_limits);

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "SCARA simulator");

    camera.position = (Vector3){ 50.0f, 50.0f, 10.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    double now = GetTime();

    double next_pid_time = now;
    double next_render_time = now;

    LOG_MESSAGE("Program started");

    while (!WindowShouldClose())
    {
        now = GetTime();

        UpdateTestPoseCycle(SCARA, now);

        bool did_work = false;

        // -------------------------------------------------
        // 1 kHz Vel_PID/control loop
        // -------------------------------------------------
        int pid_steps = 0;

        while (now >= next_pid_time)
        {
            Pos_Ctrl_Update();

            next_pid_time += CTRL_DT;
            pid_steps++;
            did_work = true;
        }

        // If PID falls too far behind, resync instead of spiraling.
        if (pid_steps >= MAX_PID_STEPS_PER_LOOP)
        {
            next_pid_time = now + CTRL_DT;
        }

        // -------------------------------------------------
        // 60 FPS screen update/draw
        // -------------------------------------------------
        if (now >= next_render_time)
        {
            UpdateDrawFrame();

            next_render_time += RENDER_DT;
            did_work = true;

            // If rendering falls behind, resync.
            if (now > next_render_time + RENDER_DT)
            {
                next_render_time = now + RENDER_DT;
            }
        }

        // -------------------------------------------------
        // Low-priority tasks
        // -------------------------------------------------
        if (!did_work)
        {
            IdleTasks();

            // Give CPU a tiny break.
            // This prevents the loop from burning 100% CPU.
            WaitTime(0.0001);
        }
    }

    ROBOT_dtor(SCARA);

    Controller_destroy(G1_vel);
    Controller_destroy(G2_vel);
    Controller_destroy(G3_vel);

    Controller_destroy(G1_pos);
    Controller_destroy(G2_pos);
    Controller_destroy(G3_pos);

    LINK_dtor(link1);
    LINK_dtor(link2);

    Actuator_dtor(actuator1);
    Actuator_dtor(actuator2);

    ZFilter_dtor(motor_plant1);
    ZFilter_dtor(motor_plant2);

    Logger_shutdown();

    CloseWindow();
    

    return 0;
}

void Pos_Ctrl_Update(void){
    ROBOT_update(SCARA);
}


// ---------------------------------------------------------
// Runs at 1 kHz
// Put PID, control, simulation, path math, etc. here.
// ---------------------------------------------------------
void Control_Update(void)
{
    PROFILER_Begin(&prof_control);

    float t = (float)GetTime();

    float joint1Angle = cosf(t * 0.25f * PI) * 90.0f * DEG2RAD;
    float joint2Angle = sinf(t * 0.25f * PI) * 90.0f * DEG2RAD;

    // Positive distance downward
    float joint3Slide =  (sinf(t * 0.5f * PI) -1) * 1.70/2;

    float link1Heading = 0.0f;
    float link2Heading = joint1Angle;
    float link3Heading = joint1Angle + joint2Angle;
    float link4Heading = joint1Angle + joint2Angle;

    LINK_Set_Heading(link1, link1Heading);
    LINK_Set_JP(link1, 0.0f);

    LINK_Set_Heading(link2, link2Heading);
    LINK_Set_JP(link2, joint1Angle);

    LINK_Set_Heading(link3, link3Heading);
    LINK_Set_JP(link3, joint3Slide);

    LINK_Set_Heading(link4, link4Heading);
    LINK_Set_JP(link4, 0.0f);

    PROFILER_End(&prof_control);
}

void UpdateTestPoseCycle(Robot* robot, double now)
{
    /*
        JP meaning assumed:
            x = joint 1 angle, radians
            y = joint 2 angle, radians
            z = prismatic joint position

        If your prismatic joint moves the opposite direction,
        flip the signs on the z values.
    */

    float const Rad30  = 30.0f  * DEG2RAD;
    float const Rad45  = 45.0f  * DEG2RAD;
    float const Rad90  = 90.0f  * DEG2RAD;
    float const Rad125 = 125.0f * DEG2RAD;

    float const Z_TOP = 0.0f;
    float const Z_MID = -17.0f / 2.0f;
    float const Z_LOW = -17.0f;

    static const Vector3 test_poses[] = {
        // Home / neutral
        {  0.0f,    0.0f,    Z_TOP },
        {  0.0f,    0.0f,    Z_MID },
        {  0.0f,    0.0f,    Z_LOW },

        // Joint 1 only
        {  Rad45,   0.0f,    Z_MID },
        { -Rad45,   0.0f,    Z_MID },
        {  Rad90,   0.0f,    Z_LOW },
        { -Rad90,   0.0f,    Z_LOW },

        // Joint 2 only
        {  0.0f,    Rad45,   Z_MID },
        {  0.0f,   -Rad45,   Z_MID },
        {  0.0f,    Rad90,   Z_LOW },
        {  0.0f,   -Rad90,   Z_LOW },

        // Same direction bends
        {  Rad45,   Rad45,   Z_MID },
        { -Rad45,  -Rad45,   Z_MID },
        {  Rad90,   Rad45,   Z_LOW },
        { -Rad90,  -Rad45,   Z_LOW },

        // Opposite direction bends
        {  Rad45,  -Rad45,   Z_MID },
        { -Rad45,   Rad45,   Z_MID },
        {  Rad90,  -Rad90,   Z_LOW },
        { -Rad90,   Rad90,   Z_LOW },

        // Near-limit stress tests
        {  Rad125,  Rad125,  Z_TOP },
        {  Rad125, -Rad125,  Z_LOW },
        { -Rad125,  Rad125,  Z_TOP },
        { -Rad125, -Rad125,  Z_LOW },

        // Smaller smooth-motion checks
        {  Rad30,  -Rad30,   Z_MID },
        { -Rad30,   Rad30,   Z_MID },
    };

    static bool initialized = false;
    static size_t pose_index = 0;
    static double next_pose_time = 0.0;

    if (!robot) {
        return;
    }

    if (!initialized) {
        ROBOT_set_JP_target(robot, test_poses[pose_index]);
        next_pose_time = now + TEST_POSE_PERIOD_SEC;
        initialized = true;
        return;
    }

    if (now >= next_pose_time) {
        pose_index = (pose_index + 1) % (sizeof(test_poses) / sizeof((test_poses)[0]));

        ROBOT_set_JP_target(robot, test_poses[pose_index]);

        // Resync from current time so it does not try to catch up.
        next_pose_time = now + TEST_POSE_PERIOD_SEC;
    }
}

// ---------------------------------------------------------
// Runs only when PID and render are not due
// Put low-priority background work here.
// ---------------------------------------------------------
void IdleTasks(void)
{
    // Examples:
    // - process queued commands
    // - update non-critical GUI state
    // - logging
    // - file checks
    // - serial/network polling
    //
    // Keep this short.
    // Do not block here.
}

void DrawWorldAxes3D(float length)
{
    const float shaftRadius = 0.0225f;
    const float headRadius  = 0.07f;
    const float headLength  = 0.20f;

    Vector3 origin = { 0.0f, 0.0f, 0.0f };

    // X axis
    DrawCylinderEx(
        origin,
        (Vector3){ length - headLength, 0.0f, 0.0f },
        shaftRadius,
        shaftRadius,
        16,
        RED
    );

    DrawCylinderEx(
        (Vector3){ length - headLength, 0.0f, 0.0f },
        (Vector3){ length, 0.0f, 0.0f },
        headRadius,
        0.0f,
        16,
        RED
    );

    // Y axis
    DrawCylinderEx(
        origin,
        (Vector3){ 0.0f, length - headLength, 0.0f },
        shaftRadius,
        shaftRadius,
        16,
        GREEN
    );

    DrawCylinderEx(
        (Vector3){ 0.0f, length - headLength, 0.0f },
        (Vector3){ 0.0f, length, 0.0f },
        headRadius,
        0.0f,
        16,
        GREEN
    );

    // Z axis
    DrawCylinderEx(
        origin,
        (Vector3){ 0.0f, 0.0f, length - headLength },
        shaftRadius,
        shaftRadius,
        16,
        BLUE
    );

    DrawCylinderEx(
        (Vector3){ 0.0f, 0.0f, length - headLength },
        (Vector3){ 0.0f, 0.0f, length },
        headRadius,
        0.0f,
        16,
        BLUE
    );

    // Origin marker
    DrawSphere(origin, 0.12f, BLACK);
}

// ---------------------------------------------------------
// Runs at 60 FPS
// Put raylib drawing here.
// ---------------------------------------------------------
void UpdateDrawFrame(void)
{
    PROFILER_Begin(&prof_frame);


    UpdateCamera(&camera, CAMERA_ORBITAL);

    float J1_rad = LINK_Get_JP(link1);
    float J2_rad = LINK_Get_JP(link2);
    float J3_pos = LINK_Get_JP(link3);   // prismatic joint, probably distance not angle


    BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

            DrawGrid(30, 5.0f);

            DrawWorldAxes3D(5.0f);

            PROFILER_Begin(&prof_robot);
            ROBOT_Draw(SCARA);
            PROFILER_End(&prof_robot);

        EndMode3D();

        PROFILER_End(&prof_frame);
        
        DrawText(TextFormat("FPS: %d", GetFPS()), 10, 10, 20, DARKGRAY);
        DrawText(TextFormat("Frame: %.3f ms", prof_frame.elapsed_ms), 10, 40, 20, DARKGRAY);
        DrawText(TextFormat("Robot Draw: %.3f ms", prof_robot.elapsed_ms), 10, 60, 20, DARKGRAY);
        DrawText(TextFormat("Control: %.6f ms", prof_control.elapsed_ms), 10, 80, 20, DARKGRAY);

        DrawText(TextFormat("J1: %.2f deg", J1_rad * RAD2DEG), 10, 110, 20, DARKGRAY);
        DrawText(TextFormat("J2: %.2f deg", J2_rad * RAD2DEG), 10, 130, 20, DARKGRAY);
        DrawText(TextFormat("J3: %.3f m", J3_pos),             10, 150, 20, DARKGRAY);

    EndDrawing();
}