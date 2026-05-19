#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <math.h>
#include <stdint.h>
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

#define RENDER_HZ 60.0
#define PID_HZ    1000.0

#define RENDER_DT (1.0 / RENDER_HZ)
#define PID_DT    (1.0 / PID_HZ)

#define MAX_PID_STEPS_PER_LOOP 5

// ---------------------------------------------------------
// Globals
// ---------------------------------------------------------
static Camera camera = { 0 };

static Robot* SCARA;

Controller* G_const1 = NULL;
Controller* G_const2 = NULL;
Controller* G_const3 = NULL;

ZFilter* motor_plant1 = NULL;
ZFilter* motor_plant2 = NULL;
ZFilter* motor_plant3 = NULL;

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
static void Control_Update(double dt);
static void IdleTasks(void);
static void UpdateDrawFrame(void);

// ---------------------------------------------------------
// Main
// ---------------------------------------------------------
int main(void)
{
    if (Logger_init("Logs/log.txt") != 0) {
       printf("Failed to initialize logger. Logging to stderr.\n");
    }

    float Gain = 2.0f;

    float plant_b[] = { 1.0f };
    float plant_a[] = { 1.0f };

    float Dead_Zone = 0.0f, Saturation = 12.0f;


    Vector3 link1Dim = { 0.35f, 1.0f, 0.35f }; // bace REVOLUTE LINK 
    Vector3 link2Dim = { 2.0f, 0.35f, 0.35f }; // link 2 REVOLUTE LINK 
    Vector3 link3Dim = { 1.5f, 0.35f, 0.35f }; // link 2 PRISMATIC_LINK
    Vector3 link4Dim = { 0.35f, 1.7f, 0.35f }; // link 2 none

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

    // setup simple P Controller for testing
    G_const1 = Controller_create(&GainController_Type, Gain);
    G_const2 = Controller_create(&GainController_Type, Gain);
    G_const3 = Controller_create(&GainController_Type, Gain);

    Controller* controllers[NUM_CTRLS] = {G_const1, G_const2, G_const3};
    Link* links[NUM_LINKS] = {link1, link2, link3, link4};
    
    // setup scara robot with simple setup for testing
    SCARA = ROBOT_ctor(controllers, links);

    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "SCARA simulator");

    camera.position = (Vector3){ 10.0f, 10.0f, 8.0f };
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

        bool did_work = false;

        // -------------------------------------------------
        // 1 kHz PID/control loop
        // -------------------------------------------------
        int pid_steps = 0;

        while (now >= next_pid_time && pid_steps < MAX_PID_STEPS_PER_LOOP)
        {
            Control_Update(PID_DT);

            next_pid_time += PID_DT;
            pid_steps++;
            did_work = true;
        }

        // If PID falls too far behind, resync instead of spiraling.
        if (pid_steps >= MAX_PID_STEPS_PER_LOOP)
        {
            next_pid_time = now + PID_DT;
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

    Controller_destroy(G_const1);
    Controller_destroy(G_const2);

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


// ---------------------------------------------------------
// Runs at 1 kHz
// Put PID, control, simulation, path math, etc. here.
// ---------------------------------------------------------
static void Control_Update(double dt)
{
    (void)dt;

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
}

// ---------------------------------------------------------
// Runs only when PID and render are not due
// Put low-priority background work here.
// ---------------------------------------------------------
static void IdleTasks(void)
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

static void DrawWorldAxes3D(float length)
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
static void UpdateDrawFrame(void)
{
    UpdateCamera(&camera, CAMERA_ORBITAL);

    BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

            DrawGrid(10, 1.0f);

            DrawWorldAxes3D(5.0f);

            ROBOT_Draw(SCARA);

        EndMode3D();

        Vector2 xLabel = GetWorldToScreen((Vector3){ 5.25f, 0.0f, 0.0f }, camera);
        Vector2 yLabel = GetWorldToScreen((Vector3){ 0.0f, 5.25f, 0.0f }, camera);
        Vector2 zLabel = GetWorldToScreen((Vector3){ 0.0f, 0.0f, 5.25f }, camera);

        DrawText("X", (int)xLabel.x, (int)xLabel.y, 24, RED);
        DrawText("Y", (int)yLabel.x, (int)yLabel.y, 24, GREEN);
        DrawText("Z", (int)zLabel.x, (int)zLabel.y, 24, BLUE);

        DrawText("Simple scheduler", 10, 40, 20, DARKGRAY);
        DrawFPS(10, 10);

    EndDrawing();
}