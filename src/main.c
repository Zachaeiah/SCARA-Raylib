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
#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 450

#define RENDER_HZ 60.0
#define PID_HZ    1000.0

#define RENDER_DT (1.0 / RENDER_HZ)
#define PID_DT    (1.0 / PID_HZ)

#define MAX_PID_STEPS_PER_LOOP 5

// ---------------------------------------------------------
// Globals
// ---------------------------------------------------------
static Camera camera = { 0 };

static double pid_time = 0.0;
static uint64_t pid_count = 0;
static Vector3 cubePosition = { 0 };

static Robot* SCARA;

// ---------------------------------------------------------
// Functions
// ---------------------------------------------------------
static void PID_Update(double dt);
static void Other_IdleTasks(void);
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
    Controller* G_const1 = NULL;
    Controller* G_const2 = NULL;
    Controller* G_const3 = NULL;

    ZFilter* motor_plant1 = NULL;
    ZFilter* motor_plant2 = NULL;
    ZFilter* motor_plant3 = NULL;
    float plant_b[] = { 1.0f };
    float plant_a[] = { 1.0f };

    Actuator* actuator1 = NULL;
    Actuator* actuator2 = NULL;
    Actuator* actuator3 = NULL;
    float Dead_Zone = 0.0f, Saturation = 12.0f;

    Link* link1 = NULL;
    Link* link2 = NULL;
    Link* link3 = NULL;

    Vector3 link1Dim = { 0.35f, 1.0f, 0.35f }; // bace REVOLUTE LINK 
    Vector3 link2Dim = { 4.0f, 0.35f, 0.35f }; // link 2 REVOLUTE LINK 
    Vector3 link3Dim = { 0.35f, 2.0f, 0.35f }; // link 2 PRISMATIC_LINK

    // setup simple plands for easy testing
    LOG_DEBUG_MSG(NO_ERROR, "Crateing Motor Plant with b = {%f}, a = {%f}", plant_b[0], plant_a[0]);
    motor_plant1 = ZFilter_ctor(plant_b, 1, plant_a, 1);
    motor_plant2 = ZFilter_ctor(plant_b, 1, plant_a, 1);
    motor_plant3 = ZFilter_ctor(plant_b, 1, plant_a, 1);

    // setup simple actuator for testing
    LOG_DEBUG_MSG(NO_ERROR, "Creating Actuator with Motor Plant, DZ = %f, SAT = %f", Dead_Zone, Saturation);
    actuator1 = Actuator_ctor(motor_plant1, Dead_Zone, Saturation);
    actuator2 = Actuator_ctor(motor_plant2, Dead_Zone, Saturation);
    actuator3 = Actuator_ctor(motor_plant3, Dead_Zone, Saturation);

    // setup simple links for testing
    LOG_DEBUG_MSG(NO_ERROR, "Creating Revolute link with actuator");
    link1 = LINK_ctor(link1Dim, ORANGE, REVOLUTE_LINK, actuator1);
    link2 = LINK_ctor(link2Dim, SKYBLUE, REVOLUTE_LINK, actuator2);
    link3 = LINK_ctor(link2Dim, SKYBLUE, PRISMATIC_LINK, actuator3);

    // setup simple P Controller for testing
    LOG_DEBUG_MSG(NO_ERROR, "crateing gain controller");
    G_const1 = Controller_create(&GainController_Type, Gain);
    G_const2 = Controller_create(&GainController_Type, Gain);
    G_const3 = Controller_create(&GainController_Type, Gain);

    Controller **controllers = {G_const1, G_const2, G_const2};
    Link **links = {link1, link2, link3};
    
    // setup scara robot with simple setup for testing
    SCARA = ROBOT_ctor(controllers, links);


    float output = ZFilter_update(motor_plant1, 2);
    LOG_DEBUG_MSG(NO_ERROR, "update motor_plant(2) -> %f", output);

    output = Actuator_update(actuator1, 2);
    LOG_DEBUG_MSG(NO_ERROR, "update actuator(2) -> %f", output);

    output = Actuator_update(actuator1, 13);
    LOG_DEBUG_MSG(NO_ERROR, "update actuator(13) -> %f", output);

    output = LINK_update(link1, 2);
    LOG_DEBUG_MSG(NO_ERROR, "update link1(2) -> %f", output);

    output = Controller_update(G_const1, 2);
    LOG_DEBUG_MSG(NO_ERROR, "update G_const(2) -> %f", output);







    ROBOT_dtor(SCARA);

    Cntroller_destroy(G_const1);
    Controller_destroy(G_const2);

    LINK_dtor(link1);
    LINK_dtor(link2);

    Actuator_dtor(actuator1);
    Actuator_dtor(actuator2);

    ZFilter_dtor(motor_plant1);
    ZFilter_dtor(motor_plant2);






    


    // LOG_MESSAGE("Program started");

    // LOG_WARN_MSG(12, "This is a warning message with no specific error code.");

    // InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "raylib simple scheduler");

    // camera.position = (Vector3){ 10.0f, 10.0f, 8.0f };
    // camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    // camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    // camera.fovy = 60.0f;
    // camera.projection = CAMERA_PERSPECTIVE;

    // double now = GetTime();

    // double next_pid_time = now;
    // double next_render_time = now;

    // while (!WindowShouldClose())
    // {
    //     now = GetTime();

    //     bool did_work = false;

    //     // -------------------------------------------------
    //     // 1 kHz PID/control loop
    //     // -------------------------------------------------
    //     int pid_steps = 0;

    //     while (now >= next_pid_time && pid_steps < MAX_PID_STEPS_PER_LOOP)
    //     {
    //         PID_Update(PID_DT);

    //         next_pid_time += PID_DT;
    //         pid_steps++;
    //         did_work = true;
    //     }

    //     // If PID falls too far behind, resync instead of spiraling.
    //     if (pid_steps >= MAX_PID_STEPS_PER_LOOP)
    //     {
    //         next_pid_time = now + PID_DT;
    //     }

    //     // -------------------------------------------------
    //     // 60 FPS screen update/draw
    //     // -------------------------------------------------
    //     if (now >= next_render_time)
    //     {
    //         UpdateDrawFrame();

    //         next_render_time += RENDER_DT;
    //         did_work = true;

    //         // If rendering falls behind, resync.
    //         if (now > next_render_time + RENDER_DT)
    //         {
    //             next_render_time = now + RENDER_DT;
    //         }
    //     }

    //     // -------------------------------------------------
    //     // Low-priority tasks
    //     // -------------------------------------------------
    //     if (!did_work)
    //     {
    //         Other_IdleTasks();

    //         // Give CPU a tiny break.
    //         // This prevents the loop from burning 100% CPU.
    //         WaitTime(0.0001);
    //     }
    // }

    // Logger_shutdown();

    // CloseWindow();
    

    return 0;
}


static Vector3 RotateAroundY(Vector3 v, float angleRad)
{
    float c = cosf(angleRad);
    float s = sinf(angleRad);

    return (Vector3){
        .x = v.x*c + v.z*s,
        .y = v.y,
        .z = -v.x*s + v.z*c
    };
}

static Vector3 DrawTestLink(Vector3 start, Vector3 dim, float angleRad, Color color)
{
    Vector3 localCenter = {
        dim.x / 2.0f,
        dim.y / 2.0f,
        0.0f
    };

    Vector3 localEnd = {
        dim.x,
        dim.y,
        0.0f
    };

    Vector3 end = Vector3Add(start, RotateAroundY(localEnd, angleRad));

    rlPushMatrix();

        rlTranslatef(start.x, start.y, start.z);

        // Rotate around vertical Y axis
        rlRotatef(angleRad * RAD2DEG, 0.0f, 1.0f, 0.0f);

        // Move cube center relative to pivot
        rlTranslatef(localCenter.x, localCenter.y, localCenter.z);

        DrawCubeV(Vector3Zero(), dim, color);
        DrawCubeWiresV(Vector3Zero(), dim, BLACK);

    rlPopMatrix();

    // Debug visuals
    DrawSphere(start, 0.08f, RED);       // pivot/start joint
    DrawSphere(end, 0.08f, BLUE);        // next joint/end
    DrawLine3D(start, end, PURPLE);      // centerline

    return end;
}

// ---------------------------------------------------------
// Runs at 1 kHz
// Put PID, control, simulation, path math, etc. here.
// ---------------------------------------------------------
static void PID_Update(double dt)
{
    pid_time += dt;
    pid_count++;

    // Example background math
    cubePosition.y = 1.0f + 0.75f * sinf((float)(pid_time * 6.2831853));
}

// ---------------------------------------------------------
// Runs only when PID and render are not due
// Put low-priority background work here.
// ---------------------------------------------------------
static void Other_IdleTasks(void)
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

// ---------------------------------------------------------
// Runs at 60 FPS
// Put raylib drawing here.
// ---------------------------------------------------------
static void UpdateDrawFrame(void)
{
    UpdateCamera(&camera, CAMERA_ORBITAL);

    float t = (float)GetTime();

    float joint1Angle = sinf(t * 1.0f) * 90.0f * DEG2RAD;
    float joint2Angle = sinf(t * 1.7f) * 90.0f * DEG2RAD;

    float link1AngleAbs = joint1Angle;
    float link2AngleAbs = joint1Angle + joint2Angle;

    Vector3 link1Start = { 1.0f, 0.0f, 0.0f };

    Vector3 link0Dim = { 0.35f, 1.0f, 0.35f };
    Vector3 link1Dim = { 4.0f, 0.35f, 0.35f };
    Vector3 link2Dim = { 3.0f, 0.30f, 0.30f };

    

    BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

            Vector3 origin = { 0.0f, 0.0f, 0.0f };
            Vector3 x_axis = { 5.0f, 0.0f, 0.0f };
            Vector3 y_axis = { 0.0f, 5.0f, 0.0f };
            Vector3 z_axis = { 0.0f, 0.0f, 5.0f };

            DrawLine3D(origin, x_axis, RED);
            DrawLine3D(origin, y_axis, GREEN);
            DrawLine3D(origin, z_axis, BLUE);

            DrawGrid(10, 1.0f);

            Vector3 link1End = DrawTestLink(link1Start, link1Dim, link1AngleAbs, ORANGE);
            Vector3 link2End = DrawTestLink(link1End, link2Dim, link2AngleAbs, SKYBLUE);

            DrawSphere(link2End, 0.12f, GREEN); // end effector marker

        EndMode3D();

        DrawText("Simple scheduler", 10, 40, 20, DARKGRAY);
        DrawText("Render: 60 FPS", 10, 70, 20, DARKGRAY);
        DrawText("PID/control: 1 kHz", 10, 100, 20, DARKGRAY);

        DrawText(TextFormat("PID updates: %llu", (unsigned long long)pid_count),
                 10, 130, 20, DARKGRAY);

        DrawText(TextFormat("PID time: %.3f s", pid_time),
                 10, 160, 20, DARKGRAY);


        DrawFPS(10, 10);

    EndDrawing();
}