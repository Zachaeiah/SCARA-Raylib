// src/main.c

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include "utils/Profiler/profiler.h"
#include "GUI/Pages/SI_main_page/GuiSim_Panel.h"
#include "Robot/Robot/robot.h"
#include "Control/Controller/Controller.h"
#include "Robot/link/link.h"
#include "Control/Ztransform/Ztransform.h"
#include "Control/Controllers/PID_s/PID_CONT.h"
#include "Control/Actuator/Actuator.h"
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/except.h"
#include "GUI/Pages/SI_main_page/GuiSim_Panel.h"
#include "GUI/ploting/XY_plot/xy_plot.h"

const ErrorType JP_CMD_CLAMPED_ErrorCode = 1;
const ErrorType JP_CMD_REJECTED_ErrorCode = 2;
const ErrorType JP_RANGE_ErrorCode = 3;

const ErrorType TCP_CMD_CLAMPED_ErrorCode = 4;
const ErrorType TCP_CMD_REJECTED_ErrorCode = 5;

const ErrorType Mem_Failed_ErrorCode = 6; /**< Represents memory allocation failure. */
const ErrorType Mem_Free_Failed_ErrorCode = 7; /**< Represents memory free failure error code. */

const ErrorType Zfilter_Failed_ErrorCode = 8; /**< Represents Z-Filter failure error code. */
const ErrorType Controller_ErrorCode = 9; /**< Represents a generic controller error code. */
const ErrorType Actuator_Failed_ErrorCode = 10; /**< Represents Actuator failure error code. */

const ErrorType XYPLOT_Failed_ErrorCode  =10; /**< Represents XYPLOT Failed  failure error code. */

// ---------------------------------------------------------
// Timing
// ---------------------------------------------------------
#define SCREEN_WIDTH  2000
#define SCREEN_HEIGHT 900

#define RENDER_HZ   120.0f
#define CTRL_FEQ    1000.0f

#define RENDER_DT (1.0f / RENDER_HZ)
#define CTRL_DT    (1.0f / CTRL_FEQ)

#define MAX_PID_STEPS_PER_LOOP 5

#define TEST_POSE_PERIOD_SEC 5.0

// ---------------------------------------------------------
// Globals
// ---------------------------------------------------------
static Camera camera = { 0 };

static Robot SCARA;

// actuator velocity controller
Controller PID1_vel = NULL;
Controller PID2_vel = NULL;
Controller PID3_vel = NULL;

// actuator posions controller
Controller PID1_pos = NULL;
Controller PID2_pos = NULL;
Controller PID3_pos = NULL;

// ===== VELOCITY controller =====
#define VELOCITY_CONTROLLER_NUM_LEN 4
#define VELOCITY_CONTROLLER_DEN_LEN 4
static const float VELOCITY_CONTROLLER_NUM[] = { 1.16725369f, -2.73364842f, 2.04479720f, -0.47840154f };
static const float VELOCITY_CONTROLLER_DEN[4] = { 1.00000000f, -2.45697901f, 1.98767596f, -0.53069696f };

// ===== VELOCITY plant =====
#define VELOCITY_PLANT_NUM_LEN 2
#define VELOCITY_PLANT_DEN_LEN 3
static const float VELOCITY_PLANT_NUM[2] = { 0.00542332f, 0.00485343f };
static const float VELOCITY_PLANT_DEN[3] = { 1.00000000f, -1.71651923f, 0.71652265f };

// ===== POSITION controller =====
#define POSITION_CONTROLLER_NUM_LEN 3
#define POSITION_CONTROLLER_DEN_LEN 3
static const float POSITION_CONTROLLER_NUM[3] = { 1.82285934f, -1.87878799f, 0.46739681f };
static const float POSITION_CONTROLLER_DEN[3] = { 1.00000000f, -1.04377111f, 0.27236453f };

// motor plands
ZFilter motor_plant1 = NULL;
ZFilter motor_plant2 = NULL;
ZFilter motor_plant3 = NULL;

// actuator to control the plands
Actuator actuator1 = NULL;
Actuator actuator2 = NULL;
Actuator actuator3 = NULL;

Link link1 = NULL;
Link link2 = NULL;
Link link3 = NULL;
Link link4 = NULL;

static GuiSimPanel gui_sim_panel;

// ---------------------------------------------------------
// Functions
// ---------------------------------------------------------
void Pos_Ctrl_Update(void);
void Control_Update(void);
void IdleTasks(void);
void UpdateDrawFrame(void);
void UpdateTestPoseCycle(Robot robot, double now);

// ---------------------------------------------------------
// Main
// ---------------------------------------------------------
int main(void)
{
    if (Logger_init("Logs/log.txt") != 0) {
       printf("Failed to initialize logger. Logging to stderr.\n");
    }

    const float THETA_MAX = 170.0f * DEG2RAD;
    const float OMEGA_MAX = 180.0f * DEG2RAD;
    const float DISPLACEMENT_MAX = 17.00f;
    const float LIN_VEL = 10.0;
    

    float Dead_Zone = 0.0f, Saturation = 12.0f;

    float jp_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS] = {
        { -THETA_MAX,        THETA_MAX },  // J1 revolute
        { -THETA_MAX,        THETA_MAX },  // J2 revolute
        { 0.0, DISPLACEMENT_MAX      }   // J3 prismatic, negative is downward
    };

    float jp_velocity_limits[ROBOT_NUM_JOINTS][ROBOT_NUM_LIMITS] = {
        { -OMEGA_MAX, OMEGA_MAX },  // J1 rad/s
        { -OMEGA_MAX, OMEGA_MAX },  // J2 rad/s
        { -LIN_VEL,   LIN_VEL   }   // J3 linear units/s
    };

    Vector3 link1Dim = { 3.5f, 10.0f, 3.5f }; // bace REVOLUTE LINK 
    Vector3 link2Dim = { 20.0f, 3.5f, 3.5f }; // link 2 REVOLUTE LINK 
    Vector3 link3Dim = { 15.0f, 3.5f, 3.5f }; // link 2 PRISMATIC_LINK
    Vector3 link4Dim = { 3.5f, 17.0f, 3.5f }; // link 2 none

    // setup simple plands for easy testing
    motor_plant1 = ZFilter_ctor(VELOCITY_PLANT_NUM, VELOCITY_PLANT_NUM_LEN, VELOCITY_PLANT_DEN, VELOCITY_PLANT_DEN_LEN);
    motor_plant2 = ZFilter_ctor(VELOCITY_PLANT_NUM, VELOCITY_PLANT_NUM_LEN, VELOCITY_PLANT_DEN, VELOCITY_PLANT_DEN_LEN);
    motor_plant3 = ZFilter_ctor(VELOCITY_PLANT_NUM, VELOCITY_PLANT_NUM_LEN, VELOCITY_PLANT_DEN, VELOCITY_PLANT_DEN_LEN);

    // setup simple actuator for testing
    actuator1 = Actuator_ctor(motor_plant1, Dead_Zone, Saturation);
    actuator2 = Actuator_ctor(motor_plant2, Dead_Zone, Saturation);
    actuator3 = Actuator_ctor(motor_plant3, Dead_Zone, Saturation);

    // Base visual only. No actuator.
    link1 = LINK_Create(link1Dim, RED, LINK_BASE, NULL);

    // Joint 1 actuator drives arm 1.
    link2 = LINK_Create(link2Dim, GREEN, LINK_REVOLUTE, actuator1);

    // Joint 2 actuator drives arm 2.
    link3 = LINK_Create(link3Dim, BLUE, LINK_REVOLUTE, actuator2);

    // Joint 3 actuator drives vertical prismatic tool slide.
    link4 = LINK_Create(link4Dim, ORANGE, LINK_PRISMATIC, actuator3);

    // P controller for actuator velocity
    PID1_vel = Controller_create(&PIDController_Type, VELOCITY_CONTROLLER_NUM, VELOCITY_CONTROLLER_NUM_LEN, VELOCITY_CONTROLLER_DEN, VELOCITY_CONTROLLER_DEN_LEN);
    PID2_vel = Controller_create(&PIDController_Type, VELOCITY_CONTROLLER_NUM, VELOCITY_CONTROLLER_NUM_LEN, VELOCITY_CONTROLLER_DEN, VELOCITY_CONTROLLER_DEN_LEN);
    PID3_vel = Controller_create(&PIDController_Type, VELOCITY_CONTROLLER_NUM, VELOCITY_CONTROLLER_NUM_LEN, VELOCITY_CONTROLLER_DEN, VELOCITY_CONTROLLER_DEN_LEN);

    // P controller for actuator posion
    PID1_pos = Controller_create(&PIDController_Type, POSITION_CONTROLLER_NUM, POSITION_CONTROLLER_NUM_LEN, POSITION_CONTROLLER_DEN, POSITION_CONTROLLER_DEN_LEN);
    PID2_pos = Controller_create(&PIDController_Type, POSITION_CONTROLLER_NUM, POSITION_CONTROLLER_NUM_LEN, POSITION_CONTROLLER_DEN, POSITION_CONTROLLER_DEN_LEN);
    PID3_pos = Controller_create(&PIDController_Type, POSITION_CONTROLLER_NUM, POSITION_CONTROLLER_NUM_LEN, POSITION_CONTROLLER_DEN, POSITION_CONTROLLER_DEN_LEN);

    Controller controllers[] = {PID1_pos, PID2_pos, PID3_pos, PID1_vel, PID2_vel, PID3_vel,};
    Link links[ROBOT_NUM_LINKS] = {link1, link2, link3, link4};
    
    // setup scara robot with simple setup for testing
    SCARA = ROBOT_Create(controllers, links);

    // setup GUI panel with initial robot state
    GUI_SIM_PANEL_Init(&gui_sim_panel, SCARA);

    // set joint limits
    ROBOT_SetJointLimits(SCARA, jp_limits, jp_velocity_limits);

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

        //UpdateTestPoseCycle(SCARA, now);

        bool did_work = false;

        // -------------------------------------------------
        // 1 kHz Vel_PID/control loop
        // -------------------------------------------------
        int pid_steps = 0;

        while (now >= next_pid_time && pid_steps < MAX_PID_STEPS_PER_LOOP)
        {
            Pos_Ctrl_Update();

            next_pid_time += CTRL_DT;
            pid_steps++;
            did_work = true;
        }

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

    ROBOT_Destroy(SCARA);

    Controller_destroy(PID1_vel);
    Controller_destroy(PID2_vel);
    Controller_destroy(PID3_vel);

    Controller_destroy(PID1_pos);
    Controller_destroy(PID2_pos);
    Controller_destroy(PID3_pos);

    LINK_Destroy(link1);
    LINK_Destroy(link2);
    LINK_Destroy(link3);
    LINK_Destroy(link4);

    Actuator_dtor(actuator1);
    Actuator_dtor(actuator2);
    Actuator_dtor(actuator3);

    ZFilter_dtor(motor_plant1);
    ZFilter_dtor(motor_plant2);
    ZFilter_dtor(motor_plant3);

    Logger_shutdown();

    CloseWindow();
    

    return 0;
}

void Pos_Ctrl_Update(void){
    ROBOT_Update(SCARA, CTRL_DT);
}


// ---------------------------------------------------------
// Runs at 1 kHz
// Put PID, control, simulation, path math, etc. here.
// ---------------------------------------------------------
void Control_Update(void)
{
    

    float t = (float)GetTime();

    float joint1Angle = cosf(t * 0.25f * PI) * 90.0f * DEG2RAD;
    float joint2Angle = sinf(t * 0.25f * PI) * 90.0f * DEG2RAD;

    // Positive distance downward
    float joint3Slide =  (sinf(t * 0.5f * PI) -1) * 1.70/2;

    float link1Heading = 0.0f;
    float link2Heading = joint1Angle;
    float link3Heading = joint1Angle + joint2Angle;
    float link4Heading = joint1Angle + joint2Angle;

    LINK_SetHeadingWorld(link1, link1Heading);
    LINK_SetJointPosition(link1, 0.0f);

    LINK_SetHeadingWorld(link2, link2Heading);
    LINK_SetJointPosition(link2, joint1Angle);

    LINK_SetHeadingWorld(link3, link3Heading);
    LINK_SetJointPosition(link3, joint3Slide);

    LINK_SetHeadingWorld(link4, link4Heading);
    LINK_SetJointPosition(link4, 0.0f);

}

void UpdateTestPoseCycle(Robot robot, double now)
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
    float const Z_MID = 17.0f / 2.0f;
    float const Z_LOW = 17.0f;

    static const Vector3 test_poses[] = {
        // Home / neutral
        {  Rad30,    0.0f,     Z_TOP },
        {  Rad30,    Rad30,    Z_TOP },
        {  Rad30,    Rad30,    Z_LOW },

        // Joint 1 only
        {  Rad45,   0.0f,    Z_TOP },
        { -Rad45,   0.0f,    Z_LOW },
        {  Rad90,   0.0f,    Z_TOP },
        { -Rad90,   0.0f,    Z_LOW },

        // Joint 2 only
        {  0.0f,    Rad45,   Z_LOW },
        {  0.0f,   -Rad45,   Z_MID },
        {  0.0f,    Rad90,   Z_LOW },
        {  0.0f,   -Rad90,   Z_MID },

        // Same direction bends
        {  Rad45,   Rad45,   Z_TOP },
        { -Rad45,  -Rad45,   Z_MID },
        {  Rad90,   Rad45,   Z_TOP },
        { -Rad90,  -Rad45,   Z_MID },

        // Opposite direction bends
        {  Rad45,  -Rad45,   Z_LOW },
        { -Rad45,   Rad45,   Z_LOW },
        {  Rad90,  -Rad90,   Z_LOW },
        { -Rad90,   Rad90,   Z_MID },

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
        ROBOT_SetJointPositionTarget(robot, test_poses[pose_index]);
        next_pose_time = now + TEST_POSE_PERIOD_SEC;
        initialized = true;
        return;
    }

    if (now >= next_pose_time) {
        pose_index = (pose_index + 1) % (sizeof(test_poses) / sizeof((test_poses)[0]));

        ROBOT_SetJointPositionTarget(robot, test_poses[pose_index]);

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
    const float shaftRadius = 0.225f;
    const float headRadius  = 0.7f;
    const float headLength  = 2.20f;

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

#define PLOT_POINTS 200

void UpdateDrawFrame(void)
{
    static float phase = 0.0f;

    float x[PLOT_POINTS];
    float y[PLOT_POINTS];

    size_t count = PLOT_POINTS;

    // Generate sine wave
    for (size_t i = 0; i < count; i++)
    {
        x[i] = ((float)i / (float)(count - 1)) * 2.0f * PI;

        y[i] = sinf(x[i] + phase);
    }

    // Move phase from 0 -> 2PI
    phase += 0.03f;

    if (phase >= 2.0f * PI)
        phase -= 2.0f * PI;


    XYPlot plot = XYPlot_Create(
        (Vector2){ 1550, 300 },
        400,
        250
    );

    XYPlot_SetRange(
        plot,
        0.0f,
        2.0f * PI,
        -1.5f,
        1.5f
    );

    XYPlot_SetGrid(
        plot,
        PI / 2.0f,
        0.5f,
        5
    );

    XYPlot_SetLabels(
        plot,
        "Angle (rad)",
        "Amplitude"
    );


    // Update camera
    UpdateCamera(&camera, CAMERA_ORBITAL);

    // Update GUI
    GUI_SIM_PANEL_Update(&gui_sim_panel, SCARA);


    BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

            DrawGrid(30, 2.5f);
            DrawWorldAxes3D(50.0f);
            ROBOT_Draw(SCARA);

        EndMode3D();


        // Draw GUI
        GUI_SIM_PANEL_Draw(&gui_sim_panel);

        XYPlot_DrawPoint(plot, 2.5f, 0.75f, "Target", BLUE);


        // Draw moving sine wave
        XYPlot_Draw(
            plot,
            x,
            y,
            count,
            RED
        );

    EndDrawing();

}