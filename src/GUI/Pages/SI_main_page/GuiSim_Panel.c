#include "GuiSim_Panel.h"
#include "raygui.h"
#include "raymath.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FIELD_WIDTH 9
#define PRECISION 3

/**
 * @brief Parses a float from a string
 * @param text The string to parse
 * @param fallback The fallback value if parsing fails
 * @return The parsed float or the fallback value
 */
static float GUI_SIM_PANEL_ParseFloat(const char* text, float fallback)
{
    if (!text) {
        return fallback;
    }

    char* end = NULL;
    float value = strtof(text, &end);

    if (end == text) {
        return fallback;
    }

    return value;
}

/**
 * @brief Draws a row for displaying a Vector3 value
 * @param label The label for the row
 * @param value The Vector3 value to display
 * @param y The y-coordinate for the row
 */
static void GUI_SIM_PANEL_DrawVector3Row(
    const char* label,
    Vector3 value,
    int y
)
{
    DrawText(label, 25, y, 16, DARKGRAY);
    int Spaceing = 80;
    int Row1X = 110 + Spaceing;
    int Row2X = Row1X + Spaceing;
    int Row3X = Row2X + Spaceing;

    DrawText(
        TextFormat("%*.*f", FIELD_WIDTH, PRECISION, value.x),
        Row1X,
        y,
        16,
        DARKGRAY
    );

    DrawText(
        TextFormat("%*.*f", FIELD_WIDTH, PRECISION, value.y),
        Row2X,
        y,
        16,
        DARKGRAY
    );

    DrawText(
        TextFormat("%*.*f", FIELD_WIDTH, PRECISION, value.z),
        Row3X,
        y,
        16,
        DARKGRAY
    );
}

/**
 * @brief Converts joint position from radians/meters to display units (degrees/meters)
 * 
 * @param joint_rad_m Joint position in radians (for revolute joints) and meters (for prismatic joint)
 * @param use_degrees Whether to convert radians to degrees for revolute joints
 * @return Vector3 Joint position in display units (degrees for revolute joints, meters for prismatic joint)
 */
static Vector3 GUI_SIM_PANEL_JointToDisplay(Vector3 joint_rad_m, bool use_degrees)
{
    Vector3 display = joint_rad_m;

    if (use_degrees) {
        display.x *= RAD2DEG;
        display.y *= RAD2DEG;
        // display.z stays meters
    }

    return display;
}

/**
 * @brief Converts joint position from display units (degrees/meters) to radians/meters
 * 
 * @param display Joint position in display units (degrees for revolute joints, meters for prismatic joint)
 * @param use_degrees Whether the input is in degrees (true) or radians (false)
 * @return Vector3 Joint position in radians (for revolute joints) and meters (for prismatic joint)
 */
static Vector3 GUI_SIM_PANEL_JointFromDisplay(Vector3 display, bool use_degrees)
{
    Vector3 joint_rad_m = display;

    if (use_degrees) {
        joint_rad_m.x *= DEG2RAD;
        joint_rad_m.y *= DEG2RAD;
        // joint_rad_m.z stays meters
    }

    return joint_rad_m;
}

/**
 * @brief Synchronizes the target text fields with the current joint position target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_SyncTargetText(GuiSimPanel* panel)
{
    Vector3 display_target =
        GUI_SIM_PANEL_JointToDisplay(panel->CommandedState.joint_position, panel->use_degrees);

    snprintf(panel->joint_target_x_text, GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.x);
    snprintf(panel->joint_target_y_text, GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.y);
    snprintf(panel->joint_target_z_text, GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.z);
}


/**
 * @brief Clears the focus from all target text boxes
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ClearTargetTextBoxFocus(GuiSimPanel* panel)
{
    panel->joint_target_x_edit = false;
    panel->joint_target_y_edit = false;
    panel->joint_target_z_edit = false;
}

/**
 * @brief Parses the target text values and updates the joint position target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ParseTargetText(GuiSimPanel* panel)
{
    Vector3 fallback_display =
        GUI_SIM_PANEL_JointToDisplay(panel->CommandedState.joint_position, panel->use_degrees);

    Vector3 display_target;

    display_target.x =
        GUI_SIM_PANEL_ParseFloat(panel->joint_target_x_text, fallback_display.x);

    display_target.y =
        GUI_SIM_PANEL_ParseFloat(panel->joint_target_y_text, fallback_display.y);

    display_target.z =
        GUI_SIM_PANEL_ParseFloat(panel->joint_target_z_text, fallback_display.z);

    panel->CommandedState.joint_position =
        GUI_SIM_PANEL_JointFromDisplay(display_target, panel->use_degrees);
}

void GUI_SIM_PANEL_Init(GuiSimPanel* panel, Robot* robot)
{
    if (!panel) {
        RAISE(NullptrError);
        return;
    }

    if (!robot) {
        RAISE(NullptrError);
        return;
    }

    memset(panel, 0, sizeof(GuiSimPanel));

    ROBOT_GetState(robot, &panel->State);

    panel->CommandedState.joint_position = panel->State.joint_position;

    panel->use_degrees = true;

    GUI_SIM_PANEL_SyncTargetText(panel);
}

void GUI_SIM_PANEL_Draw(GuiSimPanel* panel)
{
    if (!panel) {
        RAISE(NullptrError);
        return;
    }

    Rectangle panel_rect = { 10, 10, 430, 330 };

    GuiGroupBox(panel_rect, "Robot State");

    bool old_use_degrees = panel->use_degrees;

    GuiToggle(
        (Rectangle){ 25, 30, 110, 25 },
        panel->use_degrees ? "Degrees" : "Radians",
        &panel->use_degrees
    );

    if (old_use_degrees != panel->use_degrees) {
        GUI_SIM_PANEL_ParseTargetText(panel);
        GUI_SIM_PANEL_ClearTargetTextBoxFocus(panel);
        GUI_SIM_PANEL_SyncTargetText(panel);
    }


    GUI_SIM_PANEL_DrawVector3Row("TCP Position (mm):",   panel->State.tcp_position,   70);
    GUI_SIM_PANEL_DrawVector3Row("TCP Velocity (mm/s):", panel->State.tcp_velocity,   95);

    Vector3 joint_position_display =
        GUI_SIM_PANEL_JointToDisplay(panel->State.joint_position, panel->use_degrees);

    Vector3 joint_velocity_display =
        GUI_SIM_PANEL_JointToDisplay(panel->State.joint_velocity, panel->use_degrees);

    if (panel->use_degrees) {
        GUI_SIM_PANEL_DrawVector3Row("Joint Pos deg:",   joint_position_display, 120);
        GUI_SIM_PANEL_DrawVector3Row("Joint Vel deg/s:", joint_velocity_display, 145);
    } else {
        GUI_SIM_PANEL_DrawVector3Row("Joint Pos rad:",   joint_position_display, 120);
        GUI_SIM_PANEL_DrawVector3Row("Joint Vel rad/s:", joint_velocity_display, 145);
    }
    
    
    GuiLine((Rectangle){ 25, 170, 330, 10 }, "Joint Position Target");

    DrawText("J1", 25, 200, 16, DARKGRAY);
    DrawText("J2", 125, 200, 16, DARKGRAY);
    DrawText("J3", 225, 200, 16, DARKGRAY);

    if (GuiTextBox(
        (Rectangle){ 25, 220, 80, 25 },
        panel->joint_target_x_text,
        GUI_SIM_FIELD_TEXT_SIZE,
        panel->joint_target_x_edit
    )) 
    {
        panel->joint_target_x_edit = !panel->joint_target_x_edit;

        if (panel->joint_target_x_edit) {
            panel->joint_target_y_edit = false;
            panel->joint_target_z_edit = false;
        }
    }

    if (GuiTextBox(
            (Rectangle){ 125, 220, 80, 25 },
            panel->joint_target_y_text,
            GUI_SIM_FIELD_TEXT_SIZE,
            panel->joint_target_y_edit
        )) 
        {
        panel->joint_target_y_edit = !panel->joint_target_y_edit;

        if (panel->joint_target_y_edit) {
            panel->joint_target_x_edit = false;
            panel->joint_target_z_edit = false;
        }
    }

    if (GuiTextBox(
            (Rectangle){ 225, 220, 80, 25 },
            panel->joint_target_z_text,
            GUI_SIM_FIELD_TEXT_SIZE,
            panel->joint_target_z_edit
        )) 
        {
        panel->joint_target_z_edit = !panel->joint_target_z_edit;

        if (panel->joint_target_z_edit) {
            panel->joint_target_x_edit = false;
            panel->joint_target_y_edit = false;
        }
    }

    if (GuiButton((Rectangle){ 25, 265, 330, 30 }, "Set Joint Position Target")) {
        GUI_SIM_PANEL_ParseTargetText(panel);
        GUI_SIM_PANEL_ClearTargetTextBoxFocus(panel);
        GUI_SIM_PANEL_SyncTargetText(panel);

        panel->apply_joint_target_requested = true;
    }
}

void GUI_SIM_PANEL_Update(GuiSimPanel* panel, Robot* robot)
{
    if (!panel) {
        RAISE(NullptrError);
        return;
    }

    if (!robot) {
        RAISE(NullptrError);
        return;
    }

    ROBOT_GetState(robot, &panel->State);

    if (panel->apply_joint_target_requested) {
        //ROBOT_SetJointPositionTarget(robot, panel->joint_position_target);

        panel->CommandedState.joint_position = ROBOT_ApplyJointPositionLimits(robot);

        printf("Applying joint position target: %f, %f, %f\n",
            panel->CommandedState.joint_position.x,
            panel->CommandedState.joint_position.y,
            panel->CommandedState.joint_position.z
        );
        panel->apply_joint_target_requested = false;
    }
}