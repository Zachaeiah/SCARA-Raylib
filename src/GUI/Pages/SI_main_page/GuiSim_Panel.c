#include "GuiSim_Panel.h"
#include "raygui.h"
#include "raymath.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"
#include "utils/Logger/logger.h"

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
static void GUI_SIM_PANEL_SyncTargetJointText(GuiSimPanel* panel)
{
    // Convert the commanded joint position to display units for text representation
    Vector3 display_target =
        GUI_SIM_PANEL_JointToDisplay(panel->CommandedState.joint_position, panel->use_degrees);

    // Update the text fields with the display values
    snprintf(panel->joint_target_text[0], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.x);
    snprintf(panel->joint_target_text[1], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.y);
    snprintf(panel->joint_target_text[2], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.z);
}

/**
 * @brief Synchronizes the target text fields with the current joint position target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_SyncTargetTCPText(GuiSimPanel* panel)
{
    // Convert the commanded joint position to display units for text representation
    Vector3 display_target = panel->CommandedState.tcp_position;

    // Update the text fields with the display values
    snprintf(panel->TCP_target_text[0], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.x);
    snprintf(panel->TCP_target_text[1], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.y);
    snprintf(panel->TCP_target_text[2], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.z);
}



/**
 * @brief Clears the focus from all target text boxes
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ClearJointTargetTextBoxFocus(GuiSimPanel* panel)
{
    panel->joint_target_edit[0] = false;
    panel->joint_target_edit[1] = false;
    panel->joint_target_edit[2] = false;
}

/**
 * @brief Clears the focus from all target text boxes
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ClearTCPTargetTextBoxFocus(GuiSimPanel* panel)
{
    panel->TCP_target_edit[0] = false;
    panel->TCP_target_edit[1] = false;
    panel->TCP_target_edit[2] = false;
}

/**
 * @brief Parses the target text values and updates the joint position target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ParseTargetJointText(GuiSimPanel* panel)
{
    // If parsing fails for any field, fallback to the current commanded joint position for that field
    Vector3 fallback_display =
        GUI_SIM_PANEL_JointToDisplay(panel->CommandedState.joint_position, panel->use_degrees);

    Vector3 display_target;

    // Parse each field with fallback
    // For joint 1, if parsing fails, fallback to the current commanded joint position for joint 1
    display_target.x =
        GUI_SIM_PANEL_ParseFloat(panel->joint_target_text[0], fallback_display.x);

    // For joint 2, if parsing fails, fallback to the current commanded joint position for joint 2
    display_target.y =
        GUI_SIM_PANEL_ParseFloat(panel->joint_target_text[1], fallback_display.y);

    // For joint 3, if parsing fails, fallback to the current commanded joint position for joint 3
    display_target.z =
        GUI_SIM_PANEL_ParseFloat(panel->joint_target_text[2], fallback_display.z);

    // Update the commanded joint position target based on the parsed display values
    panel->CommandedState.joint_position =
        GUI_SIM_PANEL_JointFromDisplay(display_target, panel->use_degrees);
}

/**
 * @brief Parses the target text values and updates the TCP target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ParseTargetTCPText(GuiSimPanel* panel)
{
    // If parsing fails for any field, fallback to the current commanded joint position for that field
    Vector3 fallback_display = panel->CommandedState.tcp_position;

    Vector3 display_target;

    // Parse each field with fallback
    // For joint 1, if parsing fails, fallback to the current commanded joint position for joint 1
    display_target.x =
        GUI_SIM_PANEL_ParseFloat(panel->TCP_target_text[0], fallback_display.x);

    // For joint 2, if parsing fails, fallback to the current commanded joint position for joint 2
    display_target.y =
        GUI_SIM_PANEL_ParseFloat(panel->TCP_target_text[1], fallback_display.y);

    // For joint 3, if parsing fails, fallback to the current commanded joint position for joint 3
    display_target.z =
        GUI_SIM_PANEL_ParseFloat(panel->TCP_target_text[2], fallback_display.z);

    // Update the commanded joint position target based on the parsed display values
    panel->CommandedState.tcp_position =display_target; 
}

/**
 * @brief Draws the joint target editor
 * @param panel Pointer to the GuiSimPanel instance
 * @param posY The y-coordinate of the top-left corner of the joint target editor
 */
static void GUI_SIM_PANEL_DrawJointTargetEditor(GuiSimPanel* panel, int posY)
{

    int paddingY = 20;

    DrawText("J1", 25, posY, 16, DARKGRAY);
    DrawText("J2", 125, posY, 16, DARKGRAY);
    DrawText("J3", 225, posY, 16, DARKGRAY);

    for (int i = 0; i < 3; i++) {
        Rectangle box = {
            25.0f + (float)i * 100.0f,
            (float)posY + paddingY,
            80.0f,
            25.0f
        };

        if (GuiTextBox(
            box,
            panel->joint_target_text[i],
            GUI_SIM_FIELD_TEXT_SIZE,
            panel->joint_target_edit[i]
        )) {
            panel->joint_target_edit[i] = !panel->joint_target_edit[i];

            if (panel->joint_target_edit[i]) {
                for (int j = 0; j < 3; j++) {
                    if (j != i) {
                        panel->joint_target_edit[j] = false;
                    }
                }
            }
        }
    }
}

/**
 * @brief Draws the joint target editor
 * @param panel Pointer to the GuiSimPanel instance
 * @param posY The y-coordinate of the top-left corner of the TCP target editor
 */
static void GUI_SIM_PANEL_DrawTCPTargetEditor(GuiSimPanel* panel, int posY)
{
    int paddingY = 20;

    DrawText("TCP X", 25, posY, 16, DARKGRAY);
    DrawText("TCP Y", 125, posY, 16, DARKGRAY);
    DrawText("TCP Z", 225, posY, 16, DARKGRAY);

    for (int i = 0; i < 3; i++) {
        Rectangle box = {
            25.0f + (float)i * 100.0f,
            (float)posY + paddingY,
            80.0f,
            25.0f
        };

        if (GuiTextBox(
            box,
            panel->TCP_target_text[i],
            GUI_SIM_FIELD_TEXT_SIZE,
            panel->TCP_target_edit[i]
        )) {
            panel->TCP_target_edit[i] = !panel->TCP_target_edit[i];

            if (panel->TCP_target_edit[i]) {
                for (int j = 0; j < 3; j++) {
                    if (j != i) {
                        panel->TCP_target_edit[j] = false;
                    }
                }
            }
        }
    }
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

    GUI_SIM_PANEL_SyncTargetJointText(panel);

    GUI_SIM_PANEL_SyncTargetTCPText(panel);
}

void GUI_SIM_PANEL_Draw(GuiSimPanel* panel)
{
    if (!panel) {
        RAISE(NullptrError);
        return;
    }

    Rectangle panel_rect = { 10, 10, 430, 430 };

    // Draw panel background
    DrawRectangleRec(panel_rect, LIGHTGRAY);

    // Draw panel border and title
    GuiGroupBox(panel_rect, "Robot State");

    // Toggle for degrees/radians
    bool old_use_degrees = panel->use_degrees;

    GuiToggle(
        (Rectangle){ 25, 30, 110, 25 },
        panel->use_degrees ? "Degrees" : "Radians",
        &panel->use_degrees
    );

    if (old_use_degrees != panel->use_degrees) {
        GUI_SIM_PANEL_ParseTargetJointText(panel);
        GUI_SIM_PANEL_ClearJointTargetTextBoxFocus(panel);
        GUI_SIM_PANEL_SyncTargetJointText(panel);
    }

    // Display robot state
    GUI_SIM_PANEL_DrawVector3Row("TCP Position (mm):",   panel->State.tcp_position,   70);
    GUI_SIM_PANEL_DrawVector3Row("TCP Velocity (mm/s):", panel->State.tcp_velocity,   95);

    // Convert joint positions and velocities to display units based on the current use_degrees setting
    Vector3 joint_position_display =
        GUI_SIM_PANEL_JointToDisplay(panel->State.joint_position, panel->use_degrees);

    // For joint velocities, if use_degrees is true, convert from radians/s to degrees/s for revolute joints, but keep prismatic joint velocity in mm/s
    Vector3 joint_velocity_display =
        GUI_SIM_PANEL_JointToDisplay(panel->State.joint_velocity, panel->use_degrees);

    // Display joint positions and velocities with appropriate units based on the use_degrees setting
    if (panel->use_degrees) {
        GUI_SIM_PANEL_DrawVector3Row("Joint Pos deg:",   joint_position_display, 120);
        GUI_SIM_PANEL_DrawVector3Row("Joint Vel deg/s:", joint_velocity_display, 145);
    } else {
        GUI_SIM_PANEL_DrawVector3Row("Joint Pos rad:",   joint_position_display, 120);
        GUI_SIM_PANEL_DrawVector3Row("Joint Vel rad/s:", joint_velocity_display, 145);
    }
    
    // Draw joint position target editor
    GuiLine((Rectangle){ 25, 170, 330, 10 }, "Joint Position Targeting");

    // Draw the joint target editor fields
    GUI_SIM_PANEL_DrawJointTargetEditor(panel, 200);

    // Draw the "Set Joint Position Target" button and handle its click event
    if (GuiButton((Rectangle){ 25, 265, 330, 30 }, "Set Joint Position Target")) {
        GUI_SIM_PANEL_ParseTargetJointText(panel);
        GUI_SIM_PANEL_ClearJointTargetTextBoxFocus(panel);
        GUI_SIM_PANEL_SyncTargetJointText(panel);

        panel->apply_joint_target_requested = true;
    }

    GuiLine((Rectangle){ 25, 2300, 330, 10 }, "TCP Position Targeting"); // Separator line at the bottom of the panel

    GUI_SIM_PANEL_DrawTCPTargetEditor(panel, 325);

    // Draw the "Set TCP Position Target" button and handle its click event
    if (GuiButton((Rectangle){ 25, 390, 330, 30 }, "Set TCP Position Target")) {
        GUI_SIM_PANEL_ParseTargetTCPText(panel);
        GUI_SIM_PANEL_ClearTCPTargetTextBoxFocus(panel);
        GUI_SIM_PANEL_SyncTargetTCPText(panel);

        panel->TCP_joint_target_requested = true;
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
        LOG_MESSAGE("Joint Target Requested\n");
        LOG_MESSAGE("Joint target: %.3f, %.3f, %.3f\n",
            panel->CommandedState.joint_position.x,
            panel->CommandedState.joint_position.y,
            panel->CommandedState.joint_position.z
        );

        RobotCommandStatus status = ROBOT_SetJointPositionTarget(robot, panel->CommandedState.joint_position);

        if (status == ROBOT_COMMAND_REJECTED) {
            LOG_ERROR_MSG(JP_CMD_REJECTED_ErrorCode, "Joint position target command was rejected. Target may be out of bounds.\n");
        }
        else if (status == ROBOT_COMMAND_CLAMPED) {
            LOG_WARN_MSG(JP_CMD_CLAMPED_ErrorCode, "Joint position target command was clamped. Target is near bounds.\n");
        }

    
        panel->apply_joint_target_requested = false;
    }

    if (panel->TCP_joint_target_requested) {

        LOG_MESSAGE("TCP Target Requested\n");
        LOG_MESSAGE("TCP target: %.3f, %.3f, %.3f\n",
            panel->CommandedState.tcp_position.x,
            panel->CommandedState.tcp_position.y,
            panel->CommandedState.tcp_position.z
        );

        RobotCommandStatus status = ROBOT_SetTCPPositionTarget(robot, panel->CommandedState.tcp_position);

        if (status == ROBOT_COMMAND_REJECTED) {
            LOG_ERROR_MSG(TCP_CMD_REJECTED_ErrorCode, "TCP position target command was rejected. Target may be out of bounds.\n");
        }
        else if (status == ROBOT_COMMAND_CLAMPED) {
            LOG_WARN_MSG(TCP_CMD_CLAMPED_ErrorCode, "TCP position target command was clamped. Target is near bounds.\n");
        }

        panel->TCP_joint_target_requested = false;
    }
}