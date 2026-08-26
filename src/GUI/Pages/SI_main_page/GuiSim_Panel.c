#include "GuiSim_Panel.h"
#include "raygui.h"
#include "raymath.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"
#include "utils/Logger/logger.h"
#include "GUI/ploting/XY_plot/xy_plot.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FIELD_WIDTH 9
#define PRECISION 3

const Except_t GUI_Failed = {"GUI fiald"};

struct GuiSimPanel
{
    /*
     * Robot state
     */
    RobotState State;
    RobotState CommandedState;

    /*
     * Text entry
     */
    char joint_target_text[GUI_SIM_AXIS_COUNT][GUI_SIM_FIELD_TEXT_SIZE];
    char TCP_target_text[GUI_SIM_AXIS_COUNT][GUI_SIM_FIELD_TEXT_SIZE];

    bool joint_target_edit[GUI_SIM_AXIS_COUNT];
    bool TCP_target_edit[GUI_SIM_AXIS_COUNT];

    /*
     * GUI state
     */
    bool use_degrees;

    bool has_command_message;
    bool apply_joint_target_requested;
    bool TCP_joint_target_requested;

    /*
     * Layout
     */
    GuiLayout layout;


    GuiNodeID root;
    
    GuiNodeID L_Robot_report;

    GuiNodeID R_Robot_graphs;

    GuiNodeID Test_graph_section;

    GuiNodeID units_toggle;

    GuiNodeID state_section;

    GuiNodeID joint_section;
    GuiNodeID joint_fields[GUI_SIM_AXIS_COUNT];
    GuiNodeID joint_button;

    GuiNodeID tcp_section;
    GuiNodeID tcp_fields[GUI_SIM_AXIS_COUNT];
    GuiNodeID tcp_button;

};

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
 * @brief Converts joint position from radians/meters to display units (degrees/meters)
 * 
 * @param joint_rad_m Joint position in radians (for revolute joints) and meters (for prismatic joint)
 * @param use_degrees Whether to convert radians to degrees for revolute joints
 * @return Vector3 Joint position in display units (degrees for revolute joints, meters for prismatic joint)
 */
static Vector3 GUI_SIM_PANEL_JointToDisplay(Vector3 joint, bool use_degrees)
{
    Vector3 display = joint;

    if (use_degrees)
    {
        display.x *= RAD2DEG;
        display.y *= RAD2DEG;
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
    Vector3 joint = display;

    if (use_degrees)
    {
        joint.x *= DEG2RAD;
        joint.y *= DEG2RAD;
    }

    return joint;
}

/**
 * @brief 
 * 
 * @param panel 
 * @param parent 
 * @param label 
 * @param value 
 * @param y 
 */
static void GUI_SIM_PANEL_DrawVector3Row( const GuiSimPanel panel, GuiNodeID parent, const char* label, Vector3 value, float y)
{
    Vector2 label_pos = GuiNode_LocalToScreen( panel->layout, parent, (Vector2){ 0.0f, y });

    DrawText( label, (int)label_pos.x, (int)label_pos.y, 16, DARKGRAY );

    const float value_start = 165.0f;
    const float spacing = 75.0f;

    for (int i = 0; i < 3; i++)
    {
        float value_component;

        switch (i)
        {
            case 0: value_component = value.x; break;
            case 1: value_component = value.y; break;
            default: value_component = value.z; break;
        }

        Vector2 value_pos = GuiNode_LocalToScreen( panel->layout, parent, (Vector2) { value_start + spacing * (float)i, y } );

        DrawText( TextFormat( "%*.*f", FIELD_WIDTH, PRECISION, value_component ), (int)value_pos.x, (int)value_pos.y, 16, DARKGRAY );
    }
}


/**
 * @brief Synchronizes the target text fields with the current joint position target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_SyncTargetJointText(const GuiSimPanel panel)
{
    // Convert the commanded joint position to display units for text representation
    Vector3 display_target = GUI_SIM_PANEL_JointToDisplay(panel->CommandedState.joint_position, panel->use_degrees);

    // Update the text fields with the display values
    snprintf(panel->joint_target_text[0], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.x);
    snprintf(panel->joint_target_text[1], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.y);
    snprintf(panel->joint_target_text[2], GUI_SIM_FIELD_TEXT_SIZE, "%.*f", PRECISION, display_target.z);
}

/**
 * @brief Synchronizes the target text fields with the current joint position target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_SyncTargetTCPText(const GuiSimPanel panel)
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
static void GUI_SIM_PANEL_ClearJointTargetTextBoxFocus(GuiSimPanel panel)
{
    panel->joint_target_edit[0] = false;
    panel->joint_target_edit[1] = false;
    panel->joint_target_edit[2] = false;
}

/**
 * @brief Clears the focus from all target text boxes
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ClearTCPTargetTextBoxFocus(GuiSimPanel panel)
{
    panel->TCP_target_edit[0] = false;
    panel->TCP_target_edit[1] = false;
    panel->TCP_target_edit[2] = false;
}

/**
 * @brief Parses the target text values and updates the joint position target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ParseTargetJointText( GuiSimPanel panel)
{
    Vector3 fallback = GUI_SIM_PANEL_JointToDisplay( panel->CommandedState.joint_position, panel->use_degrees );

    Vector3 target;

    target.x = GUI_SIM_PANEL_ParseFloat( panel->joint_target_text[0], fallback.x );

    target.y = GUI_SIM_PANEL_ParseFloat( panel->joint_target_text[1], fallback.y );

    target.z = GUI_SIM_PANEL_ParseFloat( panel->joint_target_text[2], fallback.z );

    panel->CommandedState.joint_position = GUI_SIM_PANEL_JointFromDisplay( target, panel->use_degrees );
}

/**
 * @brief Parses the target text values and updates the TCP target
 * @param panel Pointer to the GuiSimPanel instance
 */
static void GUI_SIM_PANEL_ParseTargetTCPText(GuiSimPanel panel)
{
    Vector3 fallback = panel->CommandedState.tcp_position;

    Vector3 target;

    LOG_DEBUG_MSG(NO_ERROR, "Target to parce(X: %10s, Y:%10s, Z: %10s)", panel->TCP_target_text[0], panel->TCP_target_text[1], panel->TCP_target_text[2]);

    target.x = GUI_SIM_PANEL_ParseFloat( panel->TCP_target_text[0], fallback.x);

    target.y = GUI_SIM_PANEL_ParseFloat( panel->TCP_target_text[1], fallback.y );

    target.z = GUI_SIM_PANEL_ParseFloat( panel->TCP_target_text[2], fallback.z );

    LOG_DEBUG_MSG(NO_ERROR, "Parced target(X: %f, Y:%f, Z: %f)", target.x, target.y, target.z);

    panel->CommandedState.tcp_position = target;
}




/**
 * @brief 
 * 
 * @param panel 
 */
static void GUI_SIM_PANEL_DrawJointEditor( GuiSimPanel panel )
{
    static const char* labels[3] = { "J1", "J2", "J3" };

    for (int i = 0; i < GUI_SIM_AXIS_COUNT; i++)
    {
        Rectangle box = GuiNode_GetRect( panel->layout, panel->joint_fields[i] );

        DrawText( labels[i], (int)box.x, (int)box.y - 20, 16, DARKGRAY );

        if (GuiTextBox( box, panel->joint_target_text[i], GUI_SIM_FIELD_TEXT_SIZE, panel->joint_target_edit[i]))
        {
            panel->joint_target_edit[i] = !panel->joint_target_edit[i];

            if (panel->joint_target_edit[i])
            {
                for (int j = 0; j < GUI_SIM_AXIS_COUNT; j++)
                {
                    if (j != i){
                        panel->joint_target_edit[j] = false;
                    }
                }
            }
        }
    }
}

/**
 * @brief 
 * 
 * @param panel 
 */
static void GUI_SIM_PANEL_DrawTCPEditor( GuiSimPanel panel)
{
    static const char* labels[3] = { "TCP X", "TCP Y", "TCP Z"};

    for (int i = 0; i < GUI_SIM_AXIS_COUNT; i++)
    {
        Rectangle box = GuiNode_GetRect( panel->layout, panel->tcp_fields[i]);

        DrawText( labels[i], (int)box.x, (int)box.y - 20, 16, DARKGRAY );

        if (GuiTextBox( box, panel->TCP_target_text[i], GUI_SIM_FIELD_TEXT_SIZE, panel->TCP_target_edit[i]))
        {
            panel->TCP_target_edit[i] = !panel->TCP_target_edit[i];

            if (panel->TCP_target_edit[i])
            {
                for (int j = 0; j < GUI_SIM_AXIS_COUNT; j++)
                {
                    if (j != i){
                        panel->TCP_target_edit[j] = false;
                    }
                }
            }
        }
    }
}

static void GUI_SIM_PANEL_CreateLayout( GuiSimPanel panel, const Rectangle* panel_rect )
{
    /*
     * Root
     */
    panel->layout = GuiLayout_Create(panel_rect);

    panel->root = GuiLayout_GetRoot(panel->layout);

    GuiNode_SetPadding( panel->layout, panel->root, 15.0f );

    /**
     * all left side GUI
     */
    Rectangle Robot_report_rect = { 0.0f, 15.0f, 400.0f, 400.0f};

    panel->L_Robot_report = GuiLayout_CreateNode( panel->layout, panel->root, &Robot_report_rect );

    GuiNode_SetPadding(panel->layout, panel->L_Robot_report, 15.00f);

    /**
     * all right side GUI
     */
    Rectangle R_Robot_graphs_rect = { 1450.0f, 1.5f, 500.0f, 350.0f};

    panel->R_Robot_graphs = GuiLayout_CreateNode( panel->layout, panel->root, &R_Robot_graphs_rect );

    GuiNode_SetPadding(panel->layout, panel->L_Robot_report, 15.00f);

    /*
     * Robot state section
     */
    Rectangle state_rect = { 0.0f, 0.0f, 400.0f, 100.0f};

    panel->state_section = GuiLayout_CreateNode( panel->layout, panel->L_Robot_report, &state_rect );

    /*
     * Units toggle
     */
    Rectangle units_rect = { 0.0f, 100.0f, 75.0f, 25.0f};

    panel->units_toggle = GuiLayout_CreateNode( panel->layout, panel->L_Robot_report, &units_rect );

    /*
     * Joint target section
     */
    Rectangle joint_rect = { 0.0f, 135.0f, 400.0f, 110.0f };

    panel->joint_section = GuiLayout_CreateNode( panel->layout, panel->L_Robot_report, &joint_rect );

    GuiNode_SetPadding( panel->layout, panel->joint_section, 10.0f );


    /*
     * Joint input boxes
     */
    for (int i = 0; i < GUI_SIM_AXIS_COUNT; i++)
    {
        // each
        Rectangle field_rect = { (float)i * 100.0f,  30.0f, 80.0f, 25.0f};

        panel->joint_fields[i] = GuiLayout_CreateNode( panel->layout, panel->joint_section, &field_rect );
    }


    /*
     * Joint command button
     */
    Rectangle joint_button_rect = { 0.0f, 65.0f, 330.0f, 30.0f };

    panel->joint_button = GuiLayout_CreateNode( panel->layout, panel->joint_section, &joint_button_rect );


    /*
     * TCP target section
     */
    Rectangle tcp_rect = {0.0f, 250.0f, 400.0f, 110.0f };

    panel->tcp_section = GuiLayout_CreateNode( panel->layout, panel->L_Robot_report, &tcp_rect);

    GuiNode_SetPadding(panel->layout, panel->tcp_section, 10.0f );


    /*
     * TCP input boxes
     */
    for (int i = 0; i < GUI_SIM_AXIS_COUNT; i++)
    {
        Rectangle field_rect = { (float)i * 100.0f, 30.0f, 80.0f, 25.0f };

        panel->tcp_fields[i] = GuiLayout_CreateNode( panel->layout,  panel->tcp_section, &field_rect );
    }

    /*
     * TCP command button
     */
    Rectangle tcp_button_rect = { 0.0f, 65.0f, 330.0f, 30.0f };

    panel->tcp_button = GuiLayout_CreateNode( panel->layout, panel->tcp_section, &tcp_button_rect );

     /*
     * Graphs test layout
     */

    Rectangle Test_graph_rect = { 50.0f, 275.0f, 400.0f, 400.0f };

    panel->Test_graph_section =  GuiLayout_CreateNode( panel->layout, panel->R_Robot_graphs, &Test_graph_rect );

}

GuiSimPanel GUI_SIM_PANEL_Init(Robot robot, Rectangle* rect)
{


    assert(robot);
    assert(rect);
    
    GuiSimPanel panel;

    TRY
    {
        NEW0(panel);
    }
    EXCEPT(Mem_Failed)
    {
        LOG_ERROR_MSG(GUI_Failed_ErrorCode, "Memory allocation failed for GUI");
        RAISE(GUI_Failed);
        return NULL;

    } END_TRY;

    ROBOT_GetState( robot, &panel->State );

    panel->CommandedState = panel->State;

    panel->use_degrees = true;
    GUI_SIM_PANEL_CreateLayout(panel, rect );
    GUI_SIM_PANEL_SyncTargetJointText(panel);
    GUI_SIM_PANEL_SyncTargetTCPText(panel);

    return panel;
}

void GUI_SIM_PANEL_Draw( GuiSimPanel panel )
{
    if (!panel)
    {
        RAISE(NullptrError);
        return;
    }


    /* --------------------------------------------------------
     * Left Panel
     * -------------------------------------------------------- */

    Rectangle L_panel_rect = GuiNode_GetRect( panel->layout, panel->L_Robot_report );

    DrawRectangleRec( L_panel_rect, LIGHTGRAY );

    GuiGroupBox( L_panel_rect, "Robot State" );


    /* --------------------------------------------------------
     * Degrees / radians
     * -------------------------------------------------------- */

    bool old_use_degrees = panel->use_degrees;

    Rectangle toggle_rect =  GuiNode_GetRect( panel->layout, panel->units_toggle );

    GuiToggle( toggle_rect, panel->use_degrees ? "Degrees" : "Radians", &panel->use_degrees );

    if (old_use_degrees != panel->use_degrees)
    {
        GUI_SIM_PANEL_ParseTargetJointText(panel);

        GUI_SIM_PANEL_ClearJointTargetTextBoxFocus(panel);

        GUI_SIM_PANEL_SyncTargetJointText(panel);
    }


    /* --------------------------------------------------------
     * Current state
     * -------------------------------------------------------- */

    Vector3 joint_position = GUI_SIM_PANEL_JointToDisplay( panel->State.joint_position, panel->use_degrees );

    Vector3 joint_velocity = GUI_SIM_PANEL_JointToDisplay( panel->State.joint_velocity, panel->use_degrees );

    GUI_SIM_PANEL_DrawVector3Row( panel, panel->state_section, "TCP Pos:", panel->State.tcp_position, 0.0f );

    GUI_SIM_PANEL_DrawVector3Row( panel, panel->state_section, "TCP Vel:", panel->State.tcp_velocity, 25.0f );

    GUI_SIM_PANEL_DrawVector3Row( panel,  panel->state_section, panel->use_degrees ? "Joint Pos deg:" : "Joint Pos rad:", joint_position,50.0f );

    GUI_SIM_PANEL_DrawVector3Row( panel, panel->state_section, panel->use_degrees ? "Joint Vel deg/s:" : "Joint Vel rad/s:", joint_velocity, 75.0f );


    /* --------------------------------------------------------
     * Joint position targeting
     * -------------------------------------------------------- */

    Rectangle joint_section = GuiNode_GetRect( panel->layout, panel->joint_section );

    GuiLine((Rectangle){ joint_section.x, joint_section.y, 330.0f, 10.0f}, "Joint Position Targeting" );

    GUI_SIM_PANEL_DrawJointEditor(panel);

    if (GuiButton( GuiNode_GetRect( panel->layout, panel->joint_button), "Set Joint Position Target" ))
    {
        GUI_SIM_PANEL_ParseTargetJointText(panel);

        GUI_SIM_PANEL_ClearJointTargetTextBoxFocus(panel);

        GUI_SIM_PANEL_SyncTargetJointText(panel);

        panel->apply_joint_target_requested = true;
    }


    /* --------------------------------------------------------
     * TCP position targeting
     * -------------------------------------------------------- */

    Rectangle tcp_section = GuiNode_GetRect( panel->layout, panel->tcp_section );

    GuiLine((Rectangle) { tcp_section.x, tcp_section.y, 330.0f, 10.0f }, "TCP Position Targeting" );

    GUI_SIM_PANEL_DrawTCPEditor(panel);

    if (GuiButton( GuiNode_GetRect( panel->layout, panel->tcp_button ), "Set TCP Position Target" ))
    {
        GUI_SIM_PANEL_ParseTargetTCPText(panel);

        GUI_SIM_PANEL_ClearTCPTargetTextBoxFocus(panel);

        GUI_SIM_PANEL_SyncTargetTCPText(panel);

        panel->TCP_joint_target_requested = true;
    }

    /* --------------------------------------------------------
     * right Panel
     * -------------------------------------------------------- */

    Rectangle R_panel_rect = GuiNode_GetRect( panel->layout, panel->R_Robot_graphs );

    DrawRectangleRec( R_panel_rect, LIGHTGRAY );

    GuiGroupBox( R_panel_rect, "Robot graphs");



    /* --------------------------------------------------------
     * Test graohs
     * -------------------------------------------------------- */

     Rectangle Test_graph_rect = GuiNode_GetRect( panel->layout, panel->Test_graph_section );

    #define PLOT_POINTS 200

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

    if (phase >= 2.0f * PI) phase -= 2.0f * PI;

    XYPlot plot = XYPlot_Create( (Rectangle ){ Test_graph_rect.x, Test_graph_rect.y , 400, 250} );

    XYPlot_SetRange( plot, 0.0f, 2.0f * PI, -1.5f, 1.5f );

    XYPlot_SetGrid( plot, PI / 2.0f, 0.5f, 5 );

    XYPlot_SetLabels( plot, "Angle (rad)", "Amplitude" );

    XYPlot_DrawPoint(plot, 2.5f, 0.75f, "Target", BLUE);

    XYPlot_Draw(plot, x, y, count, RED );


}

void GUI_SIM_PANEL_Destroy( GuiSimPanel panel )
{
    assert(panel);

    GuiLayout_Destroy( &panel->layout );

    FREE(panel);

}


void GUI_SIM_PANEL_Update( GuiSimPanel panel, Robot robot )
{
    if (!panel)
    {
        RAISE(NullptrError);
        return;
    }

    if (!robot)
    {
        RAISE(NullptrError);
        return;
    }


    /*
     * Update current robot state.
     */
    ROBOT_GetState( robot, &panel->State);


    /*
     * Joint command.
     */
    if (panel->apply_joint_target_requested)
    {
        LOG_MESSAGE(
            "Joint target: %.3f, %.3f, %.3f\n",
            panel->CommandedState.joint_position.x,
            panel->CommandedState.joint_position.y,
            panel->CommandedState.joint_position.z
        );

        RobotCommandStatus status = ROBOT_SetJointPositionTarget( robot, panel->CommandedState.joint_position );

        if (status == ROBOT_COMMAND_REJECTED)
        {
            LOG_ERROR_MSG(
                JP_CMD_REJECTED_ErrorCode,
                "Joint position target command rejected.\n"
            );
        }
        else if (status == ROBOT_COMMAND_CLAMPED)
        {
            LOG_WARN_MSG(
                JP_CMD_CLAMPED_ErrorCode,
                "Joint position target command clamped.\n"
            );
        }

        panel->apply_joint_target_requested = false;
    }


    /*
     * TCP command.
     */
    if (panel->TCP_joint_target_requested)
    {
        LOG_MESSAGE(
            "TCP target: %.3f, %.3f, %.3f\n",
            panel->CommandedState.tcp_position.x,
            panel->CommandedState.tcp_position.y,
            panel->CommandedState.tcp_position.z
        );

        RobotCommandStatus status = ROBOT_SetTCPPositionTarget( robot, panel->CommandedState.tcp_position);

        

        if (status == ROBOT_COMMAND_REJECTED)
        {
            LOG_ERROR_MSG(
                TCP_CMD_REJECTED_ErrorCode,
                "TCP position target command rejected.\n"
            );
        }
        else if (status == ROBOT_COMMAND_CLAMPED)
        {
            LOG_WARN_MSG(
                TCP_CMD_CLAMPED_ErrorCode,
                "TCP position target command clamped.\n"
            );
        }

        panel->TCP_joint_target_requested = false;
    }
}