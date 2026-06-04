// GuiSimPanel.h
#ifndef GUI_SIM_PANEL_H_
#define GUI_SIM_PANEL_H_

#include "raylib.h"

#include "Robot/Robot/robot.h"

#define GUI_SIM_FIELD_TEXT_SIZE 32

typedef struct GuiSimPanel {
    RobotState State;
    RobotState CommandedState;

    char joint_target_x_text[GUI_SIM_FIELD_TEXT_SIZE];
    char joint_target_y_text[GUI_SIM_FIELD_TEXT_SIZE];
    char joint_target_z_text[GUI_SIM_FIELD_TEXT_SIZE];

    bool joint_target_x_edit;
    bool joint_target_y_edit;
    bool joint_target_z_edit;

    bool apply_joint_target_requested;

    bool use_degrees;
} GuiSimPanel;

/**
 * @brief Initializes the simulation panel
 * 
 * @param self pointer to the GuiSimPanel instance to initialize
 * @param robot pointer to the Robot instance
 */
extern void GUI_SIM_PANEL_Init(GuiSimPanel* self, Robot* robot);

/**
 * @brief Draws the simulation panel
 * 
 * @param self pointer to the GuiSimPanel instance to draw
 */
extern void GUI_SIM_PANEL_Draw(GuiSimPanel* self);

/**
 * @brief Updates the simulation panel
 * 
 * @param self pointer to the GuiSimPanel instance to update
 * @param robot pointer to the Robot instance
 */
extern void GUI_SIM_PANEL_Update(GuiSimPanel* self, Robot* robot);

#endif