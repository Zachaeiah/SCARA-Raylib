#ifndef GUI_SIM_PANEL_H_
#define GUI_SIM_PANEL_H_

#include "raylib.h"

#include "GUI/Layout/gui_layout.h"
#include "Robot/Robot/robot.h"
#include "Robot/Robot/robot_comand.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GUI_SIM_FIELD_TEXT_SIZE 32
#define GUI_SIM_AXIS_COUNT      3

#define GSP GuiSimPanel
typedef struct GSP *GSP;

extern const Except_t GUI_Failed;
extern const ErrorType GUI_Failed_ErrorCode;

/**
 * @brief Initialize the simulation panel.
 *
 * @param self Panel instance.
 * @param robot Robot instance.
 * @param rect Panel position and size.
 */
GSP GUI_SIM_PANEL_Init(Robot robot, Rectangle* rect );


/**
 * @brief Destroy resources owned by the panel.
 *
 * @param self Panel instance.
 */
void GUI_SIM_PANEL_Destroy(GSP self);


/**
 * @brief Draw the simulation panel.
 *
 * @param self Panel instance.
 */
void GUI_SIM_PANEL_Draw(GSP self);


/**
 * @brief Update robot state and process commands.
 *
 * @param self Panel instance.
 * @param robot Robot instance.
 */
void GUI_SIM_PANEL_Update(GSP self, Robot robot );

#undef GSP

#ifdef __cplusplus
}
#endif // __cplusplus

#endif