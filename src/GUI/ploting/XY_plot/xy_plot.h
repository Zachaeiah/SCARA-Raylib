#ifndef XY_PLOT_H
#define XY_PLOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "raylib.h"
#include <stddef.h>
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/MemAllocator/mem.h"

extern const Except_t XYPLOT_Failed;
extern const ErrorType XYPLOT_Failed_ErrorCode;

/**
 * @brief Represents a 2D XY plot with specified bounds and ranges for the x and y axes.
 * 
 */
#define PL XYPlot
typedef struct PL *PL;

/**
 * @brief Creates and initializes an XY plot.
 *
 * @param Rect
 *
 * @return Pointer to the created plot, or NULL on failure.
 */
PL XYPlot_Create(Rectangle Rect);


/**
 * @brief Destroys an XY plot.
 *
 * @param plot Plot to destroy.
 */
void XYPlot_Destroy(PL plot);

/**
 * @brief Sets the grid spacing for the x and y axes of the XY plot.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param xMajor The major division spacing for the x-axis.
 * @param yMajor The major division spacing for the y-axis.
 * @param minorDivisions The number of minor divisions between major divisions.
 */
void XYPlot_SetGrid(PL plot, float xMajor, float yMajor, int minorDivisions);

/**
 * @brief Draw a point of interest on the plot
 *
 * @param plot  Plot to draw on
 * @param x     X coordinate
 * @param y     Y coordinate
 * @param label Optional label, NULL for no label
 * @param color Point color
 */
void XYPlot_DrawPoint(const PL plot, float x, float y, const char* label, Color color);

/**
 * @brief Sets the labels for the x and y axes of the XY plot.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param xLabel The label for the x-axis.
 * @param yLabel The label for the y-axis.
 * 
 */

void XYPlot_SetLabels(PL plot, const char* xLabel, const char* yLabel);


/**
 * @brief Sets the range for the x and y axes of the XY plot.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param xMin Minimum value for the x-axis.
 * @param xMax Maximum value for the x-axis.
 * @param yMin Minimum value for the y-axis.
 * @param yMax Maximum value for the y-axis.
 */
void XYPlot_SetRange(PL plot, float xMin, float xMax, float yMin, float yMax);

/**
 * @brief Automatically adjusts the range of the XY plot based on the provided x and y data points.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param x Array of x values.
 * @param y Array of y values.
 * @param count Number of data points.
 */
void XYPlot_AutoRange(PL plot, const float* x, const float* y, size_t count);

/**
 * @brief Draws the XY plot with the specified data points.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param x Array of x values.
 * @param y Array of y values.
 * @param count Number of data points.
 * @param color Color of the plot.
 */
void XYPlot_Draw(const PL plot, const float* x, const float* y, size_t count, Color color);

#undef PL

#ifdef __cplusplus
}
#endif // __cplusplus


#endif // XY_PLOT_H