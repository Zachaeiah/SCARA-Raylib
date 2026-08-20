#ifndef XY_PLOT_H
#define XY_PLOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "raylib.h"
#include <stddef.h>

/**
 * @brief Represents a 2D XY plot with specified bounds and ranges for the x and y axes.
 * 
 */
typedef struct XYPlot {
    Vector2 pos;     // Bottom-left corner of graph

    float width;
    float height;

    float xMin;
    float xMax;

    float yMin;
    float yMax;

    float xMajor;
    float yMajor;

    int minorDivisions;

    const char* xLabel;
    const char* yLabel;

} XYPlot;


/**
 * @brief Initializes an XYPlot structure with the specified bounds.
 * 
 * @param plot Pointer to the XYPlot structure to initialize.
 * @param pos The position of the bottom-left corner of the plot.
 * @param width The width of the plot area.
 * @param height The height of the plot area.
 */
void XYPlot_Init(XYPlot* plot, Vector2 pos, float width, float height);

/**
 * @brief Sets the grid spacing for the x and y axes of the XY plot.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param xMajor The major division spacing for the x-axis.
 * @param yMajor The major division spacing for the y-axis.
 * @param minorDivisions The number of minor divisions between major divisions.
 */
void XYPlot_SetGrid( XYPlot* plot, float xMajor, float yMajor, int minorDivisions);

/**
 * @brief Sets the labels for the x and y axes of the XY plot.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param xLabel The label for the x-axis.
 * @param yLabel The label for the y-axis.
 * 
 */

void XYPlot_SetLabels( XYPlot* plot, const char* xLabel, const char* yLabel);


/**
 * @brief Sets the range for the x and y axes of the XY plot.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param xMin Minimum value for the x-axis.
 * @param xMax Maximum value for the x-axis.
 * @param yMin Minimum value for the y-axis.
 * @param yMax Maximum value for the y-axis.
 */
void XYPlot_SetRange( XYPlot* plot, float xMin, float xMax, float yMin, float yMax);

/**
 * @brief Automatically adjusts the range of the XY plot based on the provided x and y data points.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param x Array of x values.
 * @param y Array of y values.
 * @param count Number of data points.
 */
void XYPlot_AutoRange( XYPlot* plot, const float* x, const float* y, size_t count);

/**
 * @brief Draws the XY plot with the specified data points.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param x Array of x values.
 * @param y Array of y values.
 * @param count Number of data points.
 * @param color Color of the plot.
 */
void XYPlot_Draw( const XYPlot* plot, const float* x, const float* y, size_t count, Color color);

#ifdef __cplusplus
}
#endif // __cplusplus


#endif // XY_PLOT_H