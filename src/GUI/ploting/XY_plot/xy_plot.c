#include "xy_plot.h"
#include <math.h>

/**
 * @brief Maps a value from the data range to the pixel range for the x-axis.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param x The x-value to map.
 * @return float The mapped x-value in pixel coordinates.
 */
static float MapX(const XYPlot* plot, float x)
{
    return plot->pos.x +
        ((x - plot->xMin) / (plot->xMax - plot->xMin)) *
        plot->width;
}

/**
 * @brief Maps a value from the data range to the pixel range for the y-axis.
 * 
 * @param plot Pointer to the XYPlot structure.
 * @param y The y-value to map.
 * @return float The mapped y-value in pixel coordinates.
 */
static float MapY(const XYPlot* plot, float y)
{
    return plot->pos.y -
        ((y - plot->yMin) / (plot->yMax - plot->yMin)) *
        plot->height;
}

/**
 * @brief Draws the grid lines and ticks for the XY plot.
 * 
 * @param plot Pointer to the XYPlot structure.
 */
static void XYPlot_DrawGrid(const XYPlot* plot)
{
    float xMinor = plot->xMajor / plot->minorDivisions;
    float yMinor = plot->yMajor / plot->minorDivisions;

    const int fontSize = 10;
    const float tickSize = 5.0f;

    // ---------------------------------------------------------
    // Minor grid
    // ---------------------------------------------------------

    for (float x = plot->xMin; x <= plot->xMax; x += xMinor)
    {
        float px = MapX(plot, x);

        DrawLineEx(
            (Vector2){ px, plot->pos.y },
            (Vector2){ px, plot->pos.y - plot->height },
            1.0f,
            LIGHTGRAY
        );
    }

    for (float y = plot->yMin; y <= plot->yMax; y += yMinor)
    {
        float py = MapY(plot, y);

        DrawLineEx(
            (Vector2){ plot->pos.x, py },
            (Vector2){ plot->pos.x + plot->width, py },
            1.0f,
            LIGHTGRAY
        );
    }


    // ---------------------------------------------------------
    // Major X grid, ticks and labels
    // ---------------------------------------------------------

    for (float x = plot->xMin; x <= plot->xMax; x += plot->xMajor)
    {
        float px = MapX(plot, x);

        DrawLineEx(
            (Vector2){ px, plot->pos.y },
            (Vector2){ px, plot->pos.y - plot->height },
            1.0f,
            GRAY
        );

        DrawLineEx(
            (Vector2){ px, plot->pos.y },
            (Vector2){ px, plot->pos.y + tickSize },
            2.0f,
            DARKGRAY
        );

        const char* text = TextFormat("%.1f", x);
        int textWidth = MeasureText(text, fontSize);

        DrawText(
            text,
            (int)(px - textWidth / 2),
            (int)(plot->pos.y + tickSize + 2.0f),
            fontSize,
            DARKGRAY
        );
    }


    // ---------------------------------------------------------
    // Major Y grid, ticks and labels
    // ---------------------------------------------------------

    for (float y = plot->yMin; y <= plot->yMax; y += plot->yMajor)
    {
        float py = MapY(plot, y);

        DrawLineEx(
            (Vector2){ plot->pos.x, py },
            (Vector2){ plot->pos.x + plot->width, py },
            1.0f,
            GRAY
        );

        DrawLineEx(
            (Vector2){ plot->pos.x - tickSize, py },
            (Vector2){ plot->pos.x, py },
            2.0f,
            DARKGRAY
        );

        const char* text = TextFormat("%.1f", y);
        int textWidth = MeasureText(text, fontSize);

        DrawText(
            text,
            (int)(plot->pos.x - tickSize - textWidth - 3.0f),
            (int)(py - fontSize / 2),
            fontSize,
            DARKGRAY
        );
    }
}

static void DrawLabels(const XYPlot* plot)
{
    const int fontSize = 14;

    // X label
    if (plot->xLabel)
    {
        int width = MeasureText(plot->xLabel, fontSize);

        DrawText(
            plot->xLabel,
            (int)(plot->pos.x + plot->width / 2.0f - width / 2.0f),
            (int)(plot->pos.y + 25.0f),
            fontSize,
            DARKGRAY
        );
    }

    // Y label
    if (plot->yLabel)
    {
        Vector2 size = MeasureTextEx(
            GetFontDefault(),
            plot->yLabel,
            (float)fontSize,
            1.0f
        );

        Vector2 pos =
        {
            plot->pos.x - 40.0f,
            plot->pos.y - plot->height / 2.0f + size.x / 2.0f
        };

        DrawTextPro(
            GetFontDefault(),
            plot->yLabel,
            pos,
            (Vector2){ 0.0f, 0.0f },
            -90.0f,
            (float)fontSize,
            1.0f,
            DARKGRAY
        );
    }
}

void XYPlot_Init(
    XYPlot* plot,
    Vector2 pos,
    float width,
    float height)
{
    plot->pos = pos;

    plot->width = width;
    plot->height = height;

    plot->xMin = 0.0f;
    plot->xMax = 1.0f;

    plot->yMin = 0.0f;
    plot->yMax = 1.0f;

    plot->xMajor = 1.0f;
    plot->yMajor = 1.0f;

    plot->minorDivisions = 5;

    plot->xLabel = NULL;
    plot->yLabel = NULL;
}

void XYPlot_SetGrid(
    XYPlot* plot,
    float xMajor,
    float yMajor,
    int minorDivisions)
{
    plot->xMajor = xMajor;
    plot->yMajor = yMajor;
    plot->minorDivisions = minorDivisions;
}

void XYPlot_SetLabels(
    XYPlot* plot,
    const char* xLabel,
    const char* yLabel)
{
    plot->xLabel = xLabel;
    plot->yLabel = yLabel;
}


void XYPlot_SetRange(
    XYPlot* plot,
    float xMin,
    float xMax,
    float yMin,
    float yMax)
{
    plot->xMin = xMin;
    plot->xMax = xMax;

    plot->yMin = yMin;
    plot->yMax = yMax;
}


void XYPlot_AutoRange(
    XYPlot* plot,
    const float* x,
    const float* y,
    size_t count)
{
    if (count == 0)
        return;

    float xMin = x[0];
    float xMax = x[0];

    float yMin = y[0];
    float yMax = y[0];

    for (size_t i = 1; i < count; i++)
    {
        if (x[i] < xMin) xMin = x[i];
        if (x[i] > xMax) xMax = x[i];

        if (y[i] < yMin) yMin = y[i];
        if (y[i] > yMax) yMax = y[i];
    }

    // Handle flat data
    if (xMin == xMax)
    {
        xMin -= plot->xMajor;
        xMax += plot->xMajor;
    }

    if (yMin == yMax)
    {
        yMin -= plot->yMajor;
        yMax += plot->yMajor;
    }

    // Snap outward to major grid
    plot->xMin =
        floorf(xMin / plot->xMajor) *
        plot->xMajor;

    plot->xMax =
        ceilf(xMax / plot->xMajor) *
        plot->xMajor;

    plot->yMin =
        floorf(yMin / plot->yMajor) *
        plot->yMajor;

    plot->yMax =
        ceilf(yMax / plot->yMajor) *
        plot->yMajor;
}


void XYPlot_Draw(
    const XYPlot* plot,
    const float* x,
    const float* y,
    size_t count,
    Color color)
{
    if (count == 0)
        return;

    Rectangle bounds =
    {
        plot->pos.x,
        plot->pos.y - plot->height,
        plot->width,
        plot->height
    };

    // Grid
    XYPlot_DrawGrid(plot);

    // Border
    DrawRectangleLinesEx(bounds, 1.0f, DARKGRAY );

    DrawLabels(plot);

    // Data
    for (size_t i = 1; i < count; i++)
    {
        Vector2 p0 =
        {
            MapX(plot, x[i - 1]),
            MapY(plot, y[i - 1])
        };

        Vector2 p1 =
        {
            MapX(plot, x[i]),
            MapY(plot, y[i])
        };

        DrawCircle(p0.x, p0.y, 2.0f, color);
        DrawCircle(p1.x, p1.y, 2.0f, color);
    }
}

