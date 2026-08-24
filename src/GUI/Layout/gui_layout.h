#ifndef GUI_LAYOUT_H
#define GUI_LAYOUT_H

#include "raylib.h"
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const Except_t GUIROOT_Failed;
extern const ErrorType GUIROOT_Failed_ErrorCode;

/*
 * Owns an entire GUI layout tree.
 */
typedef struct GuiLayout *GuiLayout;


/*
 * Index into the GuiLayout node array.
 *
 * Using an integer instead of exposing GuiNode pointers gives:
 *
 *      O(1) lookup
 *      simple parent references
 *      easy validity checking
 *      stable handles even if Array_T reallocates
 */
typedef int GuiNodeID;

/*
 * Invalid / no-parent node.
 */
#define GUI_NODE_INVALID (-1)

/**
 * @brief Create a GUI layout.
 *
 * Node 0 is automatically created as the root node.
 *
 * @param rect Root rectangle in screen coordinates.
 *
 * @return New GuiLayout.
 */
GuiLayout GuiLayout_Create(const Rectangle *rect);

/**
 * @brief Destroy an entire GUI layout.
 *
 * All GuiNodes are allocated from the layout arena and are
 * destroyed together.
 * 
 * @param layout
 */
void GuiLayout_Destroy(GuiLayout *layout);

/**
 * @brief Get the root node.
 * 
 * @param layout
 * @return GuiNodeID
 */
GuiNodeID GuiLayout_GetRoot(const GuiLayout layout);

/**
 * @brief Create a child node.
 *
 * The rectangle is relative to the parent's content area.
 *
 * @param layout GUI layout.
 * @param parent Parent node.
 * @param rect Local rectangle.
 *
 * @return New GuiNodeID.
 */
GuiNodeID GuiLayout_CreateNode(GuiLayout layout, GuiNodeID parent, const Rectangle *rect
);

/**
 * @brief Get the number of nodes.
 * 
 * @param layout GUI layout.
 * @return number of Gui nodes
 */
int GuiLayout_GetNodeCount(const GuiLayout layout);

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @param x 
 * @param y 
 */
void GuiNode_SetPosition(GuiLayout layout, GuiNodeID node, float x, float y);

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @param width 
 * @param height 
 */
void GuiNode_SetSize(GuiLayout layout, GuiNodeID node, float width, float height);

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @param rect 
 */
void GuiNode_SetRect( GuiLayout layout, GuiNodeID node, const Rectangle *rect);

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @param dx 
 * @param dy 
 */
void GuiNode_Move( GuiLayout layout, GuiNodeID node, float dx, float dy );

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @param padding 
 */
void GuiNode_SetPadding( GuiLayout layout, GuiNodeID node, float padding);

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @param left 
 * @param top 
 * @param right 
 * @param bottom 
 */
void GuiNode_SetPaddingEx( GuiLayout layout, GuiNodeID node, float left, float top, float right, float bottom );


/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @return Rectangle 
 */
Rectangle GuiNode_GetRect( const GuiLayout layout, GuiNodeID node );

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @return Rectangle 
 */
Rectangle GuiNode_GetContentRect( const GuiLayout layout, GuiNodeID node );

/**
  * @brief Create a screen-space rectangle relative to a node's
  * content area.
  * 
  * @param layout 
  * @param node 
  * @param x 
  * @param y 
  * @param width 
  * @param height 
  * @return Rectangle 
  */
Rectangle GuiNode_Rect( const GuiLayout layout, GuiNodeID node, float x, float y, float width, float height );

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @return Vector2 
 */
Vector2 GuiNode_GetPosition( const GuiLayout layout, GuiNodeID node );

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @return Vector2 
 */
Vector2 GuiNode_GetSize( const GuiLayout layout, GuiNodeID node );

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @return Vector2 
 */
Vector2 GuiNode_GetContentSize( const GuiLayout layout, GuiNodeID node );

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @param localPosition 
 * @return Vector2 
 */
Vector2 GuiNode_LocalToScreen( const GuiLayout layout, GuiNodeID node, Vector2 localPosition );

/**
 * @brief 
 * 
 * @param layout 
 * @param node 
 * @param screenPosition 
 * @return Vector2 
 */
Vector2 GuiNode_ScreenToLocal( const GuiLayout layout, GuiNodeID node, Vector2 screenPosition );


#ifdef __cplusplus
}
#endif

#endif /* GUI_LAYOUT_H */