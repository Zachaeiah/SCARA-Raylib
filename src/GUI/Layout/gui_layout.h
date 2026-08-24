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
 * Owns the complete GUI layout tree.
 */
typedef struct GuiLayout *GuiLayout;


/*
 * ID used to reference a node in the layout.
 */
typedef int GuiNodeID;


/*
 * Represents an invalid node or a node with no parent.
 */
#define GUI_NODE_INVALID (-1)

/**
 * @brief Create a new GUI layout.
 *
 * The root node is automatically created as node 0.
 *
 * @param rect Root rectangle in screen coordinates.
 *
 * @return Newly created GUI layout.
 */
GuiLayout GuiLayout_Create(const Rectangle *rect);

/**
 * @brief Destroy a GUI layout and all of its nodes.
 *
 * @param layout Pointer to the GUI layout.
 */
void GuiLayout_Destroy(GuiLayout *layout);

/**
 * @brief Get the root node of the layout.
 *
 * @param layout GUI layout.
 *
 * @return Root node ID.
 */
GuiNodeID GuiLayout_GetRoot(const GuiLayout layout);

/**
 * @brief Create a child node.
 *
 * The node position is relative to the parent's content area.
 *
 * @param layout GUI layout.
 * @param parent Parent node ID.
 * @param rect Rectangle relative to the parent.
 *
 * @return ID of the newly created node.
 */
GuiNodeID GuiLayout_CreateNode(GuiLayout layout, GuiNodeID parent, const Rectangle *rect);

/**
 * @brief Get the number of nodes in the layout.
 *
 * @param layout GUI layout.
 *
 * @return Number of nodes.
 */
int GuiLayout_GetNodeCount(const GuiLayout layout);

/**
 * @brief Set a node's local position.
 *
 * The position is relative to the node's parent.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param x Local X position.
 * @param y Local Y position.
 */
void GuiNode_SetPosition(GuiLayout layout, GuiNodeID node, float x, float y);

/**
 * @brief Set a node's size.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param width Node width.
 * @param height Node height.
 */
void GuiNode_SetSize(GuiLayout layout, GuiNodeID node, float width, float height);

/**
 * @brief Set a node's local rectangle.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param rect New local rectangle.
 */
void GuiNode_SetRect( GuiLayout layout, GuiNodeID node, const Rectangle *rect);

/**
 * @brief Move a node relative to its current position.
 *
 * Child nodes will move with the node automatically.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param dx X movement.
 * @param dy Y movement.
 */
void GuiNode_Move( GuiLayout layout, GuiNodeID node, float dx, float dy );

/**
 * @brief Set equal padding on all sides of a node.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param padding Padding amount.
 */
void GuiNode_SetPadding( GuiLayout layout, GuiNodeID node, float padding);

/**
 * @brief Set padding for each side of a node.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param left Left padding.
 * @param top Top padding.
 * @param right Right padding.
 * @param bottom Bottom padding.
 */
void GuiNode_SetPaddingEx( GuiLayout layout, GuiNodeID node, float left, float top, float right, float bottom );


/**
 * @brief Get a node's rectangle in screen coordinates.
 *
 * Parent positions are automatically included.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 *
 * @return Node rectangle in screen coordinates.
 */
Rectangle GuiNode_GetRect( const GuiLayout layout, GuiNodeID node );

/**
 * @brief Get the usable content rectangle of a node.
 *
 * The returned rectangle includes the node hierarchy and removes padding.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 *
 * @return Content rectangle in screen coordinates.
 */
Rectangle GuiNode_GetContentRect( const GuiLayout layout, GuiNodeID node );

/**
 * @brief Create a screen-space rectangle inside a node.
 *
 * The position is relative to the node's content area.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param x Local X position.
 * @param y Local Y position.
 * @param width Rectangle width.
 * @param height Rectangle height.
 *
 * @return Rectangle in screen coordinates.
 */
Rectangle GuiNode_Rect( const GuiLayout layout, GuiNodeID node, float x, float y, float width, float height );

/**
 * @brief Get a node's position in screen coordinates.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 *
 * @return Screen position of the node.
 */
Vector2 GuiNode_GetPosition( const GuiLayout layout, GuiNodeID node );

/**
 * @brief Get a node's size.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 *
 * @return Node width and height.
 */
Vector2 GuiNode_GetSize( const GuiLayout layout, GuiNodeID node );

/**
 * @brief Get the usable content size of a node.
 *
 * Padding is removed from the returned size.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 *
 * @return Content width and height.
 */
Vector2 GuiNode_GetContentSize( const GuiLayout layout, GuiNodeID node );


/**
 * @brief Convert a node-local position to screen coordinates.
 *
 * The local position is relative to the node's content area.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param localPosition Position relative to the node.
 *
 * @return Position in screen coordinates.
 */
Vector2 GuiNode_LocalToScreen( const GuiLayout layout, GuiNodeID node, Vector2 localPosition );

/**
 * @brief Convert a screen position to node-local coordinates.
 *
 * The returned position is relative to the node's content area.
 *
 * @param layout GUI layout.
 * @param node Node ID.
 * @param screenPosition Position in screen coordinates.
 *
 * @return Position relative to the node.
 */
Vector2 GuiNode_ScreenToLocal( const GuiLayout layout, GuiNodeID node, Vector2 screenPosition );


#ifdef __cplusplus
}
#endif

#endif /* GUI_LAYOUT_H */