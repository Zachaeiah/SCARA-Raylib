#include "gui_layout.h"

#include "utils/DynamicArray/array.h"
#include "utils/MemAllocator/mem.h"

const Except_t GUIROOT_Failed = {"GUI Layout faild"};

/*
 * One node in the GUI layout tree.
 *
 * All GuiNodes are allocated from GuiLayout->arena.
 */
typedef struct GuiNode
{
    /*
     * Rectangle relative to the parent's content area.
     *
     * Root node is the exception:
     * its rectangle is already in screen coordinates.
     */
    Rectangle localRect;

    float paddingLeft;
    float paddingRight;
    float paddingTop;
    float paddingBottom;

    /*
     * Index of parent node in GuiLayout->nodes.
     *
     * GUI_NODE_INVALID means this is the root.
     */
    GuiNodeID parent;

} GuiNode;


/*
 * Owns the complete GUI hierarchy.
 */
struct GuiLayout
{
    /*
     * Array containing GuiNode pointers.
     *
     * Array element type:
     *
     *      GuiNode *
     */
    Array_T nodes;
};

 /**
  * @brief Check whether a node ID exists.
  * 
  * @param layout 
  * @param node 
  * @return int 
  */
static int GuiLayout_IsValidNode(const GuiLayout layout, GuiNodeID node)
{
    assert(layout);

    if (node < 0)
        return 0;

    if (node >= Array_length(layout->nodes))
        return 0;

    return 1;
}


/**
 * @brief Get a GuiNode pointer from its ID.
 *
 * Array_T stores GuiNode pointers, therefore Array_get()
 * returns a pointer to the array element:
 *
 *      GuiNode **
 *
 * We dereference that once to obtain GuiNode *.
 */

 /**
  * @brief Get a GuiNode pointer from its ID.
  * 
  * @param layout 
  * @param node 
  * @return GuiNode* 
  */
static GuiNode *GuiLayout_GetNode(const GuiLayout layout, const GuiNodeID node)
{
    if (!GuiLayout_IsValidNode(layout, node))
        return NULL;

    GuiNode **entry = (GuiNode **)Array_get(layout->nodes, node);

    if (entry == NULL)
        return NULL;

    return *entry;
}

 /**
  * @brief Append a node pointer to the dynamic array.
  * 
  * @param layout 
  * @param node 
  * @return GuiNodeID Index assigned to the node.
  */
static GuiNodeID GuiLayout_AppendNode(GuiLayout layout, GuiNode *node)
{
    assert(layout != NULL);
    assert(node != NULL);

    int index = Array_length(layout->nodes);

    /*
     * Increase array length by one.
     *
     * Example:
     *
     * old length = 3
     * valid indexes = 0, 1, 2
     *
     * resize to 4
     * new valid index = 3
     */
    Array_resize(layout->nodes, index + 1);

    /*
     * Array stores GuiNode*.
     *
     * Array_put() copies sizeof(GuiNode *) bytes from &node.
     */
    Array_put(layout->nodes, index, &node);

    return index;
}

 /**
  * @brief Allocate a GuiNode from the layout arena.
  * 
  * @param layout 
  * @return GuiNode* 
  */
static GuiNode *GuiLayout_AllocateNode(GuiLayout layout)
{
    GuiNode *node = NULL;

    NEW0(node);

    return node;
}


GuiLayout GuiLayout_Create(const Rectangle *rect)
{
    assert(rect != NULL);

    GuiLayout layout = NULL;

    NEW0(layout);

    /*
     * Dynamic array stores GuiNode pointers.
     *
     * Start with zero nodes.
     */
    layout->nodes = Array_new(0,  sizeof(GuiNode *));

    /*
     * Create root node.
     */
    GuiNode *root = GuiLayout_AllocateNode(layout);

    root->localRect = *rect;

    root->paddingLeft   = 0.0f;
    root->paddingRight  = 0.0f;
    root->paddingTop    = 0.0f;
    root->paddingBottom = 0.0f;

    root->parent = GUI_NODE_INVALID;

    /*
     * Root will always initially be node 0.
     */
    GuiLayout_AppendNode(layout, root);

    return layout;
}


void GuiLayout_Destroy(GuiLayout *layout)
{
    if (layout == NULL || *layout == NULL)
        return;

    int count = Array_length((*layout)->nodes);

    for (int i = 0; i < count; i++)
    {
        GuiNode **entry = (GuiNode **)Array_get((*layout)->nodes, i);

        if (entry != NULL && *entry != NULL)
        {
            FREE(*entry);
            *entry = NULL;
        }
    }

    Array_free(&(*layout)->nodes);

    FREE(*layout);

    *layout = NULL;
}


GuiNodeID GuiLayout_GetRoot( const GuiLayout layout)
{
    if (layout == NULL)
        return GUI_NODE_INVALID;

    if (Array_length(layout->nodes) == 0)
        return GUI_NODE_INVALID;

    return 0;
}


int GuiLayout_GetNodeCount( const GuiLayout layout)
{
    if (layout == NULL)
        return 0;

    return Array_length( layout->nodes);
}


GuiNodeID GuiLayout_CreateNode( GuiLayout layout, GuiNodeID parent, const Rectangle *rect)
{
    assert(layout != NULL);
    assert(rect != NULL);
    assert(GuiLayout_IsValidNode(layout, parent));

    GuiNode *node = GuiLayout_AllocateNode(layout);

    assert(node != NULL);

    node->localRect = *rect;

    node->paddingLeft   = 0.0f;
    node->paddingRight  = 0.0f;
    node->paddingTop    = 0.0f;
    node->paddingBottom = 0.0f;

    /*
     * localRect is relative to this parent.
     */
    node->parent = parent;

    return GuiLayout_AppendNode(layout, node);
}

void GuiNode_SetPosition( GuiLayout layout, GuiNodeID nodeID, float x, float y)
{
    GuiNode *node = GuiLayout_GetNode( layout, nodeID);

    if (node == NULL)
        return;

    node->localRect.x = x;
    node->localRect.y = y;
}


void GuiNode_SetSize( GuiLayout layout, GuiNodeID nodeID, float width, float height)
{
    GuiNode *node = GuiLayout_GetNode( layout, nodeID);

    if (node == NULL)
        return;

    node->localRect.width  = width;
    node->localRect.height = height;
}


void GuiNode_SetRect( GuiLayout layout, GuiNodeID nodeID, const Rectangle *rect)
{
    if (rect == NULL)
        return;

    GuiNode *node = GuiLayout_GetNode( layout, nodeID);

    if (node == NULL)
        return;

    node->localRect = *rect;
}


void GuiNode_Move( GuiLayout layout, GuiNodeID nodeID, float dx, float dy)
{
    GuiNode *node = GuiLayout_GetNode( layout, nodeID);

    if (node == NULL)
        return;

    node->localRect.x += dx;
    node->localRect.y += dy;
}


void GuiNode_SetPadding( GuiLayout layout, GuiNodeID nodeID, float padding)
{
    GuiNode *node = GuiLayout_GetNode( layout, nodeID);

    if (node == NULL)
        return;

    node->paddingLeft   = padding;
    node->paddingRight  = padding;
    node->paddingTop    = padding;
    node->paddingBottom = padding;
}


void GuiNode_SetPaddingEx( GuiLayout layout, GuiNodeID nodeID, float left, float top, float right, float bottom)
{
    GuiNode *node = GuiLayout_GetNode( layout, nodeID);

    if (node == NULL)
        return;

    node->paddingLeft   = left;
    node->paddingTop    = top;
    node->paddingRight  = right;
    node->paddingBottom = bottom;
}

Rectangle GuiNode_GetRect(
    const GuiLayout layout,
    GuiNodeID nodeID)
{
    GuiNode *node = GuiLayout_GetNode(layout, nodeID);

    if (node == NULL)
    {
        return (Rectangle)
        {
            .x = 0.0f,
            .y = 0.0f,
            .width = 0.0f,
            .height = 0.0f
        };
    }

    /*
     * Start in this node's local coordinate system.
     */
    Rectangle rect = node->localRect;

    /*
     * Walk upward through every parent.
     *
     * A child is positioned relative to the parent's
     * CONTENT origin, so each parent contributes:
     *
     *      parent.localRect.x
     *    + parent.paddingLeft
     *
     * and:
     *
     *      parent.localRect.y
     *    + parent.paddingTop
     */
    GuiNodeID parentID = node->parent;

    while (parentID != GUI_NODE_INVALID)
    {
        GuiNode *parent =
            GuiLayout_GetNode(layout, parentID);

        if (parent == NULL)
            break;

        rect.x +=
            parent->localRect.x +
            parent->paddingLeft;

        rect.y +=
            parent->localRect.y +
            parent->paddingTop;

        parentID = parent->parent;
    }

    return rect;
}


Rectangle GuiNode_GetContentRect( const GuiLayout layout, GuiNodeID nodeID)
{
    GuiNode *node = GuiLayout_GetNode(layout, nodeID);

    if (node == NULL)
    {
        return (Rectangle)
        {
            .x = 0.0f,
            .y = 0.0f,
            .width = 0.0f,
            .height = 0.0f
        };
    }

    Rectangle rect = GuiNode_GetRect(layout, nodeID);

    rect.x += node->paddingLeft;
    rect.y += node->paddingTop;

    rect.width -=
        node->paddingLeft +
        node->paddingRight;

    rect.height -=
        node->paddingTop +
        node->paddingBottom;

    if (rect.width < 0.0f)
        rect.width = 0.0f;

    if (rect.height < 0.0f)
        rect.height = 0.0f;

    return rect;
}


Rectangle GuiNode_Rect( const GuiLayout layout, GuiNodeID node, float x, float y, float width, float height)
{
    Rectangle content = GuiNode_GetContentRect(layout, node);

    return (Rectangle)
    {
        .x      = content.x + x,
        .y      = content.y + y,
        .width  = width,
        .height = height
    };
}

Vector2 GuiNode_GetPosition( const GuiLayout layout, GuiNodeID node)
{
    Rectangle rect = GuiNode_GetRect( layout, node);

    return (Vector2)
    {
        rect.x,
        rect.y
    };
}


Vector2 GuiNode_GetSize( const GuiLayout layout, GuiNodeID nodeID)
{
    GuiNode *node = GuiLayout_GetNode( layout, nodeID);

    if (node == NULL)
    {
        return (Vector2)
        {
            0.0f,
            0.0f
        };
    }

    return (Vector2)
    {
        node->localRect.width,
        node->localRect.height
    };
}


Vector2 GuiNode_GetContentSize( const GuiLayout layout, GuiNodeID node)
{
    Rectangle rect = GuiNode_GetContentRect( layout, node);

    return (Vector2)
    {
        rect.width,
        rect.height
    };
}


Vector2 GuiNode_LocalToScreen( const GuiLayout layout, GuiNodeID node, Vector2 localPosition)
{
    Rectangle content =  GuiNode_GetContentRect(layout, node);

    return (Vector2)
    {
        content.x + localPosition.x,
        content.y + localPosition.y
    };
}


Vector2 GuiNode_ScreenToLocal( const GuiLayout layout, GuiNodeID node, Vector2 screenPosition)
{
    Rectangle content = GuiNode_GetContentRect( layout, node);

    return (Vector2)
    {
        screenPosition.x - content.x,
        screenPosition.y - content.y
    };
}