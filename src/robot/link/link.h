//--------------------------------------------------------------------------------------------------
// File: LINK.h
// Description: Module description
// Created on: 03
//--------------------------------------------------------------------------------------------------

#ifndef LINK_LINK_H_
#define LINK_LINK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "raylib.h"
#include "raymath.h"


//---------------------------- Program Definitions --------------------------------------------------
#define LINK_VERSION_MAJOR   1
#define LINK_VERSION_MINOR   0

//---------------------------- Error Codes ----------------------------------------------------------
typedef enum {
    LINK_SUC = 0,
    LINK_ERR,
    LINK_ERR_ARG,
} LINK_status_t;


//---------------------------- Configuration Constants ----------------------------------------------
// Add user-configurable constants here


//---------------------------- Structure Definitions ------------------------------------------------
typedef struct Link_protected Link_protected;

typedef struct Link
{
    Color color;
    Vector3 dim;
    Link_protected* protected;

} Link;

//----------------------------- Globals -------------------------------------------------------------

//----------------------------- Function Prototypes --------------------------------------------------

/**
 * @brief will contructa link in the scara arm
 * 
 * @param self pointer to the link onj
 * @param dim the X:lenght Y: width Z:Hight
 * @param color the color of the link
 * @return LINK_status_t 
 */
extern LINK_status_t LINK_ctor(Link* self, Vector3 dim, Color color);

extern LINK_status_t LINK_update(Link* self, const double angle);

extern LINK_status_t LINK_dtor(Link* self);


#ifdef __cplusplus
}
#endif

#endif // LINK_H