#ifndef LINK_LINK_PROTECTED_H_
#define LINK_LINK_PROTECTED_H_

#include "link.h"
#include "raymath.h"

typedef struct Link_protected{
    Link_protected *parent;
    Matrix O_frame; // start frame matrix
    Matrix F_frame; // end frame matrix
}Link_protected;

#endif