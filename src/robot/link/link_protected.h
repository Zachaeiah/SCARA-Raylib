#ifndef LINK_LINK_PROTECTED_H_
#define LINK_LINK_PROTECTED_H_

#include "Control/Actuator/Actuator.h"
#include "link.h"
#include "raymath.h"


// the origin of the link is at x = -dim.x/2, z=-dim.z/2, y = 0 (so the link rotates around the z axis at the bottom of the link)
typedef struct Link_protected{
    Vector3 Start; // where the axis of rotation is located
    Vector3 End; // where the next link will be attached
    float JP; // current angle of the link
    Actuator* actuator; // actuator for the link V
}Link_protected;

// the actuator will take in a "volatege" and output a "speed" that will be used to update the angle of the link

#endif // LINK_LINK_PROTECTED_H_