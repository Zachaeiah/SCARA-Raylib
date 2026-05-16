#ifndef LINK_LINK_PROTECTED_H_
#define LINK_LINK_PROTECTED_H_

#include "Control/Actuator/Actuator.h"
#include "link.h"
#include "raymath.h"

typedef struct Link_protected{
    Vector3 Start; // start frame matrix
    Vector3 End; // end frame matrix
    float JP; // current angle of the link
    Actuator* actuator; // actuator for the link V
}Link_protected;

// the actuator will take in a "volatege" and output a "speed" that will be used to update the angle of the link

#endif // LINK_LINK_PROTECTED_H_