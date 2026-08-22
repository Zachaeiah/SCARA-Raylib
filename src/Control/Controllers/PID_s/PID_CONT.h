#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Control/Controller/Controller.h"

// Forward declaration of the PIDController structure.
#define PIDC PIDController
typedef struct PIDC *PIDC;

// Forward declaration of the PIDController type.
extern const ControllerType PIDController_Type;

#undef PIDC

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // PID_CONTROLLER_H_