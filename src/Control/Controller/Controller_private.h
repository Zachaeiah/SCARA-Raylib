#ifndef CONTROLLER_PRIVATE_H_
#define CONTROLLER_PRIVATE_H_

#include "Controller.h"

#include <stddef.h>
#include <stdarg.h>

/**
 * @brief The virtual function table for the Controller.
 * 
 * @param ctor A pointer to the constructor function for the controller.
 * @param update A pointer to the update function for the controller,
 * @param reset A pointer to the reset function
 * @param dtor A pointer to the destructor function for the controller, 
 */
typedef struct ControllerVTable {
    void  (*ctor)(Controller self, va_list* args);
    float (*update)(Controller self, float x);
    void  (*reset)(Controller self);
    void  (*dtor)(Controller self);
} ControllerVTable;

/**
 * @brief The type information for the Controller.
 * 
 * @param size The size of the controller structure
 * @param vtable A pointer to the virtual function table 
 * for the controller
 */
struct ControllerType {
    size_t size;
    const ControllerVTable* vtable;
};

/**
 * @brief The base Controller structure. 
 * All specific controller types
 * 
 * @param type A pointer to the ControllerType structure
 */
struct Controller {
    const ControllerType* type;
};

#endif