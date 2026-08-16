#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/Logger/logger.h"

extern const Except_t Controller_failed; /**< Represents a generic controller error. */
extern const ErrorType Controller_ErrorCode; /**< Represents a generic controller error code. */

typedef struct Controller Controller;
typedef struct ControllerType ControllerType;

/**
 * @brief Create a controller instance.
 *
 * Allocates type->size bytes and calls the controller-specific constructor.
 *
 * @param type Pointer to the ControllerType describing the controller.
 * @param ...  Additional arguments required by the specific controller type's constructor.
 *             The expected arguments and their types depend on the implementation of the controller.
 *             Refer to the documentation of the specific ControllerType for details.
 *
 * @throw NullptrError       if type or type->vtable is NULL
 * @throw ValueError         if type->size is invalid
 * @throw NotImplementedError if required functions are missing
 * @throw MemroyError        if allocation fails
 */
Controller* Controller_create(const ControllerType* type, ...);

/**
 * @brief Update the controller.
 *
 * @throw NullptrError if self is NULL
 * @throw NotImplementedError if update is not implemented
 */
float Controller_update(Controller* self, float x);

/**
 * @brief Reset the controller.
 *
 * @throw NullptrError if self is NULL
 */
void Controller_reset(Controller* self);

/**
 * @brief Destroy the controller.
 *
 * Safe to call with NULL.
 */
void Controller_destroy(Controller* self);

#endif