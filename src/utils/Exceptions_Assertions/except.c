//======================================================== Includes ===================================================
#include "except.h"

#include <stdlib.h>
#include <stdint.h>

#include "assert.h"
#include "utils/Logger/logger.h"

#define T Except_t

const T NullptrError        = { "Null pointer" };
const T ValueError          = { "Invalid value" };
const T ZeroDivisionError   = { "Zero division detected" };
const T NotImplementedError = { "Method not implemented" };

//======================================================== Data =======================================================
Except_Frame* Except_stack = NULL;   // Top of the exception stack

static const char* Except_reason(const Except_t* e)
{
    if (e == NULL) {
        return "(null exception)";
    }

    if (e->reason == NULL) {
        return "(unknown exception)";
    }

    return e->reason;
}

//======================================================== Functions ==================================================
/**
 * Raise an exception.
 * Transfers control to the nearest enclosing TRY block.
 *
 * @param e    Pointer to the exception object
 * @param file Source file where the exception was raised
 * @param line Line number where the exception was raised
 */
void Except_raise(const T* e, const char* file, int line) {
    Except_Frame *p = Except_stack;

    // Ensure the exception pointer is valid
    assert(e);

    if (p == NULL) {
        const char* reason = Except_reason(e);
        Logger_log(LOG_ERROR, file, "", line, 99, "%s", reason);

        abort();
        }

    // Fill in exception frame info
    p->exception = e;
    p->file = file;
    p->line = line;

    // Pop the frame from the stack
    Except_stack = Except_stack->prev;

    // Jump back to the corresponding TRY block
    longjmp(p->env, Excep_raised);
}
