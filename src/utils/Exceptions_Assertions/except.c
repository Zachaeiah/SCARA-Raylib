//======================================================== Includes ===================================================
#include <stdlib.h>
#include <stdio.h>
#include "assert.h"
#include "except.h"

#define T Except_t

const T NullptrError =          {" Null Pointer "};
const T ValueError =            {" Invaled Value was "};
const T ZeroDivisionError =     {" Zero Division Deteted "};
const T MemroyError =           {" Memory Allocation Failed "};
const T NotImplementedError =   {" Method Not Implemented "};

//======================================================== Data =======================================================
Except_Frame* Except_stack = NULL;   // Top of the exception stack

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
        // No TRY block to handle this exception: uncaught exception
        fprintf(stderr, "Uncaught exception");
        if (e->reason) {
            fprintf(stderr, ": %s", e->reason);
        } else {
            fprintf(stderr, " at 0x%p", (void*)e);
        }
        if (file && line > 0) {
            fprintf(stderr, " raised at %s:%d", file, line);
        }
        fprintf(stderr, "\nAborting...\n");
        fflush(stderr);
        abort();  // terminate program
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
