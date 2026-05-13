#include "assert.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * Handle a failed assertion.
 * This function is called whenever an `assert` or `assert_debug` fails.
 *
 * @param file Source file where the assertion failed
 * @param line Line number of the failed assertion
 * @param cond String representation of the failed condition
 */
void Assert_failed(const char *file, int line, const char *cond) {
    // Print a detailed error message
    fprintf(stderr, "Assertion failed: %s, file %s, line %d\n",
            cond, file, line);

    // Ensure the message is flushed immediately
    fflush(stderr);

    // Terminate the program since an assertion failure is unrecoverable
    abort();
}
