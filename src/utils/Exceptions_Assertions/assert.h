#ifndef ASSERT_INCLUDED
#define ASSERT_INCLUDED

#include <stdio.h>
#include <stdlib.h>

/*
 * Lightweight Assert Module
 * ------------------------
 * Similar to the standard <assert.h>, but implemented as a separate module.
 * Can be extended or replaced easily.
 */

#define T Assert_T
typedef struct T { 
    char dummy;  // Dummy member, actual content not used
} T;

//======================================================== Function =========================================================
/**
 * Handle a failed assertion.
 * This function is called when an assert condition fails.
 *
 * @param file  The source file where the assertion failed
 * @param line  The line number of the failed assertion
 * @param cond  The string representation of the condition that failed
 */
extern void Assert_failed(const char *file, int line, const char *cond);

//======================================================== Assertions =====================================================

// Always-on assertion: always checks the condition at runtime
#define assert(e) \
    ((e) ? (void)0 : Assert_failed(__FILE__, __LINE__, #e))

// Debug assertion: only checks the condition when NDEBUG is NOT defined
#ifdef NDEBUG
#define assert_debug(e) ((void)0)
#else
#define assert_debug(e) assert(e)
#endif

#undef T
#endif
