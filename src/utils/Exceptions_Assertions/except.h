#ifndef EXCEPT_INCLUDED
#define EXCEPT_INCLUDED

//======================================================== Includes ===================================================
#include <setjmp.h>   // for setjmp/longjmp functions

//======================================================== Data =======================================================
#define T Except_t    // Typedef alias for the exception type

//======================================================== Exception States ======================================================
enum {
    Excep_entered = 0,   // When TRY block is first entered
    Excep_raised,        // Exception has been raised
    Excep_handled,       // Exception has been caught
    Excep_finalized      // FINALLY block executed
};

//======================================================== Exception Type ======================================================
typedef struct T {
    const char* reason;  // Reason for the exception
} T;


extern const T NullptrError;
extern const T ValueError;
extern const T ZeroDivisionError;
extern const T MemroyError;
extern const T NotImplementedError;

//======================================================== Exception Frame ======================================================
typedef struct Except_Frame Except_Frame;

struct Except_Frame {
    Except_Frame *prev;      // Pointer to previous frame (stack)
    jmp_buf env;             // Environment for setjmp/longjmp
    const char* file;        // File where exception was raised
    int line;                // Line number where exception was raised
    const T* exception;      // Pointer to the exception object
};

//======================================================== Global Stack ======================================================
extern Except_Frame* Except_stack;  // Pointer to the top of exception stack

//======================================================== Prototypes ======================================================
/**
 * Raise an exception
 * @param e Pointer to the exception
 * @param file File name where the exception was raised
 * @param line Line number where the exception was raised
 */
void Except_raise(const T* e, const char* file, int line);

//======================================================== Macros =========================================================

// Raise an exception (use inside TRY blocks)
#define RAISE(e) Except_raise(&(e), __FILE__, __LINE__)

// Safe return from a function within TRY block
#define RETURN \
    switch (Except_stack = Except_stack->prev, 0) default: return

// TRY block: wraps code that may raise exceptions
#define TRY \
    do { \
        volatile int Excep_flag; \
        Except_Frame Except_frame; \
        Except_frame.prev = Except_stack; \
        Except_stack = &Except_frame; \
        Excep_flag = setjmp(Except_frame.env); \
        if (Excep_flag == Excep_entered) {

// EXCEPT block: handles a specific exception
#define EXCEPT(e) \
            if (Excep_flag == Excep_entered) Except_stack = Except_stack->prev; \
        } else if (Except_frame.exception == &(e)) { \
            Excep_flag = Excep_handled;

// ELSE block: handles any exception not previously caught
#define ELSE \
            if (Excep_flag == Excep_entered) Except_stack = Except_stack->prev; \
        } else { \
            Excep_flag = Excep_handled;

// FINALLY block: always executed regardless of exception
#define FINALLY \
            if (Excep_flag == Excep_entered) Except_stack = Except_stack->prev; \
        } { \
            if (Excep_flag == Excep_entered) \
                Excep_flag = Excep_finalized;

// END_TRY block: ends the TRY/EXCEPT/ELSE/FINALLY structure
#define END_TRY \
            if (Excep_flag == Excep_entered) Except_stack = Except_stack->prev; \
            } \
            if (Excep_flag == Excep_raised) RAISE(*Except_frame.exception); \
    } while (0)

#undef T
#endif
