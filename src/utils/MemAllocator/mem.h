#ifndef MEM_INCLUDED
#define MEM_INCLUDED

#include "utils/Exceptions_Assertions/except.h"
#include <stdint.h>

/**
 * @brief Allocate memory block.
 *
 * @param nbytes Number of bytes to allocate
 * @param file   Source file (auto-filled)
 * @param line   Line number (auto-filled)
 *
 * @return Pointer to allocated memory (never NULL)
 *
 * @raise Mem_Failed if allocation fails
 */
void* Mem_alloc(uint64_t nbytes, const char *file, int line);

/**
 * @brief Allocate zero-initialized memory.
 *
 * @param count  Number of elements
 * @param nbytes Size of each element
 *
 * @return Pointer to allocated memory (never NULL)
 *
 * @raise Mem_Failed if allocation fails or overflow occurs
 */
void* Mem_calloc(uint64_t count, uint64_t nbytes, const char *file, int line);

/**
 * @brief Free allocated memory.
 *
 * @param ptr Pointer to free
 *
 * @raise Mem_Free_Failed if:
 *  - ptr is invalid
 *  - ptr already freed
 */
void Mem_free(void *ptr, const char *file, int line);

/**
 * @brief Resize memory block.
 *
 * @param ptr    Existing pointer
 * @param nbytes New size
 *
 * @return Resized pointer (never NULL)
 *
 * @raise Mem_Failed if resize fails
 * @raise Mem_Free_Failed if ptr is invalid
 */
void* Mem_resize(void *ptr, uint64_t nbytes, const char *file, int line);

/**
 * @brief Iterate over all active allocations.
 *
 * @param apply Callback for each allocation
 * @param cl    User context
 */
void Mem_leak(void apply(void *ptr, long size,
              const char *file, int line, void *cl),
              void *cl);

/**
 * @brief Dump all current memory leaks to stderr.
 */
void Mem_dumpLeaks(void);

/* =========================
   Convenience Macros
   ========================= */

/**
 * @brief Allocate raw memory
 *
 * @raise MemroyError
 */
#define ALLOC(nbytes) Mem_alloc((nbytes), __FILE__, __LINE__)

/**
 * @brief Allocate zeroed memory
 *
 * @raise MemroyError
 */
#define CALLOC(count, nbytes) Mem_calloc((count), (nbytes), __FILE__, __LINE__)

/**
 * @brief Allocate struct (uninitialized)
 *
 * @raise MemroyError
 */
#define NEW(p) ((p) = ALLOC(sizeof *(p)))

/**
 * @brief Allocate struct (zeroed)
 *
 * @raise MemroyError
 */
#define NEW0(p) ((p) = CALLOC(1, sizeof *(p)))

/**
 * @brief Free pointer and set to NULL
 *
 * @raise Mem_Free_Failed
 */
#define FREE(ptr) ((void)(Mem_free((ptr), __FILE__, __LINE__), (ptr) = NULL))

/**
 * @brief Resize memory block
 *
 * @raise Mem_Failed
 * @raise Mem_Free_Failed
 */
#define RESIZE(ptr, nbytes) ((ptr) = Mem_resize((ptr), (nbytes), __FILE__, __LINE__))

#endif