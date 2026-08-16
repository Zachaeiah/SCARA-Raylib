#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "mem.h"

const Except_t Mem_Failed = { "Memory allocation failed" };
const Except_t Mem_Free_Failed = { "Memory free failed" };

/**
 * @brief Forces proper alignment for all allocations.
 */
union aligned {
    int i;
    long l;
    long *lp;
    void *p;
    void (*fp)(void);
    float f;
    double d;
    long double ld;
};

#define hash(p, t) (((uintptr_t)(p)>>3) & (sizeof(t)/sizeof((t)[0])-1))

/**
 * @brief Default chunk size for memory pool expansion.
 */
#define NALLOC ((4096 + sizeof(union aligned) -1)/(sizeof(union aligned)))*(sizeof(union aligned))

/**
 * @brief Descriptor pool allocation batch size.
 */
#define NDESCRIPTORS 512

struct descriptor {
    struct descriptor *free;   // next free block
    struct descriptor *link;   // hash chain
    const void *ptr;           // actual memory pointer
    long size;                // size of block
    const char *file;         // where allocated
    int line;
};

/**
 * @brief Hash table mapping ptr → descriptor
 */
static struct descriptor *htab[2048];

/**
 * @brief Freelist sentinel node
 */
static struct descriptor freelist = { &freelist };

/**
 * @brief Lookup descriptor by pointer.
 */
static struct descriptor *find(const void *ptr)
{
    struct descriptor *bp = htab[hash(ptr, htab)];

    while (bp && bp->ptr != ptr) {
        bp = bp->link;
    }
    return bp;
}

/**
 * @brief Free memory with validation.
 */
void Mem_free(void *ptr, const char *file, int line)
{
    if (!ptr) return;

    struct descriptor *bp;

    if (((uintptr_t)ptr) % sizeof(union aligned) != 0 ||
        (bp = find(ptr)) == NULL ||
        bp->free)
    {
        if (!file) RAISE(Mem_Free_Failed);
        else Except_raise(&Mem_Free_Failed, file, line);
    }

    /* Optional debug pattern (detect use-after-free) */
    memset((void*)bp->ptr, 0xDD, bp->size);

    /* Free memory */
    free((void*)bp->ptr);

    /* Mark descriptor */
    bp->ptr = NULL;
    bp->free = (struct descriptor*)1;  // freed sentinel
}


/**
 * @brief Resize memory block (realloc-style).
 */
void* Mem_resize(void *ptr, uint64_t nbytes, const char *file, int line)
{
    assert(ptr);
    assert(nbytes > 0);

    struct descriptor *bp;

    // set bp if ptr is vald
    if (((uintptr_t)ptr) % sizeof(union aligned) != 0 ||
        (bp = find(ptr)) == NULL ||
        bp->free)
    {
        if (!file) RAISE(Mem_Failed);
        else Except_raise(&Mem_Failed, file, line);
    }

    /* Allocate new */
    void *newptr = Mem_alloc(nbytes, file, line);

    /* Copy */
    uint64_t copy_size = nbytes < (uint64_t)bp->size ? nbytes : (uint64_t)bp->size;
    memcpy(newptr, ptr, copy_size);

    /* Free old */
    Mem_free(ptr, file, line);

    return newptr;
}

/**
 * @brief Allocate zero-initialized memory.
 */
void* Mem_calloc(uint64_t count, uint64_t nbytes, const char *file, int line)
{
    assert(count > 0);
    assert(nbytes > 0);

    /* overflow protection */
    if (count > UINT64_MAX / nbytes){
         if (!file) RAISE(Mem_Failed);
        else Except_raise(&Mem_Failed, file, line);
    }

    uint64_t total = count * nbytes;

    void *ptr = Mem_alloc(total, file, line);

    memset(ptr, 0, total);

    return ptr;
}


/**
 * @brief Allocate descriptor from internal pool.
 */
static struct descriptor *dalloc(void *ptr, long size, const char *file, int line)
{
    static struct descriptor *avail;
    static int nleft;

    if (nleft <= 0) {
        avail = malloc(NDESCRIPTORS * sizeof(*avail));
        if (!avail) return NULL;
        nleft = NDESCRIPTORS;
    }

    avail->ptr = ptr;
    avail->size = size;
    avail->file = file;
    avail->line = line;
    avail->free = NULL;
    avail->link = NULL;

    nleft--;
    return avail++;
}


/**
 * @brief Core allocator (pool + tracking).
 *
 * @note Except MemroyError - Memory allocashion faild
 */
void* Mem_alloc(uint64_t nbytes, const char *file, int line)
{
    assert(nbytes > 0);

    /* Overflow protection */
    if (nbytes > UINT64_MAX - sizeof(union aligned)) {
        if (!file) RAISE(Mem_Failed);
        else Except_raise(&Mem_Failed, file, line);
    }

    /* Align size */
    nbytes = ((nbytes + sizeof(union aligned) - 1) /
             sizeof(union aligned)) * sizeof(union aligned);

    /* Allocate */
    void *ptr = malloc(nbytes);
    if (!ptr) {
        if (!file) RAISE(Mem_Failed);
        else Except_raise(&Mem_Failed, file, line);
    }

    /* Debug fill (helps catch uninitialized use) */
    memset(ptr, 0xCD, nbytes);

    /* Allocate descriptor */
    struct descriptor *bp = dalloc(ptr, nbytes, file, line);
    if (!bp) {
        free(ptr);
         if (!file) RAISE(Mem_Failed);
        else Except_raise(&Mem_Failed, file, line);
    }

    /* Insert into hash table */
    unsigned h = hash(ptr, htab);
    bp->link = htab[h];
    htab[h] = bp;

    return ptr;
}

/**
 * @brief Iterate over all active allocations.
 *
 * Calls user-provided function `apply` for each allocated block.
 */
void Mem_leak(void apply(void *ptr, long size,
                         const char *file, int line, void *cl),
              void *cl)
{
    for (int i = 0; i < 2048; i++) {
        struct descriptor *bp = htab[i];

        while (bp) {
            if (!bp->free && bp->ptr != NULL) {
                apply((void*)bp->ptr,
                      bp->size,
                      bp->file,
                      bp->line,
                      cl);
            }
            bp = bp->link;
        }
    }
}

void Mem_dumpLeaks(void)
{
    size_t total = 0;
    int count = 0;

    for (int i = 0; i < 2048; i++) {
        struct descriptor *bp = htab[i];

        while (bp) {
            if (!bp->free && bp->ptr != NULL) {
                printf("LEAK: %p size=%ld (%s:%d)\n",
                       bp->ptr, bp->size, bp->file, bp->line);

                total += bp->size;
                count++;
            }
            bp = bp->link;
        }
    }

    printf("SUMMARY: %d leaks, %zu bytes total\n", count, total);
}

