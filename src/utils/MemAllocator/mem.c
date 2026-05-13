// #include <stdlib.h>
// #include <stddef.h>
// #include "utils/Exceptions_Assertions/assert.h"
// #include "utils/Exceptions_Assertions/except.h"
// #include "mem.h"

// const Except_t Mem_Failed = {"Allocation Failed"};
// const Except_t Mem_Free_Failed = {"Mem Free Failed"};

// void* Mem_alloc(uint64_t nbytes, const char *file, int line){
//     void *ptr = NULL;

//     assert(nbytes > 0);
//     ptr = malloc(nbytes);
//     if (ptr == NULL){
//         //raise Mem_failed
//         if (file == NULL) RAISE(Mem_Failed);
//         else Except_raise(&Mem_Failed, file, line);
//     }
//     return ptr;
// }

// void* Mem_calloc(uint64_t count, uint64_t nbytes, const char *file, int line){
//     void *ptr = NULL;

//     assert(count > 0);
//     assert(nbytes > 0);

//     ptr = calloc(count, nbytes);
//     if (ptr == NULL){
//         //raise Mem_failed
//         if (file == NULL) RAISE(Mem_Failed);
//         else Except_raise(&Mem_Failed, file, line);
//     }
//     return ptr;

// }

// void Mem_free(void *ptr, const char *file, int line){
//     if(ptr != NULL){
//         free(ptr);
//     }
//     else{
//         //raise Mem_failed
//         if (file == NULL) RAISE(Mem_Failed);
//         else Except_raise(&Mem_Failed, file, line);
//     }

// }

// void* Mem_resize(void *ptr, uint64_t nbytes, const char *file, int line){
//     assert(ptr);
//     assert(nbytes > 0);
//     ptr = realloc(ptr, nbytes);
//     if (ptr == NULL){
//         //raise Mem_failed
//         if (file == NULL) RAISE(Mem_Failed);
//         else Except_raise(&Mem_Failed, file, line);
//     }
//     return ptr;
// }

