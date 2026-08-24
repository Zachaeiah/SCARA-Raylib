/* $Id$ */
#ifndef ARRAYREP_INCLUDED
#define ARRAYREP_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#define T Array_T

struct T {
	int length; // number of elements in the array
	int size; // size of each element in bytes
	char *array // pointer to the memory block containing the array elements
};

/**
 * @brief Initialize the dynamic array representation.
 * 
 * @param array The dynamic array to initialize.
 * @param length The length of the array.
 * @param size The size of each element in the array.
 * @param ary A pointer to the memory block for the array elements.
 */
extern void ArrayRep_init(T array, int length, int size, void *ary);

#undef T

#ifdef __cplusplus
}
#endif

#endif