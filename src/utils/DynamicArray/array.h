/* $Id$ */
#ifndef ARRAY_H_
#define ARRAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#define T Array_T
typedef struct T *T;

extern const Except_t ARRAY_Failed;
extern const ErrorType ARRAY_Failed_ErrorCode;

/**
 * @brief Create a new dynamic array with specified length and element size.
 * 
 * @param length The initial number of elements in the array. Must be non-negative.
 * @param size The size of each element in bytes. Must be positive.
 * @return T 
 */
extern T    Array_new (int length, int size);

/**
 * @brief Free the memory allocated for the dynamic array.
 * 
 * @param array The dynamic array to free.
 */
extern void Array_free(T *array);

/**
 * @brief Get the length of the dynamic array.
 * 
 * @param array The dynamic array.
 * @return int The number of elements in the array.
 */
extern int Array_length(T array);

/**
 * @brief Get the size of each element in the dynamic array.
 * 
 * @param array The dynamic array.
 * @return int The size of each element in bytes.
 */
extern int Array_size  (T array);

/**
 * @brief Get a pointer to the element at the specified index.
 * 
 * @param array The dynamic array.
 * @param i The index of the element to retrieve.
 * @return void* A pointer to the element at the specified index.
 */
extern void *Array_get(T array, int i);

/**
 * @brief Set the element at the specified index to the provided value.
 * 
 * @param array The dynamic array.
 * @param i The index of the element to set.
 * @param elem A pointer to the value to set at the specified index.
 * @return void* A pointer to the value that was set.
 */
extern void *Array_put(T array, int i, void *elem);

/**
 * @brief Resize the dynamic array to the specified length. If the new length is greater than the current length, new elements are uninitialized. If the new length is less than the current length, elements beyond the new length are discarded.
 * 
 * @param array The dynamic array to resize.
 * @param length The new length for the array. Must be non-negative.
 */
extern void Array_resize(T array, int length);

/**
 * @brief Create a copy of the dynamic array with the specified length.
 * 
 * @param array The dynamic array to copy.
 * @param length The length of the new array.
 * @return T A new dynamic array containing a copy of the elements from the original array.
 */
extern T    Array_copy  (T array, int length);

#undef T

#ifdef __cplusplus
}
#endif

#endif