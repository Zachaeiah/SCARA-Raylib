#include <stdlib.h>
#include <string.h>
#include "utils/Logger/logger.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/Exceptions_Assertions/assert.h"
#include "utils/MemAllocator/mem.h"
#include "array.h"
#include "arrayrep.h"

const Except_t ARRAY_Failed = {"Array Faild"};

Array_T Array_new(int length, int size) 
{
	Array_T array;
	void *ary = NULL;

	NEW(array);

	if (length > 0){
		TRY
		{
			ary = CALLOC(length, size);
		}
		EXCEPT(Mem_Failed)
		{
			LOG_ERROR_MSG(ARRAY_Failed_ErrorCode, "Memory allocation failed for Array");
			RAISE(ARRAY_Failed);

		} END_TRY;


		ArrayRep_init(array, length, size, ary);
	}
	else{
		ArrayRep_init(array, length, size, NULL);
	}
	return array;
}

void ArrayRep_init(Array_T array, int length, int size, void *ary) 
{
	assert(array);
    assert(size > 0);
    assert((length == 0 && ary == NULL) || (length > 0  && ary != NULL));

	array->length = length;
	array->size   = size;

	if (length > 0)
		array->array = ary; 
	else
		array->array = NULL;
}

void Array_free(Array_T *array) 
{
	assert(array && *array);
	FREE((*array)->array);
	FREE(*array);
}

int Array_length(Array_T array) 
{
	assert(array);
	return array->length;
}

int Array_size(Array_T array) {
	assert(array);
	return array->size;
}


void *Array_get(Array_T array, int i) 
{
	assert(array);
	assert(i >= 0 && i < array->length);

	return array->array + i*array->size;
}

void *Array_put(Array_T array, int i, void *elem) 
{
	assert(array);
	assert(i >= 0 && i < array->length);
	assert(elem);

	memcpy(array->array + i*array->size, elem, array->size);
	return elem;
}

void *Array_iterate(Array_T array, int direction)
{
    static Array_T currentArray = NULL;
    static int index = 0;
    static int currentDirection = 0;

    /* Reset iterator */
    if (array == NULL)
    {
        currentArray = NULL;
        index = 0;
        currentDirection = 0;
        return NULL;
    }

    if (array != currentArray || direction != currentDirection)
    {
        currentArray = array;
        currentDirection = direction;

        if (direction == 0)
            index = 0;
        else
            index = Array_length(array) - 1;
    }

    /* Left -> Right */
    if (direction == 0)
    {
        if (index >= Array_length(array))
            return NULL;

        return Array_get(array, index++);
    }

    /* Right -> Left */
    if (index < 0)
        return NULL;

    return Array_get(array, index--);
}


void Array_resize(Array_T array, int length) 
{
	assert(array);
	assert(length >= 0);

	if (length == 0)
		FREE(array->array);

	else if (array->length == 0)
		array->array = ALLOC(length*array->size);
	else
		RESIZE(array->array, length*array->size);

	array->length = length;
}

Array_T Array_copy(Array_T array, int length) 
{

	Array_T copy;

	assert(array);
	assert(length >= 0);

	copy = Array_new(length, array->size);

	if (copy->length >= array->length && array->length > 0)
		memcpy(copy->array, array->array, array->length * array->size);

	else if (array->length > copy->length && copy->length > 0)
		memcpy(copy->array, array->array, copy->length*array->size);

	return copy;
}

void Array_apply(Array_T array, ArrayApplyFunc func)
{
    assert(array);
    assert(func);

    int length = Array_length(array);

    for (int i = 0; i < length; i++)
    {
        func(Array_get(array, i));
    }
}

void Array_applyContext(Array_T array, ArrayApplyContextFunc func, void *context)
{
    assert(array);
    assert(func);

    int length = Array_length(array);

    for (int i = 0; i < length; i++)
    {
        func(Array_get(array, i), context);
    }
}