#ifndef CONTROL_ACTUATOR_H_
#define CONTROL_ACTUATOR_H_

#include <stdint.h>

extern const Except_t Actuator_Failed;

/**
 * @brief 
 * 
 */
typedef struct Actuator Actuator;


/**
 * @brief 
 * 
 * @param filter 
 * @param coeff 
 * @return Actuator* 
 */
extern Actuator* Actuator_ctor(
                        const float *b, uint32_t nb,
                        const float *a, uint32_t na, 
                        float Dead_Zone, float Saturation);

/**
 * @brief 
 * 
 * @param self 
 * @param x 
 */
extern void Actuator_update(Actuator* self, float x);

/**
 * @brief 
 * 
 * @param self 
 */
extern void Actuator_reset(Actuator* self);


/**
 * @brief Construct a new Actuator_dtor object
 * 
 * @param self 
 */
extern Actuator_dtor(Actuator* self);

#endif