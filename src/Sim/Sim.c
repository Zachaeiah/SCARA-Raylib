#include "Sim.h"

int Sim_initialized = 0;

typedef struct Sim
{
    

}Sim;

Sim_status_t Sim_ctor(Sim* self)
{

    if (Sim_initialized) return Sim_ERR_ARG;

    // TODO

    Sim_initialized = 1;
    return Sim_SUC;

}

Sim_status_t Sim_loop(Sim* self, const double dt)
{
    // TODO
    return Sim_SUC;

}
Sim_status_t Sim_dtor(Sim* self)
{
    // TODO
    return Sim_SUC;
}