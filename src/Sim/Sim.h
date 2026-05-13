//--------------------------------------------------------------------------------------------------
// File: Sim.h
// Description: Module description
// Created on: 02
//--------------------------------------------------------------------------------------------------

#ifndef SIM_H_
#define SIM_H_

#ifdef __cplusplus
extern "C" {
#endif


//---------------------------- Program Definitions --------------------------------------------------
#define Sim_VERSION_MAJOR   1
#define Sim_VERSION_MINOR   0

//---------------------------- Error Codes ----------------------------------------------------------
typedef enum {
    Sim_SUC = 0,
    Sim_ERR,
    Sim_ERR_ARG,
} Sim_status_t;


//---------------------------- Configuration Constants ----------------------------------------------
// Add user-configurable constants here


//---------------------------- Structure Definitions ------------------------------------------------
typedef struct Sim Sim;



//----------------------------- Globals -------------------------------------------------------------
extern int Sim_initialized;


//----------------------------- Function Prototypes --------------------------------------------------
extern Sim_status_t Sim_ctor(Sim* self);
extern Sim_status_t Sim_loop(Sim* self, const double dt);
extern Sim_status_t Sim_dtor(Sim* self);


#ifdef __cplusplus
}
#endif

#endif // Sim_H