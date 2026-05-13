#include "Link.h"
#include "link_protected.h"

#include "utils/Exceptions_Assertions/assert.h"
#include "utils/Exceptions_Assertions/except.h"
#include "utils/MemAllocator/mem.h"

#include "raylib.h"
#include "raymath.h"

//---------------------------- Structure Definitions ------------------------------------------------

static Link_protected* Link_protected_ctor()
{
    Link_protected* protected;

    TRY{
        printf("inside try");
        
        NEW(protected);

        protected->O_frame = MatrixIdentity();
        protected->F_frame = MatrixIdentity();

    } EXCEPT(Mem_Failed){
        printf("Could not allocate Link: %s\n", Except_frame.exception->reason);
        if (protected) {
            FREE(protected);
        }
        protected = NULL;
    } END_TRY;

    return protected;
}


LINK_status_t LINK_ctor(Link* self, Vector3 dim, Color color)
{
    // TODO
    self->protected = Link_protected_ctor();
    self->protected->F_frame = MatrixTranslate(dim.x, 0, 0); // 

    return LINK_SUC;
}

LINK_status_t LINK_update(Link* self, const double angle)
{
    // TODO
    return LINK_SUC;
}
LINK_status_t LINK_dtor(Link* self)
{
    // TODO
    return LINK_SUC;
}



