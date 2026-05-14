#include "assert.h"

#include <stdlib.h>
#include <stdint.h>

#include "utils/Logger/logger.h"

static uint16_t Assert_clamp_line(int line)
{
    if (line < 0) {
        return 0;
    }

    if (line > UINT16_MAX) {
        return UINT16_MAX;
    }

    return (uint16_t)line;
}

void Assert_failed(const char *file, int line, const char *cond)
{
    Logger_log(LOG_ERROR,
        file ? file : "unknown",
        __FUNCTION__,
        Assert_clamp_line(line),
        NO_ERROR,
        "Assertion failed: %s",
        cond ? cond : "(null)"
    );

    abort();
}