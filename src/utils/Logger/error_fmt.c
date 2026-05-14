#include "error_fmt.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "raylib.h"

/*
    Console format.

    Do not include severity here because raylib TraceLog()
    already prints INFO:, WARNING:, ERROR:, etc.
*/
#define FMT_ERROR_CMD "%s: %u (Error:%d) in function %s: %s"
#define FMT_MSG_CMD   "%s"

/*
    File format.

    Include timestamp and severity because fprintf()
    does not add them automatically.
*/
#define FMT_ERROR_FILE "[%s] %s: %s: %u (Error:%d) in function %s: %s"
#define FMT_MSG_FILE   "[%s] %s: %s"

#define MAX_ERROR_MESSAGE_LENGTH 256 /**< Maximum length of a formatted error message. */

ErrorType NO_ERROR = 0; /**< Represents no error condition. */

static const char* severity_to_string(int severity)
{
    switch (severity) {
        case LOG_TRACE:
            return "TRACE";

        case LOG_DEBUG:
            return "DEBUG";

        case LOG_INFO:
            return "INFO";

        case LOG_WARNING:
            return "WARNING";

        case LOG_ERROR:
            return "ERROR";

        case LOG_FATAL:
            return "FATAL";

        default:
            return "UNKNOWN";
    }
}

static void make_timestamp(char* buffer, size_t buffer_size)
{
    time_t now = time(NULL);

    if (now == (time_t)-1) {
        snprintf(buffer, buffer_size, "unknown-time");
        return;
    }

    struct tm* tm_info = localtime(&now);

    if (tm_info == NULL) {
        snprintf(buffer, buffer_size, "unknown-time");
        return;
    }

    strftime(buffer, buffer_size, "%Y-%m-%d %H:%M:%S", tm_info);
}

/*
    Console error formatter.

    Example output passed into TraceLog():

        src/main.c: 50 (Error:12) in function main: This is a warning message

    TraceLog() will print:

        WARNING: src/main.c: 50 (Error:12) in function main: This is a warning message
*/
int formatError_v(char* error_buff,
                  const uint32_t error_buff_size,
                  const int severity,
                  const char* file,
                  const char* func,
                  const uint16_t line,
                  const ErrorType error,
                  const char* strError,
                  va_list args)
{
    (void)severity;

    char userMsg[MAX_ERROR_MESSAGE_LENGTH];

    if (strError == NULL) {
        strError = "";
    }

    vsnprintf(userMsg, sizeof(userMsg), strError, args);

    return snprintf(error_buff,
                    error_buff_size,
                    FMT_ERROR_CMD,
                    file,
                    (unsigned int)line,
                    error,
                    func,
                    userMsg);
}

/*
    Console normal message formatter.

    Example output passed into TraceLog():

        Program started

    TraceLog() will print:

        INFO: Program started
*/
int formatMsg_v(char* msg_buff,
                const uint32_t msg_buff_size,
                const int severity,
                const char* strMsg,
                va_list args)
{
    (void)severity;

    char userMsg[MAX_ERROR_MESSAGE_LENGTH];

    if (strMsg == NULL) {
        strMsg = "";
    }

    vsnprintf(userMsg, sizeof(userMsg), strMsg, args);

    return snprintf(msg_buff, msg_buff_size, FMT_MSG_CMD, userMsg);
}

/*
    File error formatter.

    Example file output:

        [2026-05-14 10:52:30] WARNING: src/main.c: 50 (Error:12) in function main: This is a warning message
*/
int formatErrorFile_v(char* error_buff,
                      const uint32_t error_buff_size,
                      const int severity,
                      const char* file,
                      const char* func,
                      const uint16_t line,
                      const ErrorType error,
                      const char* strError,
                      va_list args)
{
    char timestamp[32];
    char userMsg[MAX_ERROR_MESSAGE_LENGTH];

    if (strError == NULL) {
        strError = "";
    }

    make_timestamp(timestamp, sizeof(timestamp));
    vsnprintf(userMsg, sizeof(userMsg), strError, args);

    return snprintf(error_buff,
                    error_buff_size,
                    FMT_ERROR_FILE,
                    timestamp,
                    severity_to_string(severity),
                    file,
                    (unsigned int)line,
                    error,
                    func,
                    userMsg);
}

/*
    File normal message formatter.

    Example file output:

        [2026-05-14 10:52:30] INFO: Program started
*/
int formatMsgFile_v(char* msg_buff,
                    const uint32_t msg_buff_size,
                    const int severity,
                    const char* strMsg,
                    va_list args)
{
    char timestamp[32];
    char userMsg[MAX_ERROR_MESSAGE_LENGTH];

    if (strMsg == NULL) {
        strMsg = "";
    }

    make_timestamp(timestamp, sizeof(timestamp));
    vsnprintf(userMsg, sizeof(userMsg), strMsg, args);

    return snprintf(msg_buff,
                    msg_buff_size,
                    FMT_MSG_FILE,
                    timestamp,
                    severity_to_string(severity),
                    userMsg);
}