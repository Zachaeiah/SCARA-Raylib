#include "logger.h"
#include "Sinck.h"
#include <stdarg.h>
#include <stdio.h>

#define MAX_ERROR_MESSAGE_LENGTH 512 /**< Maximum length of a formatted error message. */

/*
 * @brief Log a message with the specified severity level.
 * @param severity The severity level of the log message.
 * @param funcError The name of the function where the log message is generated.
 * @param line The line number in the source code where the log message is generated.
 * @param error The error code associated with the log message.
 * @param strError A format string describing the log message (printf-style).
 * @param ... Additional arguments to be formatted into strError, as required by the format string.
 */
void Logger_log(LogLevel severity, const char* funcError, uint16_t line, ErrorType error, const char* strError, ...)
{
    char errorBuff[MAX_ERROR_MESSAGE_LENGTH]; // Buffer to hold the formatted error message

    va_list args; // Declare the variable argument list
    va_start(args, strError); // Initialize the variable argument list

    if (severity == LOG_DEBUG || severity == LOG_INFO) {
        // For debug and info messages, we can use a simpler format without error codes and location
        formatMsg_v(errorBuff, sizeof(errorBuff), severity, strError, args);
    } else {
        // For other severity levels, include error code and location in the message

        // Format the error message using the formatError function
        formatError_v(errorBuff, sizeof(errorBuff), severity, funcError, line, error, strError, args);
    }

    va_end(args); // Clean up the variable argument list

    // Send the formatted error message to all registered sinks
    Sinck_brodcast_printf("%s", errorBuff);
}