#include "utils/Logger/logger.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>

#define MAX_ERROR_MESSAGE_LENGTH 256 /**< Maximum length of a formatted error message. */

FILE* flog = NULL;

static int Logger_atexit_registered = 0;

/**
 * @brief Initialize the logger file.
 *
 * Opens the log file in append mode.
 * If path is NULL or empty, "log.txt" is used.
 *
 * @param path Path to the log file.
 * @return 0 on success, -1 on failure.
 */
int Logger_init(const char* path)
{
    const char* log_path = path;

    // If no path provided, use default "log.txt"
    if (log_path == NULL || log_path[0] == '\0') {
        printf("No log file path provided. Using default: log.txt\n");
        log_path = "log.txt";
    }

    // Close existing log file if open
    if (flog != NULL && flog != stderr) {
        printf("Closing existing log file before opening new one.\n");
        fflush(flog);
        fclose(flog);
        flog = NULL;
    }

    // Attempt to open the log file in append mode
    flog = fopen(log_path, "a");

    if (flog == NULL) {
        printf("Failed to open log file: %s. Logging to stderr.\n", log_path);
        flog = stderr;
        return -1;
    }

    // Set line buffering for the log file to ensure timely writes
    setvbuf(flog, NULL, _IOLBF, 0);

    // Register Logger_shutdown to be called at program exit, but only once.
    if (!Logger_atexit_registered) {
        printf("Registering Logger_shutdown to be called at program exit.\n");
        atexit(Logger_shutdown);
        Logger_atexit_registered = 1;
    }


    LOG_DEBUG_MSG(NO_ERROR, "Logger initialized with file: %s", log_path);

    return 0;
}

/**
 * @brief Close the logger file.
 */
void Logger_shutdown(void)
{
    if (!Logger_atexit_registered || flog == NULL) {
        printf("Logger_shutdown called but Logger was never initialized. Nothing to do.\n");
        return; // Logger was never initialized, nothing to do
    }

    // Log the shutdown message before closing the file, if it's not stderr.
    if (flog != NULL && flog != stderr) {
        LOG_DEBUG_MSG(NO_ERROR, "Logger shutting down");

        // Flush and close the log file if it's not stderr.
        fflush(flog);
        fclose(flog);
    }

    flog = NULL;
}

/**
 * @brief Log a message with the specified severity level.
 * @param severity The severity level of the log message.
 * @param file The name of the source file where the log message is generated.
 * @param func The function name where the log message is generated.
 * @param line The line number in the source code where the log message is generated.
 * @param error The error code associated with the log message.
 * @param strError A format string describing the log message (printf-style).
 * @param ... Additional arguments to be formatted into strError, as required by the format string.
 */
void Logger_log(int severity, const char* file, const char* func, const uint16_t line, ErrorType error, const char* strError, ...)
{
    char consoleBuff[MAX_ERROR_MESSAGE_LENGTH];
    char fileBuff[MAX_ERROR_MESSAGE_LENGTH];

    va_list consoleArgs;
    va_list fileArgs;

    if (flog == NULL) {
        flog = stderr;
    }


    va_start(consoleArgs, strError);
    va_copy(fileArgs, consoleArgs);

    
    if (error == NO_ERROR) {
        formatMsg_v(consoleBuff, sizeof(consoleBuff), severity,
                    strError, consoleArgs);

        formatMsgFile_v(fileBuff, sizeof(fileBuff), severity,
                        strError, fileArgs);
    } 
    else 
    {
        formatError_v(consoleBuff, sizeof(consoleBuff),
                      severity, file, func, line, error,
                      strError, consoleArgs);

        formatErrorFile_v(fileBuff, sizeof(fileBuff),
                          severity, file, func, line,
                          error, strError, fileArgs);
    }

    va_end(fileArgs);
    va_end(consoleArgs);

    /*
        Console:
        TraceLog already adds INFO:, WARNING:, ERROR:, etc.
    */
    TraceLog(severity, "%s", consoleBuff);

    /*
        File:
        fileBuff already contains timestamp and severity.
    */
    if (flog != NULL && flog != stderr) {
        fprintf(flog, "%s", fileBuff);

        size_t len = strlen(fileBuff);

        if (len == 0 || fileBuff[len - 1] != '\n') {
            fprintf(flog, "\n");
        }

        fflush(flog);
    }
}