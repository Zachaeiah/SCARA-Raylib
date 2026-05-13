#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "error_fmt.h"

/*
 * @brief Log a message with the specified severity level.
 * @param severity The severity level of the log message.
 * @param funcError The name of the function where the log message is generated.
 * @param line The line number in the source code where the log message is generated.
 * @param error The error code associated with the log message.
 * @param strError A format string describing the log message (printf-style).
 * @param ... Additional arguments to be formatted into strError, as required by the format string.
 */
void Logger_log(LogLevel severity, const char* funcError, uint16_t line, ErrorType error, const char* strError, ...);

/**
 * @brief log a message with the INFO severity level, using a simplified format without error codes and location.
 */
#define LOG_MESSAGE(msg, ...) Logger_log(LOG_INFO, __FUNCTION__, __LINE__, NO_ERROR, msg, ##__VA_ARGS__)

/**
 * @brief Log a debug message with automatic function and line metadata.
 */
#define LOG_DEBUG_MSG(errType, msg, ...) Logger_log(LOG_DEBUG, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

/**
 * @brief Log an info message with automatic function and line metadata.
 */
#define LOG_INFO_MSG(errType, msg, ...) Logger_log(LOG_INFO, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

/**
 * @brief Log a warning message with automatic function and line metadata.
 */
#define LOG_WARN_MSG(errType, msg, ...) Logger_log(LOG_WARNING, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

/**
 * @brief Log an error message with automatic function and line metadata.
 */
#define LOG_ERROR_MSG(errType, msg, ...) Logger_log(LOG_ERROR, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

/**
 * @brief Log a critical error message with automatic function and line metadata.
 */
#define LOG_CRITICAL_MSG(errType, msg, ...) Logger_log(LOG_CRITICAL, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // LOGGER_H