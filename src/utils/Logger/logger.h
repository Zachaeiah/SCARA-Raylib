#ifndef SRC_LOGGER_H
#define SRC_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "error_fmt.h"

extern FILE* flog; /**< File pointer for logging output. */

/**
 * @brief Log a message with the specified severity level.
 * @param severity The severity level of the log message.
 * @param file The name of the source file where the log message is generated.
 * @param funcError The name of the function where the log message is generated.
 * @param line The line number in the source code where the log message is generated.
 * @param error The error code associated with the log message.
 * @param strError A format string describing the log message (printf-style).
 * @param ... Additional arguments to be formatted into strError, as required by the format string.
 */
void Logger_log(LogLevel severity, const char * file, const char* funcError, uint16_t line, ErrorType error, const char* strError, ...);

/**
 * @brief A helper function that formats a message and prints it to both the console and the log file.
 * 
 * @param fmt The format string (printf-style) for the message to be logged.
 * @param ... Additional arguments to be formatted into the message, as required by the format string.
 * @return int The number of characters printed, or a negative value if an error occurs.
 */
int dsprintf(char const* fmt, ...);


/**
 * @brief log a message with the INFO severity level, using a simplified format without error codes and location.
 * 
 * @param msg The format string (printf-style) for the message to be logged.
 * @param ... Additional arguments to be formatted into the message, as required by the format string
 */
#define LOG_MESSAGE(msg, ...) Logger_log(LOG_INFO, __FILE__, __FUNCTION__, __LINE__, NO_ERROR, msg, ##__VA_ARGS__)

/**
 * @brief Log a debug message with automatic function and line metadata.
 */
#define LOG_DEBUG_MSG(errType, msg, ...) Logger_log(LOG_DEBUG, __FILE__, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

/**
 * @brief Log an info message with automatic function and line metadata.
 * 
 * @param errType The error code associated with the log message. Use NO_ERROR if there is no specific error code.
 * @param msg The format string (printf-style) for the message to be logged.
 */
#define LOG_INFO_MSG(errType, msg, ...) Logger_log(LOG_INFO, __FILE__, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

/**
 * @brief Log a warning message with automatic function and line metadata.
 * 
 * @param errType The error code associated with the log message. Use NO_ERROR if there is no specific error code.
 * @param msg The format string (printf-style) for the message to be logged.
 */
#define LOG_WARN_MSG(errType, msg, ...) Logger_log(LOG_WARNING, __FILE__, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

/**
 * @brief Log an error message with automatic function and line metadata.
 * 
 * @param errType The error code associated with the log message. Use NO_ERROR if there is no specific error code.
 * @param msg The format string (printf-style) for the message to be logged.
 */
#define LOG_ERROR_MSG(errType, msg, ...) Logger_log(LOG_ERROR, __FILE__, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

/**
 * @brief Log a critical error message with automatic function and line metadata.
 * 
 * @param errType The error code associated with the log message. Use NO_ERROR if there is no specific error code.
 * @param msg The format string (printf-style) for the message to be logged.
 */
#define LOG_CRITICAL_MSG(errType, msg, ...) Logger_log(LOG_CRITICAL, __FILE__, __FUNCTION__, __LINE__, errType, msg, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // SRC_LOGGER_H