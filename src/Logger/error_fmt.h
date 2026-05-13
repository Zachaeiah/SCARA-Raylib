#ifndef ERROR_FMT_H
#define ERROR_FMT_H

// @file Error_fmt.h
// @brief Header file for error formatting utilities.
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdarg.h>

/**
 * @enum LogLevel
 * @brief Defines severity levels for log messages.
 */
typedef enum LogLevel {
  LOG_DEBUG,   /**< Detailed debug information, for development use. */
  LOG_INFO,    /**< General system information messages. */
  LOG_WARNING, /**< Warning messages that indicate potential issues. */
  LOG_ERROR,   /**< Error messages that indicate failures. */
  LOG_CRITICAL /**< Critical error messages that indicate severe failures. */
} LogLevel;


typedef uint8_t ErrorType; /**< Type definition for error codes. */

extern ErrorType NO_ERROR; /**< Represents no error condition. */

/**
 * @brief Format an error message based on the error type and log level.
 * 
 * @param error_buff Buffer to store the formatted error message. Must be writable and at least @p error_buff_size bytes. The buffer will be null-terminated.
 * @param error_buff_size Size of the @p error_buff buffer in bytes.
 * @param severity The severity level of the log message.
 * @param funcError The name of the function where the error occurred.
 * @param line The line number in the source code where the error occurred.
 * @param error The specific error code to format.
 * @param strError A format string describing the error (printf-style).
 * @param args Additional arguments to be formatted into strError, as required by the format string.
 */
void formatError_v(char* error_buff, const uint32_t error_buff_size, const LogLevel severity, const char* funcError, const uint16_t line, const ErrorType error, const char* strError, va_list args);

/**
 * @brief Format a log message.
 * @param msg_buff Buffer to store the formatted message. Must be writable and at least @p msg_buff_size bytes. The buffer will be null-terminated.
 * @param msg_buff_size Size of the @p msg_buff buffer in bytes.
 * @param severity The severity level of the log message.
 * @param strMsg A format string for the message (printf-style).
 * @param args Additional arguments to be formatted into strMsg, as required by the format string.
 */
void formatMsg_v(char* msg_buff, const uint32_t msg_buff_size, const LogLevel severity, const char* strMsg, va_list args);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ERROR_FMT_H


