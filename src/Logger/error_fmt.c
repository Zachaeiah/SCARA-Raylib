#include "error_fmt.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#define MAX_ERROR_MESSAGE_LENGTH 512 /**< Maximum length of a formatted error message. */
#define FMT_ERROR "[%s] %s (Error Code: %d) at %s:%d" /**< Format string for error messages. */
#define FMT_MSG "[%s] %s" /**< Format string for general messages. */

ErrorType NO_ERROR = 0; /**< Represents no error condition. */

/**
 * @brief Convert a log level enum to a string.
 *
 * @param level LogLevel to convert.
 * @return Human-readable string.
 * @ingroup Logger
 */
static const char* levelToStr(LogLevel level) {
  switch (level) {
    case LOG_DEBUG: return "DEBUG";
    case LOG_INFO: return "INFO";
    case LOG_WARNING: return "WARNING";
    case LOG_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

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
void formatError_v(char* error_buff, const uint32_t error_buff_size, const LogLevel severity, const char* funcError, const uint16_t line, const ErrorType error, const char* strError, va_list args)
{

  char userMsg[MAX_ERROR_MESSAGE_LENGTH]; // Buffer for the user-provided and final formatted message

  // Format the user-provided message
  vsnprintf(userMsg, sizeof(userMsg), strError, args);

  // Combine all parts into the final error message
  snprintf(error_buff, error_buff_size, FMT_ERROR, levelToStr(severity), userMsg, error, funcError, line);
}

/**
 * @brief Format a log message.
 * @param msg_buff Buffer to store the formatted message. Must be writable and at least @p msg_buff_size bytes. The buffer will be null-terminated.
 * @param msg_buff_size Size of the @p msg_buff buffer in bytes.
 * @param severity The severity level of the log message.
 * @param strMsg A format string for the message (printf-style).
 * @param args Additional arguments to be formatted into strMsg, as required by the format string.
 */
void formatMsg_v(char* msg_buff, const uint32_t msg_buff_size, const LogLevel severity, const char* strMsg, va_list args)
{
  char userMsg[MAX_ERROR_MESSAGE_LENGTH]; // Buffer for the user-provided and final formatted message

  // Format the user-provided message
  vsnprintf(userMsg, sizeof(userMsg), strMsg, args);

  // Combine all parts into the final error message
  snprintf(msg_buff, msg_buff_size, FMT_MSG, levelToStr(severity), userMsg);
}