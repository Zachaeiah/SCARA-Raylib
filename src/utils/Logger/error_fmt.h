#ifndef ERROR_FMT_H
#define ERROR_FMT_H

// @file Error_fmt.h
// @brief Header file for error formatting utilities.
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdarg.h>

typedef uint8_t ErrorType; /**< Type definition for error codes. */
extern ErrorType NO_ERROR; /**< Represents no error condition. */
extern ErrorType JP_RANGE; /**< Represents joint position out of range. */
extern ErrorType JP_CMD_CLAMPED; /**< Represents clamped joint position command. */
extern ErrorType JP_CMD_REJECTED; /**< Represents rejected joint position command. */
extern ErrorType TCP_CMD_CLAMPED; /**< Represents clamped TCP command. */
extern ErrorType TCP_CMD_REJECTED; /**< Represents rejected TCP command. */


/**
 * @brief Format an error message for console output.
 * 
 * @param error_buff Buffer to hold the formatted error message.
 * @param error_buff_size Size of the error_buff buffer.
 * @param severity The severity level of the error (e.g., LOG_ERROR, LOG_WARNING).
 * @param file The name of the source file where the error occurred.
 * @param func The name of the function where the error occurred.
 * @param line The line number in the source file where the error occurred.
 * @param error The error code associated with the error.
 * @param strError A format string describing the error message (printf-style).
 * @param args A va_list of arguments to be formatted into strError, as required by the format string.
 * @return int The number of characters written to error_buff, excluding the null terminator.
 */
int formatError_v(char* error_buff,
                  uint32_t error_buff_size,
                  int severity,
                  const char* file,
                  const char* func,
                  uint16_t line,
                  ErrorType error,
                  const char* strError,
                  va_list args);

/**
 * @brief Format a normal message for console output.
 * 
 * @param msg_buff Buffer to hold the formatted message.
 * @param msg_buff_size Size of the msg_buff buffer.
 * @param severity The severity level of the message (e.g., LOG_INFO, LOG_DEBUG).
 * @param strMsg A format string describing the message (printf-style).
 * @param args A va_list of arguments to be formatted into strMsg, as required by the format string.
 * @return int The number of characters written to msg_buff, excluding the null terminator.
 */
int formatMsg_v(char* msg_buff,
                uint32_t msg_buff_size,
                int severity,
                const char* strMsg,
                va_list args);

/**
 * @brief Format an error message for file output.
 * 
 * @param error_buff Buffer to hold the formatted error message.
 * @param error_buff_size Size of the error_buff buffer.
 * @param severity The severity level of the error (e.g., LOG_ERROR, LOG_WARNING).
 * @param file The name of the source file where the error occurred.
 * @param func The name of the function where the error occurred.
 * @param line The line number in the source file where the error occurred.
 * @param error The error code associated with the error.
 * @param strError A format string describing the error message (printf-style).
 * @param args A va_list of arguments to be formatted into strError, as required by the format string.
 * @return int The number of characters written to error_buff, excluding the null terminator.
 */
int formatErrorFile_v(char* error_buff,
                      uint32_t error_buff_size,
                      int severity,
                      const char* file,
                      const char* func,
                      uint16_t line,
                      ErrorType error,
                      const char* strError,
                      va_list args);

/**
 * @brief Format a normal message for file output.
 * 
 * @param msg_buff Buffer to hold the formatted message.
 * @param msg_buff_size Size of the msg_buff buffer.
 * @param severity The severity level of the message (e.g., LOG_INFO, LOG_DEBUG).
 * @param strMsg A format string describing the message (printf-style).
 * @param args A va_list of arguments to be formatted into strMsg, as required by the format string.
 * @return int The number of characters written to msg_buff, excluding the null terminator.
 */
int formatMsgFile_v(char* msg_buff,
                    uint32_t msg_buff_size,
                    int severity,
                    const char* strMsg,
                    va_list args);
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ERROR_FMT_H


