#include "logger.h"
#include <stdarg.h>
#include <stdio.h>

#define MAX_ERROR_MESSAGE_LENGTH 512 /**< Maximum length of a formatted error message. */

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
void Logger_log(LogLevel severity, const char* file, const char* funcError, uint16_t line, ErrorType error, const char* strError, ...)
{
    char errorBuff[MAX_ERROR_MESSAGE_LENGTH]; // Buffer to hold the formatted error message

    va_list args; // Declare the variable argument list
    va_start(args, strError); // Initialize the variable argument list

    if (severity == LOG_DEBUG || severity == LOG_INFO) {
        // For debug and info messages, we can use a simpler format without error codes and location
        formatMsg_v(errorBuff, sizeof(errorBuff), severity, strError, args);
    } else {
        // For warning, error, and critical messages, include error codes and location
        formatError_v(errorBuff, sizeof(errorBuff), severity, file, funcError, line, error, strError, args);
    }

    va_end(args); // Clean up the variable argument list

    dsprintf("%s", errorBuff); // Print the formatted error message to the console and log file
}

/**
 * @brief A helper function that formats a message and prints it to both the console and the log file.
 * 
 * @param fmt The format string (printf-style) for the message to be logged.
 * @param ... Additional arguments to be formatted into the message, as required by the format string.
 * @return int The number of characters printed, or a negative value if an error occurs.
 */
int dsprintf(char const* fmt, ...)
{
// Declare the variable argument list
   va_list args;

   // Variable to store the number of characters printed to the log file
   int n1 = -1; 

   // Variable to store the number of characters printed to the console
   int n2 = -1; 

   // Print to log file if it's open
   if (flog != NULL)
   {
      va_start(args, fmt);
      n1 = vfprintf(flog, fmt, args);
      va_end(args);
   }
   // Always print to console
   va_start(args, fmt);

   // Print to console and get the number of characters printed
   n2 = vfprintf(stdout, fmt, args);

   // Clean up the variable argument list
   va_end(args);

   if (n2 < n1) n1 = n2;
   return n1;
}