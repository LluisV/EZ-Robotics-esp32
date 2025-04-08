/**
 * @file Debug.h
 * @brief Global debugging facility for the CNC controller
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Debug level macros for convenience
#define DEBUG_ERROR 0
#define DEBUG_WARNING 1
#define DEBUG_INFO 2
#define DEBUG_VERBOSE 3

/**
 * @brief Debug utility class
 */
class Debug
{
public:
    /**
     * @brief Initialize the debug system
     * @param enable Whether debugging is enabled
     * @param serialBaud Baud rate for serial communication
     */
    static void begin(bool enable, unsigned long serialBaud = 115200);

    /**
     * @brief Set the debug level
     * @param level Debug level (0-3)
     */
    static void setLevel(uint8_t level);

    /**
     * @brief Print a debug message
     * @param component Component name
     * @param message Debug message
     * @param level Debug level (0=ERROR, 1=WARNING, 2=INFO, 3=VERBOSE)
     */
    static void print(const String &component, const String &message, uint8_t level = 2);

    /**
     * @brief Print a debug message at ERROR level
     * @param component Component name
     * @param message Debug message
     */
    static void error(const String &component, const String &message);

    /**
     * @brief Print a debug message at WARNING level
     * @param component Component name
     * @param message Debug message
     */
    static void warning(const String &component, const String &message);

    /**
     * @brief Print a debug message at INFO level
     * @param component Component name
     * @param message Debug message
     */
    static void info(const String &component, const String &message);

    /**
     * @brief Print a debug message at VERBOSE level
     * @param component Component name
     * @param message Debug message
     */
    static void verbose(const String &component, const String &message);

    /**
     * @brief Print system diagnostics
     */
    static void printDiagnostics();

    /**
     * @brief Check if debug is enabled
     * @return True if debug is enabled
     */
    static bool isEnabled();

    /**
     * @brief Start a performance timing section
     * @param component Component name
     * @param section Section name
     */
    static void timerStart(const String &component, const String &section);

    /**
     * @brief End a performance timing section and print elapsed time
     * @param component Component name
     * @param section Section name
     */
    static void timerEnd(const String &component, const String &section);

private:
    static bool enabled;       ///< Whether debugging is enabled
    static uint8_t debugLevel; ///< Debug level (0-3)

    static const int MAX_TIMERS = 10; ///< Maximum number of active timers

    /**
     * @brief Timer structure for performance measurements
     */
    struct Timer
    {
        String key;              ///< Timer identifier (component:section)
        unsigned long startTime; ///< Start time in microseconds
    };

    static Timer timers[MAX_TIMERS]; ///< Timer array
    static int timerCount;           ///< Current timer index
};

#endif // DEBUG_H