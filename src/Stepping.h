#ifndef STEPPING_H
#define STEPPING_H

#include <Arduino.h>
#include <vector>
#include "CommonTypes.h"

// Forward declarations
class Motor;

/**
 * @brief Handles low-level step generation timing and coordination
 */
class Stepping {
public:
    // Step engine constants
    static constexpr uint32_t STEPPER_TIMER_FREQ = 20000000; // 20 MHz step timer
    static constexpr uint32_t ACCELERATION_TICKS_PER_SECOND = 1000; // FluidNC compatible

    /**
     * @brief Initialize the stepping engine
     */
    static void init();

    /**
     * @brief Reset the stepping engine
     */
    static void reset();

    /**
     * @brief Register a motor with the stepping engine
     * @param motor Pointer to the motor
     * @param axis Index of the axis
     * @param motorNum Index of the motor on the axis
     */
    static void registerMotor(Motor* motor, size_t axis, size_t motorNum);

    /**
     * @brief Generate step pulses based on step and direction masks
     * @param stepMask Bitmask of axes that should step
     * @param dirMask Bitmask of axis directions (1 = negative)
     */
    static void step(uint8_t stepMask, uint8_t dirMask);

    /**
     * @brief Turn off all step pins
     */
    static void unstep();

    /**
     * @brief Set the period for step pulse timer
     * @param timerTicks Timer period in ticks
     */
    static void setTimerPeriod(uint32_t timerTicks);

    /**
     * @brief Start the step timer (schedules timer start)
     */
    static void startTimer();
    
    /**
     * @brief Process delayed timer start from main loop context
     * This should be called from the main loop to safely start the timer
     */
    static void processDelayedStart();

    /**
     * @brief Stop the step timer
     */
    static void stopTimer();

    /**
     * @brief Set direction pins according to mask
     * @param dirMask Direction mask
     */
    static void setDirectionMask(uint8_t dirMask);

    /**
     * @brief Get maximum possible steps per second
     * @return Max steps per second
     */
    static uint32_t maxPulsesPerSec();

    /**
     * @brief Configure step pulse length
     * @param pulseUsecs Step pulse duration in microseconds
     */
    static void setPulseLength(uint32_t pulseUsecs);

    /**
     * @brief Configure direction change delay
     * @param delayUsecs Direction change delay in microseconds
     */
    static void setDirectionDelay(uint32_t delayUsecs);
    
    /**
     * @brief Get total number of steps executed
     * @return Total step count
     */
    static uint32_t getTotalStepCount();
    
    /**
     * @brief Get number of steps executed by a specific axis
     * @param axis Axis index
     * @return Axis step count
     */
    static uint32_t getAxisStepCount(int axis);
    
    /**
     * @brief Print step counts to debug output
     */
    static void printStepCounts();
    
    /**
     * @brief Check if timer is currently running
     * @return True if timer is active
     */
    static bool isTimerRunning();
    
    /**
     * @brief Check if timer start is pending (waiting for main loop)
     * @return True if start is pending
     */
    static bool isTimerStartPending();
    
    static void forceTimerStart();

private:
    static hw_timer_t* stepTimer;
    static portMUX_TYPE timerMux;
    static volatile bool timerRunning;
    
    static uint32_t pulseUsecs;
    static uint32_t directionDelayUsecs;
    
    static std::vector<Motor*> motors[MAX_N_AXIS][2]; // Motors indexed by [axis][motorNum]
    
    static uint8_t lastDirMask;
    static int32_t axisSteps[MAX_N_AXIS];
    
    // Step counters for debugging
    static volatile uint32_t totalStepCount;
    static volatile uint32_t stepCounts[MAX_N_AXIS];
    
    static bool pulseFunc(); // Called by the timer ISR
    
    static void IRAM_ATTR onStepperTimer();
};

#endif // STEPPING_H