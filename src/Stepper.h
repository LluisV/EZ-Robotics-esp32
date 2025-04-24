/**
 * @file Stepper.h
 * @brief Stepper motor driver: executes motion plans using stepper motors
 * Adapted from FluidNC's Stepper module
 */

 #ifndef STEPPER_H
 #define STEPPER_H
 
 #include <Arduino.h>
 #include "Stepping.h"
 
 // Segment preparation data structures
 enum PrepFlag {
     PREP_FLAG_RECALCULATE = 0,
     PREP_FLAG_HOLD_PARTIAL_BLOCK,
     PREP_FLAG_PARKING,
     PREP_FLAG_DECEL_OVERRIDE
 };
 
 // Ramp states used by step segment preparation
 #define RAMP_ACCEL 0
 #define RAMP_CRUISE 1
 #define RAMP_DECEL 2
 #define RAMP_DECEL_OVERRIDE 3
 
 // Some useful constants for the stepper subsystem
 #define DT_SEGMENT (1.0f / (float(Stepping::ACCELERATION_TICKS_PER_SECOND) * 60.0f)) // min/segment
 #define REQ_MM_INCREMENT_SCALAR 1.25f
 #define MINIMUM_JUNCTION_SPEED 0.1f // (mm/min)
 #define MINIMUM_FEED_RATE 1.0f // (mm/min)
 #define SOME_LARGE_VALUE 1.0E+38f // Used for dummy/default values that should never be used
 
 // Define Adaptive Multi-Axis Step-Smoothing (AMASS) levels
 #define MAX_AMASS_LEVEL 3
 // Each level halves the step frequency and increases steps per ISR call
 // The default thresholds are conservative to ensure stability
 
 namespace Stepper {
     /**
      * @brief Initialize the stepper subsystem
      */
     void init();
 
     /**
      * @brief Reset and clear stepper subsystem variables
      */
     void reset();
 
     /**
      * @brief Pulse step generation function
      * @return True if step generation should continue, false to stop timer
      */
     bool IRAM_ATTR pulse_func();
 
     /**
      * @brief Enable steppers and start timer interrupt
      */
     void wake_up();
 
     /**
      * @brief Disable steppers and timer interrupt
      */
     void go_idle();
 
     /**
      * @brief Stop stepping operation immediately
      */
     void IRAM_ATTR stop_stepping();
 
     /**
      * @brief Cancel the current jog motion
      */
     void cancel_jog();
 
     /**
      * @brief Update current plan block parameters
      * @return True if updated successfully
      */
     bool update_plan_block_parameters();
 
     /**
      * @brief Prepare parking motion for homing/probing
      */
     void parking_setup_buffer();
 
     /**
      * @brief Restore normal motion after parking operations
      */
     void parking_restore_buffer();
 
     /**
      * @brief Prepare segment buffer for next moves
      * Continuously called from main program to prep buffer.
      */
     void prep_buffer();
 
     /**
      * @brief Get the current real-time motion rate
      * @return Current speed in mm/min
      */
     float get_realtime_rate();
 
     /**
      * @brief Update the step segment buffer when the current block is completed
      */
     void discard_current_block();
 
     /**
      * @brief Get the current block if available
      * @return Pointer to current plan block or nullptr
      */
     struct plan_block_t* get_current_block();
 }
 
 #endif // STEPPER_H