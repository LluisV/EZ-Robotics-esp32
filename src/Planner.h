/**
 * @file Planner.h
 * @brief Motion planner for CNC movement
 * Adapted from FluidNC's Planner implementation
 */

 #ifndef PLANNER_H
 #define PLANNER_H
 
 #include <Arduino.h>
 #include <cmath>
 #include "CommonTypes.h"
 


 
 /**
  * @namespace Planner
  * @brief Functions for velocity planning, acceleration profiles, and trajectory control
  */
 namespace Planner {
     /**
      * @brief Initialize the motion plan subsystem
      */
     void init();
 
     /**
      * @brief Reset the motion plan subsystem
      */
     void reset();
     
     /**
      * @brief Reset only the planner buffer
      */
     void reset_buffer();
 
     /**
      * @brief Add a new linear movement to the buffer
      * @param target Target position in mm
      * @param pl_data Planning data for the move
      * @return True on success
      */
     bool buffer_line(float* target, plan_line_data_t* pl_data);
 
     /**
      * @brief Get the currently executing block
      * @return Pointer to current plan block or NULL if buffer empty
      */
     plan_block_t* get_current_block();
 
     /**
      * @brief Discard the current block when it's completed
      */
     void discard_current_block();
 
     /**
      * @brief Get special system motion block for homing/parking
      * @return Pointer to system motion block
      */
     plan_block_t* get_system_motion_block();
 
     /**
      * @brief Get the exit speed of the executing block
      * @return Exit speed squared in (mm/min)^2
      */
     float get_exec_block_exit_speed_sqr();
 
     /**
      * @brief Check if the buffer is full
      * @return True if buffer is full, false otherwise
      */
     bool check_full_buffer();
 
     /**
      * @brief Compute the nominal speed profile based on overrides
      * @param block Block to compute speed for
      * @return Nominal speed in mm/min
      */
     float compute_profile_nominal_speed(plan_block_t* block);
 
     /**
      * @brief Update velocity profile parameters after an override change
      */
     void update_velocity_profile_parameters();
 
     /**
      * @brief Sync the planner position with the machine position
      */
     void sync_position();
 
     /**
      * @brief Get available blocks in the planner buffer
      * @return Number of available blocks
      */
     uint8_t get_block_buffer_available();
 
     /**
      * @brief Reinitialize the buffer for a hold cycle
      */
     void cycle_reinitialize();
 }
 
 #endif // PLANNER_H