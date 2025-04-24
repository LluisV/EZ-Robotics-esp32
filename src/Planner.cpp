/**
 * @file Planner.cpp
 * @brief Implementation of motion planning system
 * Adapted from FluidNC's Planner implementation (spindle/coolant removed)
 */

 #include "Planner.h"
 #include "Stepper.h"
 #include "Debug.h"
 #include "ConfigManager.h"
 #include "MotorManager.h"
 
 #include <cstdlib>
 #include <cmath>
 
 // Planner buffers and state variables
 static plan_block_t* block_buffer = nullptr;
 static uint8_t block_buffer_tail;    // Index of the block to process now
 static uint8_t block_buffer_head;    // Index of the next block to be pushed
 static uint8_t next_buffer_head;     // Next buffer head
 static uint8_t block_buffer_planned; // Index of optimally planned block
 
 // Define planner variables
 typedef struct {
     int32_t position[MAX_N_AXIS];          // Planner position (in steps)
     float previous_unit_vec[MAX_N_AXIS];   // Unit vector of previous segment
     float previous_nominal_speed;          // Nominal speed of previous segment
 } planner_t;
 static planner_t pl;
 
 // Override values
 static struct {
     uint8_t feed_rate;       // Feed rate override (0-255%)
     uint8_t rapid_rate;      // Rapid rate override (0-100%)
 } overrides = {100, 100}; // Default 100%
 
 // Buffer size setting
 static constexpr uint8_t PLANNER_BUFFER_SIZE = 16;
 
 // Helper function to get next block index in ring buffer
 static uint8_t plan_next_block_index(uint8_t block_index) {
     block_index++;
     if (block_index == PLANNER_BUFFER_SIZE) {
         block_index = 0;
     }
     return block_index;
 }
 
 // Helper function to get previous block index in ring buffer
 static uint8_t plan_prev_block_index(uint8_t block_index) {
     if (block_index == 0) {
         block_index = PLANNER_BUFFER_SIZE;
     }
     block_index--;
     return block_index;
 }
 
 /**
  * @brief Initialize the motion plan subsystem
  */
 void Planner::init() {
    Debug::info("Planner", "Initializing planner");
    
    // Define the number of blocks to allocate (avoid magic numbers)
    const uint8_t BUFFER_SIZE = 16;
    
    // Delete old block buffer if it exists
    if (block_buffer != nullptr) {
        Debug::verbose("Planner", "Releasing old block buffer");
        delete[] block_buffer;
        block_buffer = nullptr;
    }
    
    // Allocate new block buffer with error handling
    try {
        block_buffer = new plan_block_t[BUFFER_SIZE];
        if (block_buffer == nullptr) {
            Debug::error("Planner", "Failed to allocate block buffer");
            return;
        }
        // Initialize the buffer to all zeros
        memset(block_buffer, 0, sizeof(plan_block_t) * BUFFER_SIZE);
        Debug::verbose("Planner", "Block buffer allocated and initialized");
    } catch (std::bad_alloc& e) {
        Debug::error("Planner", "Memory allocation failed for block buffer: " + String(e.what()));
        return;
    } catch (...) {
        Debug::error("Planner", "Unknown error allocating block buffer");
        return;
    }
    
    // Initialize the planner state
    try {
        // Reset planner state
        memset(&pl, 0, sizeof(planner_t));
        
        // Initialize position vectors to safe defaults
        for (int i = 0; i < MAX_N_AXIS; i++) {
            pl.position[i] = 0;
            pl.previous_unit_vec[i] = 0;
        }
        
        pl.previous_nominal_speed = 0.0f;
        
        // Reset buffer pointers
        block_buffer_tail = 0;
        block_buffer_head = 0;
        next_buffer_head = 1;
        block_buffer_planned = 0;
        
        // Reset overrides to 100%
        overrides.feed_rate = 100;
        overrides.rapid_rate = 100;
        
        Debug::info("Planner", "Planner state initialized successfully");
    } catch (...) {
        Debug::error("Planner", "Error initializing planner state");
        return;
    }
}
 
 /**
  * @brief Reset the motion plan subsystem
  */
 void Planner::reset() {
     memset(&pl, 0, sizeof(planner_t));
     reset_buffer();
 }
 
 /**
  * @brief Reset only the planner buffer
  */
 void Planner::reset_buffer() {
     block_buffer_tail = 0;
     block_buffer_head = 0;
     next_buffer_head = 1;
     block_buffer_planned = 0;
 }
 
 /**
  * @brief Recalculate the motion plan
  * Optimizes velocities between blocks based on junction constraints
  */
 static void planner_recalculate() {
     // Skip if buffer is empty
     if (block_buffer_head == block_buffer_tail) {
         return;
     }
     
     // Initialize block index to the last block in the buffer
     uint8_t block_index = plan_prev_block_index(block_buffer_head);
     
     // Exit if only one block to plan
     if (block_index == block_buffer_planned) {
         return;
     }
     
     // Reverse Pass: Traverse the buffer backwards and maximize deceleration
     float entry_speed_sqr;
     plan_block_t* next;
     plan_block_t* current = &block_buffer[block_index];
     
     // Calculate maximum entry speed for last block (exit speed is always zero)
     current->entry_speed_sqr = std::min(current->max_entry_speed_sqr, 
                                        2.0f * current->acceleration * current->millimeters);
     
     block_index = plan_prev_block_index(block_index);
     
     // Only two blocks? Reverse pass is complete.
     if (block_index == block_buffer_planned) {
         // Notify stepper module of new block if it's at the tail
         if (block_index == block_buffer_tail) {
             Stepper::update_plan_block_parameters();
         }
     } else {
         // Three or more blocks - continue reverse pass
         while (block_index != block_buffer_planned) {
             next = current;
             current = &block_buffer[block_index];
             block_index = plan_prev_block_index(block_index);
             
             // Update stepper parameters for the tail block
             if (block_index == block_buffer_tail) {
                 Stepper::update_plan_block_parameters();
             }
             
             // Calculate maximum entry speed based on deceleration to next block's entry speed
             if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
                 entry_speed_sqr = next->entry_speed_sqr + 
                                  2.0f * current->acceleration * current->millimeters;
                 
                 if (entry_speed_sqr < current->max_entry_speed_sqr) {
                     current->entry_speed_sqr = entry_speed_sqr;
                 } else {
                     current->entry_speed_sqr = current->max_entry_speed_sqr;
                 }
             }
         }
     }
     
     // Forward Pass: Find acceleration limits going forward
     next = &block_buffer[block_buffer_planned];
     block_index = plan_next_block_index(block_buffer_planned);
     
     while (block_index != block_buffer_head) {
         current = next;
         next = &block_buffer[block_index];
         
         // If acceleration is detected, move the optimal plan pointer forward
         if (current->entry_speed_sqr < next->entry_speed_sqr) {
             entry_speed_sqr = current->entry_speed_sqr + 
                              2.0f * current->acceleration * current->millimeters;
             
             // If true, current block is accelerating and we can move the planned pointer
             if (entry_speed_sqr < next->entry_speed_sqr) {
                 next->entry_speed_sqr = entry_speed_sqr;
                 block_buffer_planned = block_index;
             }
         }
         
         // If next block is at max entry speed, we can move the planned pointer forward as well
         if (next->entry_speed_sqr == next->max_entry_speed_sqr) {
             block_buffer_planned = block_index;
         }
         
         block_index = plan_next_block_index(block_index);
     }
 }
 
 /**
  * @brief Convert delta vector to unit vector and return magnitude
  * @param vector Vector to be converted (modified in place)
  * @return Magnitude of the vector
  */
 static float convert_delta_vector_to_unit_vector(float* vector) {
     // Calculate the vector's magnitude
     float magnitude = 0.0f;
     for (int i = 0; i < MAX_N_AXIS; i++) {
         magnitude += vector[i] * vector[i];
     }
     magnitude = sqrtf(magnitude);
     
     // Normalize the vector if not zero
     if (magnitude > 0.0f) {
         for (int i = 0; i < MAX_N_AXIS; i++) {
             vector[i] /= magnitude;
         }
     }
     
     return magnitude;
 }
 
 /**
  * @brief Limit the acceleration based on axis constraints
  * @param unit_vec Unit vector of the move
  * @return Limited acceleration value
  */
 static float limit_acceleration_by_axis_maximum(const float* unit_vec) {
     // Start with a high value and reduce based on constraints
     float limit = SOME_LARGE_VALUE;
     
     // This would be a full implementation that checks each axis's max acceleration
     // For now, using a simple default value
     float max_accel = 500.0f; // Default mm/min^2
     
     // In a full implementation, we'd check each axis max accel
     // and scale based on the unit vector component
     for (int i = 0; i < MAX_N_AXIS; i++) {
         if (fabsf(unit_vec[i]) > 0.0f) {
             float axis_limit = max_accel / fabsf(unit_vec[i]);
             limit = fminf(limit, axis_limit);
         }
     }
     
     return limit;
 }
 
 /**
  * @brief Limit the feed rate based on axis constraints
  * @param unit_vec Unit vector of the move
  * @return Limited feed rate value
  */
 static float limit_rate_by_axis_maximum(const float* unit_vec) {
     // Start with a high value and reduce based on constraints
     float limit = SOME_LARGE_VALUE;
     
     // Default max rates per axis
     float max_rate = 3000.0f; // Default mm/min
     
     // In a full implementation, we'd check each axis max rate
     // and scale based on the unit vector component
     for (int i = 0; i < MAX_N_AXIS; i++) {
         if (fabsf(unit_vec[i]) > 0.0f) {
             float axis_limit = max_rate / fabsf(unit_vec[i]);
             limit = fminf(limit, axis_limit);
         }
     }
     
     return limit;
 }
 
/**
 * @brief Convert millimeters to steps
 * @param mm Distance in millimeters
 * @param axis Axis index
 * @return Number of steps
 */
int32_t mm_to_steps(float mm, int axis) {
    // Safety check for global variable
    extern MotorManager* g_motorManager; // Declare external reference 
    
    // Default conversion factor if motor manager is unavailable
    float stepsPerMm = 80.0f; // Standard value, will be overridden if motor info available
    
    // Safely attempt to get more accurate information if motorManager is available
    if (g_motorManager != nullptr) {
        Motor* motor = g_motorManager->getMotor(axis);
        if (motor != nullptr) {
            // Use proper conversion from the motor's configuration
            return motor->unitsToSteps(mm);
        }
    }
    
    // Fallback to simple calculation if motor information not available
    return (int32_t)(mm * stepsPerMm);
}

/**
 * @brief Convert steps to millimeters
 * @param steps Number of steps
 * @param axis Axis index
 * @return Distance in millimeters
 */
float steps_to_mm(int32_t steps, int axis) {
    // Safety check for global variable
    extern MotorManager* g_motorManager; // Declare external reference
    
    // Default conversion factor if motor manager is unavailable
    float stepsPerMm = 80.0f; // Standard value, will be overridden if motor info available
    
    // Safely attempt to get more accurate information if motorManager is available
    if (g_motorManager != nullptr) {
        Motor* motor = g_motorManager->getMotor(axis);
        if (motor != nullptr) {
            // Use proper conversion from the motor's configuration
            return motor->stepsToUnits(steps);
        }
    }
    
    // Fallback to simple calculation if motor information not available
    return steps / stepsPerMm;
}
 
 /**
  * @brief Add a new linear movement to the buffer
  * @param target Target position in mm
  * @param pl_data Planning data for the move
  * @return True on success
  */
 bool Planner::buffer_line(float* target, plan_line_data_t* pl_data) {
    // Validate inputs with more robust error handling
    if (target == nullptr) {
        Debug::error("Planner", "Buffer line called with null target");
        return false;
    }
    
    if (pl_data == nullptr) {
        Debug::error("Planner", "Buffer line called with null planning data");
        return false;
    }
    
    // Validate block buffer exists
    if (block_buffer == nullptr) {
        Debug::error("Planner", "Block buffer is null - Planner not initialized");
        return false;
    }
    
    // Prepare and initialize new block
    if (next_buffer_head == block_buffer_tail) {
        Debug::warning("Planner", "Buffer full - cannot add move");
        return false; // Buffer is full
    }
    
    plan_block_t* block = &block_buffer[block_buffer_head];
    if (block == nullptr) {
        Debug::error("Planner", "Invalid block pointer");
        return false;
    }
    
    // Initialize the block with zeros for safety
    try {
        memset(block, 0, sizeof(plan_block_t));
    } catch (...) {
        Debug::error("Planner", "Failed to initialize block");
        return false;
    }
    
    // Copy motion parameters with error handling
    try {
        block->motion = pl_data->motion;
        block->line_number = pl_data->line_number;
        block->is_jog = pl_data->is_jog;
    } catch (...) {
        Debug::error("Planner", "Failed to copy motion parameters");
        return false;
    }
    
    
    // Convert target position to steps and calculate move distance
    int32_t target_steps[MAX_N_AXIS], position_steps[MAX_N_AXIS];
    float unit_vec[MAX_N_AXIS], delta_mm;
    
    // Get current position
    for (int i = 0; i < MAX_N_AXIS; i++) {
        try {
            position_steps[i] = pl.position[i];
            
            // Calculate target position in steps - safely handle potential exceptions
            target_steps[i] = mm_to_steps(target[i], i);
            
            // Calculate steps to move
            block->steps[i] = labs(target_steps[i] - position_steps[i]);
            
            // Update maximum step count - safely check boundaries
            if (block->steps[i] > block->step_event_count) {
                block->step_event_count = block->steps[i];
            }
            
            // Calculate distance in mm
            delta_mm = steps_to_mm(target_steps[i] - position_steps[i], i);
            unit_vec[i] = delta_mm; // Temporarily store for unit vector calculation
            
            // Set direction bits - bit enabled means negative direction
            if (delta_mm < 0.0f) {
                block->direction_bits |= (1 << i);
            }
        } catch (std::exception& e) {
            Debug::error("Planner", "Exception in buffer_line: " + String(e.what()));
            return false;
        } catch (...) {
            Debug::error("Planner", "Unknown exception in buffer_line");
            return false;
        }
    }
    
    // Check if this is a zero-length block (rare but possible)
    if (block->step_event_count == 0) {
        Debug::verbose("Planner", "Zero-length move rejected");
        return false;
    }
    
    // Calculate unit vector, millimeters of travel, and limit feed rate and acceleration
    float moveLength = 0.0f;
    try {
        // Calculate the vector magnitude
        for (int i = 0; i < MAX_N_AXIS; i++) {
            moveLength += unit_vec[i] * unit_vec[i];
        }
        moveLength = sqrtf(moveLength);
        
        // Normalize the vector
        if (moveLength > 0.000001f) {  // Avoid division by near-zero
            for (int i = 0; i < MAX_N_AXIS; i++) {
                unit_vec[i] /= moveLength;
            }
        }
        
        block->millimeters = moveLength;
    } catch (...) {
        Debug::error("Planner", "Exception calculating move length");
        return false;
    }
    
    // Set programmed feed rate based on motion type
    try {
        if (block->motion.rapidMotion) {
            block->programmed_rate = limit_rate_by_axis_maximum(unit_vec);
        } else {
            block->programmed_rate = pl_data->feed_rate;
            if (block->motion.inverseTime && moveLength > 0.0001f) {
                block->programmed_rate *= moveLength;
            }
        }
        
        // Set a minimum feedrate
        if (block->programmed_rate < MINIMUM_FEED_RATE) {
            block->programmed_rate = MINIMUM_FEED_RATE;
        }
        
        // Set a safe acceleration value
        block->acceleration = limit_acceleration_by_axis_maximum(unit_vec);
    } catch (...) {
        Debug::error("Planner", "Exception setting feed rate");
        return false;
    }
    
    // Calculate the junction speed limits
    try {
        // First block - start from rest
        if (block_buffer_head == block_buffer_tail) {
            block->entry_speed_sqr = 0.0f;
            block->max_junction_speed_sqr = 0.0f; 
        } else {
            // Calculate junction deviation based on cornering algorithm
            float junction_cos_theta = 0.0f;
            for (int i = 0; i < MAX_N_AXIS; i++) {
                junction_cos_theta -= pl.previous_unit_vec[i] * unit_vec[i];
            }
            
            // Compute junction limit
            if (junction_cos_theta > 0.999999f) {
                // Straight line or slight curve - minimal junction deviation
                block->max_junction_speed_sqr = 0.01f; // Small value to ensure smooth transitions
            } else {
                // Use a conservative junction speed limit
                block->max_junction_speed_sqr = std::min(
                    pl.previous_nominal_speed * pl.previous_nominal_speed,
                    block->programmed_rate * block->programmed_rate
                ) * 0.5f;
            }
            
            // Initial entry speed is junction limit
            block->entry_speed_sqr = block->max_junction_speed_sqr;
        }
    } catch (...) {
        Debug::error("Planner", "Exception calculating junction speed");
        return false;
    }
    
    try {
        // Update planner variables
        float nominal_speed = block->programmed_rate;
        pl.previous_nominal_speed = nominal_speed;
        
        // Update previous unit vector and planner position
        for (int i = 0; i < MAX_N_AXIS; i++) {
            pl.previous_unit_vec[i] = unit_vec[i];
            pl.position[i] = target_steps[i];
        }
        
        // Update buffer head and next buffer head
        block_buffer_head = next_buffer_head;
        next_buffer_head = plan_next_block_index(block_buffer_head);
        
        // Recalculate the plan with the new block
        planner_recalculate();
        
        Debug::verbose("Planner", "Successfully added move to buffer");
        return true;
    } catch (...) {
        Debug::error("Planner", "Exception finalizing planner state");
        return false;
    }
}
 
 /**
  * @brief Compute the nominal speed profile for a block
  * @param block Block to compute speed for
  * @return Nominal speed in mm/min
  */
 float Planner::compute_profile_nominal_speed(plan_block_t* block) {
     float nominal_speed = block->programmed_rate;
     
     // Apply overrides
     if (block->motion.rapidMotion) {
         nominal_speed *= (0.01f * overrides.rapid_rate);
     } else if (!(block->motion.noFeedOverride)) {
         nominal_speed *= (0.01f * overrides.feed_rate);
     }
     
     // Check for maximum feedrate limit
     if (nominal_speed > block->rapid_rate) {
         nominal_speed = block->rapid_rate;
     }
     
     // Ensure minimum feedrate
     if (nominal_speed < MINIMUM_FEED_RATE) {
         nominal_speed = MINIMUM_FEED_RATE;
     }
     
     return nominal_speed;
 }
 
 /**
  * @brief Update velocity profile parameters after an override change
  */
 void Planner::update_velocity_profile_parameters() {
     uint8_t block_index = block_buffer_tail;
     plan_block_t* block;
     float nominal_speed;
     float prev_nominal_speed = SOME_LARGE_VALUE; // Set high for first block
     
     // Iterate through all blocks and recompute speed profiles
     while (block_index != block_buffer_head) {
         block = &block_buffer[block_index];
         nominal_speed = compute_profile_nominal_speed(block);
         
         // Recompute max entry speed
         if (nominal_speed > prev_nominal_speed) {
             block->max_entry_speed_sqr = prev_nominal_speed * prev_nominal_speed;
         } else {
             block->max_entry_speed_sqr = nominal_speed * nominal_speed;
         }
         
         // Ensure it doesn't exceed junction limit
         if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) {
             block->max_entry_speed_sqr = block->max_junction_speed_sqr;
         }
         
         prev_nominal_speed = nominal_speed;
         block_index = plan_next_block_index(block_index);
     }
     
     // Update previous nominal speed for next incoming block
     pl.previous_nominal_speed = prev_nominal_speed;
     
     // Force planner recalculation if there are blocks in the buffer
     if (block_buffer_tail != block_buffer_head) {
         planner_recalculate();
         cycle_reinitialize();
     }
 }
 
 /**
  * @brief Sync the planner position with the machine position
  */
 void Planner::sync_position() {
     // Get current motor steps from machine
     // In a full implementation, this would get the actual motor positions
     // For now, just clear the planner positions
     memset(pl.position, 0, sizeof(pl.position));
     
     // Sync with actual motor positions
     // This would interface with your motor management system
 }
 
 /**
  * @brief Get the currently executing block
  * @return Pointer to current plan block or NULL if buffer empty
  */
 plan_block_t* Planner::get_current_block() {
     // Check if buffer is empty
     if (block_buffer_head == block_buffer_tail) {
         return nullptr;
     }
     
     return &block_buffer[block_buffer_tail];
 }
 
 /**
  * @brief Get special system motion block for homing/parking
  * @return Pointer to system motion block
  */
 plan_block_t* Planner::get_system_motion_block() {
     return &block_buffer[block_buffer_head]; // Use head for system motion
 }
 
 /**
  * @brief Discard the current block when it's completed
  */
 void Planner::discard_current_block() {
     // Only discard if buffer is not empty
     if (block_buffer_head != block_buffer_tail) {
         // Update planned pointer if needed
         if (block_buffer_tail == block_buffer_planned) {
             block_buffer_planned = plan_next_block_index(block_buffer_tail);
         }
         
         // Move to next block
         block_buffer_tail = plan_next_block_index(block_buffer_tail);
     }
 }
 
 /**
  * @brief Get the exit speed of the executing block
  * @return Exit speed squared in (mm/min)^2
  */
 float Planner::get_exec_block_exit_speed_sqr() {
     uint8_t block_index = plan_next_block_index(block_buffer_tail);
     if (block_index == block_buffer_head) {
         return 0.0f; // Exit speed is zero at end of buffer
     }
     return block_buffer[block_index].entry_speed_sqr;
 }
 
 /**
  * @brief Check if the buffer is full
  * @return True if buffer is full, false otherwise
  */
 bool Planner::check_full_buffer() {
     return block_buffer_tail == next_buffer_head;
 }
 
 /**
  * @brief Get available blocks in the planner buffer
  * @return Number of available blocks
  */
 uint8_t Planner::get_block_buffer_available() {
     if (block_buffer_head >= block_buffer_tail) {
         return (PLANNER_BUFFER_SIZE - 1) - (block_buffer_head - block_buffer_tail);
     } else {
         return block_buffer_tail - block_buffer_head - 1;
     }
 }
 
 /**
  * @brief Reinitialize the buffer for a hold cycle
  */
 void Planner::cycle_reinitialize() {
     // Reset entry speeds for blocks that are in the planner
     block_buffer_planned = block_buffer_tail;
     planner_recalculate();
 }