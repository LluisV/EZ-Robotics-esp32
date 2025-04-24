/**
 * @file Stepper.cpp
 * @brief Implementation of the stepper module
 * Adapted from FluidNC's Stepper implementation
 */

 #include "Stepper.h"
 #include "Planner.h"
 #include "MotorManager.h"
 #include "Debug.h"
 
 // Define step segment buffer structures
 struct st_block_t {
     uint32_t steps[MAX_N_AXIS];
     uint32_t step_event_count;
     uint8_t  direction_bits;
     bool     is_pwm_rate_adjusted;
 };
 
 // Primary stepper segment buffer
 struct segment_t {
     uint16_t n_step;              // Number of steps to execute
     uint16_t isrPeriod;           // Timer period for this segment
     uint8_t  st_block_index;      // Block data index for this segment
     uint8_t  amass_level;         // AMASS level for this segment
 };
 
 // Buffer sizes
 #define SEGMENT_BUFFER_SIZE 12
 #define BLOCK_BUFFER_SIZE 16
 
 // Variables and buffers
 static st_block_t* st_block_buffer = nullptr;
 static segment_t* segment_buffer = nullptr;
 
 // Stepper ISR data
 static struct {
     uint32_t counter[MAX_N_AXIS];  // Bresenham counters
     uint8_t  step_bits;            // Step output bits
     uint8_t  execute_step;         // Flag to execute steps
     uint8_t  step_outbits;         // Next step bits to output
     uint8_t  dir_outbits;          // Direction bits
     uint32_t steps[MAX_N_AXIS];    // Steps for current segment
     
     uint16_t step_count;           // Steps remaining in segment
     uint8_t  exec_block_index;     // Current block index
     st_block_t* exec_block;        // Pointer to current block
     segment_t*  exec_segment;      // Pointer to current segment
 } st;
 
 // Segment buffer indices
 static volatile uint32_t segment_buffer_tail;
 static volatile uint32_t segment_buffer_head;
 static uint32_t segment_next_head;
 
 // Prep data - Use the plan_block_t from Planner
 static plan_block_t* pl_block;
 static st_block_t* st_prep_block;
 
 // Segment preparation data
 static struct {
     uint8_t  st_block_index;
     uint8_t  recalculate_flag;
     
     float dt_remainder;
     float steps_remaining;
     float step_per_mm;
     float req_mm_increment;
     
     uint8_t last_st_block_index;
     float   last_steps_remaining;
     float   last_step_per_mm;
     float   last_dt_remainder;
     
     uint8_t ramp_type;
     float   mm_complete;
     float   current_speed;
     float   maximum_speed;
     float   exit_speed;
     float   accelerate_until;
     float   decelerate_after;
     
     float inv_rate;
 } prep;
 
 // AMASS level thresholds (based on timer ticks)
 static const uint32_t amass_threshold = Stepping::STEPPER_TIMER_FREQ / 8000;
 
 static bool awake = false;
 static void* mc_pl_data_inflight = nullptr;
 
 // Convert mm position to motor steps
 static inline int32_t mm_to_steps(float mm, int axis) {
     // This would need to be implemented based on your machine configuration
     // For now, just using a simple conversion factor
     float steps_per_mm = 80.0f; // Example value
     return (int32_t)(mm * steps_per_mm);
 }
 
 // Convert steps to mm position
 static inline float steps_to_mm(int32_t steps, int axis) {
     // Inverse of mm_to_steps
     float steps_per_mm = 80.0f; // Example value
     return steps / steps_per_mm;
 }
 
 namespace Stepper {
 
 void IRAM_ATTR stop_stepping() {
     Stepping::unstep();
     st.step_outbits = 0;
 }
 
 bool IRAM_ATTR pulse_func() {
     // This is called by the stepping interrupt - it's the heart of the step generation system
     
     // Safety check - if we're not supposed to be awake
     if (!awake) {
         return false;
     }
     
     // Reset step outputs
     Stepping::unstep();
     st.step_outbits = 0;
     
     // If there is no current segment, try to get one from the buffer
     if (st.exec_segment == nullptr) {
         // Check if we have segments to execute
         if (segment_buffer_head != segment_buffer_tail) {
             // Initialize new step segment and load number of steps to execute
             st.exec_segment = &segment_buffer[segment_buffer_tail];
             
             // Update ISR timer period for this segment
             Stepping::setTimerPeriod(st.exec_segment->isrPeriod);
             st.step_count = st.exec_segment->n_step;
             
             // If new block, initialize stepper variables and counters
             if (st.exec_block_index != st.exec_segment->st_block_index) {
                 st.exec_block_index = st.exec_segment->st_block_index;
                 st.exec_block = &st_block_buffer[st.exec_block_index];
                 
                 // Initialize Bresenham counters
                 for (int axis = 0; axis < MAX_N_AXIS; axis++) {
                     st.counter[axis] = st.exec_block->step_event_count >> 1;
                 }
             }
             
             // Set direction bits
             st.dir_outbits = st.exec_block->direction_bits;
             
             // Adjust Bresenham axis increments based on AMASS level
             for (int axis = 0; axis < MAX_N_AXIS; axis++) {
                 st.steps[axis] = st.exec_block->steps[axis] >> st.exec_segment->amass_level;
             }
         } else {
             // Segment buffer empty. Shutdown.
             stop_stepping();
             awake = false;
             return false; // Tell timer to stop
         }
     }
     
     // Execute step displacement profile using Bresenham line algorithm
     for (int axis = 0; axis < MAX_N_AXIS; axis++) {
         st.counter[axis] += st.steps[axis];
         if (st.counter[axis] > st.exec_block->step_event_count) {
             bitSet(st.step_outbits, axis);
             st.counter[axis] -= st.exec_block->step_event_count;
         }
     }
     
     // Generate the step pulses for all axes
     Stepping::step(st.step_outbits, st.dir_outbits);
     
     // Track remaining steps
     st.step_count--;
     if (st.step_count == 0) {
         // Segment is complete. Discard current segment and advance index.
         st.exec_segment = nullptr;
         segment_buffer_tail = (segment_buffer_tail + 1) % SEGMENT_BUFFER_SIZE;
     }
     
     return true; // Continue pulsing
 }
 
 void wake_up() {
     if (awake) {
         return;
     }
     
     awake = true;
     
     // Enable stepper drivers, if needed
     // This would interface with your motor management system
     
     // Start the step timer
     Stepping::startTimer();
 }
 
 void go_idle() {
     awake = false;
     stop_stepping();
     
     // In your system, you'd want to disable motors after a timeout
     // This could involve a call to your motor management system
 }
 
 void prep_buffer() {
     // Skip processing if motion should be stopped
     if (!awake) {
         return;
     }
     
     // Keep filling the buffer until it's full
     while (segment_buffer_tail != segment_next_head) {
         // Determine if we need to load a new planner block or recompute the current one
         if (pl_block == nullptr) {
             // Get next planner block from the buffer
             pl_block = Planner::get_current_block();
             
             if (pl_block == nullptr) {
                 return; // No more blocks available
             }
             
             // Check if we need to recompute or load a new block
             if (prep.recalculate_flag & (1 << PREP_FLAG_RECALCULATE)) {
                 prep.recalculate_flag &= ~(1 << PREP_FLAG_RECALCULATE);
             } else {
                 // Load Bresenham stepping data for the new block
                 prep.st_block_index = (prep.st_block_index + 1) % SEGMENT_BUFFER_SIZE;
                 st_prep_block = &st_block_buffer[prep.st_block_index];
                 
                 // Copy direction bits and calculate steps
                 st_prep_block->direction_bits = pl_block->direction_bits;
                 
                 // Scale steps by AMASS max level to avoid rounding errors
                 for (int idx = 0; idx < MAX_N_AXIS; idx++) {
                     st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL;
                 }
                 st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
                 
                 // Initialize segment buffer data
                 prep.steps_remaining = (float)pl_block->step_event_count;
                 prep.step_per_mm = prep.steps_remaining / pl_block->millimeters;
                 prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR / prep.step_per_mm;
                 prep.dt_remainder = 0.0f;
                 
                 // Determine initial speed
                 prep.current_speed = sqrtf(pl_block->entry_speed_sqr);
                 
                 // Set pwm adjustments flag
                 st_prep_block->is_pwm_rate_adjusted = false;
             }
             
             // Compute the velocity profile and acceleration parameters
             prep.mm_complete = 0.0f;
             float inv_2_accel = 0.5f / pl_block->acceleration;
             
             // Determine ramp type: acceleration, cruise, or deceleration
             prep.ramp_type = RAMP_ACCEL;
             prep.accelerate_until = pl_block->millimeters;
             
             // Calculate the exit speed and deceleration point
             prep.exit_speed = sqrtf(Planner::get_exec_block_exit_speed_sqr());
             
             float nominal_speed = Planner::compute_profile_nominal_speed(pl_block);
             float nominal_speed_sqr = nominal_speed * nominal_speed;
             
             // Compute acceleration and deceleration points
             if (nominal_speed_sqr > pl_block->entry_speed_sqr) {
                 // Accelerate to nominal speed
                 prep.accelerate_until = inv_2_accel * (nominal_speed_sqr - pl_block->entry_speed_sqr);
                 
                 // Calculate deceleration point
                 if (nominal_speed_sqr > 0.0f) {
                     prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - prep.exit_speed * prep.exit_speed);
                 } else {
                     // Pure deceleration move
                     prep.decelerate_after = 0.0f;
                 }
                 
                 // Determine if the move is a pure acceleration, acceleration+deceleration, or pure deceleration
                 if (prep.accelerate_until >= pl_block->millimeters) {
                     // Pure acceleration or deceleration
                     prep.accelerate_until = pl_block->millimeters;
                     prep.decelerate_after = pl_block->millimeters;
                     prep.ramp_type = (prep.accelerate_until == 0.0f) ? RAMP_DECEL : RAMP_ACCEL;
                 } else if (prep.decelerate_after >= pl_block->millimeters) {
                     // Pure acceleration
                     prep.decelerate_after = pl_block->millimeters;
                 } else {
                     // Acceleration + deceleration
                     // No adjustments needed
                 }
             } else {
                 // Pure deceleration
                 prep.ramp_type = RAMP_DECEL;
                 prep.accelerate_until = 0.0f;
                 prep.decelerate_after = 0.0f;
             }
             
             prep.maximum_speed = nominal_speed;
         }
         
         // Initialize new segment
         segment_t* prep_segment = &segment_buffer[segment_buffer_head];
         
         // Set new segment to point to the current segment data block
         prep_segment->st_block_index = prep.st_block_index;
         
         /* --- Compute new segment parameters ---
          * This is where we calculate the motion trajectory for the segment. We iterate
          * through time, computing speeds and positions to create a velocity profile.
          */
         float dt_max = DT_SEGMENT;  // Maximum segment time
         float dt = 0.0f;            // Initialize segment time
         float time_var = dt_max;    // Time working variable
         float mm_var;               // mm-Distance working variable
         float speed_var;            // Speed working variable
         float mm_remaining = pl_block->millimeters; // New segment distance
         float minimum_mm = mm_remaining - prep.req_mm_increment; // Guarantee at least one step
         
         if (minimum_mm < 0.0f) {
             minimum_mm = 0.0f;
         }
         
         do {
             // Determine acceleration, cruise, or deceleration phase
             switch (prep.ramp_type) {
                 case RAMP_ACCEL:
                     // Compute acceleration
                     speed_var = pl_block->acceleration * time_var;
                     mm_remaining -= time_var * (prep.current_speed + 0.5f * speed_var);
                     
                     if (mm_remaining <= prep.accelerate_until) {
                         // End of acceleration ramp
                         mm_remaining = prep.accelerate_until;
                         
                         // Check if we need to transition to cruise or deceleration
                         if (mm_remaining == prep.decelerate_after) {
                             prep.ramp_type = RAMP_DECEL;
                         } else {
                             prep.ramp_type = RAMP_CRUISE;
                         }
                         
                         prep.current_speed = prep.maximum_speed;
                     } else {
                         // Continue acceleration
                         prep.current_speed += speed_var;
                     }
                     break;
                     
                 case RAMP_CRUISE:
                     // Cruising at constant speed
                     mm_var = mm_remaining - prep.maximum_speed * time_var;
                     
                     if (mm_var < prep.decelerate_after) {
                         // Transition to deceleration
                         time_var = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
                         mm_remaining = prep.decelerate_after;
                         prep.ramp_type = RAMP_DECEL;
                     } else {
                         // Continue cruising
                         mm_remaining = mm_var;
                     }
                     break;
                     
                 case RAMP_DECEL:
                     // Compute deceleration
                     speed_var = pl_block->acceleration * time_var;
                     
                     if (prep.current_speed > speed_var) {
                         // Normal deceleration
                         mm_var = mm_remaining - time_var * (prep.current_speed - 0.5f * speed_var);
                         
                         if (mm_var > prep.mm_complete) {
                             // Continue deceleration
                             mm_remaining = mm_var;
                             prep.current_speed -= speed_var;
                             break;
                         }
                     }
                     
                     // Either at or below zero speed, or at end of block
                     time_var = 2.0f * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
                     mm_remaining = prep.mm_complete;
                     prep.current_speed = prep.exit_speed;
                     break;
                     
                 case RAMP_DECEL_OVERRIDE:
                     // Handle deceleration override (for feed holds)
                     speed_var = pl_block->acceleration * time_var;
                     
                     if (prep.current_speed > speed_var) {
                         mm_var = mm_remaining - time_var * (prep.current_speed - 0.5f * speed_var);
                         
                         if (mm_var > prep.mm_complete) {
                             mm_remaining = mm_var;
                             prep.current_speed -= speed_var;
                             break;
                         }
                     }
                     
                     // At end of deceleration
                     time_var = 2.0f * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
                     mm_remaining = prep.mm_complete;
                     prep.current_speed = prep.exit_speed;
                     break;
             }
             
             dt += time_var; // Add computed time to total segment time
             
             // If segment time is incomplete, keep computing
             if (dt < dt_max) {
                 time_var = dt_max - dt;
             } else {
                 // Segment time complete or minimum steps reached
                 if (mm_remaining > minimum_mm) {
                     // Ensure at least one step in segment
                     dt_max += DT_SEGMENT;
                     time_var = dt_max - dt;
                 } else {
                     break; // Exit loop, segment is complete
                 }
             }
         } while (mm_remaining > prep.mm_complete);
         
         /* --- Compute segment timing and steps --- */
         float step_dist_remaining = prep.step_per_mm * mm_remaining;
         float n_steps_remaining = ceilf(step_dist_remaining);
         float last_n_steps_remaining = ceilf(prep.steps_remaining);
         
         // Compute number of steps for this segment
         prep_segment->n_step = (uint16_t)(last_n_steps_remaining - n_steps_remaining);
         
         // Bail if no steps (can happen at end of a feed hold)
         if (prep_segment->n_step == 0) {
             // If in hold, this would exit
             return;
         }
         
         // Compute segment step timing
         dt += prep.dt_remainder; // Apply previous segment remainder
         float inv_rate = dt / (last_n_steps_remaining - step_dist_remaining);
         
         // Convert to timer ticks
         uint32_t timer_ticks = (uint32_t)ceilf((Stepping::STEPPER_TIMER_FREQ * 60.0f) * inv_rate);
         
         // Compute AMASS level for smooth stepping
         int level;
         for (level = 0; level < MAX_AMASS_LEVEL; level++) {
             if (timer_ticks < (amass_threshold << level)) {
                 break;
             }
         }
         
         // Apply the AMASS level
         prep_segment->amass_level = level;
         prep_segment->n_step <<= level;
         
         // Ensure timer period is within 16-bit limit
         prep_segment->isrPeriod = (timer_ticks > 0xffff) ? 0xffff : timer_ticks;
         
         // Segment complete! Update buffer indices
         segment_buffer_head = segment_next_head;
         segment_next_head = (segment_next_head + 1) % SEGMENT_BUFFER_SIZE;
         
         // Update planner and prep data for next segment
         pl_block->millimeters = mm_remaining;
         prep.steps_remaining = n_steps_remaining;
         prep.dt_remainder = (n_steps_remaining - step_dist_remaining) * inv_rate;
         
         // Check for end of block conditions
         if (mm_remaining == prep.mm_complete) {
             // End of block
             if (mm_remaining > 0.0f) {
                 // End of forced-termination
                 // This would handle a feed hold or abort
                 return;
             } else {
                 // Normal end of block
                 pl_block = nullptr; // Mark block as completed
                 Planner::discard_current_block();
             }
         }
     }
 }
 
 bool update_plan_block_parameters() {
     if (pl_block != nullptr) {
         prep.recalculate_flag |= (1 << PREP_FLAG_RECALCULATE);
         pl_block->entry_speed_sqr = prep.current_speed * prep.current_speed;
         pl_block = nullptr; // Force reload and check of active velocity profile
         return true;
     }
     return false;
 }
 
 void parking_setup_buffer() {
     // Store execution data for partial block, if needed
     if (prep.recalculate_flag & (1 << PREP_FLAG_HOLD_PARTIAL_BLOCK)) {
         prep.last_st_block_index = prep.st_block_index;
         prep.last_steps_remaining = prep.steps_remaining;
         prep.last_dt_remainder = prep.dt_remainder;
         prep.last_step_per_mm = prep.step_per_mm;
     }
     
     // Set flags for parking mode
     prep.recalculate_flag |= (1 << PREP_FLAG_PARKING);
     prep.recalculate_flag &= ~(1 << PREP_FLAG_RECALCULATE);
     pl_block = nullptr; // Always reset on parking maneuver
 }
 
 void parking_restore_buffer() {
     // Restore execution data from partial block, if needed
     if (prep.recalculate_flag & (1 << PREP_FLAG_HOLD_PARTIAL_BLOCK)) {
         st_prep_block = &st_block_buffer[prep.last_st_block_index];
         prep.st_block_index = prep.last_st_block_index;
         prep.steps_remaining = prep.last_steps_remaining;
         prep.dt_remainder = prep.last_dt_remainder;
         prep.step_per_mm = prep.last_step_per_mm;
         prep.recalculate_flag |= (1 << PREP_FLAG_RECALCULATE);
         prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR / prep.step_per_mm;
     } else {
         prep.recalculate_flag = 0;
     }
     
     pl_block = nullptr; // Set to reload next block
 }
 
 float get_realtime_rate() {
     // Return the current move speed (useful for reporting)
     return prep.current_speed;
 }
 
 void cancel_jog() {
     if (mc_pl_data_inflight != nullptr) {
         plan_line_data_t* pl_data = (plan_line_data_t*)mc_pl_data_inflight;
         if (pl_data->is_jog) {
             mc_pl_data_inflight = nullptr;
         }
     }
 }
 
 // Fix the get_current_block function to match header declaration
 // and properly return the type from the Planner module
 struct plan_block_t* get_current_block() {
     // This would return the currently executing block from the planner
     return Planner::get_current_block();
 }
 
 void discard_current_block() {
     // Forward to planner
     Planner::discard_current_block();
 }
 
 void init() {
     Debug::info("Stepper", "Initializing stepper system");
     
     // Allocate buffers
     if (st_block_buffer) {
         delete[] st_block_buffer;
     }
     st_block_buffer = new st_block_t[SEGMENT_BUFFER_SIZE];
     
     if (segment_buffer) {
         delete[] segment_buffer;
     }
     segment_buffer = new segment_t[SEGMENT_BUFFER_SIZE];
     
     // Initialize everything to a clean state
     reset();
 }
 
 void reset() {
     // Initialize stepping driver
     Stepping::reset();
     
     // Go to idle state
     go_idle();
     
     // Initialize stepper algorithm variables
     memset(&prep, 0, sizeof(prep));
     memset(&st, 0, sizeof(st));
     
     st.exec_segment = nullptr;
     pl_block = nullptr;
     
     segment_buffer_tail = 0;
     segment_buffer_head = 0;
     segment_next_head = 1;
     
     st.step_outbits = 0;
     st.dir_outbits = 0;
 }
 
 } // namespace Stepper