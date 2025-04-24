/**
 * @file Scheduler.cpp
 * @brief Implementation of FluidNC-style motion scheduler
 */

 #include "Scheduler.h"
 #include "Planner.h"
 #include "Stepper.h"
 #include "MachineController.h"
 
 Scheduler::Scheduler(MachineController* machineController, MotorManager* motorManager, ConfigManager* configManager)
     : machineController(machineController),
       motorManager(motorManager),
       configManager(configManager),
       feedOverride(100),
       rapidOverride(100),
       motionInProgress(false)
 {
     // Initialize position vector to zero
     currentPosition.resize(MAX_N_AXIS, 0.0f);
     currentVelocity.resize(MAX_N_AXIS, 0.0f);
 }
 
 bool Scheduler::initialize() {
     Debug::info("Scheduler", "Initializing motion scheduler");
     
     if (!motorManager || !configManager) {
         Debug::error("Scheduler", "Invalid motor or config manager");
         return false;
     }
     
     // Initialize motion control systems
     Planner::init();
     Stepper::init();
     
     // Initialize stepping configuration
     const MachineConfig& config = configManager->getMachineConfig();
     Stepping::setPulseLength(config.stepPulseUsecs);
     Stepping::setDirectionDelay(config.directionDelayUsecs);
     
     // Get initial position from motors
     updatePositionFromMotors();
     
     // Sync planner position
     Planner::sync_position();
     
     Debug::info("Scheduler", "Motion scheduler initialized");
     return true;
 }
 
 void Scheduler::updatePositionFromMotors() {
     // Get current position from motors
     for (int axis = 0; axis < MAX_N_AXIS; axis++) {
         Motor* motor = motorManager->getMotorByName(String(axis == 0 ? 'X' : (axis == 1 ? 'Y' : (axis == 2 ? 'Z' : '?'))));
         if (motor) {
             currentPosition[axis] = motor->getPositionInUnits();
         }
     }
 }
 
 bool Scheduler::addLinearMove(const std::vector<float>& targetPos, float feedrate) {
     return planLinearMotion(targetPos, feedrate, false);
 }
 
 bool Scheduler::addRapidMove(const std::vector<float>& targetPos) {
     // Get max feedrate from config
     const MachineConfig& config = configManager->getMachineConfig();
     float rapidFeedrate = config.maxFeedrate;
     
     return planLinearMotion(targetPos, rapidFeedrate, true);
 }
 
 bool Scheduler::planLinearMotion(const std::vector<float>& targetPos, float feedrate, bool isRapid) {
     // Validate target position
     if (targetPos.size() < MAX_N_AXIS) {
         Debug::error("Scheduler", "Target position has incorrect size");
         return false;
     }
     
     // Create planning data structure for the move
     plan_line_data_t pl_data;
     memset(&pl_data, 0, sizeof(pl_data));
     
     // Configure motion parameters
     pl_data.feed_rate = feedrate;
     
     // Set motion type (rapid or feed)
     if (isRapid) {
         pl_data.motion.rapidMotion = 1;
     }  
     
     // Copy target position to float array for Planner system
     float target[MAX_N_AXIS];
     for (size_t i = 0; i < MAX_N_AXIS; i++) {
         if (i < targetPos.size()) {
             target[i] = targetPos[i];
         } else {
             target[i] = 0.0f;
         }
     }
     
     // Add move to planner buffer
     if (!Planner::buffer_line(target, &pl_data)) {
         Debug::error("Scheduler", "Failed to add move to planner");
         return false;
     }
     
     // Prepare stepper buffer for execution
     Stepper::prep_buffer();
     
     // Wake up stepper module if this is the first move
     if (!motionInProgress) {
         Stepper::wake_up();
         motionInProgress = true;
     }
     
     Debug::verbose("Scheduler", "Added " + String(isRapid ? "rapid" : "linear") + 
                                " move to " + String(targetPos[0]) + "," + 
                                String(targetPos[1]) + "," + String(targetPos[2]) + 
                                " at " + String(feedrate) + " mm/min");
     
     return true;
 }
 
 void Scheduler::processMotion() {
     // Prepare stepper buffer for execution by segmenting planner blocks
     Stepper::prep_buffer();
     
     // If stepper system has gone idle, update our motion status
     if (!Stepper::pulse_func()) {
         if (motionInProgress) {
             motionInProgress = false;
             
             // Update position from current motor positions
             updatePositionFromMotors();
             
             Debug::verbose("Scheduler", "Motion complete");
         }
     }
 }
 
 bool Scheduler::executeNextStep() {
     // For compatibility, just call process motion
     processMotion();
     return motionInProgress;
 }
 
 void Scheduler::clear() {
     Debug::info("Scheduler", "Clearing motion queue");
     
     // Reset planner and stepper systems
     Planner::reset();
     Stepper::reset();
     
     // Update our state
     motionInProgress = false;
 }
 
 bool Scheduler::hasMove() const {
     // Check if there are blocks in the planner or if motion is in progress
     return motionInProgress || !Planner::check_full_buffer();
 }
 
 bool Scheduler::isFull() const {
     // Check if planner buffer is full
     return Planner::check_full_buffer();
 }
 
 float Scheduler::getCurrentVelocity() const {
     // Get current velocity from stepper module
     return Stepper::get_realtime_rate();
 }
 
 void Scheduler::setOverrides(uint8_t feedOverride, uint8_t rapidOverride) {
     bool changed = false;
     
     // Apply feed override if changed
     if (this->feedOverride != feedOverride) {
         this->feedOverride = feedOverride;
         changed = true;
     }
     
     // Apply rapid override if changed
     if (this->rapidOverride != rapidOverride) {
         this->rapidOverride = rapidOverride;
         changed = true;
     }
     
     // Update velocity profiles if overrides changed
     if (changed) {
         Planner::update_velocity_profile_parameters();
     }
 }