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
    // Initialize position vector to zero for the number of available motors
    int numMotors = motorManager ? motorManager->getNumMotors() : MAX_N_AXIS;
    currentPosition.resize(numMotors, 0.0f);
    currentVelocity.resize(numMotors, 0.0f);
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
        int numMotors = motorManager->getNumMotors();
        currentPosition.resize(numMotors, 0.0f);
        
        for (int i = 0; i < numMotors; i++) {
            Motor* motor = motorManager->getMotor(i);
            if (motor) {
                currentPosition[i] = motor->getPositionInUnits();
            }
        }
    }
 
    bool Scheduler::addLinearMove(const std::vector<float>& targetPos, float feedrate) {
        try {
            if (targetPos.empty()) {
                Debug::error("Scheduler", "Empty target position in addLinearMove");
                return false;
            }
            
            if (feedrate <= 0) {
                Debug::warning("Scheduler", "Invalid feedrate in addLinearMove, using default");
                feedrate = 1000.0f; // Use safe default
            }
            
            Debug::info("Scheduler", "Adding linear move at " + String(feedrate) + " mm/min");
            return planLinearMotion(targetPos, feedrate, false);
        } catch (const std::exception& e) {
            Debug::error("Scheduler", "Exception in addLinearMove: " + String(e.what()));
            return false;
        } catch (...) {
            Debug::error("Scheduler", "Unknown exception in addLinearMove");
            return false;
        }
    }
    
    bool Scheduler::addRapidMove(const std::vector<float>& targetPos) {
        try {
            if (targetPos.empty()) {
                Debug::error("Scheduler", "Empty target position in addRapidMove");
                return false;
            }
            
            // Get max feedrate from config, with a safe default
            float rapidFeedrate = 3000.0f; // Safe default
            
            if (configManager) {
                const MachineConfig& config = configManager->getMachineConfig();
                rapidFeedrate = config.maxFeedrate;
                
                // Safety check
                if (rapidFeedrate <= 0) {
                    rapidFeedrate = 3000.0f;
                }
            }
            
            Debug::info("Scheduler", "Adding rapid move at " + String(rapidFeedrate) + " mm/min");
            return planLinearMotion(targetPos, rapidFeedrate, true);
        } catch (const std::exception& e) {
            Debug::error("Scheduler", "Exception in addRapidMove: " + String(e.what()));
            return false;
        } catch (...) {
            Debug::error("Scheduler", "Unknown exception in addRapidMove");
            return false;
        }
    }
 
    bool Scheduler::planLinearMotion(const std::vector<float>& targetPos, float feedrate, bool isRapid) {
        // Validate that we have some target position data
        if (targetPos.empty()) {
            Debug::error("Scheduler", "Target position is empty");
            return false;
        }
        
        // Validate that we have properly initialized components
        if (!motorManager) {
            Debug::error("Scheduler", "Motor manager is not available");
            return false;
        }
        
        // Get number of motors/axes in the system
        int numMotors = motorManager->getNumMotors();
        if (numMotors == 0) {
            Debug::error("Scheduler", "No motors configured");
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
        // Always ensure we have MAX_N_AXIS positions (Planner expects this)
        float target[MAX_N_AXIS];
        
        // First initialize all positions with current position
        for (int i = 0; i < MAX_N_AXIS; i++) {
            if (i < currentPosition.size()) {
                target[i] = currentPosition[i];
            } else {
                target[i] = 0.0f;
            }
        }
        
        // Then update the positions we want to move
        // Only update the axes that were specified in the targetPos
        for (size_t i = 0; i < targetPos.size() && i < MAX_N_AXIS; i++) {
            target[i] = targetPos[i];
        }
        
        // Output debug info
        String targetDebug = "Target: ";
        for (int i = 0; i < MAX_N_AXIS; i++) {
            targetDebug += String(target[i]) + " ";
        }
        Debug::verbose("Scheduler", targetDebug);
        
        // Add move to planner buffer - with error handling
        try {
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
                                       " move to " + String(targetPos[0]) + 
                                       (targetPos.size() > 1 ? "," + String(targetPos[1]) : "") + 
                                       (targetPos.size() > 2 ? "," + String(targetPos[2]) : "") + 
                                       " at " + String(feedrate) + " mm/min");
            
            return true;
        }
        catch (const std::exception& e) {
            Debug::error("Scheduler", "Exception in buffer_line: " + String(e.what()));
            return false;
        }
        catch (...) {
            Debug::error("Scheduler", "Unknown exception in buffer_line");
            return false;
        }
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