/**
 * @file MotionSystem.cpp
 * @brief Implementation of motion system integration helpers
 */

 #include "MotionSystem.h"
 #include "Debug.h"
 
 namespace MotionSystem {
 
 bool init(MachineController* machineController, MotorManager* motorManager, ConfigManager* configManager) {
     Debug::info("MotionSystem", "Initializing FluidNC-style motion system");
     
     if (!machineController || !motorManager || !configManager) {
         Debug::error("MotionSystem", "Invalid controller or manager pointers");
         return false;
     }
     
     // Initialize stepping subsystem
     Stepping::init();
     
     // Apply configuration parameters
     applyConfiguration(configManager);
     
     // Register all motors with the stepping system
     registerMotors(motorManager);
     
     // Initialize planning modules
     Planner::init();
     Stepper::init();
     
     Debug::info("MotionSystem", "Motion system initialization complete");
     return true;
 }
 
 void registerMotors(MotorManager* motorManager) {
    if (!motorManager) {
        Debug::error("MotionSystem", "Cannot register motors: Motor manager is null");
        return;
    }
    
    Debug::info("MotionSystem", "Registering motors with stepping system");
    
    // Register all motors with the stepping system
    int numMotors = motorManager->getNumMotors();
    Debug::info("MotionSystem", "Found " + String(numMotors) + " motors to register");
    
    if (numMotors == 0) {
        Debug::warning("MotionSystem", "No motors found to register!");
        return;
    }
    
    // Map motors to axes based on their index, not their name
    // This assumes a standard mapping:
    // Motor 0 → X axis (axis 0)
    // Motor 1 → Y axis (axis 1) 
    // Motor 2 → Z axis (axis 2)
    for (int i = 0; i < numMotors && i < MAX_N_AXIS; i++) {
        Motor* motor = motorManager->getMotor(i);
        if (motor) {
            // Map motor index directly to corresponding axis
            int axis = i;  // Simple 1:1 mapping
            int motorNum = 0;  // Primary motor for this axis
            
            // Register motor with the stepping system
            Stepping::registerMotor(motor, axis, motorNum);
            Debug::info("MotionSystem", "Registered motor index " + String(i) + 
                                     " as axis " + String(axis) + 
                                     " motor " + String(motorNum));
        }
    }
    
    Debug::info("MotionSystem", "Motor registration complete");
}
 
 void applyConfiguration(ConfigManager* configManager) {
     if (!configManager) {
         return;
     }
     
     const MachineConfig& config = configManager->getMachineConfig();
     
     // Apply stepping parameters from the configuration
     Stepping::setPulseLength(config.stepPulseUsecs);
     Stepping::setDirectionDelay(config.directionDelayUsecs);
     
     Debug::info("MotionSystem", "Applied stepping configuration: pulseWidth=" + 
                                String(config.stepPulseUsecs) + "us, dirDelay=" + 
                                String(config.directionDelayUsecs) + "us");
 }
 
 void update() {
     // Call stepper preparation routine to load more segments
     Stepper::prep_buffer();
 }
 
 void emergencyStop() {
     Debug::warning("MotionSystem", "Emergency stop requested");
     
     // Stop stepper execution
     Stepper::stop_stepping();
     
     // Reset planner and stepper systems
     Planner::reset();
     Stepper::reset();
 }
 
 }  // namespace MotionSystem