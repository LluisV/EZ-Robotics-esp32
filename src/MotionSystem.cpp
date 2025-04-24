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
         return;
     }
     
     Debug::info("MotionSystem", "Registering motors with stepping system");
     
     // Register all motors with the stepping system
     int numMotors = motorManager->getNumMotors();
     for (int i = 0; i < numMotors; i++) {
         Motor* motor = motorManager->getMotor(i);
         if (motor) {
             const MotorConfig* config = motor->getConfig();
             String name = motor->getName();
             
             // Determine axis and motor number
             int axis = -1;
             int motorNum = 0;
             
             if (name == "X") {
                 axis = 0;
             } else if (name == "Y") {
                 axis = 1;
             } else if (name == "Z") {
                 axis = 2;
             } else if (name.startsWith("X2")) {
                 axis = 0;
                 motorNum = 1;
             } else if (name.startsWith("Y2")) {
                 axis = 1;
                 motorNum = 1;
             } else if (name.startsWith("Z2")) {
                 axis = 2;
                 motorNum = 1;
             } else if (name == "A") {
                 axis = 3;
             } else if (name == "B") {
                 axis = 4;
             } else if (name == "C") {
                 axis = 5;
             }
             
             if (axis >= 0) {
                 // Register motor with the stepping system
                 Stepping::registerMotor(motor, axis, motorNum);
                 Debug::info("MotionSystem", "Registered motor " + name + 
                                            " as axis " + String(axis) + 
                                            " motor " + String(motorNum));
             }
         }
     }
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