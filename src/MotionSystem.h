/**
 * @file MotionSystem.h
 * @brief Integration helpers for the FluidNC-style motion system
 */

 #ifndef MOTION_SYSTEM_H
 #define MOTION_SYSTEM_H
 
 #include "Stepping.h"
 #include "Stepper.h"
 #include "Planner.h"
 #include "MachineController.h"
 #include "MotorManager.h"
 #include "ConfigManager.h"
 
 namespace MotionSystem {
     /**
      * @brief Initialize the complete motion system
      * @param machineController Machine controller
      * @param motorManager Motor manager
      * @param configManager Configuration manager
      * @return True if initialization was successful
      */
     bool init(MachineController* machineController, MotorManager* motorManager, ConfigManager* configManager);
     
     /**
      * @brief Register motors with the stepping system
      * @param motorManager Motor manager with configured motors
      */
     void registerMotors(MotorManager* motorManager);
     
     /**
      * @brief Apply machine configuration to motion system
      * @param configManager Configuration manager
      */
     void applyConfiguration(ConfigManager* configManager);
     
     /**
      * @brief Background update function to call periodically
      * Should be called from the main loop
      */
     void update();
     
     /**
      * @brief Stop all motion immediately
      * Emergency stop for the motion system
      */
     void emergencyStop();
 }
 
 #endif // MOTION_SYSTEM_H