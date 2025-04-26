/**
 * @file MachineController.h
 * @brief Simplified machine controller for the pipeline architecture
 */

 #ifndef MACHINE_CONTROLLER_H
 #define MACHINE_CONTROLLER_H
 
 #include <Arduino.h>
 #include <vector>
 #include "MotorManager.h"
 #include "ConfigManager.h"
 #include "CommonTypes.h"
 
 /**
  * @brief Controller class for the CNC machine
  * 
  * Handles machine state, position tracking, and basic machine operations.
  * In the pipeline architecture, this serves as the interface between the
  * higher-level components and the physical machine.
  */
 class MachineController
 {
 public:
   /**
    * @brief Construct a new Machine Controller object
    * @param motorManager Motor manager
    * @param configManager Configuration manager
    */
   MachineController(MotorManager *motorManager, ConfigManager *configManager);
 
   /**
    * @brief Initialize the machine controller
    * @return True if successful, false otherwise
    */
   bool initialize();
 
   /**
    * @brief Home all axes
    * @return True if successful, false otherwise
    */
   bool homeAll();
 
   /**
    * @brief Home specific axis
    * @param axisName Axis name
    * @return True if successful, false otherwise
    */
   bool homeAxis(const String &axisName);
 
   /**
    * @brief Get the current position in world coordinates (machine coordinates)
    * @return Vector of current world positions
    */
   std::vector<float> getCurrentWorldPosition() const;
 
   /**
    * @brief Get the current position in work coordinates
    * @return Vector of current work positions
    */
   std::vector<float> getCurrentWorkPosition() const;
 
   /**
    * @brief Get the work offset
    * @return Vector of work offsets
    */
   std::vector<float> getWorkOffset() const
   {
     return workOffset;
   }
 
   /**
    * @brief Set the work coordinate system offset
    * @param offsets Array of offsets for each axis
    */
   void setWorkOffset(const std::vector<float> &offsets);
 
   /**
    * @brief Convert work coordinates to machine coordinates
    * @param workPos Work position
    * @return Machine positions
    */
   std::vector<float> workToMachinePositions(const std::vector<float> &workPos) const;
 
   /**
    * @brief Convert machine coordinates to work coordinates
    * @param machinePos Machine position
    * @return Work positions
    */
   std::vector<float> machineToWorkPositions(const std::vector<float> &machinePos) const;
 
   /**
    * @brief Get the current axis velocity vector
    * @return Vector of velocities for each axis (mm/min)
    */
   std::vector<float> getCurrentVelocityVector() const;
 
   /**
    * @brief Get the current velocity magnitude
    * @return Current velocity in mm/min
    */
   float getCurrentVelocity() const;
 
   /**
    * @brief Get the desired velocity vector
    * @return Vector of desired velocities (mm/min)
    */
   std::vector<float> getCurrentDesiredVelocityVector() const;
 
   /**
    * @brief Set the current desired velocity vector
    * @param velocities Vector of velocities (mm/min)
    */
   void setCurrentDesiredVelocityVector(std::vector<float> velocities);
 
   /**
    * @brief Get the current feedrate
    * @return Current feedrate in mm/min
    */
   float getCurrentFeedrate() const;
 
   /**
    * @brief Check if the machine is currently moving
    * @return True if any axis is moving, false otherwise
    */
   bool isMoving() const;
 
   /**
    * @brief Emergency stop
    */
   void emergencyStop();
 
   /**
    * @brief Pause current movement (for GRBL feedhold support)
    */
   void pauseMovement();
 
   /**
    * @brief Resume movement after pause (for GRBL resume support)
    */
   void resumeMovement();
 
   /**
    * @brief Get the MotorManager
    * @return MotorManager pointer
    */
   MotorManager *getMotorManager() const
   {
     return motorManager;
   }
 
   /**
    * @brief Get pointer to the ConfigManager
    * @return Pointer to the ConfigManager instance
    */
   ConfigManager *getConfigManager() const
   {
     return configManager;
   }
 
   /**
    * @brief Check if the machine is in absolute mode
    * @return True if in absolute mode, false if relative
    */
   bool isAbsoluteMode() const
   {
     return absoluteMode;
   }
 
   /**
    * @brief Set absolute or relative mode
    * @param absolute True for absolute mode, false for relative
    */
   void setAbsoluteMode(bool absolute)
   {
     absoluteMode = absolute;
   }
 
 private:
   MotorManager *motorManager;     ///< Motor manager
   ConfigManager *configManager;   ///< Configuration manager
   std::vector<float> workOffset;  ///< Work coordinate system offset
   std::vector<float> desiredVelocityVector;  ///< Desired axis velocities
   float currentFeedrate;          ///< Current feedrate in mm/min
   bool absoluteMode;              ///< True if in absolute mode, false if in relative mode
 
   /**
    * @brief Apply machine limits to positions
    * @param machinePos Machine positions
    * @return Constrained machine positions
    */
   std::vector<float> applyMachineLimits(const std::vector<float> &machinePos) const;
 
   /**
    * @brief Validate a position against machine limits
    * @param motorName Motor name
    * @param position Position to validate
    * @param clampToLimits Whether to clamp to limits
    * @param clampedPosition Output for clamped position
    * @return True if valid, false otherwise
    */
   bool validatePosition(const String &motorName, float position, bool clampToLimits, float &clampedPosition);
 };
 
 #endif // MACHINE_CONTROLLER_H