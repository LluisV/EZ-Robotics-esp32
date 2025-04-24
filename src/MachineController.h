/**
 * @file MachineController.h
 * @brief Machine controller with segmented motion planner integration
 */

 #ifndef MACHINE_CONTROLLER_H
 #define MACHINE_CONTROLLER_H
 
 #include <Arduino.h>
 #include <vector>
 #include "MotorManager.h"
 class Scheduler;
 #include "Kinematics.h"
 #include "CommonTypes.h"
 
 /**
  * @brief Controller class for the CNC machine with segmented motion planning
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
    * @brief Set the segmented motion planner
    * @param motionPlanner Segmented motion planner
    */
   void setMotionPlanner(Scheduler* motionPlanner);
 
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
    * @brief Move to a position in machine coordinates
    * @param positions Array of positions for each axis
    * @param feedrate Feedrate in mm/min
    * @param movementType Movement type (RAPID, LINEAR, etc.)
    * @return True if successful, false otherwise
    */
   bool moveTo(const std::vector<float> &positions, float feedrate, MovementType movementType);
 
   /**
    * @brief Move to a position in machine coordinates
    * @param x X position
    * @param y Y position
    * @param z Z position
    * @param feedrate Feedrate in mm/min
    * @param movementType Movement type (RAPID, LINEAR, etc.)
    * @return True if successful, false otherwise
    */
   bool moveTo(float x, float y, float z, float feedrate, MovementType movementType);
 
   /**
    * @brief Get motion planner
    * @return Pointer to the motion planner
    */
   Scheduler* getMotionPlanner() { return motionPlanner; }

 
   /**
    * @brief Get the current position in world coordinates
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
    * @brief Get the current position of a specific axis
    * @param axisName Axis name
    * @return Current position in user units
    */
   float getCurrentAxisPosition(const String &axisName) const;
 
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
    * @brief Set the work coordinate system offset
    * @param offsets Array of offsets for each axis
    */
   void setWorkOffset(const std::vector<float> &offsets);
 
   /**
    * @brief Get the MotorManager
    * @return MotorManager pointer
    */
   MotorManager *getMotorManager() const
   {
     return motorManager;
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
 
   /**
    * @brief Get pointer to the ConfigManager
    * @return Pointer to the ConfigManager instance
    */
   ConfigManager *getConfigManager()
   {
     return configManager;
   }

   /**
    * @brief Get the current end effector velocity vector
    * @return Vector of velocities (mm/min) for each axis
    */
   std::vector<float> getCurrentVelocityVector() const;

   /**
    * @brief Get the current end effector scalar velocity
    * @return Current velocity in mm/min
    */
   float getCurrentVelocity() const;

    /**
    * @brief Get the current end effector desired velocity vector
    * @return Vector of velocities (mm/min) for each axis
    */
   std::vector<float> getCurrentDesiredVelocityVector() const;

   /**
    * @brief Get the current end effector desired scalar velocity
    * @return Current velocity in mm/min
    */
   float getCurrentDesiredVelocity() const;


   void setCurrentDesiredVelocityVector(std::vector<float>);
 
 private:
   MotorManager *motorManager;            ///< Motor manager
   Kinematics *kinematics;                ///< Kinematics calculator (for transform)
   std::vector<float> workOffset;         ///< Work coordinate system offset
   float currentFeedrate;                 ///< Current feedrate in mm/min
   bool absoluteMode;                     ///< True if in absolute mode, false if in relative mode
   Scheduler *motionPlanner; ///< Segmented motion planner
   std::vector<float> desiredVelocityVector; 

   /**
    * @brief Convert machine coordinates to motor positions
    * @param machinePos Machine position
    * @return Motor positions
    */
   std::vector<float> machineToMotorPositions(const std::vector<float> &machinePos);
 
   /**
    * @brief Convert work coordinates to machine coordinates
    * @param workPos Work position
    * @return Machine positions
    */
   std::vector<float> workToMachinePositions(const std::vector<float> &workPos);
 
   /**
    * @brief Validate and optionally clamp position to machine limits
    * @param motorName Motor name
    * @param position Proposed position
    * @param clampToLimits If true, clamps the value to limits; if false, returns false when out of limits
    * @param clampedPosition Output parameter for the clamped position (if clamping)
    * @return True if within limits (or clamped successfully), false if out of limits (and not clamping)
    */
   bool validatePosition(const String &motorName, float position, bool clampToLimits, float &clampedPosition);
 
   /**
    * @brief Apply machine limits to ensure positions are within bounds
    * @param machinePos Machine positions
    * @return Constrained machine positions
    */
   std::vector<float> applyMachineLimits(const std::vector<float> &machinePos);
 
   ConfigManager *configManager;
 };
 
 #endif // MACHINE_CONTROLLER_H