/**
 * @file Scheduler.h
 * @brief Motion scheduler that integrates FluidNC motion planning and execution
 */

 #ifndef SCHEDULER_H
 #define SCHEDULER_H
 
 #include <vector>
 #include "MotorManager.h"
 #include "ConfigManager.h"
 #include "CommonTypes.h"
 #include "Debug.h"
 #include "MachineController.h"
 
 class Scheduler {
 public:
     /**
      * @brief Constructor
      * @param machineController Reference to machine controller
      * @param motorManager Reference to motor manager
      * @param configManager Reference to config manager
      */
     Scheduler(MachineController* machineController, MotorManager* motorManager, ConfigManager* configManager);
     
     /**
      * @brief Initialize the scheduler
      * @return True if successful
      */
     bool initialize();
     
     /**
      * @brief Add a linear move to the queue
      * @param targetPos Target position
      * @param feedrate Feedrate in mm/min
      * @return True if successful
      */
     bool addLinearMove(const std::vector<float>& targetPos, float feedrate);
     
     /**
      * @brief Add a rapid move to the queue
      * @param targetPos Target position
      * @return True if successful
      */
     bool addRapidMove(const std::vector<float>& targetPos);
     
     /**
      * @brief Process motion commands from the queue
      * This should be called regularly from the main loop
      */
     void processMotion();
     
     /**
      * @brief Clear all queued moves
      */
     void clear();
     
     /**
      * @brief Check if scheduler has queued moves
      * @return True if there are moves in the queue
      */
     bool hasMove() const;
 
     /**
      * @brief Check if the scheduler queue is full
      * @return True if queue is full
      */
     bool isFull() const;
     
     /**
      * @brief Get current machine velocity
      * @return Current velocity in mm/min
      */
     float getCurrentVelocity() const;
     
     /**
      * @brief Execute next step in the current segment
      * @return True if a step was executed
      */
     bool executeNextStep();
     
     /**
      * @brief Set override values for feed and rapid rates
      * @param feedOverride Feed override percentage (0-255%)
      * @param rapidOverride Rapid override percentage (0-100%)
      */
     void setOverrides(uint8_t feedOverride, uint8_t rapidOverride);
 
 private:
     MachineController* machineController;  // Reference to machine controller
     MotorManager* motorManager;            // Reference to motor manager
     ConfigManager* configManager;          // Reference to config manager
     
     // Helper methods for motion planning
     bool planLinearMotion(const std::vector<float>& targetPos, float feedrate, bool isRapid);
     void updatePositionFromMotors();
     
     // Motion controller state
     std::vector<float> currentPosition;    // Current position in mm
     std::vector<float> currentVelocity;    // Current velocity vector
     
     uint8_t feedOverride;    // Feed override percentage (0-255%)
     uint8_t rapidOverride;   // Rapid override percentage (0-100%)
     
     // Status tracking
     bool motionInProgress;   // Flag indicating if motion is in progress
 };
 
 #endif // SCHEDULER_H