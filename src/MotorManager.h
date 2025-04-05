/**
 * @file MotorManager.h
 * @brief Motor manager for the CNC controller
 */

 #ifndef MOTOR_MANAGER_H
 #define MOTOR_MANAGER_H
 
 #include <Arduino.h>
 #include <FastAccelStepper.h>
 #include <vector>
 #include "ConfigManager.h"
 
 /**
  * @brief Motor status enumeration
  */
 enum MotorStatus {
   IDLE,       ///< Motor is idle
   MOVING,     ///< Motor is moving
   HOMING,     ///< Motor is homing
   ERROR       ///< Motor has an error
 };
 
 /**
  * @brief Motor class representing a single motor
  */
 class Motor {
 public:
   /**
    * @brief Construct a new Motor object
    * @param config Motor configuration
    */
   Motor(const MotorConfig* config);
   
   /**
    * @brief Initialize the motor
    * @param engine FastAccelStepperEngine reference
    * @return True if successful, false otherwise
    */
   bool initialize(FastAccelStepperEngine* engine);
   
   /**
    * @brief Set the current motor position
    * @param position Position in steps
    */
   void setPosition(long position);
   
   /**
    * @brief Get the current motor position
    * @return Current position in steps
    */
   long getPosition() const;
   
   /**
    * @brief Get the current motor position in user units (mm or degrees)
    * @return Current position in user units
    */
   float getPositionInUnits() const;
   
   /**
    * @brief Get the motor position at the last move command
    * @return Target position in steps
    */
   long getTargetPosition() const;
   
   /**
    * @brief Convert user units to steps
    * @param units User units (mm or degrees)
    * @return Steps
    */
   long unitsToSteps(float units) const;
   
   /**
    * @brief Convert steps to user units
    * @param steps Steps
    * @return User units (mm or degrees)
    */
   float stepsToUnits(long steps) const;
   
   /**
    * @brief Move the motor to a target position
    * @param position Target position in steps
    * @param speed Speed in steps per second (optional)
    * @return True if the move was started, false otherwise
    */
   bool moveTo(long position, int speed = 0);
   
   /**
    * @brief Move the motor to a target position in user units
    * @param position Target position in user units (mm or degrees)
    * @param speed Speed in user units per second (optional)
    * @return True if the move was started, false otherwise
    */
   bool moveToUnits(float position, float speed = 0.0f);
   
   /**
    * @brief Move the motor relative to current position
    * @param steps Number of steps to move
    * @param speed Speed in steps per second (optional)
    * @return True if the move was started, false otherwise
    */
   bool moveRelative(long steps, int speed = 0);
   
   /**
    * @brief Move the motor relative to current position in user units
    * @param units User units to move (mm or degrees)
    * @param speed Speed in user units per second (optional)
    * @return True if the move was started, false otherwise
    */
   bool moveRelativeUnits(float units, float speed = 0.0f);
   
   /**
    * @brief Stop the motor
    * @param immediate If true, stop immediately without deceleration
    */
   void stop(bool immediate = false);
   
   /**
    * @brief Check if the motor is moving
    * @return True if the motor is moving, false otherwise
    */
   bool isMoving() const;
   
   /**
    * @brief Get the motor status
    * @return Motor status
    */
   MotorStatus getStatus() const;
   
   /**
    * @brief Start the homing sequence
    * @return True if homing was started, false otherwise
    */
   bool startHoming();
   
   /**
    * @brief Update the motor state (must be called regularly)
    * @return True if the motor state was updated, false otherwise
    */
   bool update();
   
   /**
    * @brief Get the motor configuration
    * @return Motor configuration
    */
   const MotorConfig* getConfig() const;
   
   /**
    * @brief Get the motor name
    * @return Motor name
    */
   String getName() const;
   
   /**
    * @brief Check if the endstop is triggered
    * @return True if endstop is triggered, false otherwise
    */
   bool isEndstopTriggered() const;
   
 private:
   const MotorConfig* config;          ///< Motor configuration
   FastAccelStepper* stepper;          ///< FastAccelStepper object
   MotorStatus status;                 ///< Current motor status
   bool endstopTriggered;              ///< Flag to track if endstop was triggered during homing
   long homingStartPosition;           ///< Starting position for homing
   unsigned long lastEndstopCheckTime; ///< Time of last endstop check
   int homingPhase;                    ///< Current phase in the homing sequence
 };
 
 /**
  * @brief Manager class for multiple motors
  */
 class MotorManager {
 public:
   MotorManager();
   ~MotorManager();
   
   /**
    * @brief Initialize all motors based on configuration
    * @param configManager Configuration manager
    * @return True if all motors were initialized, false otherwise
    */
   bool initialize(ConfigManager* configManager);
   
   /**
    * @brief Get motor by index
    * @param index Motor index
    * @return Motor pointer or nullptr if not found
    */
   Motor* getMotor(int index);
   
   /**
    * @brief Get motor by name
    * @param name Motor name
    * @return Motor pointer or nullptr if not found
    */
   Motor* getMotorByName(const String& name);
   
   /**
    * @brief Get number of motors
    * @return Number of motors
    */
   int getNumMotors() const;
   
   /**
    * @brief Update all motors
    * @return True if all motors were updated, false otherwise
    */
   bool update();
   
   /**
    * @brief Check if any motor is moving
    * @return True if any motor is moving, false otherwise
    */
   bool isAnyMotorMoving() const;
   
   /**
    * @brief Perform synchronized move of multiple motors
    * @param positions Array of target positions in steps
    * @return True if the move was started, false otherwise
    */
   bool moveToSynchronized(const std::vector<long>& positions);
   
   /**
    * @brief Perform synchronized move of multiple motors in user units
    * @param positions Array of target positions in user units
    * @return True if the move was started, false otherwise
    */
   bool moveToSynchronizedUnits(const std::vector<float>& positions);
   
   /**
    * @brief Stop all motors
    * @param immediate If true, stop immediately without deceleration
    */
   void stopAll(bool immediate = false);
   
   /**
    * @brief Start homing sequence for all motors
    * @return True if homing was started for all motors, false otherwise
    */
   bool homeAll();
   
   /**
    * @brief Home a specific motor
    * @param motorIndex Motor index
    * @return True if homing was started, false otherwise
    */
   bool homeMotor(int motorIndex);
   
   /**
    * @brief Home a specific motor by name
    * @param motorName Motor name
    * @return True if homing was started, false otherwise
    */
   bool homeMotorByName(const String& motorName);
   
 private:
   FastAccelStepperEngine engine;         ///< FastAccelStepper engine
   std::vector<Motor*> motors;            ///< Motors
   ConfigManager* configManager;          ///< Configuration manager
 };
 
 #endif // MOTOR_MANAGER_H