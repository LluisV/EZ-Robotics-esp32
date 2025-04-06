/**
 * @file Motor.h
 * @brief Individual stepper motor control with endstop support
 */

 #ifndef MOTOR_H
 #define MOTOR_H
 
 #include <Arduino.h>
 #include <FastAccelStepper.h>
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

   /**
   * @brief Set the acceleration for this motor
   * @param acceleration Acceleration in steps/second²
   */
  void setAcceleration(int acceleration);
  
  /**
   * @brief Set the speed for this motor
   * @param speedInSteps Speed in steps/second
   */
  void setSpeed(int speedInSteps);
   
 private:
   const MotorConfig* config;          ///< Motor configuration
   FastAccelStepper* stepper;          ///< FastAccelStepper object
   MotorStatus status;                 ///< Current motor status
   bool endstopTriggered;              ///< Flag to track if endstop was triggered during homing
   long homingStartPosition;           ///< Starting position for homing
   unsigned long lastEndstopCheckTime; ///< Time of last endstop check
   int homingPhase;                    ///< Current phase in the homing sequence
   int currentSpeed;                   ///< Current speed in steps/second
   int currentAcceleration; ///< Current acceleration in steps/second²
 };
 
 #endif // MOTOR_H