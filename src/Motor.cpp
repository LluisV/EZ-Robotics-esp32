/**
 * @file Motor.cpp
 * @brief Implementation of the Motor class adapted to use FluidNC-style stepping
 */

 #include "Motor.h"
 #include "Debug.h"
 #include "Stepping.h"
 
 Motor::Motor(const MotorConfig *config)
     : config(config),
       status(IDLE),
       endstopTriggered(false),
       homingStartPosition(0),
       lastEndstopCheckTime(0),
       homingPhase(0),
       currentSpeed(0),
       currentAcceleration(0),
       axis(-1),
       motorNum(-1),
       currentPosition(0),
       targetPosition(0)
 {
 }
 
 bool Motor::initialize() {
     if (!config) {
         Debug::error("Motor", "Invalid motor configuration");
         return false;
     }
 
     // Set pin modes
     pinMode(config->stepPin, OUTPUT);
     pinMode(config->dirPin, OUTPUT);
     
     if (config->endstopPin >= 0) {
         pinMode(config->endstopPin, INPUT_PULLUP);
         Debug::verbose("Motor", "Configured endstop pin " + String(config->endstopPin) +
                                 " for motor " + config->name);
     }
 
     // Initial pin states
     digitalWrite(config->stepPin, config->invertStep ? HIGH : LOW);
     digitalWrite(config->dirPin, config->invertDirection ? HIGH : LOW);
 
     // Set initial position to home position
     currentPosition = unitsToSteps(config->homePosition);
     targetPosition = currentPosition;
 
     Debug::info("Motor", "Initialized motor " + config->name +
                          " (Step: " + String(config->stepPin) +
                          ", Dir: " + String(config->dirPin) +
                          ", MaxSpeed: " + String(config->maxSpeed) +
                          ", Initial Pos: " + String(config->homePosition) + ")");
     
     return true;
 }
 
 void Motor::setPosition(long position) {
     currentPosition = position;
     targetPosition = position;
 }
 
 long Motor::getPosition() const {
     return currentPosition;
 }
 
 float Motor::getPositionInUnits() const {
     return stepsToUnits(currentPosition);
 }
 
 long Motor::getTargetPosition() const {
     return targetPosition;
 }
 
 long Motor::unitsToSteps(float units) const {
     if (!config)
         return 0;
 
     float stepsPerUnit;
     if (config->type == LINEAR_AXIS) {
         // For linear axes (using lead screw)
         stepsPerUnit = (config->stepsPerRev * config->reduction) / config->leadScrewPitch;
     } else {
         // For angular axes (direct rotation)
         stepsPerUnit = (config->stepsPerRev * config->reduction) / 360.0;
     }
 
     // Apply position inversion if configured
     if (config->invertDirection) {
         stepsPerUnit = -stepsPerUnit;
     }
 
     return (long)(units * stepsPerUnit);
 }
 
 float Motor::stepsToUnits(long steps) const {
     if (!config)
         return 0.0;
 
     float unitsPerStep;
     if (config->type == LINEAR_AXIS) {
         // For linear axes (using lead screw)
         unitsPerStep = config->leadScrewPitch / (config->stepsPerRev * config->reduction);
     } else {
         // For angular axes (direct rotation)
         unitsPerStep = 360.0 / (config->stepsPerRev * config->reduction);
     }
 
     // Apply position inversion if configured
     if (config->invertDirection) {
         unitsPerStep = -unitsPerStep;
     }
 
     return steps * unitsPerStep;
 }
 
 bool Motor::moveTo(long position, float speedInHz) {
     if (status == HOMING) {
         Debug::warning("Motor", "Cannot move motor " + getName() + " during homing");
         return false;
     }
 
     // Set target position
     targetPosition = position;
     
     // Set speed if specified, use default max speed otherwise
     if (speedInHz > 0) {
         currentSpeed = speedInHz;
     } else if (currentSpeed > 0) {
         // Use stored speed
     } else {
         currentSpeed = config->maxSpeed;
     }
     
     // Note: The actual motion control is handled by the Stepper/Planner system
     // This just registers the intent to move
     
     status = MOVING;
     
     Debug::verbose("Motor", getName() + " moving to position " + String(position) +
                           " at speed " + String(currentSpeed));
     
     return true;
 }
 
 bool Motor::moveToUnits(float position, float speed) {
     long steps = unitsToSteps(position);
     int stepsPerSec = speed > 0.0f ? unitsToSteps(speed) : 0;
     return moveTo(steps, stepsPerSec);
 }
 
 bool Motor::moveRelative(long steps, float speed) {
     return moveTo(currentPosition + steps, speed);
 }
 
 bool Motor::moveRelativeUnits(float units, float speed) {
     long steps = unitsToSteps(units);
     int stepsPerSec = speed > 0.0f ? unitsToSteps(speed) : 0;
     return moveRelative(steps, stepsPerSec);
 }
 
 void Motor::stop(bool immediate) {
     // In a FluidNC-style system, stops are handled at the planner level
     // This would interface with that system to stop this specific motor
     
     if (status != HOMING) {
         status = IDLE;
     }
 }
 
 bool Motor::isMoving() const {
     // In a real implementation, this would check if the motor is still
     // executing steps from the Stepper module
     return status == MOVING;
 }
 
 MotorStatus Motor::getStatus() const {
     return status;
 }
 
 bool Motor::startHoming() {
     if (!config || config->endstopPin < 0) {
         Debug::error("Motor", "Cannot home " + config->name + ": No endstop configured");
         return false;
     }
 
     // Save current position
     homingStartPosition = currentPosition;
 
     // Set initial status
     endstopTriggered = false;
     status = HOMING;
 
     Debug::info("Motor", "Starting homing sequence for " + config->name);
 
     // Check if endstop is already triggered
     if (isEndstopTriggered()) {
         Debug::warning("Motor", config->name + " endstop already triggered at start, backing off first");
         homingPhase = 2;
         
         // Calculate backoff distance in steps
         long backoffSteps;
         if (config->type == LINEAR_AXIS) {
             backoffSteps = unitsToSteps(config->backoffDistance);
         } else {
             backoffSteps = unitsToSteps(config->backoffDistance);
         }
         
         // Move in the opposite direction of homing
         moveRelative(-config->homingDirection * backoffSteps);
         
         Debug::info("Motor", config->name + " backing off " + String(config->backoffDistance) + 
                            " units, steps=" + String(backoffSteps) + 
                            ", direction=" + String(-config->homingDirection));
     }
     else
     {
         // Normal case: Start with phase 1 (moving to endstop)
         homingPhase = 1;
         // Set homing speed
         setSpeed(config->homeSpeed);
         // Start moving toward the endstop
         moveRelative(config->homingDirection * 1000000); // Large value to ensure we hit the endstop
         Debug::info("Motor", "Moving toward endstop in direction " + String(config->homingDirection));
     }
 
     lastEndstopCheckTime = millis();
     return true;
 }
 
 bool Motor::update() {
     // In a full implementation, this would check motion completion
     // and update the current position based on the stepper system
     
     // For now, simulate completion by updating position if we're close to target
     if (status == MOVING && abs(targetPosition - currentPosition) < 10) {
         currentPosition = targetPosition;
         status = IDLE;
         Debug::verbose("Motor", config->name + " movement completed. Status: IDLE");
     }
 
     // Handle homing state
     if (status == HOMING) {
         const unsigned long currentTime = millis();
 
         // Check endstop every 10ms
         if (currentTime - lastEndstopCheckTime >= 10) {
             lastEndstopCheckTime = currentTime;
 
             switch (homingPhase) {
                 case 1: // Phase 1: Moving to endstop
                 if (isEndstopTriggered()) {
                     Debug::info("Motor", config->name + " endstop triggered in phase 1");
                     
                     // Endstop hit, stop immediately
                     stop(true);
                     delay(100); // Short delay for physical stop
                     
                     // Phase 2: Back off from endstop
                     homingPhase = 2;
                     setSpeed(config->maxSpeed);
                     
                     // Calculate backoff steps
                     long backoffSteps = unitsToSteps(config->backoffDistance);
                     
                     // Move in the opposite direction of homing
                     moveRelative(-config->homingDirection * backoffSteps);
                     
                     Debug::info("Motor", "Backing off " + String(config->backoffDistance) +
                                      " units in direction " + String(-config->homingDirection));
                 }
                 break;
 
                 case 2: // Phase 2: Backing off from endstop
                 if (!isMoving()) {
                     // Backoff complete, approach endstop slowly
                     homingPhase = 3;
                     setSpeed(config->homeSpeed / 4);     // Slower approach
                     moveRelative(config->homingDirection * 1000000); // Large value to ensure we hit the endstop
                     
                     Debug::verbose("Motor", "Slowly approaching " + config->name + " endstop");
                 }
                 break;
 
                 case 3: // Phase 3: Slow approach to endstop
                 if (isEndstopTriggered()) {
                     Debug::info("Motor", config->name + " endstop triggered in phase 3");
                     
                     // Endstop hit again, stop immediately
                     stop(true);
                     delay(100); // Short delay for physical stop
                     
                     // Set current position to endstop position
                     setPosition(unitsToSteps(config->endstopPosition));
                     
                     // If home position differs from endstop position, move to home
                     if (fabs(config->homePosition - config->endstopPosition) > 0.001f) {
                         // Calculate steps to move
                         long homeSteps = unitsToSteps(config->homePosition);
                         long endstopSteps = unitsToSteps(config->endstopPosition);
                         long stepsToMove = homeSteps - endstopSteps;
                         
                         Debug::info("Motor", config->name + " moving from endstop to home position");
                         
                         // Move to home position
                         setSpeed(config->homeSpeed / 2);
                         moveRelative(stepsToMove);
                         
                         // Set phase 4 to wait for this movement to complete
                         homingPhase = 4;
                     }
                     else {
                         // Home is same as endstop, we're done
                         status = IDLE;
                         homingPhase = 0;
                         Debug::info("Motor", config->name + " homing complete");
                     }
                 }
                 break;
 
                 case 4: // Phase 4: Moving from endstop to home position
                 if (!isMoving()) {
                     status = IDLE;
                     homingPhase = 0;
                     Debug::info("Motor", config->name + " homing complete");
                 }
                 break;
             }
         }
     }
 
     return true;
 }
 
 const MotorConfig *Motor::getConfig() const {
     return config;
 }
 
 String Motor::getName() const {
     return config ? config->name : "Unknown";
 }
 
 bool Motor::isEndstopTriggered() const {
     if (config->endstopPin < 0) {
         return false;
     }
 
     bool state = digitalRead(config->endstopPin) == HIGH;
     bool triggered = config->endstopInverted ? !state : state;
 
     Debug::verbose("Motor", config->name + " endstop check: " +
                               String(triggered ? "TRIGGERED" : "OPEN") +
                               " (raw state: " + String(state) +
                               ", inverted: " + String(config->endstopInverted) + ")");
 
     return triggered;
 }
 
 void Motor::setSpeed(int speedInSteps) {
     currentSpeed = speedInSteps;
 }
 
 void Motor::setAcceleration(int acceleration) {
     currentAcceleration = acceleration;
 }
 
 int Motor::getCurrentSpeedSteps() const {
     if (!isMoving()) {
         return 0;
     }
     
     return currentSpeed;
 }