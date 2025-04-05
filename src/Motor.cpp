/**
 * @file Motor.cpp
 * @brief Implementation of the Motor class
 */

 #include "Motor.h"

 Motor::Motor(const MotorConfig* config) 
   : config(config), 
     stepper(nullptr), 
     status(IDLE), 
     endstopTriggered(false), 
     homingStartPosition(0),
     lastEndstopCheckTime(0),
     homingPhase(0)
 {
 }
 
 bool Motor::initialize(FastAccelStepperEngine* engine) {
   if (!config || !engine) {
     Serial.println("Invalid motor configuration or engine");
     return false;
   }
   
   // Create and configure the stepper
   stepper = engine->stepperConnectToPin(config->stepPin);
   if (!stepper) {
     Serial.println("Failed to connect stepper to pin " + String(config->stepPin));
     return false;
   }
   
   stepper->setDirectionPin(config->dirPin);
   stepper->setAcceleration(config->acceleration);
   stepper->setSpeedInHz(config->maxSpeed);
   
   // Configure endstop pin if defined
   if (config->endstopPin >= 0) {
     pinMode(config->endstopPin, INPUT_PULLUP);
   }
   
   Serial.println("Motor " + config->name + " initialized successfully");
   return true;
 }
 
 void Motor::setPosition(long position) {
   if (stepper) {
     stepper->setCurrentPosition(position);
   }
 }
 
 long Motor::getPosition() const {
   if (stepper) {
     return stepper->getCurrentPosition();
   }
   return 0;
 }
 
 float Motor::getPositionInUnits() const {
   return stepsToUnits(getPosition());
 }
 
 long Motor::getTargetPosition() const {
   if (stepper) {
     return stepper->targetPos();
   }
   return 0;
 }
 
 long Motor::unitsToSteps(float units) const {
   if (!config) return 0;
   
   float stepsPerUnit;
   if (config->type == LINEAR_AXIS) {
     // For linear axes (using lead screw)
     stepsPerUnit = (config->stepsPerRev * config->reduction) / config->leadScrewPitch;
   } else {
     // For angular axes (direct rotation)
     stepsPerUnit = (config->stepsPerRev * config->reduction) / 360.0;
   }
   
   return (long)(units * stepsPerUnit);
 }
 
 float Motor::stepsToUnits(long steps) const {
   if (!config) return 0.0;
   
   float unitsPerStep;
   if (config->type == LINEAR_AXIS) {
     // For linear axes (using lead screw)
     unitsPerStep = config->leadScrewPitch / (config->stepsPerRev * config->reduction);
   } else {
     // For angular axes (direct rotation)
     unitsPerStep = 360.0 / (config->stepsPerRev * config->reduction);
   }
   
   return steps * unitsPerStep;
 }
 
 bool Motor::moveTo(long position, int speed) {
   if (!stepper || status == HOMING) {
     return false;
   }
   
   // Set speed if specified
   if (speed > 0) {
     stepper->setSpeedInHz(speed);
   } else {
     stepper->setSpeedInHz(config->maxSpeed);
   }
   
   // Start the move
   stepper->moveTo(position);
   status = MOVING;
   
   return true;
 }
 
 bool Motor::moveToUnits(float position, float speed) {
   long steps = unitsToSteps(position);
   int stepsPerSec = speed > 0.0f ? unitsToSteps(speed) : 0;
   return moveTo(steps, stepsPerSec);
 }
 
 bool Motor::moveRelative(long steps, int speed) {
   if (!stepper || status == HOMING) {
     return false;
   }
   
   // Set speed if specified
   if (speed > 0) {
     stepper->setSpeedInHz(speed);
   } else {
     stepper->setSpeedInHz(config->maxSpeed);
   }
   
   // Start the move
   stepper->move(steps);
   status = MOVING;
   
   return true;
 }
 
 bool Motor::moveRelativeUnits(float units, float speed) {
   long steps = unitsToSteps(units);
   int stepsPerSec = speed > 0.0f ? unitsToSteps(speed) : 0;
   return moveRelative(steps, stepsPerSec);
 }
 
 void Motor::stop(bool immediate) {
   if (!stepper) {
     return;
   }
   
   if (immediate) {
     stepper->forceStop();
   } else {
     stepper->stopMove();
   }
   
   if (status != HOMING) {
     status = IDLE;
   }
 }
 
 bool Motor::isMoving() const {
   if (!stepper) {
     return false;
   }
   
   return stepper->isRunning();
 }
 
 MotorStatus Motor::getStatus() const {
   return status;
 }
 
 bool Motor::startHoming() {
   if (!stepper || config->endstopPin < 0) {
     Serial.println("Cannot home " + config->name + ": No endstop configured");
     return false;
   }
   
   // Save current position
   homingStartPosition = getPosition();
   
   // Set initial status
   endstopTriggered = false;
   status = HOMING;
   
   // Check if endstop is already triggered
   if (isEndstopTriggered()) {
     Serial.println(config->name + " endstop already triggered at start, backing off first");
     // Start with phase 2 (backing off)
     homingPhase = 2;
     stepper->setSpeedInHz(config->maxSpeed);
     stepper->move(-config->homingDirection * unitsToSteps(config->backoffDistance));
   } else {
     // Normal case: Start with phase 1 (moving to endstop)
     homingPhase = 1;
     // Set homing speed
     stepper->setSpeedInHz(config->homeSpeed);
     // Start moving toward the endstop
     stepper->move(config->homingDirection * 1000000); // Large value to ensure we hit the endstop
     Serial.println("Started homing for " + config->name);
   }
   
   lastEndstopCheckTime = millis();
   return true;
 }
 
 bool Motor::update() {
   if (!stepper) {
     return false;
   }
   
   // Update status if movement finished
   if (status == MOVING && !stepper->isRunning()) {
     status = IDLE;
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
             // Endstop hit, stop immediately
             stepper->forceStop();
             delay(100); // Short delay for physical stop
             
             // Phase 2: Back off from endstop
             homingPhase = 2;
             stepper->setSpeedInHz(config->maxSpeed);
             stepper->move(-config->homingDirection * unitsToSteps(config->backoffDistance));
             
             Serial.println(config->name + " endstop triggered, backing off");
           }
           break;
           
         case 2: // Phase 2: Backing off from endstop
           if (!stepper->isRunning()) {
             // Backoff complete, approach endstop slowly
             homingPhase = 3;
             stepper->setSpeedInHz(config->homeSpeed / 4); // Slower approach
             stepper->move(config->homingDirection * 1000000); // Large value to ensure we hit the endstop
             
             Serial.println(config->name + " approaching endstop slowly");
           }
           break;
           
         case 3: // Phase 3: Slow approach to endstop
           if (isEndstopTriggered()) {
             // Endstop hit again, stop immediately
             stepper->forceStop();
             delay(100); // Short delay for physical stop
             
             // Set current position as zero or reference position
             stepper->setCurrentPosition(0);
             status = IDLE;
             homingPhase = 0;
             
             Serial.println(config->name + " homing complete");
           }
           break;
       }
     }
   }
   
   return true;
 }
 
 const MotorConfig* Motor::getConfig() const {
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
   return config->endstopInverted ? !state : state;
 }