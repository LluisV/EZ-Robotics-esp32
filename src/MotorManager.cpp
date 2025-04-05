/**
 * @file MotorManager.cpp
 * @brief Implementation of the MotorManager class
 */

 #include "MotorManager.h"

 // Motor class implementation
 
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
 
 // MotorManager class implementation
 
 MotorManager::MotorManager() : configManager(nullptr) {
 }
 
 MotorManager::~MotorManager() {
   // Clean up motors
   for (Motor* motor : motors) {
     delete motor;
   }
   motors.clear();
 }
 
 bool MotorManager::initialize(ConfigManager* configManager) {
   this->configManager = configManager;
   if (!configManager) {
     Serial.println("Invalid configuration manager");
     return false;
   }
   
   // Clean up existing motors
   for (Motor* motor : motors) {
     delete motor;
   }
   motors.clear();
   
   // Initialize the engine
   engine.init();
   
   // Create motors based on configuration
   int numMotors = configManager->getNumMotors();
   for (int i = 0; i < numMotors; i++) {
     const MotorConfig* motorConfig = configManager->getMotorConfig(i);
     if (motorConfig) {
       Motor* motor = new Motor(motorConfig);
       if (motor->initialize(&engine)) {
         motors.push_back(motor);
       } else {
         delete motor;
         Serial.println("Failed to initialize motor " + motorConfig->name);
       }
     }
   }
   
   Serial.println("Initialized " + String(motors.size()) + " motors");
   return motors.size() > 0;
 }
 
 Motor* MotorManager::getMotor(int index) {
   if (index >= 0 && index < motors.size()) {
     return motors[index];
   }
   return nullptr;
 }
 
 Motor* MotorManager::getMotorByName(const String& name) {
   for (Motor* motor : motors) {
     if (motor->getName() == name) {
       return motor;
     }
   }
   return nullptr;
 }
 
 int MotorManager::getNumMotors() const {
   return motors.size();
 }
 
 bool MotorManager::update() {
   for (Motor* motor : motors) {
     motor->update();
   }
   return true;
 }
 
 bool MotorManager::isAnyMotorMoving() const {
   for (const Motor* motor : motors) {
     if (motor->isMoving()) {
       return true;
     }
   }
   return false;
 }
 
 bool MotorManager::moveToSynchronized(const std::vector<long>& positions) {
   if (positions.size() != motors.size()) {
     Serial.println("Position array size mismatch");
     return false;
   }
   
   // Check if any motor is homing
   for (const Motor* motor : motors) {
     if (motor->getStatus() == HOMING) {
       Serial.println("Cannot move while homing");
       return false;
     }
   }
   
   // Calculate the move duration for each motor
   struct MotorMoveInfo {
     long distance;
     long maxStepsPerSecond;
     float duration;
   };
   
   std::vector<MotorMoveInfo> moveInfos(motors.size());
   float maxDuration = 0.0f;
   
   for (size_t i = 0; i < motors.size(); i++) {
     MotorMoveInfo& info = moveInfos[i];
     
     // Calculate distance to move
     info.distance = positions[i] - motors[i]->getPosition();
     
     // Get max speed
     info.maxStepsPerSecond = motors[i]->getConfig()->maxSpeed;
     
     // Calculate unconstrained duration
     info.duration = abs(info.distance) / (float)info.maxStepsPerSecond;
     
     // Track maximum duration
     if (info.duration > maxDuration) {
       maxDuration = info.duration;
     }
   }
   
   // Start moves with adjusted speeds to synchronize completion
   for (size_t i = 0; i < motors.size(); i++) {
     const MotorMoveInfo& info = moveInfos[i];
     
     if (abs(info.distance) > 0) {
       // Calculate adjusted speed to match the longest duration
       int adjustedSpeed;
       if (maxDuration > 0.0f && info.duration > 0.0f) {
         adjustedSpeed = abs(info.distance) / maxDuration;
       } else {
         adjustedSpeed = info.maxStepsPerSecond;
       }
       
       // Ensure speed is within limits
       adjustedSpeed = constrain(adjustedSpeed, 1, info.maxStepsPerSecond);
       
       // Start the move
       motors[i]->moveTo(positions[i], adjustedSpeed);
     }
   }
   
   return true;
 }
 
 bool MotorManager::moveToSynchronizedUnits(const std::vector<float>& positions) {
   std::vector<long> stepsPositions(positions.size());
   
   for (size_t i = 0; i < positions.size() && i < motors.size(); i++) {
     stepsPositions[i] = motors[i]->unitsToSteps(positions[i]);
   }
   
   return moveToSynchronized(stepsPositions);
 }
 
 void MotorManager::stopAll(bool immediate) {
   for (Motor* motor : motors) {
     motor->stop(immediate);
   }
 }
 
 bool MotorManager::homeAll() {
   bool success = true;
   
   // Home Z first (if it exists)
   Motor* zMotor = getMotorByName("Z");
   if (zMotor) {
     success &= zMotor->startHoming();
     
     // Wait for Z homing to complete
     while (zMotor->getStatus() == HOMING) {
       update();
       delay(10);
     }
   }
   
   // Home X next
   Motor* xMotor = getMotorByName("X");
   if (xMotor) {
     success &= xMotor->startHoming();
     
     // Wait for X homing to complete
     while (xMotor->getStatus() == HOMING) {
       update();
       delay(10);
     }
   }
   
   // Home Y last
   Motor* yMotor = getMotorByName("Y");
   if (yMotor) {
     success &= yMotor->startHoming();
     
     // Wait for Y homing to complete
     while (yMotor->getStatus() == HOMING) {
       update();
       delay(10);
     }
   }
   
   // Home any remaining motors
   for (Motor* motor : motors) {
     if (motor->getName() != "X" && motor->getName() != "Y" && motor->getName() != "Z") {
       success &= motor->startHoming();
       
       // Wait for this motor's homing to complete
       while (motor->getStatus() == HOMING) {
         update();
         delay(10);
       }
     }
   }
   
   return success;
 }
 
 bool MotorManager::homeMotor(int motorIndex) {
   Motor* motor = getMotor(motorIndex);
   if (motor) {
     return motor->startHoming();
   }
   return false;
 }
 
 bool MotorManager::homeMotorByName(const String& motorName) {
   Motor* motor = getMotorByName(motorName);
   if (motor) {
     return motor->startHoming();
   }
   return false;
 }