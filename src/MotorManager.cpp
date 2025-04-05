/**
 * @file MotorManager.cpp
 * @brief Implementation of the MotorManager class
 */

 #include "MotorManager.h"

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