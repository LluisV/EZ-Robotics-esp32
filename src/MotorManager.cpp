/**
 * @file MotorManager.cpp
 * @brief Implementation of the MotorManager class
 */

 #include "MotorManager.h"
 #include "Debug.h"

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
  Debug::info("MotorManager", "Starting motor initialization");

  this->configManager = configManager;
  if (!configManager) {
    Debug::error("MotorManager", "Invalid configuration manager");
    return false;
  }
  
  // Clean up existing motors
  Debug::verbose("MotorManager", "Cleaning up existing motors");
  for (Motor* motor : motors) {
    Debug::verbose("MotorManager", "Deleting motor: " + (motor ? motor->getName() : "Unknown"));
    delete motor;
  }
  motors.clear();
  
  // Initialize the engine
  Debug::info("MotorManager", "Initializing stepper engine");
  engine.init();
  
  // Create motors based on configuration
  int numMotors = configManager->getNumMotors();
  Debug::verbose("MotorManager", "Total motors in configuration: " + String(numMotors));

  int successfulInitCount = 0;
  int failedInitCount = 0;

  for (int i = 0; i < numMotors; i++) {
    const MotorConfig* motorConfig = configManager->getMotorConfig(i);
    if (motorConfig) {
      Debug::verbose("MotorManager", "Attempting to initialize motor: " + motorConfig->name);
      
      Motor* motor = new Motor(motorConfig);
      if (motor->initialize(&engine)) {
        motors.push_back(motor);
        successfulInitCount++;
        Debug::info("MotorManager", "Successfully initialized motor: " + motorConfig->name);
      } else {
        Debug::error("MotorManager", "Failed to initialize motor: " + motorConfig->name);
        delete motor;
        failedInitCount++;
      }
    } else {
      Debug::warning("MotorManager", "Null motor configuration at index " + String(i));
      failedInitCount++;
    }
  }
  
  Debug::info("MotorManager", "Motor initialization summary: " + 
              String(successfulInitCount) + " successful, " + 
              String(failedInitCount) + " failed");

  return motors.size() > 0;
}
 
Motor* MotorManager::getMotor(int index) {
  if (index >= 0 && index < motors.size()) {
    return motors[index];
  }
  
  Debug::warning("MotorManager", "Invalid motor index requested: " + String(index) + 
                 ". Total motors: " + String(motors.size()));
  return nullptr;
}
 
Motor* MotorManager::getMotorByName(const String& name) {
  for (Motor* motor : motors) {
    if (motor->getName() == name) {
      return motor;
    }
  }
  
  Debug::warning("MotorManager", "Motor not found with name: " + name);
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
      Debug::verbose("MotorManager", "Motor " + motor->getName() + " is currently moving");
      return true;
    }
  }
  
  Debug::verbose("MotorManager", "No motors are currently moving");
  return false;
}
 
 bool MotorManager::moveToSynchronized(const std::vector<long>& positions) {
  if (positions.size() != motors.size()) {
    Debug::error("MotorManager", "Position array size mismatch: " + 
                 String(positions.size()) + " vs " + String(motors.size()) + " motors");
    return false;
  }
  
  // Check if any motor is homing
  for (const Motor* motor : motors) {
    if (motor->getStatus() == HOMING) {
      Debug::warning("MotorManager", "Cannot move while " + motor->getName() + " is homing");
      return false;
    }
  }
  
  Debug::verbose("MotorManager", "Starting synchronized move with " + String(positions.size()) + " motors");
  
  // Calculate the move duration for each motor
  struct MotorMoveInfo {
    long distance;
    long maxStepsPerSecond;
    float duration;
  };
  
  std::vector<MotorMoveInfo> moveInfos(motors.size());
  float maxDuration = 0.0f;
  
  // Log move details for debugging
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
    
    Debug::verbose("MotorManager", motors[i]->getName() + 
                   " move details: Distance=" + String(info.distance) + 
                   " steps, Max Speed=" + String(info.maxStepsPerSecond) + 
                   " steps/s, Duration=" + String(info.duration) + " s");
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
      
      Debug::verbose("MotorManager", motors[i]->getName() + 
                     " adjusted speed: " + String(adjustedSpeed) + 
                     " steps/s (original distance duration: " + String(info.duration) + 
                     " s, max duration: " + String(maxDuration) + " s)");
      
      // Start the move
      motors[i]->moveTo(positions[i], adjustedSpeed);
    }
  }
  
  return true;
}
 
bool MotorManager::moveToSynchronizedUnits(const std::vector<float>& positions) {
  Debug::verbose("MotorManager", "Attempting synchronized move in units");
  
  std::vector<long> stepsPositions(positions.size());
  
  for (size_t i = 0; i < positions.size() && i < motors.size(); i++) {
    stepsPositions[i] = motors[i]->unitsToSteps(positions[i]);
    
    Debug::verbose("MotorManager", motors[i]->getName() + 
                   " units: " + String(positions[i]) + 
                   " -> steps: " + String(stepsPositions[i]));
  }
  
  return moveToSynchronized(stepsPositions);
}
 
 void MotorManager::stopAll(bool immediate) {
  Debug::info("MotorManager", "Stopping all motors" + 
              String(immediate ? " (IMMEDIATE)" : " (DECELERATE)"));
  
  for (Motor* motor : motors) {
    Debug::verbose("MotorManager", "Stopping motor " + motor->getName() + 
                   String(immediate ? " immediately" : " with deceleration"));
    motor->stop(immediate);
  }
}
 
bool MotorManager::homeAll() {
  Debug::info("MotorManager", "Starting home all sequence");
  bool success = true;
  unsigned long startTime = millis();
  
  // Home sequence prioritized order
  std::vector<String> homeOrder = {"Z", "X", "Y"};
  
  for (const String& axisName : homeOrder) {
    Motor* motor = getMotorByName(axisName);
    if (motor) {
      Debug::verbose("MotorManager", "Homing " + axisName + " axis");
      bool axisHomeSuccess = motor->startHoming();
      
      if (!axisHomeSuccess) {
        Debug::error("MotorManager", "Failed to start homing for " + axisName + " axis");
        success = false;
        continue;
      }
      
      // Wait for homing to complete
      unsigned long axisStartTime = millis();
      while (motor->getStatus() == HOMING) {
        update();
        delay(10);
        
        // Timeout check (30 seconds)
        if (millis() - axisStartTime > 30000) {
          Debug::error("MotorManager", axisName + " axis homing timed out");
          success = false;
          break;
        }
      }
      
      Debug::info("MotorManager", axisName + " axis homing " + 
                  String(motor->getStatus() != HOMING ? "COMPLETED" : "FAILED"));
    } else {
      Debug::warning("MotorManager", axisName + " axis motor not found");
    }
  }
  
  // Home any remaining motors not in the main axes
  Debug::verbose("MotorManager", "Checking for additional motors to home");
  for (Motor* motor : motors) {
    if (motor->getName() != "X" && motor->getName() != "Y" && motor->getName() != "Z") {
      Debug::info("MotorManager", "Homing additional motor: " + motor->getName());
      
      bool additionalAxisSuccess = motor->startHoming();
      
      if (!additionalAxisSuccess) {
        Debug::error("MotorManager", "Failed to start homing for additional motor: " + motor->getName());
        success = false;
        continue;
      }
      
      // Wait for homing to complete
      unsigned long additionalAxisStartTime = millis();
      while (motor->getStatus() == HOMING) {
        update();
        delay(10);
        
        // Timeout check (30 seconds)
        if (millis() - additionalAxisStartTime > 30000) {
          Debug::error("MotorManager", motor->getName() + " additional axis homing timed out");
          success = false;
          break;
        }
      }
      
      Debug::info("MotorManager", motor->getName() + " additional axis homing " + 
                  String(motor->getStatus() != HOMING ? "COMPLETED" : "FAILED"));
    }
  }
  
  // Overall homing process summary
  unsigned long totalTime = millis() - startTime;
  Debug::info("MotorManager", "Home all sequence " + 
              String(success ? "COMPLETED SUCCESSFULLY" : "FAILED") + 
              " in " + String(totalTime) + " ms");
  
  return success;
}
 
 bool MotorManager::homeMotor(int motorIndex) {
  Motor* motor = getMotor(motorIndex);
  if (motor) {
    Debug::info("MotorManager", "Starting homing for motor at index " + String(motorIndex));
    return motor->startHoming();
  }
  
  Debug::error("MotorManager", "Cannot home motor at index " + String(motorIndex) + 
               ". Motor not found.");
  return false;
}
 
bool MotorManager::homeMotorByName(const String& motorName) {
  Motor* motor = getMotorByName(motorName);
  if (motor) {
    Debug::info("MotorManager", "Starting homing for motor: " + motorName);
    return motor->startHoming();
  }
  
  Debug::error("MotorManager", "Cannot home motor: " + motorName + 
               ". Motor not found.");
  return false;
}