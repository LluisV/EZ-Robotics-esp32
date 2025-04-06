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
 
bool MotorManager::moveToFeedrate(const std::vector<float>& positions, float feedrate) {
  // For feedrate moves (G1), we need to ensure all motors start and finish together
  Debug::verbose("MotorManager", "Starting synchronized move with feedrate: " + String(feedrate) + " mm/min");
  
  if (positions.size() != motors.size()) {
    Debug::error("MotorManager", "Position array size mismatch in feedrate move");
    return false;
  }
  
  // Get current positions and calculate distances
  std::vector<float> currentPositions(motors.size());
  std::vector<float> distances(motors.size());
  
  // Calculate total Cartesian distance for the move (for proper feedrate calculation)
  float totalDistanceSquared = 0.0f;
  
  for (size_t i = 0; i < motors.size(); i++) {
    currentPositions[i] = motors[i]->getPositionInUnits();
    distances[i] = positions[i] - currentPositions[i];
    totalDistanceSquared += distances[i] * distances[i];
    
    Debug::verbose("MotorManager", motors[i]->getName() + 
                  " move distance: " + String(distances[i]) + " units");
  }
  
  float totalDistance = sqrt(totalDistanceSquared);
  
  // Calculate the move duration based on the feedrate
  float feedrateInUnitsPerSec = feedrate / 60.0f; // Convert mm/min to mm/sec
  float moveDuration = (totalDistance > 0.0001f) ? (totalDistance / feedrateInUnitsPerSec) : 0.0f;
  
  Debug::verbose("MotorManager", "Feedrate move: total distance=" + String(totalDistance) + 
                " units, feedrate=" + String(feedrateInUnitsPerSec) + 
                " units/sec, ideal duration=" + String(moveDuration) + " sec");
  
  // Calculate the required speeds for each motor
  std::vector<float> requiredSpeeds(motors.size());
  bool needToScaleFeedrate = false;
  float maxSpeedRatio = 0.0f;
  
  for (size_t i = 0; i < motors.size(); i++) {
    // Calculate the speed needed for this motor to complete at the same time as others
    if (moveDuration > 0.0001f) {
      requiredSpeeds[i] = abs(distances[i]) / moveDuration; // units/sec
    } else {
      requiredSpeeds[i] = 0.0f;
    }
    
    // Check if speed exceeds motor's capability
    const MotorConfig* config = motors[i]->getConfig();
    int speedInSteps = motors[i]->unitsToSteps(requiredSpeeds[i]);
    
    if (config->maxSpeed > 0 && speedInSteps > 0) {
      float speedRatio = (float)speedInSteps / (float)config->maxSpeed;
      if (speedRatio > maxSpeedRatio) {
        maxSpeedRatio = speedRatio;
        if (speedRatio > 1.0f) {
          needToScaleFeedrate = true;
        }
      }
    }
    
    Debug::verbose("MotorManager", motors[i]->getName() + " requires " + 
                  String(requiredSpeeds[i]) + " units/sec (" + 
                  String(speedInSteps) + " steps/sec), max: " + 
                  String(config->maxSpeed) + " steps/sec");
  }
  
  // If any motor's required speed exceeds its maximum, scale all speeds down proportionally
  float speedScaleFactor = 1.0f;
  if (needToScaleFeedrate && maxSpeedRatio > 1.0f) {
    speedScaleFactor = 1.0f / maxSpeedRatio;
    moveDuration *= maxSpeedRatio; // Extend duration proportionally
    
    Debug::warning("MotorManager", "Feedrate exceeds machine capabilities by factor of " + 
                  String(maxSpeedRatio) + ". Scaling down speeds by " + 
                  String(speedScaleFactor));
  }
  
  // Now set up each motor with adjusted speeds and execute moves
  bool success = true;
  
  // Track max acceleration time to ensure all motors have proper acceleration profiles
  float maxAccelTime = 0;
  std::vector<float> accelTimes(motors.size());
  
  // First pass: calculate acceleration times for each motor
  for (size_t i = 0; i < motors.size(); i++) {
    if (abs(distances[i]) < 0.0001f) {
      accelTimes[i] = 0;
      continue;
    }
    
    const MotorConfig* config = motors[i]->getConfig();
    float adjustedSpeed = requiredSpeeds[i] * speedScaleFactor;
    
    // Calculate time to reach target speed with configured acceleration
    float accelInUnits = motors[i]->stepsToUnits(config->acceleration);
    accelTimes[i] = adjustedSpeed / accelInUnits;
    
    if (accelTimes[i] > maxAccelTime) {
      maxAccelTime = accelTimes[i];
    }
  }
  
  // Second pass: adjust accelerations to ensure synchronized movement
  for (size_t i = 0; i < motors.size(); i++) {
    if (abs(distances[i]) < 0.0001f) {
      continue; // Skip motors that don't need to move
    }
    
    const MotorConfig* config = motors[i]->getConfig();
    
    // Calculate adjusted speed (scaled if necessary)
    float adjustedSpeed = requiredSpeeds[i] * speedScaleFactor;
    int speedInSteps = motors[i]->unitsToSteps(adjustedSpeed);
    
    // Adjust acceleration to match the acceleration profile of the slowest motor
    // This ensures all motors reach their speeds in the same amount of time
    float accelRatio = 1.0f;
    if (accelTimes[i] > 0.0001f) {
      accelRatio = accelTimes[i] / maxAccelTime;
    }
    
    int adjustedAccel = config->acceleration * accelRatio;
    // Ensure a minimum acceleration
    adjustedAccel = max(adjustedAccel, 100);
    
    // Configure motor parameters
    motors[i]->setAcceleration(adjustedAccel);
    motors[i]->setSpeed(speedInSteps);
    
    Debug::verbose("MotorManager", motors[i]->getName() + " config: " +
                  "speed=" + String(adjustedSpeed) + " units/sec, " +
                  "accel=" + String(adjustedAccel) + " steps/sec² " +
                  "(ratio: " + String(accelRatio) + ")");
    
    // Start the move
    success &= motors[i]->moveToUnits(positions[i], adjustedSpeed);
  }
  
  return success;
}

bool MotorManager::moveToRapid(const std::vector<float>& positions) {
  // For rapid moves (G0), we aim for synchronized motion at max speeds
  Debug::verbose("MotorManager", "Starting rapid synchronized move");
  
  if (positions.size() != motors.size()) {
    Debug::error("MotorManager", "Position array size mismatch in rapid move");
    return false;
  }
  
  // Calculate distances and the time each motor would take at max speed
  std::vector<float> currentPositions(motors.size());
  std::vector<float> distances(motors.size());
  std::vector<float> moveTimes(motors.size());
  float maxMoveTime = 0.0f;
  
  for (size_t i = 0; i < motors.size(); i++) {
    currentPositions[i] = motors[i]->getPositionInUnits();
    distances[i] = positions[i] - currentPositions[i];
    
    // Skip calculation if distance is negligible
    if (abs(distances[i]) < 0.0001f) {
      moveTimes[i] = 0.0f;
      continue;
    }
    
    // Calculate time to move this distance at max speed
    const MotorConfig* config = motors[i]->getConfig();
    float maxSpeedInUnits = motors[i]->stepsToUnits(config->maxSpeed);
    moveTimes[i] = abs(distances[i]) / maxSpeedInUnits;
    
    // Find the maximum move time
    if (moveTimes[i] > maxMoveTime) {
      maxMoveTime = moveTimes[i];
    }
    
    Debug::verbose("MotorManager", motors[i]->getName() + " rapid move: " + 
                  "distance=" + String(distances[i]) + " units, " +
                  "max speed=" + String(maxSpeedInUnits) + " units/sec, " +
                  "time=" + String(moveTimes[i]) + " sec");
  }
  
  // Adjust speeds to ensure all motors finish simultaneously
  bool success = true;
  
  // Calculate acceleration times at max acceleration
  float maxAccelTime = 0;
  std::vector<float> maxSpeedTimes(motors.size());
  std::vector<float> adjustedSpeeds(motors.size());
  
  for (size_t i = 0; i < motors.size(); i++) {
    if (abs(distances[i]) < 0.0001f) {
      continue; // Skip motors that don't need to move
    }
    
    // Calculate adjusted speed to match the longest move time
    adjustedSpeeds[i] = abs(distances[i]) / maxMoveTime;
    
    // Calculate time to reach this speed at max acceleration
    const MotorConfig* config = motors[i]->getConfig();
    float accelInUnits = motors[i]->stepsToUnits(config->acceleration);
    maxSpeedTimes[i] = adjustedSpeeds[i] / accelInUnits;
    
    if (maxSpeedTimes[i] > maxAccelTime) {
      maxAccelTime = maxSpeedTimes[i];
    }
  }
  
  // Now adjust accelerations to ensure all motors reach their speeds in sync
  for (size_t i = 0; i < motors.size(); i++) {
    if (abs(distances[i]) < 0.0001f) {
      continue; // Skip motors that don't need to move
    }
    
    const MotorConfig* config = motors[i]->getConfig();
    int speedInSteps = motors[i]->unitsToSteps(adjustedSpeeds[i]);
    
    // Adjust acceleration to ensure synchronization
    float accelRatio = 1.0f;
    if (maxSpeedTimes[i] > 0.0001f) {
      accelRatio = maxSpeedTimes[i] / maxAccelTime;
    }
    
    int adjustedAccel = config->acceleration * accelRatio;
    // Ensure a minimum acceleration
    adjustedAccel = max(adjustedAccel, 100);
    
    motors[i]->setAcceleration(adjustedAccel);
    motors[i]->setSpeed(speedInSteps);
    
    Debug::verbose("MotorManager", motors[i]->getName() + " rapid config: " +
                  "adjusted speed=" + String(adjustedSpeeds[i]) + " units/sec, " +
                  "accel=" + String(adjustedAccel) + " steps/sec² " +
                  "(ratio: " + String(accelRatio) + ")");
    
    // Start the move
    success &= motors[i]->moveToUnits(positions[i], adjustedSpeeds[i]);
  }
  
  return success;
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