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
 
bool MotorManager::moveToRapid(const std::vector<float>& positions) {
  // For rapid moves, we use the maximum speed of each motor while still
  // ensuring synchronization of completion time
  
  Debug::verbose("MotorManager", "Starting rapid synchronized move");
  
  if (positions.size() != motors.size()) {
    Debug::error("MotorManager", "Position array size mismatch in rapid move");
    return false;
  }
  
  // Calculate distances and max durations for each motor
  std::vector<float> currentPositions(motors.size());
  std::vector<float> distances(motors.size());
  std::vector<float> durations(motors.size());
  float maxDuration = 0.0f;
  
  // Get current positions and calculate distances
  for (size_t i = 0; i < motors.size(); i++) {
    currentPositions[i] = motors[i]->getPositionInUnits();
    distances[i] = positions[i] - currentPositions[i];
    
    // Calculate time using maximum speed
    const MotorConfig* config = motors[i]->getConfig();
    float maxSpeedInUnitsPerSec = motors[i]->stepsToUnits(config->maxSpeed); // Convert steps/sec to units/sec
    
    // Time to travel the distance at max speed
    if (abs(distances[i]) > 0.0001f && maxSpeedInUnitsPerSec > 0.0001f) {
      durations[i] = abs(distances[i]) / maxSpeedInUnitsPerSec;
    } else {
      durations[i] = 0.0f;
    }
    
    // Find the maximum duration
    if (durations[i] > maxDuration) {
      maxDuration = durations[i];
    }
    
    Debug::verbose("MotorManager", motors[i]->getName() + " rapid move: distance=" + 
                  String(distances[i]) + ", max speed=" + String(maxSpeedInUnitsPerSec) + 
                  ", duration=" + String(durations[i]));
  }
  
  // Calculate acceleration scaling to ensure synchronized arrival
  // This is a key improvement for proper synchronization
  std::vector<float> accelerationScaling(motors.size(), 1.0f);
  for (size_t i = 0; i < motors.size(); i++) {
    if (maxDuration > 0.0001f && durations[i] > 0.0001f) {
      // Scale both speed and acceleration proportionally to match longest move
      accelerationScaling[i] = durations[i] / maxDuration;
      
      // We scale acceleration by square of time ratio to maintain proper profiles
      accelerationScaling[i] = accelerationScaling[i] * accelerationScaling[i];
    }
  }
  
  // Execute the moves with adjusted acceleration and speed to synchronize
  bool success = true;
  for (size_t i = 0; i < motors.size(); i++) {
    if (abs(distances[i]) > 0.0001f) {
      const MotorConfig* config = motors[i]->getConfig();
      
      // Calculate adjusted speed and acceleration
      int adjustedAccel = config->acceleration * accelerationScaling[i];
      // Ensure acceleration is at least 100 steps/s² to avoid extremely slow starts
      adjustedAccel = max(100, adjustedAccel);
      
      // Set the acceleration
      motors[i]->setAcceleration(adjustedAccel);
      
      // Calculate adjusted speed to match the longest duration
      float adjustedSpeed = abs(distances[i]) / maxDuration;
      int speedInSteps = motors[i]->unitsToSteps(adjustedSpeed);
      motors[i]->setSpeed(speedInSteps);
      
      // Start the move using the motor's maximum speed for rapid movements
      // Pass the adjusted speed explicitly to ensure it's used
      success &= motors[i]->moveToUnits(positions[i], adjustedSpeed);
      
      Debug::verbose("MotorManager", motors[i]->getName() + " rapid move configured: accel scaling=" + 
                     String(accelerationScaling[i]) + ", adjusted accel=" + String(adjustedAccel) +
                     ", adjusted speed=" + String(adjustedSpeed) + " units/sec");
    }
  }
  
  return success;
}

bool MotorManager::moveToFeedrate(const std::vector<float>& positions, float feedrate) {
  // For feedrate moves (G1), we respect the specified feedrate and ensure 
  // synchronized completion with proper acceleration profiles
  
  Debug::verbose("MotorManager", "Starting synchronized move with feedrate: " + String(feedrate) + " mm/min");
  
  if (positions.size() != motors.size()) {
    Debug::error("MotorManager", "Position array size mismatch in feedrate move");
    return false;
  }
  
  // Get current positions in user units
  std::vector<float> currentPositions(motors.size());
  for (size_t i = 0; i < motors.size(); i++) {
    currentPositions[i] = motors[i]->getPositionInUnits();
  }
  
  // Calculate Cartesian distance for the move
  float totalDistanceSquared = 0.0f;
  for (size_t i = 0; i < motors.size(); i++) {
    float delta = positions[i] - currentPositions[i];
    totalDistanceSquared += delta * delta;
  }
  float totalDistance = sqrt(totalDistanceSquared);
  
  // Calculate ideal move duration based on feedrate
  float feedrateInUnitsPerSec = feedrate / 60.0f; // Convert mm/min to mm/sec
  float idealDuration = (totalDistance > 0.0001f) ? (totalDistance / feedrateInUnitsPerSec) : 0.0f;
  
  Debug::verbose("MotorManager", "Feedrate move: total distance=" + String(totalDistance) + 
                " units, feedrate=" + String(feedrateInUnitsPerSec) + 
                " units/sec, ideal duration=" + String(idealDuration) + " sec");
  
  // Calculate the required speeds and check against max speeds
  std::vector<float> speeds(motors.size());
  std::vector<float> distances(motors.size());
  bool feedrateExceedsLimit = false;
  
  for (size_t i = 0; i < motors.size(); i++) {
    distances[i] = positions[i] - currentPositions[i];
    
    // Calculate the required speed for this motor to follow the feedrate
    if (idealDuration > 0.0001f) {
      speeds[i] = abs(distances[i]) / idealDuration;
    } else {
      speeds[i] = 0.0f;
    }
    
    // Convert to steps/sec for comparison with motor limits
    const MotorConfig* config = motors[i]->getConfig();
    int speedInSteps = motors[i]->unitsToSteps(speeds[i]);
    
    // Check if speed exceeds motor's capability
    if (speedInSteps > config->maxSpeed) {
      feedrateExceedsLimit = true;
      Debug::warning("MotorManager", motors[i]->getName() + " required speed " + 
                    String(speedInSteps) + " steps/sec exceeds max " + 
                    String(config->maxSpeed) + " steps/sec");
    }
    
    Debug::verbose("MotorManager", motors[i]->getName() + " move: distance=" + 
                  String(distances[i]) + ", required speed=" + String(speeds[i]) + 
                  " units/sec (" + String(speedInSteps) + " steps/sec)");
  }
  
  // If feedrate exceeds machine capabilities, scale it down
  float durationScaling = 1.0f;
  if (feedrateExceedsLimit) {
    // Find the most limiting motor
    float maxSpeedRatio = 0.0f;
    for (size_t i = 0; i < motors.size(); i++) {
      const MotorConfig* config = motors[i]->getConfig();
      int speedInSteps = motors[i]->unitsToSteps(speeds[i]);
      
      float speedRatio = (config->maxSpeed > 0) ? 
                         ((float)speedInSteps / (float)config->maxSpeed) : 0.0f;
                         
      if (speedRatio > maxSpeedRatio) {
        maxSpeedRatio = speedRatio;
      }
    }
    
    // Scale the duration to ensure we don't exceed max speeds
    if (maxSpeedRatio > 1.0f) {
      durationScaling = maxSpeedRatio;
      Debug::warning("MotorManager", "Feedrate exceeds machine capabilities by factor of " + 
                    String(maxSpeedRatio) + ". Scaling down.");
    }
  }
  
  // Calculate adjusted duration and speeds
  float adjustedDuration = idealDuration * durationScaling;
  
  // Calculate acceleration scaling to ensure synchronized arrival
  // For professional-grade movement, we need to consider acceleration profiles
  std::vector<float> accelerationScaling(motors.size(), 1.0f);
  
  for (size_t i = 0; i < motors.size(); i++) {
    const MotorConfig* config = motors[i]->getConfig();
    
    // Calculate time needed for acceleration and deceleration at max acceleration
    float accelTimeToReachSpeed = speeds[i] / motors[i]->stepsToUnits(config->acceleration);
    
    // Total time spent in acceleration/deceleration phases
    float totalAccelTime = accelTimeToReachSpeed * 2.0f; // Accel + decel
    
    // Calculate distance covered during acceleration/deceleration
    float accelDistance = 0.5f * speeds[i] * accelTimeToReachSpeed * 2.0f;
    
    // If acceleration distance exceeds total distance, we need to adjust
    if (accelDistance > abs(distances[i]) && abs(distances[i]) > 0.0001f) {
      // This is a "triangle" velocity profile (no constant velocity phase)
      // Scale acceleration to fit the shorter distance
      float accelScale = sqrt(abs(distances[i]) / accelDistance);
      accelerationScaling[i] = accelScale;
      
      Debug::verbose("MotorManager", motors[i]->getName() + " uses triangle profile, " +
                    "accel scaling=" + String(accelScale));
    } else {
      Debug::verbose("MotorManager", motors[i]->getName() + " uses trapezoidal profile");
    }
  }
  
  // Execute the moves with properly adjusted parameters
  bool success = true;
  for (size_t i = 0; i < motors.size(); i++) {
    if (abs(distances[i]) > 0.0001f) {
      const MotorConfig* config = motors[i]->getConfig();
      
      // Calculate adjusted speed
      float adjustedSpeed = speeds[i] / durationScaling;
      int speedInSteps = motors[i]->unitsToSteps(adjustedSpeed);
      
      // Apply acceleration scaling for proper synchronization
      int adjustedAccel = config->acceleration * accelerationScaling[i];
      // Ensure acceleration is at least 100 steps/s² to avoid extremely slow starts
      adjustedAccel = max(100, adjustedAccel);
      
      // Set parameters and execute move
      motors[i]->setAcceleration(adjustedAccel);
      motors[i]->setSpeed(speedInSteps);
      
      // Instead of just calling moveToUnits, explicitly pass the speed
      success &= motors[i]->moveToUnits(positions[i], adjustedSpeed);
      
      Debug::verbose("MotorManager", motors[i]->getName() + " feedrate move configured: " +
                    "speed=" + String(adjustedSpeed) + " units/sec (" + String(speedInSteps) + 
                    " steps/sec), accel=" + String(adjustedAccel) + " steps/sec²");
    }
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