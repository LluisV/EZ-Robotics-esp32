/**
 * @file MachineController.cpp
 * @brief Implementation of the simplified MachineController class
 */

 #include "MachineController.h"
 #include "Debug.h"
 
 MachineController::MachineController(MotorManager *motorManager, ConfigManager *configManager)
     : motorManager(motorManager),
       configManager(configManager),
       currentFeedrate(1000.0f),
       absoluteMode(true)
 {
   // Initialize work offset to zero
   if (motorManager) {
     workOffset.resize(motorManager->getNumMotors(), 0.0f);
     desiredVelocityVector.resize(motorManager->getNumMotors(), 0.0f);
   }
 }
 
 bool MachineController::initialize()
 {
   if (!motorManager) {
     Serial.println("Invalid motor manager");
     return false;
   }
   if (!configManager) {
     Serial.println("Invalid config manager");
     return false;
   }
 
   // Initialize work offset to zero for all motors
   workOffset.resize(motorManager->getNumMotors(), 0.0f);
 
   Serial.println("Machine controller initialized");
   return true;
 }
 
 std::vector<float> MachineController::getCurrentWorldPosition() const
 {
   std::vector<float> positions;
 
   if (!motorManager) {
     return positions;
   }
 
   // Get current position in machine coordinates (world coordinates)
   int numMotors = motorManager->getNumMotors();
   for (int i = 0; i < numMotors; i++) {
     Motor *motor = motorManager->getMotor(i);
     if (motor) {
       positions.push_back(motor->getPositionInUnits());
     }
   }
 
   return positions;
 }
 
 std::vector<float> MachineController::getCurrentWorkPosition() const
 {
   // Get world positions
   std::vector<float> worldPositions = getCurrentWorldPosition();
 
   // Apply work offset to get work coordinates
   return machineToWorkPositions(worldPositions);
 }
 
 std::vector<float> MachineController::workToMachinePositions(const std::vector<float> &workPos) const
 {
   std::vector<float> machinePos = workPos;
 
   // Apply work offset to get machine coordinates
   for (size_t i = 0; i < machinePos.size() && i < workOffset.size(); i++) {
     machinePos[i] += workOffset[i];
   }
 
   // Apply machine limits to ensure we stay within bounds
   return applyMachineLimits(machinePos);
 }
 
 std::vector<float> MachineController::machineToWorkPositions(const std::vector<float> &machinePos) const
 {
   std::vector<float> workPos = machinePos;
 
   // Apply work offset to get work coordinates
   for (size_t i = 0; i < workPos.size() && i < workOffset.size(); i++) {
     workPos[i] -= workOffset[i];
   }
 
   return workPos;
 }
 
 bool MachineController::homeAll()
 {
   if (!motorManager) {
     return false;
   }
 
   return motorManager->homeAll();
 }
 
 bool MachineController::homeAxis(const String &axisName)
 {
   if (!motorManager) {
     return false;
   }
 
   return motorManager->homeMotorByName(axisName);
 }
 
 void MachineController::setWorkOffset(const std::vector<float> &offsets)
 {
   if (offsets.size() == workOffset.size()) {
     workOffset = offsets;
   }
 }
 
 std::vector<float> MachineController::getCurrentVelocityVector() const
 {
   std::vector<float> velocityVector;
   
   if (!motorManager) {
     return velocityVector;
   }
   
   int numMotors = motorManager->getNumMotors();
   velocityVector.resize(numMotors, 0.0f);
   
   // Get the current velocities of all motors in their native units
   for (int i = 0; i < numMotors; i++) {
     Motor *motor = motorManager->getMotor(i);
     if (motor && motor->isMoving()) {
       // Get motor speed in steps/sec (already converted in getCurrentSpeedSteps)
       float currentSpeedSteps = motor->getCurrentSpeedSteps();
       
       // Convert from steps/sec to units/sec (mm/sec or deg/sec)
       float unitsPerStep = motor->stepsToUnits(1);
       float speedInUnitsPerSec = currentSpeedSteps * unitsPerStep;
       
       // Convert from units/sec to units/min (mm/min or deg/min)
       float speedInUnitsPerMin = speedInUnitsPerSec * 60.0f;
       
       velocityVector[i] = speedInUnitsPerMin;
     }
   }
 
   return velocityVector;
 }
 
 float MachineController::getCurrentVelocity() const
 {
   if (!isMoving()) {
     return 0.0f;
   }
   
   // Get velocities in cartesian space (already in mm/min after the modification above)
   std::vector<float> velocityVector = getCurrentVelocityVector();
   
   // Calculate magnitude of velocity vector - Euclidean norm
   float sumOfSquares = 0.0f;
   for (const float &axisVelocity : velocityVector) {
     sumOfSquares += axisVelocity * axisVelocity;
   }
   
   return sqrt(sumOfSquares);
 }
 
 void MachineController::setCurrentDesiredVelocityVector(std::vector<float> velocities)
 {
   desiredVelocityVector = velocities;
 }
 
 std::vector<float> MachineController::getCurrentDesiredVelocityVector() const
 {
   return desiredVelocityVector;
 }
 
 float MachineController::getCurrentFeedrate() const {
   return currentFeedrate;
 }
 
 bool MachineController::isMoving() const
 {
   if (!motorManager) {
     return false;
   }
 
   return motorManager->isAnyMotorMoving();
 }
 
 void MachineController::emergencyStop()
 {
   if (!motorManager) {
     return;
   }
 
   Serial.println("EMERGENCY STOP");
   motorManager->stopAll(true);
 }
 
 void MachineController::pauseMovement()
 {
   if (!motorManager) return;
   
   // Gradually slow down motors
   for (int i = 0; i < motorManager->getNumMotors(); i++) {
     Motor* motor = motorManager->getMotor(i);
     if (motor && motor->isMoving()) {
       motor->stop(false); // False means decelerate
     }
   }
   
   Debug::info("MachineController", "Movement paused");
 }
 
 void MachineController::resumeMovement()
 {
   // In our new architecture, resuming motion is handled at the planner level
   Debug::info("MachineController", "Resume requested - handled by planner");
 }
 
 std::vector<float> MachineController::applyMachineLimits(const std::vector<float> &machinePos) const
 {
   std::vector<float> constrainedPos = machinePos;
 
   // Apply soft limits by clamping to machine boundaries
   for (size_t i = 0; i < constrainedPos.size(); i++) {
     if (motorManager && i < motorManager->getNumMotors()) {
       Motor *motor = motorManager->getMotor(i);
       if (motor) {
         const MotorConfig *config = motor->getConfig();
         if (config) {
           // Clamp to machine limits
           if (constrainedPos[i] < config->minPosition) {
             constrainedPos[i] = config->minPosition;
           }
           else if (constrainedPos[i] > config->maxPosition) {
             constrainedPos[i] = config->maxPosition;
           }
         }
       }
     }
   }
 
   return constrainedPos;
 }
 
 bool MachineController::validatePosition(const String &motorName, float position, bool clampToLimits, float &clampedPosition)
 {
   // Default to the input position
   clampedPosition = position;
 
   // Find the motor's configuration
   const MotorConfig *motorConfig = nullptr;
 
   if (configManager) {
     motorConfig = configManager->getMotorConfigByName(motorName);
   }
 
   if (!motorConfig) {
     return true; // Allow movement if we can't find config (safer than blocking)
   }
 
   // Use the min and max position limits
   const float minLimit = motorConfig->minPosition;
   const float maxLimit = motorConfig->maxPosition;
 
   // Check if position is within limits
   bool withinLimits = true;
 
   if (position < minLimit) {
     withinLimits = false;
 
     if (clampToLimits) {
       clampedPosition = minLimit;
     }
   }
   else if (position > maxLimit) {
     withinLimits = false;
 
     if (clampToLimits) {
       clampedPosition = maxLimit;
     }
   }
 
   return withinLimits || clampToLimits;
 }