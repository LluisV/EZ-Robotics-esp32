/**
 * @file MachineController.cpp
 * @brief Implementation of the MachineController class with segmented motion planner
 */

 #include "MachineController.h"
 #include "Debug.h"
 #include "Scheduler.h"

 
 MachineController::MachineController(MotorManager *motorManager, ConfigManager *configManager)
     : motorManager(motorManager),
       configManager(configManager),
       kinematics(nullptr),
       currentFeedrate(1000.0f),
       absoluteMode(true),
       motionPlanner(nullptr)
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
 
   // TODO: Initialize kinematics module
   // kinematics = new CartesianKinematics();
 
   // Initialize work offset to zero for all motors
   workOffset.resize(motorManager->getNumMotors(), 0.0f);
 
   Serial.println("Machine controller initialized");
   return true;
 }
 
 void MachineController::setMotionPlanner(Scheduler* motionPlanner) {
   this->motionPlanner = motionPlanner;
   Debug::info("MachineController", "Segmented motion planner connected");
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
   for (size_t i = 0; i < worldPositions.size() && i < workOffset.size(); i++) {
     worldPositions[i] -= workOffset[i];
   }
 
   return worldPositions;
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
 
 bool MachineController::moveTo(const std::vector<float> &positions, float feedrate, MovementType movementType)
 {
   if (!motorManager) {
     Debug::error("MachineController", "No motor manager available");
     return false;
   }
 
   if (positions.size() != motorManager->getNumMotors()) {
     Debug::error("MachineController", "Position array size mismatch: " +
                                           String(positions.size()) + " vs " + String(motorManager->getNumMotors()));
     return false;
   }
 
   // First convert work coordinates to machine coordinates WITHOUT clamping
   std::vector<float> machinePos = positions;
   for (size_t i = 0; i < machinePos.size() && i < workOffset.size(); i++) {
     machinePos[i] += workOffset[i];
   }
 
   // Now validate the machine (world) coordinates against limits
   bool positionOutOfBounds = false;
   String outOfBoundsMessage = "Position out of bounds: ";
   std::vector<float> validatedMachinePos = machinePos;
 
   for (int i = 0; i < motorManager->getNumMotors(); i++) {
     Motor *motor = motorManager->getMotor(i);
     if (motor && !isnan(machinePos[i])) {
       float clampedPos = machinePos[i];
       if (!validatePosition(motor->getName(), machinePos[i], false, clampedPos)) {
         positionOutOfBounds = true;
         outOfBoundsMessage += motor->getName() + "=" + String(machinePos[i]) + " ";
       }
       validatedMachinePos[i] = clampedPos;
     }
   }
 
   // If any position is out of bounds, reject the move
   if (positionOutOfBounds) {
     Debug::error("MachineController", outOfBoundsMessage);
     Serial.println("Error: " + outOfBoundsMessage);
     return false;
   }
 
   // Set current feedrate for future commands
   currentFeedrate = feedrate;
 
   // Apply soft limits to ensure machine boundaries are respected
   std::vector<float> constrainedMachinePos = applyMachineLimits(validatedMachinePos);
 
   // For kinematic machines, convert machine coordinates to motor positions
   std::vector<float> targetMotorPos = machineToMotorPositions(constrainedMachinePos);
 
   // Use segmented motion planner if available
   if (motionPlanner) {
    Debug::verbose("MachineController", "Move sent to the motion planner");
    if (movementType == RAPID_MOVE) {
      return motionPlanner->addRapidMove(targetMotorPos);
    } else {
     return motionPlanner->addLinearMove(targetMotorPos, feedrate);
    }
  } else {
     Debug::error("MachineController", "No motion planner instanced");
   }
 }
 
 bool MachineController::moveTo(float x, float y, float z, float feedrate, MovementType movementType)
 {
   std::vector<float> positions;
 
   // Find motors by name and set their positions
   Motor *xMotor = motorManager->getMotorByName("X");
   Motor *yMotor = motorManager->getMotorByName("Y");
   Motor *zMotor = motorManager->getMotorByName("Z");
 
   // Prepare position array with current positions as defaults
   std::vector<float> currentPos = getCurrentWorkPosition();
   positions = currentPos;
 
   // Update positions for X, Y, Z if they exist and values are specified
   if (xMotor && !isnan(x))
   {
     int index = motorManager->getMotor(0)->getName() == "X" ? 0 : motorManager->getMotor(1)->getName() == "X" ? 1
                                                                                                               : 2; // Assuming X, Y, Z are the first three motors
     positions[index] = x;
   }
 
   if (yMotor && !isnan(y))
   {
     int index = motorManager->getMotor(0)->getName() == "Y" ? 0 : motorManager->getMotor(1)->getName() == "Y" ? 1
                                                                                                               : 2; // Assuming X, Y, Z are the first three motors
     positions[index] = y;
   }
 
   if (zMotor && !isnan(z))
   {
     int index = motorManager->getMotor(0)->getName() == "Z" ? 0 : motorManager->getMotor(1)->getName() == "Z" ? 1
                                                                                                               : 2; // Assuming X, Y, Z are the first three motors
     positions[index] = z;
   }
 
   return moveTo(positions, feedrate, movementType);
 }
 
 
 bool MachineController::executeGCode(const String &command)
 {
   // This is a simplified G-code interpreter
 
   // Remove comments and trim
   String code = command;
   int commentIndex = code.indexOf(';');
   if (commentIndex != -1)
   {
     code = code.substring(0, commentIndex);
   }
   code.trim();
 
   if (code.length() == 0)
   {
     return true; // Empty line or comment-only line
   }
 
   // Extract command letter and number
   char letter = code.charAt(0);
   float value = 0.0f;
 
   if (code.length() > 1)
   {
     value = code.substring(1).toFloat();
   }
 
   // Extract parameters
   float x = NAN, y = NAN, z = NAN, f = NAN;
 
   int idx = 0;
   while (idx < code.length())
   {
     if (idx + 1 < code.length())
     {
       char paramLetter = code.charAt(idx);
 
       // Find the next parameter or end of string
       int nextIdx = idx + 1;
       while (nextIdx < code.length() &&
              !isalpha(code.charAt(nextIdx)))
       {
         nextIdx++;
       }
 
       // Extract the value
       if (nextIdx > idx + 1)
       {
         String valueStr = code.substring(idx + 1, nextIdx);
         float paramValue = valueStr.toFloat();
 
         // Store the parameter
         switch (paramLetter)
         {
         case 'X':
         case 'x':
           x = paramValue;
           break;
         case 'Y':
         case 'y':
           y = paramValue;
           break;
         case 'Z':
         case 'z':
           z = paramValue;
           break;
         case 'F':
         case 'f':
           f = paramValue;
           break;
         }
       }
 
       idx = nextIdx;
     }
     else
     {
       idx++;
     }
   }
 
   // Interpret G-codes
   if (letter == 'G' || letter == 'g')
   {
     if (value == 0)
     {
       // G0: Rapid move
       if (!isnan(f))
       {
         currentFeedrate = f;
       }
 
       // Handle relative mode
       std::vector<float> currentPos = getCurrentWorkPosition();
       if (!absoluteMode)
       {
         if (!isnan(x))
           x += currentPos[0];
         if (!isnan(y))
           y += currentPos[1];
         if (!isnan(z))
           z += currentPos[2];
       }
 
       if (!isnan(x) || !isnan(y) || !isnan(z))
       {
         float xPos = isnan(x) ? currentPos[0] : x;
         float yPos = isnan(y) ? currentPos[1] : y;
         float zPos = isnan(z) ? currentPos[2] : z;
 
         return moveTo(xPos, yPos, zPos, currentFeedrate, RAPID_MOVE);
       }
     }
     else if (value == 1)
     {
       // G1: Linear move
       if (!isnan(f))
       {
         currentFeedrate = f;
       }
 
       // Handle relative mode
       std::vector<float> currentPos = getCurrentWorkPosition();
       if (!absoluteMode)
       {
         if (!isnan(x))
           x += currentPos[0];
         if (!isnan(y))
           y += currentPos[1];
         if (!isnan(z))
           z += currentPos[2];
       }
 
       if (!isnan(x) || !isnan(y) || !isnan(z))
       {
         float xPos = isnan(x) ? currentPos[0] : x;
         float yPos = isnan(y) ? currentPos[1] : y;
         float zPos = isnan(z) ? currentPos[2] : z;
 
         return moveTo(xPos, yPos, zPos, currentFeedrate, LINEAR_MOVE);
       }
     }
     else if (value == 28)
     {
       // G28: Home
       if (isnan(x) && isnan(y) && isnan(z))
       {
         // Home all axes
         return homeAll();
       }
       else
       {
         // Home specific axes
         bool success = true;
         if (!isnan(x))
           success &= homeAxis("X");
         if (!isnan(y))
           success &= homeAxis("Y");
         if (!isnan(z))
           success &= homeAxis("Z");
         return success;
       }
     }
     else if (value == 90)
     {
       // G90: Set to absolute positioning
       absoluteMode = true;
       return true;
     }
     else if (value == 91)
     {
       // G91: Set to relative positioning
       absoluteMode = false;
       return true;
     }
     else if (value == 92)
     {
       // G92: Set position
       std::vector<float> currentPos = getCurrentWorldPosition();
 
       if (!isnan(x))
         workOffset[0] = motorManager->getMotorByName("X")->getPositionInUnits() - x;
       if (!isnan(y))
         workOffset[1] = motorManager->getMotorByName("Y")->getPositionInUnits() - y;
       if (!isnan(z))
         workOffset[2] = motorManager->getMotorByName("Z")->getPositionInUnits() - z;
 
       return true;
     }
   }
   else if (letter == 'M' || letter == 'm')
   {
     if (value == 0 || value == 1)
     {
       // M0/M1: Program pause
       // No implementation needed here
       return true;
     }
     else if (value == 2 || value == 30)
     {
       // M2/M30: Program end
       // No implementation needed here
       return true;
     }
     else if (value == 3)
     {
       // M3: Spindle on clockwise
       // Not implemented in this version
       return true;
     }
     else if (value == 4)
     {
       // M4: Spindle on counterclockwise
       // Not implemented in this version
       return true;
     }
     else if (value == 5)
     {
       // M5: Spindle off
       // Not implemented in this version
       return true;
     }
     else if (value == 112)
     {
       // M112: Emergency stop
       emergencyStop();
       return true;
     }
   }
 
   // Unrecognized command
   Serial.println("Unrecognized G-code: " + command);
   return false;
 }
 
 float MachineController::getCurrentAxisPosition(const String &axisName) const
 {
   if (!motorManager)
   {
     return 0.0f;
   }
 
   Motor *motor = motorManager->getMotorByName(axisName);
   if (motor)
   {
     int index = 0;
     for (int i = 0; i < motorManager->getNumMotors(); i++)
     {
       if (motorManager->getMotor(i)->getName() == axisName)
       {
         index = i;
         break;
       }
     }
 
     return motor->getPositionInUnits() - workOffset[index];
   }
 
   return 0.0f;
 }
 
 bool MachineController::isMoving() const
 {
   if (!motorManager)
   {
     return false;
   }
 
   return motorManager->isAnyMotorMoving();
 }
 
 void MachineController::emergencyStop()
 {
   if (!motorManager)
   {
     return;
   }
 
   Serial.println("EMERGENCY STOP");
   motorManager->stopAll(true);
 
   // Clear motion planner if available
   if (motionPlanner) {
     motionPlanner->clear();
   }
 }
 
 void MachineController::setWorkOffset(const std::vector<float> &offsets)
 {
   if (offsets.size() == workOffset.size())
   {
     workOffset = offsets;
   }
 }
 
 std::vector<float> MachineController::machineToMotorPositions(const std::vector<float> &machinePos)
 {
   // In a simple Cartesian machine, machine coordinates are the same as motor positions
   // For more complex kinematics, this would transform coordinates
   // TODO: Implement kinematics transformation when kinematics module is added
   return machinePos;
 }
 
 std::vector<float> MachineController::workToMachinePositions(const std::vector<float> &workPos)
 {
   std::vector<float> machinePos = workPos;
 
   // Apply work offset to get machine coordinates
   for (size_t i = 0; i < machinePos.size() && i < workOffset.size(); i++)
   {
     machinePos[i] += workOffset[i];
   }
 
   // Apply machine limits to ensure we stay within bounds
   return applyMachineLimits(machinePos);
 }
 
 bool MachineController::validatePosition(const String &motorName, float position, bool clampToLimits, float &clampedPosition)
 {
   // Default to the input position
   clampedPosition = position;
 
   // Find the motor's configuration
   const MotorConfig *motorConfig = nullptr;
 
   if (configManager)
   {
     motorConfig = configManager->getMotorConfigByName(motorName);
   }
 
   if (!motorConfig)
   {
     Debug::warning("MachineController", "No config found for motor " + motorName + " during limit validation");
     return true; // Allow movement if we can't find config (safer than blocking)
   }
 
   // Use the min and max position limits
   const float minLimit = motorConfig->minPosition;
   const float maxLimit = motorConfig->maxPosition;
 
   // Check if position is within limits
   bool withinLimits = true;
 
   if (position < minLimit)
   {
     Debug::warning("MachineController", motorName + " position " + String(position) +
                                             " below minimum limit of " + String(minLimit));
     withinLimits = false;
 
     if (clampToLimits)
     {
       clampedPosition = minLimit;
       Debug::info("MachineController", "Clamping " + motorName + " to minimum limit " + String(minLimit));
     }
   }
   else if (position > maxLimit)
   {
     Debug::warning("MachineController", motorName + " position " + String(position) +
                                             " exceeds maximum limit of " + String(maxLimit));
     withinLimits = false;
 
     if (clampToLimits)
     {
       clampedPosition = maxLimit;
       Debug::info("MachineController", "Clamping " + motorName + " to maximum limit " + String(maxLimit));
     }
   }
 
   return withinLimits || clampToLimits;
 }
 
 std::vector<float> MachineController::applyMachineLimits(const std::vector<float> &machinePos)
 {
   std::vector<float> constrainedPos = machinePos;
 
   // Apply soft limits by clamping to machine boundaries
   for (size_t i = 0; i < constrainedPos.size(); i++)
   {
     if (i < motorManager->getNumMotors())
     {
       Motor *motor = motorManager->getMotor(i);
       if (motor)
       {
         const MotorConfig *config = motor->getConfig();
         if (config)
         {
           // Clamp to machine limits
           if (constrainedPos[i] < config->minPosition)
           {
             Debug::warning("MachineController", "Clamping " + motor->getName() +
                                                     " position to minimum limit (" + String(config->minPosition) + ")");
             constrainedPos[i] = config->minPosition;
           }
           else if (constrainedPos[i] > config->maxPosition)
           {
             Debug::warning("MachineController", "Clamping " + motor->getName() +
                                                     " position to maximum limit (" + String(config->maxPosition) + ")");
             constrainedPos[i] = config->maxPosition;
           }
         }
       }
     }
   }
 
   return constrainedPos;
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

   // Apply kinematics if needed (left as-is)
   if (kinematics) {
     // For now, just pass through motor velocities
     return velocityVector;
   } else {
     // Default case - assume Cartesian kinematics
     return velocityVector;
   }
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

 float MachineController::getCurrentDesiredVelocity() const
{
  
  // Get velocities in cartesian space (already in mm/min after the modification above)
  std::vector<float> velocityVector = getCurrentDesiredVelocityVector();
  
  // Calculate magnitude of velocity vector - Euclidean norm
  float sumOfSquares = 0.0f;
  for (const float &axisVelocity : velocityVector) {
    sumOfSquares += axisVelocity * axisVelocity;
  }
  
  return sqrt(sumOfSquares);
}