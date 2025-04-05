/**
 * @file MachineController.cpp
 * @brief Implementation of the MachineController class
 */

 #include "MachineController.h"

 MachineController::MachineController(MotorManager* motorManager)
   : motorManager(motorManager),
     kinematics(nullptr),
     currentFeedrate(1000.0f),
     absoluteMode(true)
 {
   // Initialize work offset to zero
   if (motorManager) {
     workOffset.resize(motorManager->getNumMotors(), 0.0f);
   }
 }
 
 bool MachineController::initialize() {
   if (!motorManager) {
     Serial.println("Invalid motor manager");
     return false;
   }
   
   // TODO: Initialize kinematics module
   // kinematics = new CartesianKinematics();
   
   // Initialize work offset to zero for all motors
   workOffset.resize(motorManager->getNumMotors(), 0.0f);
   
   Serial.println("Machine controller initialized");
   return true;
 }
 
 bool MachineController::homeAll() {
   if (!motorManager) {
     return false;
   }
   
   return motorManager->homeAll();
 }
 
 bool MachineController::homeAxis(const String& axisName) {
   if (!motorManager) {
     return false;
   }
   
   return motorManager->homeMotorByName(axisName);
 }
 
 bool MachineController::moveTo(const std::vector<float>& positions, float feedrate, MovementType movementType) {
   if (!motorManager) {
     return false;
   }
   
   if (positions.size() != motorManager->getNumMotors()) {
     Serial.println("Position array size mismatch");
     return false;
   }
   
   // Set current feedrate
   currentFeedrate = feedrate;
   
   // Convert work coordinates to machine coordinates
   std::vector<float> machinePos = workToMachinePositions(positions);
   
   // For kinematic machines, convert machine coordinates to motor positions
   std::vector<float> motorPos = machineToMotorPositions(machinePos);
   
   // Calculate speed based on feedrate
   float speedInUnitsPerSec = feedrate / 60.0f; // Convert from mm/min to mm/s
   
   // Handle rapid moves at maximum speed
   if (movementType == RAPID_MOVE) {
     motorManager->moveToSynchronizedUnits(motorPos); // Use max speed
   } else {
     // Implement feedrate for synchronized movements
     // This is simplified - a more advanced implementation would handle proper
     // acceleration planning and feedrate control based on path
     motorManager->moveToSynchronizedUnits(motorPos);
   }
   
   return true;
 }
 
 bool MachineController::moveTo(float x, float y, float z, float feedrate, MovementType movementType) {
   std::vector<float> positions;
   
   // Find motors by name and set their positions
   Motor* xMotor = motorManager->getMotorByName("X");
   Motor* yMotor = motorManager->getMotorByName("Y");
   Motor* zMotor = motorManager->getMotorByName("Z");
   
   // Prepare position array with current positions as defaults
   std::vector<float> currentPos = getCurrentPosition();
   positions = currentPos;
   
   // Update positions for X, Y, Z if they exist
   if (xMotor) {
     int index = motorManager->getMotor(0)->getName() == "X" ? 0 :
                 motorManager->getMotor(1)->getName() == "X" ? 1 :
                 2; // Assuming X, Y, Z are the first three motors
     positions[index] = x;
   }
   
   if (yMotor) {
     int index = motorManager->getMotor(0)->getName() == "Y" ? 0 :
                 motorManager->getMotor(1)->getName() == "Y" ? 1 :
                 2; // Assuming X, Y, Z are the first three motors
     positions[index] = y;
   }
   
   if (zMotor) {
     int index = motorManager->getMotor(0)->getName() == "Z" ? 0 :
                 motorManager->getMotor(1)->getName() == "Z" ? 1 :
                 2; // Assuming X, Y, Z are the first three motors
     positions[index] = z;
   }
   
   return moveTo(positions, feedrate, movementType);
 }
 
 bool MachineController::executeGCode(const String& command) {
   // This is a simplified G-code interpreter
   // A full implementation would handle more codes and parameters
   
   // Remove comments and trim
   String code = command;
   int commentIndex = code.indexOf(';');
   if (commentIndex != -1) {
     code = code.substring(0, commentIndex);
   }
   code.trim();
   
   if (code.length() == 0) {
     return true; // Empty line or comment-only line
   }
   
   // Extract command letter and number
   char letter = code.charAt(0);
   float value = 0.0f;
   
   if (code.length() > 1) {
     value = code.substring(1).toFloat();
   }
   
   // Extract parameters
   float x = NAN, y = NAN, z = NAN, f = NAN;
   
   int idx = 0;
   while (idx < code.length()) {
     if (idx + 1 < code.length()) {
       char paramLetter = code.charAt(idx);
       
       // Find the next parameter or end of string
       int nextIdx = idx + 1;
       while (nextIdx < code.length() && 
              !isalpha(code.charAt(nextIdx))) {
         nextIdx++;
       }
       
       // Extract the value
       if (nextIdx > idx + 1) {
         String valueStr = code.substring(idx + 1, nextIdx);
         float paramValue = valueStr.toFloat();
         
         // Store the parameter
         switch (paramLetter) {
           case 'X': case 'x': x = paramValue; break;
           case 'Y': case 'y': y = paramValue; break;
           case 'Z': case 'z': z = paramValue; break;
           case 'F': case 'f': f = paramValue; break;
         }
       }
       
       idx = nextIdx;
     } else {
       idx++;
     }
   }
   
   // Interpret G-codes
   if (letter == 'G' || letter == 'g') {
     if (value == 0) {
       // G0: Rapid move
       if (!isnan(f)) {
         currentFeedrate = f;
       }
       
       // Handle relative mode
       std::vector<float> currentPos = getCurrentPosition();
       if (!absoluteMode) {
         if (!isnan(x)) x += currentPos[0];
         if (!isnan(y)) x += currentPos[1];
         if (!isnan(z)) x += currentPos[2];
       }
       
       if (!isnan(x) || !isnan(y) || !isnan(z)) {
         float xPos = isnan(x) ? currentPos[0] : x;
         float yPos = isnan(y) ? currentPos[1] : y;
         float zPos = isnan(z) ? currentPos[2] : z;
         
         return moveTo(xPos, yPos, zPos, currentFeedrate, RAPID_MOVE);
       }
       
     } else if (value == 1) {
       // G1: Linear move
       if (!isnan(f)) {
         currentFeedrate = f;
       }
       
       // Handle relative mode
       std::vector<float> currentPos = getCurrentPosition();
       if (!absoluteMode) {
         if (!isnan(x)) x += currentPos[0];
         if (!isnan(y)) x += currentPos[1];
         if (!isnan(z)) x += currentPos[2];
       }
       
       if (!isnan(x) || !isnan(y) || !isnan(z)) {
         float xPos = isnan(x) ? currentPos[0] : x;
         float yPos = isnan(y) ? currentPos[1] : y;
         float zPos = isnan(z) ? currentPos[2] : z;
         
         return moveTo(xPos, yPos, zPos, currentFeedrate, LINEAR_MOVE);
       }
       
     } else if (value == 28) {
       // G28: Home
       if (isnan(x) && isnan(y) && isnan(z)) {
         // Home all axes
         return homeAll();
       } else {
         // Home specific axes
         bool success = true;
         if (!isnan(x)) success &= homeAxis("X");
         if (!isnan(y)) success &= homeAxis("Y");
         if (!isnan(z)) success &= homeAxis("Z");
         return success;
       }
       
     } else if (value == 90) {
       // G90: Set to absolute positioning
       absoluteMode = true;
       return true;
       
     } else if (value == 91) {
       // G91: Set to relative positioning
       absoluteMode = false;
       return true;
       
     } else if (value == 92) {
       // G92: Set position
       std::vector<float> currentPos = getCurrentPosition();
       
       if (!isnan(x)) workOffset[0] = motorManager->getMotorByName("X")->getPositionInUnits() - x;
       if (!isnan(y)) workOffset[1] = motorManager->getMotorByName("Y")->getPositionInUnits() - y;
       if (!isnan(z)) workOffset[2] = motorManager->getMotorByName("Z")->getPositionInUnits() - z;
       
       return true;
     }
   } else if (letter == 'M' || letter == 'm') {
     if (value == 0 || value == 1) {
       // M0/M1: Program pause
       // No implementation needed here
       return true;
       
     } else if (value == 2 || value == 30) {
       // M2/M30: Program end
       // No implementation needed here
       return true;
       
     } else if (value == 3) {
       // M3: Spindle on clockwise
       // Not implemented in this version
       return true;
       
     } else if (value == 4) {
       // M4: Spindle on counterclockwise
       // Not implemented in this version
       return true;
       
     } else if (value == 5) {
       // M5: Spindle off
       // Not implemented in this version
       return true;
       
     } else if (value == 112) {
       // M112: Emergency stop
       emergencyStop();
       return true;
     }
   }
   
   // Unrecognized command
   Serial.println("Unrecognized G-code: " + command);
   return false;
 }
 
 std::vector<float> MachineController::getCurrentPosition() const {
   std::vector<float> positions;
   
   if (!motorManager) {
     return positions;
   }
   
   // Get current position in machine coordinates
   int numMotors = motorManager->getNumMotors();
   for (int i = 0; i < numMotors; i++) {
     Motor* motor = motorManager->getMotor(i);
     if (motor) {
       positions.push_back(motor->getPositionInUnits());
     }
   }
   
   // Apply work offset
   for (size_t i = 0; i < positions.size() && i < workOffset.size(); i++) {
     positions[i] -= workOffset[i];
   }
   
   return positions;
 }
 
 float MachineController::getCurrentAxisPosition(const String& axisName) const {
   if (!motorManager) {
     return 0.0f;
   }
   
   Motor* motor = motorManager->getMotorByName(axisName);
   if (motor) {
     int index = 0;
     for (int i = 0; i < motorManager->getNumMotors(); i++) {
       if (motorManager->getMotor(i)->getName() == axisName) {
         index = i;
         break;
       }
     }
     
     return motor->getPositionInUnits() - workOffset[index];
   }
   
   return 0.0f;
 }
 
 bool MachineController::isMoving() const {
   if (!motorManager) {
     return false;
   }
   
   return motorManager->isAnyMotorMoving();
 }
 
 void MachineController::emergencyStop() {
   if (!motorManager) {
     return;
   }
   
   Serial.println("EMERGENCY STOP");
   motorManager->stopAll(true);
 }
 
 void MachineController::setWorkOffset(const std::vector<float>& offsets) {
   if (offsets.size() == workOffset.size()) {
     workOffset = offsets;
   }
 }
 
 std::vector<float> MachineController::machineToMotorPositions(const std::vector<float>& machinePos) {
   // In a simple Cartesian machine, machine coordinates are the same as motor positions
   // For more complex kinematics, this would transform coordinates
   // TODO: Implement kinematics transformation when kinematics module is added
   return machinePos;
 }
 
 std::vector<float> MachineController::workToMachinePositions(const std::vector<float>& workPos) {
   std::vector<float> machinePos = workPos;
   
   // Apply work offset
   for (size_t i = 0; i < machinePos.size() && i < workOffset.size(); i++) {
     machinePos[i] += workOffset[i];
   }
   
   return machinePos;
 }