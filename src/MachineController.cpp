/**
 * @file MachineController.cpp
 * @brief Implementation of the MachineController class with segmented motion planner
 */

 #include "MachineController.h"
 #include "Debug.h"
 #include "Scheduler.h"
 #include "Stepper.h"
 
 // External access to the scheduler for direct velocity reporting
 extern Scheduler* scheduler;
  
 MachineController::MachineController(MotorManager *motorManager, ConfigManager *configManager)
     : motorManager(motorManager),
       configManager(configManager),
       kinematics(nullptr),
       currentFeedrate(1000.0f),
       absoluteMode(true),
       motionPlanner(nullptr)
 {
   Debug::info("MachineController", "Initializing MachineController");
   
   // Initialize work offset to zero
   if (motorManager) {
     workOffset.resize(motorManager->getNumMotors(), 0.0f);
     desiredVelocityVector.resize(motorManager->getNumMotors(), 0.0f);
   }
 }
  
 bool MachineController::initialize()
 {
   if (!motorManager) {
     Debug::error("MachineController", "Invalid motor manager");
     return false;
   }
   if (!configManager) {
     Debug::error("MachineController", "Invalid config manager");
     return false;
   }
 
   // TODO: Initialize kinematics module
   // kinematics = new CartesianKinematics();
 
   // Initialize work offset to zero for all motors
   workOffset.resize(motorManager->getNumMotors(), 0.0f);
   
   Debug::info("MachineController", "Machine controller initialized successfully");
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
     Debug::warning("MachineController", "No motor manager available for position query");
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
     Debug::error("MachineController", "Cannot home - no motor manager available");
     return false;
   }
 
   Debug::info("MachineController", "Homing all axes");
   return motorManager->homeAll();
 }
  
 bool MachineController::homeAxis(const String &axisName)
 {
   if (!motorManager) {
     Debug::error("MachineController", "Cannot home axis - no motor manager available");
     return false;
   }
 
   Debug::info("MachineController", "Homing axis: " + axisName);
   return motorManager->homeMotorByName(axisName);
 }
  
 bool MachineController::moveTo(const std::vector<float> &positions, float feedrate, MovementType movementType)
 {
   if (!motorManager) {
     Debug::error("MachineController", "No motor manager available");
     return false;
   }
 
   int numMotors = motorManager->getNumMotors();
   
   // Make sure we have at least some positions to move to
   if (positions.empty()) {
     Debug::error("MachineController", "Position array is empty");
     return false;
   }
   
   // If positions vector is smaller than the number of motors, pad it with current positions
   std::vector<float> fullPositions = positions;
   if (fullPositions.size() < numMotors) {
     std::vector<float> currentPos = getCurrentWorkPosition();
     fullPositions.resize(numMotors);
     
     // Copy current positions for any missing axes
     for (int i = positions.size(); i < numMotors; i++) {
       if (i < currentPos.size()) {
         fullPositions[i] = currentPos[i];
       } else {
         fullPositions[i] = 0.0f;
       }
     }
   }
 
   // First convert work coordinates to machine coordinates WITHOUT clamping
   std::vector<float> machinePos = fullPositions;
   for (size_t i = 0; i < machinePos.size() && i < workOffset.size(); i++) {
     machinePos[i] += workOffset[i];
   }
 
   // Now validate the machine (world) coordinates against limits
   bool positionOutOfBounds = false;
   String outOfBoundsMessage = "Position out of bounds: ";
   std::vector<float> validatedMachinePos = machinePos;
 
   for (int i = 0; i < motorManager->getNumMotors(); i++) {
     Motor *motor = motorManager->getMotor(i);
     if (motor && i < machinePos.size() && !isnan(machinePos[i])) {
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
     
     // Create a target position vector with the right number of elements
     std::vector<float> motionTargetPos;
     
     // Calculate target positions based on work coordinates
     for (int i = 0; i < numMotors; i++) {
       // If this position was in our original target, use it
       if (i < positions.size()) {
         motionTargetPos.push_back(positions[i]);
       } 
       // Otherwise use the current position
       else if (i < getCurrentWorkPosition().size()) {
         motionTargetPos.push_back(getCurrentWorkPosition()[i]);
       }
       // In case we're missing data, use 0
       else {
         motionTargetPos.push_back(0.0f);
       }
     }
     
     // Update velocity vector for telemetry
     float feedrateInMMperSec = feedrate / 60.0f; // Convert from mm/min to mm/sec
     
     std::vector<float> velVector;
     velVector.resize(numMotors, 0.0f);
     
     float moveDistance = 0.0f;
     for (size_t i = 0; i < motionTargetPos.size() && i < getCurrentWorkPosition().size(); i++) {
       float delta = motionTargetPos[i] - getCurrentWorkPosition()[i];
       moveDistance += delta * delta;
     }
     moveDistance = sqrtf(moveDistance);
     
     if (moveDistance > 0.0001f) {
       for (size_t i = 0; i < motionTargetPos.size() && i < getCurrentWorkPosition().size(); i++) {
         float delta = motionTargetPos[i] - getCurrentWorkPosition()[i];
         // Scale by feedrate to get velocity vector (convert back to mm/min)
         velVector[i] = (delta / moveDistance) * feedrate;
       }
     }
     
     // Store velocity vector for telemetry
     setCurrentDesiredVelocityVector(velVector);
     
     // Log the move distance and velocity for debugging
     Debug::info("MachineController", "Sending move: distance=" + String(moveDistance) + 
                "mm, feedrate=" + String(feedrate) + "mm/min");
     
     // If there's a previous move pending, make sure to wake the stepper system
     if (Stepper::get_current_block() == nullptr) {
       Debug::info("MachineController", "No current move in progress - ensuring stepper is awake");
       Stepper::wake_up();
     }
     
     // Send move to appropriate planner function
     try {
       bool result;
       if (movementType == RAPID_MOTION) {
         result = motionPlanner->addRapidMove(motionTargetPos);
       } else {
         result = motionPlanner->addLinearMove(motionTargetPos, feedrate);
       }
       
       if (!result) {
         Debug::error("MachineController", "Motion planner rejected the move");
         return false;
       }
       
       Debug::info("MachineController", "Move accepted by motion planner");
       return true;
     } catch (std::exception& e) {
       Debug::error("MachineController", "Exception in motion planning: " + String(e.what()));
       return false;
     } catch (...) {
       Debug::error("MachineController", "Unknown exception in motion planning");
       return false;
     }
   } else {
     Debug::error("MachineController", "No motion planner available");
     return false;
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
   if (xMotor && !isnan(x)) {
     int index = motorManager->getMotor(0)->getName() == "X" ? 0 : 
                 motorManager->getMotor(1)->getName() == "X" ? 1 : 2;
     positions[index] = x;
   }
 
   if (yMotor && !isnan(y)) {
     int index = motorManager->getMotor(0)->getName() == "Y" ? 0 : 
                 motorManager->getMotor(1)->getName() == "Y" ? 1 : 2;
     positions[index] = y;
   }
 
   if (zMotor && !isnan(z)) {
     int index = motorManager->getMotor(0)->getName() == "Z" ? 0 : 
                 motorManager->getMotor(1)->getName() == "Z" ? 1 : 2;
     positions[index] = z;
   }
 
   Debug::info("MachineController", "Move to: X=" + String(x) + " Y=" + String(y) + 
               " Z=" + String(z) + " F=" + String(feedrate));
   return moveTo(positions, feedrate, movementType);
 }

  
 float MachineController::getCurrentAxisPosition(const String &axisName) const
 {
   if (!motorManager) {
     return 0.0f;
   }
 
   Motor *motor = motorManager->getMotorByName(axisName);
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
  
 bool MachineController::isMoving() const
 {
   if (!motorManager) {
     return false;
   }
 
   // Check both the motor manager and the stepper system for active movement
   if (motorManager->isAnyMotorMoving()) {
     return true;
   }
   
   // Also check if the scheduler or stepper system reports active movement
   if (Stepper::get_current_block() != nullptr) {
     Debug::verbose("MachineController", "Motion detected in stepper system");
     return true;
   }
   
   return false;
 }
  
 void MachineController::emergencyStop()
 {
   if (!motorManager) {
     return;
   }
 
   Debug::error("MachineController", "EMERGENCY STOP TRIGGERED");
   Serial.println("EMERGENCY STOP");
   motorManager->stopAll(true);
 
   // Clear motion planner if available
   if (motionPlanner) {
     motionPlanner->clear();
   }
   
   // Also stop the stepper execution system
   Stepper::stop_stepping();
 }
  
 void MachineController::setWorkOffset(const std::vector<float> &offsets)
 {
   if (offsets.size() == workOffset.size()) {
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
   for (size_t i = 0; i < machinePos.size() && i < workOffset.size(); i++) {
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
 
   if (configManager) {
     motorConfig = configManager->getMotorConfigByName(motorName);
   }
 
   if (!motorConfig) {
     Debug::warning("MachineController", "No config found for motor " + motorName + " during limit validation");
     return true; // Allow movement if we can't find config (safer than blocking)
   }
 
   // Use the min and max position limits
   const float minLimit = motorConfig->minPosition;
   const float maxLimit = motorConfig->maxPosition;
 
   // Check if position is within limits
   bool withinLimits = true;
 
   if (position < minLimit) {
     Debug::warning("MachineController", motorName + " position " + String(position) +
                                            " below minimum limit of " + String(minLimit));
     withinLimits = false;
 
     if (clampToLimits) {
       clampedPosition = minLimit;
       Debug::info("MachineController", "Clamping " + motorName + " to minimum limit " + String(minLimit));
     }
   } else if (position > maxLimit) {
     Debug::warning("MachineController", motorName + " position " + String(position) +
                                            " exceeds maximum limit of " + String(maxLimit));
     withinLimits = false;
 
     if (clampToLimits) {
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
   for (size_t i = 0; i < constrainedPos.size(); i++) {
     if (i < motorManager->getNumMotors()) {
       Motor *motor = motorManager->getMotor(i);
       if (motor) {
         const MotorConfig *config = motor->getConfig();
         if (config) {
           // Clamp to machine limits
           if (constrainedPos[i] < config->minPosition) {
             Debug::warning("MachineController", "Clamping " + motor->getName() +
                                                    " position to minimum limit (" + String(config->minPosition) + ")");
             constrainedPos[i] = config->minPosition;
           } else if (constrainedPos[i] > config->maxPosition) {
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
   // This has been completely rewritten to use the stepper system directly
   std::vector<float> velocityVector;
    
   if (!motorManager) {
     Debug::warning("MachineController", "No motor manager available for velocity query");
     return velocityVector;
   }
    
   int numMotors = motorManager->getNumMotors();
   velocityVector.resize(numMotors, 0.0f);
   
   // Get velocity from the stepper module if we're actually moving
   if (Stepper::get_current_block() != nullptr) {
     // Get scalar velocity from stepper
     float realtime_rate = Stepper::get_realtime_rate();
     
     // There's no easy way to get the per-axis velocity from the stepper module directly,
     // so we approximate it based on the desired velocity vector direction
     // but scale it to the actual current velocity magnitude
     if (desiredVelocityVector.size() > 0) {
       // First calculate the magnitude of our desired vector
       float desired_magnitude = 0.0f;
       for (size_t i = 0; i < desiredVelocityVector.size(); i++) {
         desired_magnitude += desiredVelocityVector[i] * desiredVelocityVector[i];
       }
       desired_magnitude = sqrtf(desired_magnitude);
       
       // Now scale the desired vector direction to the actual velocity magnitude
       if (desired_magnitude > 0.001f) {
         for (size_t i = 0; i < desiredVelocityVector.size() && i < velocityVector.size(); i++) {
           velocityVector[i] = (desiredVelocityVector[i] / desired_magnitude) * realtime_rate;
         }
         
         Debug::verbose("MachineController", "Velocity from stepper: " + String(realtime_rate) + " mm/min");
       }
     }
   }
   
   return velocityVector;
 }
 
 float MachineController::getCurrentVelocity() const
 {
   // Get the current velocity directly from the stepper system for accuracy
   if (Stepper::get_current_block() != nullptr) {
     float velocity = Stepper::get_realtime_rate();
     Debug::verbose("MachineController", "Current velocity from stepper: " + String(velocity) + " mm/min");
     return velocity;
   }
   
   // If no blocks are active, we're not moving
   Debug::verbose("MachineController", "No active stepper blocks - velocity is 0");
   return 0.0f;
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
   // Calculate magnitude of velocity vector - Euclidean norm
   float sumOfSquares = 0.0f;
   for (const float &axisVelocity : desiredVelocityVector) {
     sumOfSquares += axisVelocity * axisVelocity;
   }
   
   return sqrt(sumOfSquares);
 }