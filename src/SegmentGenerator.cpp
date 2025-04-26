/**
 * @file SegmentGenerator.cpp
 * @brief Implementation of the SegmentGenerator class
 */

 #include "SegmentGenerator.h"
 #include "MachineController.h"
 #include "MotorManager.h"
 #include "Planner.h"
 #include <cmath>
 #include <algorithm>
 
 SegmentGenerator::SegmentGenerator(MachineController* machineController, MotorManager* motorManager)
     : machineController(machineController), motorManager(motorManager), planner(nullptr)
 {
 }
 
 bool SegmentGenerator::initialize()
 {
     // Initialize the current position
     if (motorManager) {
         int numMotors = motorManager->getNumMotors();
         currentSteps.resize(numMotors, 0);
         
         // Get current position from motors
         for (int i = 0; i < numMotors; i++) {
             Motor* motor = motorManager->getMotor(i);
             if (motor) {
                 currentSteps[i] = motor->getPosition();
             }
         }
     }
     
     // Clear the segment buffer
     segmentBuffer.clear();
     
     Debug::info("SegmentGenerator", "Initialized");
     return true;
 }
 
 void SegmentGenerator::generateSegments()
 {
     // Don't do anything if the planner isn't set
     if (!planner) {
         return;
     }
     
     // Skip if segment buffer is already full enough
     if (segmentBuffer.size() >= SEGMENT_BUFFER_SIZE / 2) {
         return;
     }
     
     // Get the next move from the planner
     const PlannerMove* move = planner->getNextMove();
     if (!move) {
         return;
     }
     
     // Get the current position as the starting point
     std::vector<float> startPos;
     
     if (machineController) {
         startPos = machineController->getCurrentWorldPosition();
     } else {
         // Fallback to zeros if machine controller is not available
         startPos.resize(move->targetPosition.size(), 0.0f);
     }
     
     // Segment the move
     segmentMove(startPos, move->targetPosition, 
                move->entrySpeed, move->exitSpeed, move->maxSpeed, 
                move->acceleration, move->unitVector);
     
     // Remove the move from the planner
     planner->removeCurrentMove();
 }
 
 bool SegmentGenerator::executeNextSegment()
 {
     // Skip if no segments available
     if (segmentBuffer.empty()) {
         return false;
     }
     
     // Get the next segment
     const Segment& segment = segmentBuffer.front();
     
     // Check if any motors are still moving
     if (motorManager && motorManager->isAnyMotorMoving()) {
         return false;
     }
     
     // Execute the segment
     bool result = false;
     
     if (motorManager) {
         int numMotors = motorManager->getNumMotors();
         
         // Update machine controller about the current desired velocity
         if (machineController) {
             machineController->setCurrentDesiredVelocityVector(segment.desiredVelocities);
         }
         
         // Configure and start each motor
         for (int i = 0; i < numMotors && i < segment.jointPositions.size(); i++) {
             Motor* motor = motorManager->getMotor(i);
             if (!motor) continue;
             
             // Convert position to steps
             int32_t targetSteps = motor->unitsToSteps(segment.jointPositions[i]);
             
             // Calculate steps to move
             int32_t stepsToMove = targetSteps - currentSteps[i];
             
             // Skip motors that don't need to move
             if (stepsToMove == 0) continue;
             
             // Calculate speed in steps/s from mm/min
             float speed = std::abs(segment.adjustedVelocities[i]); // Already in steps/s
             
             // Start the move
             motor->moveTo(targetSteps, speed);
             
             // Update current steps
             currentSteps[i] = targetSteps;
         }
         
         result = true;
     }
     
     // Remove the segment from the buffer
     if (result) {
         segmentBuffer.pop_front();
     }
     
     return result;
 }
 
 bool SegmentGenerator::isEmpty() const
 {
     return segmentBuffer.empty();
 }
 
 bool SegmentGenerator::isFull() const
 {
     return segmentBuffer.size() >= SEGMENT_BUFFER_SIZE;
 }
 
 void SegmentGenerator::clear()
 {
     segmentBuffer.clear();
     Debug::info("SegmentGenerator", "Segment buffer cleared");
 }
 
 bool SegmentGenerator::update()
 {
     bool actionTaken = false;
     
     // Generate segments if needed and planner has moves
     if (!isFull() && planner && !planner->isEmpty()) {
         generateSegments();
         actionTaken = true;
     }
     
     // Execute next segment if motors are idle
     if (!isEmpty() && motorManager && !motorManager->isAnyMotorMoving()) {
         if (executeNextSegment()) {
             actionTaken = true;
         }
     }
     
     return actionTaken;
 }
 
 void SegmentGenerator::segmentMove(const std::vector<float>& startPos, const std::vector<float>& endPos,
                                  float startSpeed, float endSpeed, float maxSpeed, float acceleration,
                                  const std::vector<float>& moveVector)
 {
     // Calculate total move distance
     float moveDistance = 0.0f;
     for (size_t i = 0; i < std::min(startPos.size(), endPos.size()); i++) {
         float delta = endPos[i] - startPos[i];
         moveDistance += delta * delta;
     }
     moveDistance = std::sqrt(moveDistance);
     
     // Skip extremely short moves
     if (moveDistance < 0.001f) {
         return;
     }
     
     // Convert speeds from mm/min to mm/s for calculations
     float startSpeedMmS = startSpeed / 60.0f;
     float endSpeedMmS = endSpeed / 60.0f;
     float maxSpeedMmS = maxSpeed / 60.0f;
     
     // Calculate accelerating and decelerating distances
     float accelDist = 0.0f;
     float decelDist = 0.0f;
     float cruiseDist = 0.0f;
     
     // Can we reach max speed?
     float accelDistToMax = calculateDistance(startSpeedMmS, maxSpeedMmS, acceleration);
     float decelDistFromMax = calculateDistance(maxSpeedMmS, endSpeedMmS, -acceleration);
     
     if (accelDistToMax + decelDistFromMax <= moveDistance) {
         // We can reach max speed
         accelDist = accelDistToMax;
         decelDist = decelDistFromMax;
         cruiseDist = moveDistance - accelDist - decelDist;
     } else {
         // We can't reach max speed, find crossover point
         // Solve for the peak velocity where we transition from accelerating to decelerating
         float crossoverSpeed = std::sqrt((startSpeedMmS * startSpeedMmS + endSpeedMmS * endSpeedMmS) / 2.0f + 
                                        acceleration * moveDistance);
         
         accelDist = calculateDistance(startSpeedMmS, crossoverSpeed, acceleration);
         decelDist = calculateDistance(crossoverSpeed, endSpeedMmS, -acceleration);
         cruiseDist = 0.0f;
     }
     
     // Calculate how many segments we need
     int segmentCount = calculateSegmentCount(moveDistance, startSpeed, endSpeed, maxSpeed, acceleration);
     
     // Prepare to generate segments
     std::vector<float> prevPos = startPos;
     float prevSpeed = startSpeedMmS;
     float distanceTraveled = 0.0f;
     
     for (int i = 1; i <= segmentCount; i++) {
         // Calculate the position at this segment
         float segmentFraction = static_cast<float>(i) / segmentCount;
         float segmentDistance = moveDistance * segmentFraction;
         
         // Determine the current speed at this distance
         float currentSpeed;
         
         if (segmentDistance <= accelDist) {
             // Accelerating phase
             currentSpeed = calculateSpeed(startSpeedMmS, segmentDistance, acceleration);
         } else if (segmentDistance <= accelDist + cruiseDist) {
             // Cruise phase
             currentSpeed = maxSpeedMmS;
         } else {
             // Decelerating phase
             float decelDistance = segmentDistance - accelDist - cruiseDist;
             currentSpeed = calculateSpeed(maxSpeedMmS, decelDistance, -acceleration);
         }
         
         // Calculate the position at this point
         std::vector<float> segmentPos = startPos;
         for (size_t j = 0; j < segmentPos.size(); j++) {
             if (j < moveVector.size()) {
                 segmentPos[j] += moveVector[j] * segmentDistance;
             }
         }
         
         // Create a segment
         Segment segment;
         segment.jointPositions = segmentPos;
         segment.distance = segmentDistance - distanceTraveled;
         
         // Calculate the velocities for each joint in steps/sec
         segment.desiredVelocities.resize(segmentPos.size());
         segment.adjustedVelocities.resize(segmentPos.size());
         
         // Get the motor speeds in steps/sec
         if (motorManager) {
             for (size_t j = 0; j < segmentPos.size(); j++) {
                 Motor* motor = j < motorManager->getNumMotors() ? motorManager->getMotor(j) : nullptr;
                 if (motor) {
                     // Calculate the distance this motor needs to travel for this segment
                     float motorDistance = segmentPos[j] - prevPos[j];
                     
                     // Calculate time for this segment
                     float segmentTime = segment.distance / ((currentSpeed + prevSpeed) / 2.0f);
                     
                     if (segmentTime > 0.000001f) {
                         // Calculate the velocity in mm/s
                         float motorVelocity = motorDistance / segmentTime;
                         
                         // Convert to steps/s
                         float stepsPerUnit = std::abs(motor->unitsToSteps(1.0f));
                         float velocityInSteps = motorVelocity * stepsPerUnit;
                         
                         segment.desiredVelocities[j] = velocityInSteps;
                         segment.adjustedVelocities[j] = velocityInSteps;
                     } else {
                         segment.desiredVelocities[j] = 0.0f;
                         segment.adjustedVelocities[j] = 0.0f;
                     }
                 }
             }
         }
         
         // Add the segment to the buffer
         if (!addSegment(segment)) {
             // Buffer is full, we'll continue later
             break;
         }
         
         // Update for next segment
         prevPos = segmentPos;
         prevSpeed = currentSpeed;
         distanceTraveled = segmentDistance;
     }
 }
 
 int SegmentGenerator::calculateSegmentCount(float moveDistance, float startSpeed, float endSpeed, 
                                           float maxSpeed, float acceleration)
 {
     // Convert speeds to mm/s
     float startSpeedMmS = startSpeed / 60.0f;
     float endSpeedMmS = endSpeed / 60.0f;
     float maxSpeedMmS = maxSpeed / 60.0f;
     
     // Calculate time to execute the move
     float accelTime = 0.0f;
     float cruiseTime = 0.0f;
     float decelTime = 0.0f;
     
     // Calculate accelerating and decelerating distances
     float accelDist = 0.0f;
     float decelDist = 0.0f;
     float cruiseDist = 0.0f;
     
     // Can we reach max speed?
     float accelDistToMax = calculateDistance(startSpeedMmS, maxSpeedMmS, acceleration);
     float decelDistFromMax = calculateDistance(maxSpeedMmS, endSpeedMmS, -acceleration);
     
     if (accelDistToMax + decelDistFromMax <= moveDistance) {
         // We can reach max speed
         accelDist = accelDistToMax;
         decelDist = decelDistFromMax;
         cruiseDist = moveDistance - accelDist - decelDist;
         
         // Calculate times
         accelTime = (maxSpeedMmS - startSpeedMmS) / acceleration;
         decelTime = (maxSpeedMmS - endSpeedMmS) / acceleration;
         cruiseTime = cruiseDist / maxSpeedMmS;
     } else {
         // We can't reach max speed, find crossover point
         // Solve for the peak velocity where we transition from accelerating to decelerating
         float crossoverSpeed = std::sqrt((startSpeedMmS * startSpeedMmS + endSpeedMmS * endSpeedMmS) / 2.0f + 
                                        acceleration * moveDistance);
         
         accelDist = calculateDistance(startSpeedMmS, crossoverSpeed, acceleration);
         decelDist = calculateDistance(crossoverSpeed, endSpeedMmS, -acceleration);
         
         // Calculate times
         accelTime = (crossoverSpeed - startSpeedMmS) / acceleration;
         decelTime = (crossoverSpeed - endSpeedMmS) / acceleration;
         cruiseTime = 0.0f;
     }
     
     // Total time for the move
     float totalTime = accelTime + cruiseTime + decelTime;
     
     // Calculate number of segments based on time and segment parameters
     int timeSegments = std::ceil(totalTime / MIN_SEGMENT_TIME);
     int distanceSegments = std::ceil(moveDistance / SEGMENT_MAX_LENGTH);
     
     // Use the maximum of these two values, but at least 1 segment
     return std::max(1, std::max(timeSegments, distanceSegments));
 }
 
 bool SegmentGenerator::addSegment(const Segment& segment)
 {
     // Check if buffer is full
     if (isFull()) {
         return false;
     }
     
     // Add to buffer
     segmentBuffer.push_back(segment);
     return true;
 }
 
 float SegmentGenerator::calculateSpeed(float initialSpeed, float distance, float acceleration)
 {
     // v^2 = v0^2 + 2*a*d
     float speedSquared = initialSpeed * initialSpeed + 2.0f * acceleration * distance;
     return std::sqrt(std::max(0.0f, speedSquared));
 }
 
 float SegmentGenerator::calculateDistance(float initialSpeed, float targetSpeed, float acceleration)
 {
     // d = (v_f^2 - v_i^2) / (2*a)
     return (targetSpeed * targetSpeed - initialSpeed * initialSpeed) / (2.0f * acceleration);
 }