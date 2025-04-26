/**
 * @file Planner.cpp
 * @brief Implementation of the Planner class
 */

 #include "Planner.h"
 #include "SegmentGenerator.h"
 #include "MathUtils.h"
 #include <cmath>
 #include <algorithm>
 
 Planner::Planner(SegmentGenerator* segmentGenerator)
     : segmentGenerator(segmentGenerator), 
       junctionDeviation(0.01f),
       maxVelocity(2000.0f),
       maxAcceleration(20.0f)
 {
 }
 
 bool Planner::initialize()
 {
     // Reserve space for the move queue to avoid reallocations
     moveQueue.clear();
     
     Debug::info("Planner", "Initialized with junction deviation: " + String(junctionDeviation, 4) + 
                           ", max velocity: " + String(maxVelocity, 1) + 
                           ", max acceleration: " + String(maxAcceleration, 1));
     return true;
 }
 
 bool Planner::addMove(const PlannerMove& move)
 {
     // Check if the queue is full
     if (moveQueue.size() >= MAX_MOVE_QUEUE_SIZE) {
         Debug::warning("Planner", "Move queue is full, cannot add move");
         return false;
     }
     
     // Don't add zero-distance moves
     if (move.distance < 0.001f) {
         return true;
     }
     
     // Create a copy of the move for our queue
     PlannerMove newMove = move;
     
     // Calculate unit vector for the move
     if (!calculateUnitVector(newMove)) {
         Debug::warning("Planner", "Could not calculate unit vector for move");
         return false;
     }
     
     // Calculate maximum speed and acceleration for this move
     calculateMaxSpeed(newMove);
     
     // Start with conservative speed settings
     newMove.entrySpeed = 0;
     newMove.exitSpeed = 0;
     
     // Set the maximum allowable entry and exit speeds
     if (moveQueue.empty()) {
         newMove.maxEntrySpeed = 0; // Start from stand-still
     } else {
         // Calculate the maximum junction speed between the previous move and this one
         float junctionSpeed = calculateJunctionSpeed(moveQueue.back(), newMove);
         newMove.maxEntrySpeed = junctionSpeed;
         moveQueue.back().maxExitSpeed = junctionSpeed;
         
         // Mark previous move for recalculation
         moveQueue.back().recalculate = true;
     }
     
     // Add the move to the queue
     moveQueue.push_back(newMove);
     
     // Recalculate the plan
     recalculatePlannerMoves();
     
     return true;
 }
 
 bool Planner::addRapidMove(const std::vector<float>& targetPos)
 {
     // Create a rapid move
     PlannerMove rapidMove;
     rapidMove.targetPosition = targetPos;
     rapidMove.type = RAPID_MOVE;
     rapidMove.feedrate = maxVelocity; // Use max velocity for rapid moves
     
     // Calculate the distance
     if (!moveQueue.empty()) {
         const std::vector<float>& currentPos = moveQueue.back().targetPosition;
         float distSquared = 0.0f;
         for (size_t i = 0; i < std::min(currentPos.size(), targetPos.size()); i++) {
             float delta = targetPos[i] - currentPos[i];
             distSquared += delta * delta;
         }
         rapidMove.distance = std::sqrt(distSquared);
     } else {
         // We don't know the starting position, will be updated later
         rapidMove.distance = 0.0f;
     }
     
     // Add to the planner
     return addMove(rapidMove);
 }
 
 bool Planner::addLinearMove(const std::vector<float>& targetPos, float feedrate)
 {
     // Create a linear move
     PlannerMove linearMove;
     linearMove.targetPosition = targetPos;
     linearMove.type = LINEAR_MOVE;
     linearMove.feedrate = feedrate;
     
     // Calculate the distance
     if (!moveQueue.empty()) {
         const std::vector<float>& currentPos = moveQueue.back().targetPosition;
         float distSquared = 0.0f;
         for (size_t i = 0; i < std::min(currentPos.size(), targetPos.size()); i++) {
             float delta = targetPos[i] - currentPos[i];
             distSquared += delta * delta;
         }
         linearMove.distance = std::sqrt(distSquared);
     } else {
         // We don't know the starting position, will be updated later
         linearMove.distance = 0.0f;
     }
     
     // Add to the planner
     return addMove(linearMove);
 }
 
 const PlannerMove* Planner::getNextMove()
 {
     if (moveQueue.empty()) {
         return nullptr;
     }
     
     return &moveQueue.front();
 }
 
 bool Planner::removeCurrentMove()
 {
     if (moveQueue.empty()) {
         return false;
     }
     
     moveQueue.pop_front();
     
     // If there are still moves in the queue, recalculate
     if (!moveQueue.empty()) {
         moveQueue.front().maxEntrySpeed = 0; // Start from stand-still
         moveQueue.front().recalculate = true;
         recalculatePlannerMoves();
     }
     
     return true;
 }
 
 bool Planner::isEmpty() const
 {
     return moveQueue.empty();
 }
 
 bool Planner::isFull() const
 {
     return moveQueue.size() >= MAX_MOVE_QUEUE_SIZE;
 }
 
 void Planner::clear()
 {
     moveQueue.clear();
     Debug::info("Planner", "Move queue cleared");
 }
 
 bool Planner::waitForCompletion(float dwell_time)
 {
     // Wait for all planned moves to complete
     while (!isEmpty() || (segmentGenerator && !segmentGenerator->isEmpty())) {
         // Allow for system tasks and motion execution
         delay(10);
         
         // TODO: Check for interruption or emergency stop
     }
     
     // Additional dwell if requested
     if (dwell_time > 0.0f) {
         delay(dwell_time * 1000.0f);
     }
     
     return true;
 }
 
 size_t Planner::moveCount() const
 {
     return moveQueue.size();
 }
 
 void Planner::recalculatePlannerMoves()
 {
     if (moveQueue.empty()) {
         return;
     }
     
     // Ensure all moves have their max speeds calculated
     for (auto& move : moveQueue) {
         if (!move.unitVector.size()) {
             calculateUnitVector(move);
         }
         calculateMaxSpeed(move);
     }
     
     // Forward planning pass
     forwardPass();
     
     // Backward planning pass
     backwardPass();
     
     // Clear recalculation flags
     for (auto& move : moveQueue) {
         move.recalculate = false;
     }
 }
 
 float Planner::calculateJunctionSpeed(const PlannerMove& previous, const PlannerMove& current)
 {
     // Check if we have valid unit vectors
     if (previous.unitVector.empty() || current.unitVector.empty()) {
         return 0.0f;
     }
     
     // Calculate the cosine of the angle between movement vectors
     float cosTheta = MathUtils::dotProductVectors(previous.unitVector, current.unitVector);
     
     // Limit cos(theta) to valid range
     cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));
     
     // Check if the vectors are in the same direction (or nearly so)
     if (cosTheta > 0.9999f) {
         return std::min(previous.maxSpeed, current.maxSpeed);
     }
     
     // Check if the junction is too sharp (nearly reverse direction)
     if (cosTheta < -0.9999f) {
         return 0.0f;
     }
     
     // Calculate the junction angle
     float theta = std::acos(cosTheta);
     
     // Calculate the junction speed based on centripetal acceleration
     // This formula uses the junction deviation setting to control how aggressive the corners can be
     float junctionVelocitySq = junctionDeviation * maxAcceleration / 
                               std::sqrt((1.0f - cosTheta) / 2.0f);
     
     // Convert junction velocity to mm/min and apply limits
     float junctionSpeed = std::sqrt(junctionVelocitySq) * 60.0f;
     
     // Limit to the maximum speed of both moves
     junctionSpeed = std::min(junctionSpeed, std::min(previous.maxSpeed, current.maxSpeed));
     
     return junctionSpeed;
 }
 
 bool Planner::calculateUnitVector(PlannerMove& move)
 {
     // We need a previous position to calculate direction
     if (moveQueue.empty() && move.distance <= 0.001f) {
         // Can't calculate unit vector without a distance
         return false;
     }
     
     // Get the starting position, either from the previous move or use a default
     std::vector<float> startPos;
     
     if (!moveQueue.empty()) {
         startPos = moveQueue.back().targetPosition;
     } else {
         // This is the first move, so assume starting from zeros
         startPos.resize(move.targetPosition.size(), 0.0f);
     }
     
     // Calculate movement vector and distance
     std::vector<float> moveVector;
     float distanceSquared = 0.0f;
     
     // Ensure vectors are same size
     if (startPos.size() != move.targetPosition.size()) {
         startPos.resize(move.targetPosition.size(), 0.0f);
     }
     
     moveVector.resize(move.targetPosition.size());
     
     for (size_t i = 0; i < moveVector.size(); i++) {
         moveVector[i] = move.targetPosition[i] - startPos[i];
         distanceSquared += moveVector[i] * moveVector[i];
     }
     
     // Update move distance
     move.distance = std::sqrt(distanceSquared);
     
     // Handle zero-distance moves
     if (move.distance < 0.001f) {
         move.unitVector.clear();
         return false;
     }
     
     // Calculate unit vector
     move.unitVector.resize(moveVector.size());
     
     for (size_t i = 0; i < moveVector.size(); i++) {
         move.unitVector[i] = moveVector[i] / move.distance;
     }
     
     return true;
 }
 
 void Planner::calculateMaxSpeed(PlannerMove& move)
 {
     // For rapid moves, always use the maximum velocity
     if (move.type == RAPID_MOVE) {
         move.maxSpeed = maxVelocity;
         move.acceleration = maxAcceleration;
         return;
     }
     
     // For normal moves, use specified feedrate with limits
     move.maxSpeed = std::min(move.feedrate, maxVelocity);
     move.acceleration = maxAcceleration;
     
     // TODO: Apply axis-specific acceleration and velocity limits
     // This would need to check the limits for each axis based on the direction of travel
 }
 
 void Planner::forwardPass()
 {
     float prevExitSpeed = 0.0f;
     
     for (auto& move : moveQueue) {
         // Entry speed is limited by the exit speed of the previous move
         // and our own maximum entry speed
         move.entrySpeed = std::min(prevExitSpeed, move.maxEntrySpeed);
         
         // Calculate the maximum possible exit speed based on distance and acceleration
         // v_f^2 = v_i^2 + 2*a*d
         float maxReachableSpeed = std::sqrt(move.entrySpeed * move.entrySpeed + 
                                            2.0f * move.acceleration * move.distance / 60.0f) * 60.0f;
         
         // Exit speed is limited by max speed, next move's entry constraint, and what we can reach
         move.exitSpeed = std::min(maxReachableSpeed, move.maxSpeed);
         
         // Remember this exit speed for the next move
         prevExitSpeed = move.exitSpeed;
     }
 }
 
 void Planner::backwardPass()
 {
     // Start from the last move with the current exit speed
     for (auto it = moveQueue.rbegin(); it != moveQueue.rend(); ++it) {
         PlannerMove& move = *it;
         
         if (it != moveQueue.rbegin()) {
             // This move's exit speed is limited by the next move's entry speed
             PlannerMove& nextMove = *(it - 1);
             move.exitSpeed = std::min(move.exitSpeed, nextMove.entrySpeed);
         }
         
         // Calculate the maximum entry speed that allows us to reach the exit speed
         // v_i^2 = v_f^2 - 2*a*d (reversed acceleration formula)
         float maxAllowedEntrySpeed = std::sqrt(std::max(0.0f, 
                                              move.exitSpeed * move.exitSpeed - 
                                              2.0f * move.acceleration * move.distance / 60.0f)) * 60.0f;
         
         // Limit entry speed by the calculated maximum and the original constraint
         move.entrySpeed = std::min(move.entrySpeed, maxAllowedEntrySpeed);
         
         // Also limit by the absolute maximum entry speed for this move
         move.entrySpeed = std::min(move.entrySpeed, move.maxEntrySpeed);
     }
 }