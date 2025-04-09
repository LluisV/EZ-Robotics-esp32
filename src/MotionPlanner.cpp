/**
 * @file MotionPlanner.cpp
 * @brief Implementation of the MotionPlanner class
 */

 #include "MotionPlanner.h"
 #include <cmath>
 
 MotionPlanner::MotionPlanner(MotorManager* motorManager, ConfigManager* configManager)
     : motorManager(motorManager),
       configManager(configManager),
       junctionDeviation(0.05f),
       running(false)
 {
     // Initialize default current position to zeros
     if (motorManager) {
         currentPosition.resize(motorManager->getNumMotors(), 0.0f);
     }
     
     // Initialize junction velocity calculation
     junctionVelocitySquared = 2.0f * 9.8f * junctionDeviation;  // 2*g*h formula
 }
 
 bool MotionPlanner::initialize() {
     if (!motorManager || !configManager) {
         Debug::error("MotionPlanner", "Invalid motor manager or config manager");
         return false;
     }
 
     // Get junction deviation from configuration
     const MachineConfig& config = configManager->getMachineConfig();
     this->junctionDeviation = config.junctionDeviation;
     junctionVelocitySquared = 2.0f * 9.8f * junctionDeviation;

     Debug::info("MotionPlanner", "Junction deviation: " + String(junctionDeviation, 4) + 
                                " mm, junction velocity squared: " + 
                                String(junctionVelocitySquared, 4) + " mm²/s²");
     return true;
 }
 
 bool MotionPlanner::addMove(const std::vector<float>& targetPos, float feedrate, MovementType movementType) {
     if (targetPos.size() != currentPosition.size()) {
         Debug::error("MotionPlanner", "Position array size mismatch in addMove");
         return false;
     }
 
     // Skip if the move is too small (less than 0.001mm)
     bool anyMovement = false;
     for (size_t i = 0; i < targetPos.size(); i++) {
         if (fabs(targetPos[i] - currentPosition[i]) > 0.001f) {
             anyMovement = true;
             break;
         }
     }
 
     if (!anyMovement) {
         Debug::verbose("MotionPlanner", "Skipping zero-distance move");
         return true;  // Consider this a success, just skip it
     }
 
     // Convert feedrate from mm/min to mm/s
     float feedrateMMPerSec = feedrate / 60.0f;
 
     // Create a new motion segment
     MotionSegment segment;
     segment.startPos = currentPosition;
     segment.endPos = targetPos;
     segment.type = movementType;
     segment.recalculate = true;
 
     // Calculate unit vector and distance
     calculateUnitVector(segment);
     segment.distance = calculateDistance(segment);
     
     // Set initial velocities
     if (movementType == RAPID_MOVE) {
         // For rapid moves, use maximum machine feedrate
         const MachineConfig& config = configManager->getMachineConfig();
         segment.maxVelocity = config.maxFeedrate / 60.0f;  // Convert to mm/s
     } else {
         // For linear moves, use the specified feedrate
         segment.maxVelocity = feedrateMMPerSec;
     }
     
     // Initial acceleration/deceleration values
     // These will be adjusted in velocity profile planning
     segment.acceleration = 500.0f;  // Default 500 mm/s²
     segment.deceleration = 500.0f;
 
     // Initial entry/exit velocities (to be recalculated during planning)
     segment.entryVelocity = 0.0f;
     segment.exitVelocity = 0.0f;
 
     // Add to planning queue
     planningQueue.push_back(segment);
 
     // Update current position
     currentPosition = targetPos;
 
     // If queue has enough segments, perform planning
     if (planningQueue.size() >= MIN_SEGMENTS_PLANNING) {
         planVelocityProfile();
     }
 
     // If queue is getting too large, execute the oldest move
     if (planningQueue.size() > MAX_QUEUE_SIZE - 1) {
         executeMove();
     }
 
     return true;
 }
 
 bool MotionPlanner::executeMove() {
     if (planningQueue.empty()) {
         return false;
     }
 
     // Ensure velocity profile is up-to-date
     planVelocityProfile();
 
     // Get the oldest segment
     const MotionSegment& segment = planningQueue.front();
     
     // Execute the segment
     bool success = executeSegment(segment);
     
     // Remove the segment from the queue
     planningQueue.pop_front();
     
     return success;
 }
 
 bool MotionPlanner::executeSegment(const MotionSegment& segment) {
     // Here we translate the planned segment into motor movement commands
     
     // Calculate target position vector for motor manager
     const std::vector<float>& targetPos = segment.endPos;
     
     Debug::verbose("MotionPlanner", "Executing move to: " + 
                    String(targetPos[0], 3) + ", " + 
                    String(targetPos[1], 3) + ", " + 
                    String(targetPos[2], 3) + 
                    " at " + String(segment.maxVelocity * 60.0f) + " mm/min");
     
     Debug::verbose("MotionPlanner", "Entry velocity: " + String(segment.entryVelocity) + 
                    " mm/s, Exit velocity: " + String(segment.exitVelocity) + " mm/s");
     
     // Convert velocities to corresponding speeds for motor manager
     float feedrate = segment.maxVelocity * 60.0f;  // Convert back to mm/min for motor manager
     
     // Call the appropriate move function based on movement type
     if (segment.type == RAPID_MOVE) {
         return motorManager->moveToRapid(targetPos);
     } else {
         return motorManager->moveToFeedrate(targetPos, feedrate);
     }
 }
 
 void MotionPlanner::clear() {
     planningQueue.clear();
 }
 
 bool MotionPlanner::hasMove() const {
     return !planningQueue.empty();
 }
 
 size_t MotionPlanner::queueSize() const {
     return planningQueue.size();
 }
 
 void MotionPlanner::setJunctionDeviation(float junctionDeviation) {
     this->junctionDeviation = junctionDeviation;
     this->junctionVelocitySquared = 2.0f * 9.8f * junctionDeviation;  // 2*g*h formula
 }
 
 void MotionPlanner::setCurrentPosition(const std::vector<float>& position) {
     currentPosition = position;
 }
 
 void MotionPlanner::planVelocityProfile() {
     if (planningQueue.size() < 2) {
         // Need at least 2 segments to plan velocities between them
         if (planningQueue.size() == 1) {
             // If only one segment, set entry and exit velocity to zero
             MotionSegment& segment = planningQueue.front();
             segment.entryVelocity = 0.0f;
             segment.exitVelocity = 0.0f;
             calculateTrapezoidProfile(segment);
         }
         return;
     }
 
     // First pass: Calculate maximum velocities at junctions
     for (size_t i = 0; i < planningQueue.size() - 1; i++) {
         MotionSegment& current = planningQueue[i];
         MotionSegment& next = planningQueue[i + 1];
         
         // Calculate maximum junction velocity between current and next segment
         float junctionVel = calculateJunctionVelocity(current, next);
         
         // Set exit velocity of current segment and entry velocity of next segment
         current.exitVelocity = junctionVel;
         next.entryVelocity = junctionVel;
     }
 
     // First segment entry and last segment exit velocity should be zero
     // for complete start/stop
     planningQueue.front().entryVelocity = 0.0f;
     planningQueue.back().exitVelocity = 0.0f;
 
     // Forward pass: Adjust exit velocities based on maximum allowed acceleration
     forwardPass();
     
     // Backward pass: Adjust entry velocities based on exit velocities
     backwardPass();
     
     // Calculate trapezoid profiles for each segment
     for (auto& segment : planningQueue) {
         calculateTrapezoidProfile(segment);
     }
 }
 
 float MotionPlanner::calculateJunctionVelocity(const MotionSegment& prev, const MotionSegment& current) {
     // Calculate the angle between segments
     float cosTheta = dotProduct(prev.unitVector, current.unitVector);
     
     // Clamp to valid range (-1 to 1)
     cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));
     
     // If segments are nearly parallel (within 10 degrees), use full speed
     if (cosTheta > 0.984f) {  // cos(10°) ≈ 0.984
         // Find the minimum max_velocity between the two segments
         return std::min(prev.maxVelocity, current.maxVelocity);
     }
     
     // Calculate the angle in radians
     float theta = acosf(cosTheta);
     
     // Calculate the direction change factor
     // 1.0 means same direction, -1.0 means opposite direction
     float directionChangeFactor = 1.0f - (theta / M_PI);
     
     // Calculate the junction velocity using the junction deviation formula
     // This is derived from the centripetal acceleration formula
     // v² = r * a = 2 * g * h, where h is junction_deviation
     float sinHalfTheta = sinf(theta * 0.5f);
     float junctionVel;
     
     if (sinHalfTheta <= 0.001f) {
         // Avoid division by zero for nearly parallel moves
         junctionVel = std::min(prev.maxVelocity, current.maxVelocity);
     } else {
         // Apply junction deviation formula
         // The underlying physics model is based on considering the junction as a circular
         // path with radius r, where v²/r = a (the centripetal acceleration)
         // We rearrange to get v = sqrt(r * a) = sqrt(junctionDeviation / sinHalfTheta * 2 * g)
         junctionVel = sqrtf(junctionVelocitySquared / sinHalfTheta);
         
         // Limit to the lesser of the two segment max velocities
         junctionVel = std::min(junctionVel, std::min(prev.maxVelocity, current.maxVelocity));
     }
     
     Debug::info("MotionPlanner", "Angle: " + String(theta * 180.0f / M_PI) + 
            " deg, cosTheta: " + String(cosTheta) + 
            ", Junction v: " + String(junctionVel));
                    
     return junctionVel;
 }
 
 void MotionPlanner::calculateUnitVector(MotionSegment& segment) {
     segment.unitVector.clear();
     segment.unitVector.resize(segment.endPos.size(), 0.0f);
     
     float sumSquares = 0.0f;
     
     for (size_t i = 0; i < segment.endPos.size(); i++) {
         float delta = segment.endPos[i] - segment.startPos[i];
         segment.unitVector[i] = delta;
         sumSquares += delta * delta;
     }
     
     if (sumSquares > 0.000001f) {
         // Normalize the vector
         float length = sqrtf(sumSquares);
         for (size_t i = 0; i < segment.unitVector.size(); i++) {
             segment.unitVector[i] /= length;
         }
     }
 }
 
 float MotionPlanner::calculateDistance(const MotionSegment& segment) {
     float sumSquares = 0.0f;
     
     for (size_t i = 0; i < segment.endPos.size(); i++) {
         float delta = segment.endPos[i] - segment.startPos[i];
         sumSquares += delta * delta;
     }
     
     return sqrtf(sumSquares);
 }
 
 float MotionPlanner::dotProduct(const std::vector<float>& v1, const std::vector<float>& v2) {
     if (v1.size() != v2.size()) {
         return 0.0f;
     }
     
     float sum = 0.0f;
     
     for (size_t i = 0; i < v1.size(); i++) {
         sum += v1[i] * v2[i];
     }
     
     return sum;
 }
 
 void MotionPlanner::calculateTrapezoidProfile(MotionSegment& segment) {
     // Calculate the trapezoid velocity profile for a segment based on
     // entry velocity, exit velocity, max velocity, and acceleration/deceleration
     
     // Time and distance to accelerate from entry_velocity to max_velocity
     float entryToMaxTime = (segment.maxVelocity - segment.entryVelocity) / segment.acceleration;
     float entryToMaxDist = 0.5f * (segment.entryVelocity + segment.maxVelocity) * entryToMaxTime;
     
     // Time and distance to decelerate from max_velocity to exit_velocity
     float maxToExitTime = (segment.maxVelocity - segment.exitVelocity) / segment.deceleration;
     float maxToExitDist = 0.5f * (segment.maxVelocity + segment.exitVelocity) * maxToExitTime;
     
     // Check if we have enough distance for a trapezoid (with plateau at max velocity)
     if (entryToMaxDist + maxToExitDist <= segment.distance) {
         // We can reach max velocity - this is a trapezoid profile
         Debug::verbose("MotionPlanner", "Trapezoid profile");
     } else {
         // Not enough distance to reach max velocity - this is a triangular profile
         // We need to solve for the peak velocity that satisfies the entry and exit constraints
         // This involves solving a quadratic equation
         float a = 1.0f / segment.acceleration + 1.0f / segment.deceleration;
         float b = 2.0f * (segment.entryVelocity / segment.acceleration + segment.exitVelocity / segment.deceleration);
         float c = -(2.0f * segment.distance + 
                    segment.entryVelocity * segment.entryVelocity / segment.acceleration + 
                    segment.exitVelocity * segment.exitVelocity / segment.deceleration);
         
         // Solve for the peak velocity using the quadratic formula
         float discriminant = b * b - 4.0f * a * c;
         if (discriminant < 0.0f) {
             // This shouldn't happen with valid inputs, but handle it anyway
             discriminant = 0.0f;
         }
         
         float peakVelocity = (-b + sqrtf(discriminant)) / (2.0f * a);
         
         // Ensure the peak velocity is at least the greater of entry and exit velocities
         peakVelocity = std::max(peakVelocity, std::max(segment.entryVelocity, segment.exitVelocity));
         
         // Update the segment's max velocity to this peak
         segment.maxVelocity = peakVelocity;
         
         Debug::verbose("MotionPlanner", "Triangular profile with peak: " + String(peakVelocity) + " mm/s");
     }
 }
 
 void MotionPlanner::forwardPass() {
     float prev_exit_velocity = 0.0f;
     
     for (auto& segment : planningQueue) {
         // Entry velocity constrained by previous segment's exit velocity
         segment.entryVelocity = prev_exit_velocity;
         
         // Calculate the maximum velocity reachable from entry_velocity with the given
         // acceleration and segment distance
         float max_reachable_velocity = sqrtf(segment.entryVelocity * segment.entryVelocity + 
                                            2.0f * segment.acceleration * segment.distance);
         
         // Constrain exit velocity by maximum reachable velocity
         segment.exitVelocity = std::min(segment.exitVelocity, max_reachable_velocity);
         segment.maxVelocity = std::min(segment.maxVelocity, max_reachable_velocity);
         
         prev_exit_velocity = segment.exitVelocity;
     }
 }
 
 void MotionPlanner::backwardPass() {
     float next_entry_velocity = 0.0f;  // Last segment must stop
     
     for (auto it = planningQueue.rbegin(); it != planningQueue.rend(); ++it) {
         MotionSegment& segment = *it;
         
         // Exit velocity constrained by next segment's entry velocity
         segment.exitVelocity = next_entry_velocity;
         
         // Calculate the maximum velocity reachable working backward from exit_velocity
         // with the given deceleration and segment distance
         float max_reverse_velocity = sqrtf(segment.exitVelocity * segment.exitVelocity + 
                                          2.0f * segment.deceleration * segment.distance);
         
         // Constrain entry velocity by maximum reverse velocity
         segment.entryVelocity = std::min(segment.entryVelocity, max_reverse_velocity);
         segment.maxVelocity = std::min(segment.maxVelocity, max_reverse_velocity);
         
         next_entry_velocity = segment.entryVelocity;
     }
 }

 bool MotionPlanner::isFull() const {
    return planningQueue.size() >= MAX_QUEUE_SIZE;
 }