/**
 * @file MotionControl.cpp
 * @brief Implementation of the MotionControl class
 */

 #include "MotionControl.h"
 #include "Planner.h"
 #include "Kinematics.h"
 #include <cmath>
 
 MotionControl::MotionControl(MachineController* machineController, Planner* planner)
     : machineController(machineController), planner(planner), kinematics(nullptr)
 {
 }
 
 bool MotionControl::initialize()
 {
     // Create a default Cartesian kinematics implementation
     kinematics = new CartesianKinematics();
     
     Debug::info("MotionControl", "Initialized with Cartesian kinematics");
     return true;
 }
 
 bool MotionControl::executeLinearMove(const std::vector<float>& targetPos, float feedrate, MovementType moveType)
 {
     if (!machineController || !planner) {
         Debug::error("MotionControl", "Invalid machine controller or planner");
         return false;
     }
     
     // Get current position as starting point
     std::vector<float> currentPos = machineController->getCurrentWorkPosition();
     
     // Calculate movement in work coordinates
     float distance = 0.0f;
     for (size_t i = 0; i < std::min(currentPos.size(), targetPos.size()); i++) {
         float delta = targetPos[i] - currentPos[i];
         distance += delta * delta;
     }
     distance = std::sqrt(distance);
     
     // Skip zero-distance moves
     if (distance < 0.001f) {
         return true;
     }
     
     // Convert to machine coordinates using kinematics
     std::vector<float> machineCurrentPos = machineController->getCurrentWorldPosition();
     std::vector<float> machineTargetPos;
     
     // Calculate machine target position from work target position
     machineTargetPos = machineController->workToMachinePositions(targetPos);
     
     // Create a move for the planner
     PlannerMove move;
     move.type = moveType;
     move.targetPosition = machineTargetPos;
     move.feedrate = feedrate;
     move.distance = distance;
     
     // Add move to the planner
     bool result = planner->addMove(move);
     
     if (!result) {
         Debug::warning("MotionControl", "Failed to add move to planner - queue may be full");
     }
     
     return result;
 }
 
 bool MotionControl::executeArcMove(const std::vector<float>& endPos, const std::vector<float>& centerOffset, 
                       float feedrate, bool isClockwise)
 {
     if (!machineController || !planner) {
         Debug::error("MotionControl", "Invalid machine controller or planner");
         return false;
     }
     
     // Get current position as starting point for the arc
     std::vector<float> startPos = machineController->getCurrentWorkPosition();
     
     // Need at least 2D coordinates
     if (startPos.size() < 2 || endPos.size() < 2 || centerOffset.size() < 2) {
         Debug::error("MotionControl", "Arc move requires at least 2D coordinates");
         return false;
     }
     
     // Break the arc into small linear segments
     return segmentArc(startPos, endPos, centerOffset, feedrate, isClockwise);
 }
 
 bool MotionControl::executeDwell(float seconds)
 {
     // For dwell, we need to wait for the planner to complete all moves
     if (!planner->waitForCompletion(std::max(0.0f, seconds))) {
         Debug::warning("MotionControl", "Dwell execution interrupted");
         return false;
     }
     
     return true;
 }
 
 bool MotionControl::executeHoming(const std::vector<String>& axes)
 {
     if (!machineController) {
         Debug::error("MotionControl", "Invalid machine controller");
         return false;
     }
     
     // Wait for any ongoing movement to complete
     if (!planner->waitForCompletion(0)) {
         Debug::warning("MotionControl", "Could not wait for planner completion before homing");
         return false;
     }
     
     // If no specific axes are provided, home all axes
     if (axes.empty()) {
         return machineController->homeAll();
     }
     
     // Home the specified axes one by one
     bool success = true;
     for (const String& axis : axes) {
         success &= machineController->homeAxis(axis);
     }
     
     return success;
 }
 
 bool MotionControl::hasSpace() const
 {
     return planner ? !planner->isFull() : false;
 }
 
 bool MotionControl::segmentArc(const std::vector<float>& startPos, const std::vector<float>& endPos,
                                const std::vector<float>& centerOffset, float feedrate, bool isClockwise)
 {
     // This is a simplified arc segmentation that works in 2D (XY plane)
     // For 3D arcs (helical interpolation), additional math would be needed
     
     // Calculate arc radius and angular span
     float radius = std::sqrt(centerOffset[0] * centerOffset[0] + centerOffset[1] * centerOffset[1]);
     
     // Calculate start and end angles
     float startAngle = std::atan2(-centerOffset[1], -centerOffset[0]);
     float endAngle = std::atan2(endPos[1] - (startPos[1] + centerOffset[1]), 
                                endPos[0] - (startPos[0] + centerOffset[0]));
     
     // Adjust angles based on direction
     float angleDelta = endAngle - startAngle;
     
     // Fix angle for proper direction
     if (isClockwise) {
         if (angleDelta > 0) angleDelta -= 2 * M_PI;
     } else {
         if (angleDelta < 0) angleDelta += 2 * M_PI;
     }
     
     // Make sure angleDelta has the correct sign for the arc direction
     if ((isClockwise && angleDelta > 0) || (!isClockwise && angleDelta < 0)) {
         angleDelta = isClockwise ? -2 * M_PI + angleDelta : 2 * M_PI + angleDelta;
     }
     
     // Calculate how many segments to use
     int segments = calculateArcSegments(radius, std::abs(angleDelta));
     
     // Center point of the arc
     float centerX = startPos[0] + centerOffset[0];
     float centerY = startPos[1] + centerOffset[1];
     
     // Z-axis incremental movement per segment (for helical moves)
     float zIncrement = 0;
     if (startPos.size() > 2 && endPos.size() > 2) {
         zIncrement = (endPos[2] - startPos[2]) / segments;
     }
     
     // Generate segments
     bool success = true;
     
     for (int i = 1; i <= segments; i++) {
         float angle = startAngle + angleDelta * i / segments;
         
         // Calculate segment endpoint
         std::vector<float> segmentEnd = startPos;
         segmentEnd[0] = centerX + radius * std::cos(angle);
         segmentEnd[1] = centerY + radius * std::sin(angle);
         
         // If this is a helical move, update Z coordinate
         if (startPos.size() > 2 && endPos.size() > 2) {
             segmentEnd[2] = startPos[2] + zIncrement * i;
         }
         
         // Add segment to the planner
         if (!executeLinearMove(segmentEnd, feedrate, LINEAR_MOVE)) {
             success = false;
             break;
         }
     }
     
     return success;
 }
 
 int MotionControl::calculateArcSegments(float radius, float angleDelta)
 {
     // Get arc tolerance from configuration
     float arcTolerance = 0.002f; // Default is 0.002mm, can be loaded from config
     if (machineController && machineController->getConfigManager()) {
         const MachineConfig& config = machineController->getConfigManager()->getMachineConfig();
         arcTolerance = config.arcTolerance;
     }
     
     // Calculate minimum number of segments needed to stay within tolerance
     float segmentLength = 2 * std::sqrt(arcTolerance * (2 * radius - arcTolerance));
     float arcLength = radius * angleDelta;
     
     int segments = std::max(1, (int)std::ceil(std::abs(arcLength) / segmentLength));
     
     // Ensure we have a reasonable number of segments
     segments = std::min(segments, 100); // Cap at 100 segments to prevent excessive segmentation
     
     return segments;
 }