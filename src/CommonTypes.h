/**
 * @file CommonTypes.h
 * @brief Common type definitions shared across multiple components
 */

 #ifndef COMMON_TYPES_H
 #define COMMON_TYPES_H
 
 #include <Arduino.h>
 #include <vector>
 
 /**
  * @brief Movement types
  */
 enum MovementType
 {
   RAPID_MOVE,  ///< Rapid movement (G0)
   LINEAR_MOVE, ///< Linear movement (G1)
   ARC_CW_MOVE, ///< Clockwise arc (G2)
   ARC_CCW_MOVE ///< Counter-clockwise arc (G3)
 };

 /**
 * @brief Enum for G-code parsing and queueing results
 */
enum class GCodeParseResult
{
    PARSE_ERROR = -1,     ///< Parsing failed, command is invalid
    QUEUE_FULL = 0,       ///< Motion planner queue is full, retry later
    SUCCESS = 1           ///< Successfully parsed and queued
};

/**
 * @brief Structure representing a motion segment
 */
struct Segment {
  std::vector<float> jointPositions;     // Target positions for each joint (mm)
  std::vector<float> desiredVelocities;         // Desired velocities for each joint (steps/s)
  std::vector<float> adjustedVelocities; // Adjusted velocities after look-ahead (steps/s)
  float distance;                        // Segment distance (mm)
};

/**
* @brief Structure representing a planned move
*/
struct ScheduledMove {
  std::vector<float> targetPosition;  // Target position in machine units
  MovementType type;                  // Type of movement
  float feedrate;                     // Feedrate in mm/min
  
  float segmentationProgress;        // Progress of segmentation (0.0 to 1.0)
  int totalSegments;                 // Total number of segments for this move
  float totalDistance;               // Total distance of the move
  std::vector<float> startPosition;  // Start position of the move
  std::vector<float> moveVector;     // Movement vector from start to end
};
 
 #endif // COMMON_TYPES_H