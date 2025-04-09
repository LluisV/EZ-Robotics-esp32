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
 
 #endif // COMMON_TYPES_H