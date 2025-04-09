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
 
 #endif // COMMON_TYPES_H