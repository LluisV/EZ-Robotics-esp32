/**
 * @file MotionControl.h
 * @brief Motion control module for translating parsed GCode into planned movements
 */

 #ifndef MOTION_CONTROL_H
 #define MOTION_CONTROL_H
 
 #include <Arduino.h>
 #include <vector>
 #include "MachineController.h"
 #include "Kinematics.h"
 #include "Planner.h"
 #include "CommonTypes.h"
 #include "Debug.h"
 
 // Forward declarations
 class Planner;
 
 /**
  * @brief Motion control class for transforming GCode into machine movements
  * 
  * This class takes parsed GCode commands and transforms them into planned linear
  * movements that can be executed by the machine. It handles coordinate transformations,
  * arc generation, and other motion-related operations.
  */
 class MotionControl {
 public:
     /**
      * @brief Construct a new Motion Control object
      * 
      * @param machineController Reference to the machine controller
      * @param planner Reference to the motion planner
      */
     MotionControl(MachineController* machineController, Planner* planner);
 
     /**
      * @brief Initialize the motion control system
      * 
      * @return true if initialization successful
      * @return false if initialization failed
      */
     bool initialize();
 
     /**
      * @brief Execute a linear move
      * 
      * @param targetPos Target position in work coordinates
      * @param feedrate Feedrate in mm/min
      * @param moveType Type of movement (rapid, linear, etc.)
      * @return true if move was scheduled successfully
      * @return false if move could not be scheduled
      */
     bool executeLinearMove(const std::vector<float>& targetPos, float feedrate, MovementType moveType);
 
     /**
      * @brief Execute an arc move
      * 
      * @param endPos End position of the arc
      * @param centerOffset Center offset from start position
      * @param feedrate Feedrate in mm/min
      * @param isClockwise True if clockwise arc, false for counter-clockwise
      * @return true if arc was scheduled successfully
      * @return false if arc could not be scheduled
      */
     bool executeArcMove(const std::vector<float>& endPos, const std::vector<float>& centerOffset, 
                         float feedrate, bool isClockwise);
 
     /**
      * @brief Execute a dwell (pause)
      * 
      * @param seconds Dwell time in seconds
      * @return true if dwell was scheduled successfully
      * @return false if dwell could not be scheduled
      */
     bool executeDwell(float seconds);
 
     /**
      * @brief Execute a homing operation
      * 
      * @param axes Vector of axes to home (empty means all axes)
      * @return true if homing was scheduled successfully
      * @return false if homing could not be scheduled
      */
     bool executeHoming(const std::vector<String>& axes);
 
     /**
      * @brief Check if the planner queue has space for more movements
      * 
      * @return true if planner has space
      * @return false if planner is full
      */
     bool hasSpace() const;
 
     /**
      * @brief Get the machine controller
      * 
      * @return Pointer to the machine controller
      */
     MachineController* getMachineController() const { return machineController; }
 
 private:
     MachineController* machineController;  ///< Reference to the machine controller
     Planner* planner;                      ///< Reference to the motion planner
     Kinematics* kinematics;                ///< Kinematics calculator for coordinate transformations
     
     /**
      * @brief Break an arc into small linear segments
      * 
      * @param startPos Starting position
      * @param endPos Ending position
      * @param centerOffset Center offset from start position
      * @param feedrate Feedrate in mm/min
      * @param isClockwise True if clockwise arc, false if counter-clockwise
      * @return true if segmentation and scheduling successful
      * @return false if segmentation or scheduling failed
      */
     bool segmentArc(const std::vector<float>& startPos, const std::vector<float>& endPos,
                     const std::vector<float>& centerOffset, float feedrate, bool isClockwise);
     
     /**
      * @brief Calculate the number of segments needed for an arc
      * 
      * @param radius Arc radius
      * @param angleDelta Angular span of the arc in radians
      * @return Number of segments
      */
     int calculateArcSegments(float radius, float angleDelta);
 };
 
 #endif // MOTION_CONTROL_H