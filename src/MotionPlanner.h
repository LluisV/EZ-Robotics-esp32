/**
 * @file MotionPlanner.h
 * @brief Motion planner with look-ahead and junction deviation for smooth path execution
 */

 #ifndef MOTION_PLANNER_H
 #define MOTION_PLANNER_H
 
 #include <Arduino.h>
 #include <vector>
 #include <deque>
 #include "MotorManager.h"
 #include "ConfigManager.h"
 #include "CommonTypes.h"
 #include "Debug.h"
 
 // Forward declarations
 class MotorManager;
 class ConfigManager;
 
 /**
  * @brief Structure representing a planned motion segment
  */
 struct MotionSegment {
     std::vector<float> startPos;      ///< Starting position for each axis
     std::vector<float> endPos;        ///< Ending position for each axis
     std::vector<float> unitVector;    ///< Unit vector of the direction of travel
     float distance;                   ///< Euclidean distance of the segment
     float entryVelocity;              ///< Entry velocity in mm/s
     float exitVelocity;               ///< Exit velocity in mm/s
     float maxVelocity;                ///< Maximum allowed velocity for this segment in mm/s
     float acceleration;               ///< Acceleration in mm/s²
     float deceleration;               ///< Deceleration in mm/s² (may be different from acceleration)
     bool recalculate;                 ///< Flag to indicate if this segment needs recalculation
     MovementType type;                ///< Type of movement (RAPID or LINEAR)
 };
 
 /**
  * @brief Motion planner class that implements look-ahead and junction deviation
  */
 class MotionPlanner {
 public:
     /**
      * @brief Construct a new Motion Planner
      * @param motorManager Reference to the motor manager
      * @param configManager Reference to the configuration manager
      */
     MotionPlanner(MotorManager* motorManager, ConfigManager* configManager);
 
     /**
      * @brief Initialize the motion planner
      * @return True if successful
      */
     bool initialize();
 
     /**
      * @brief Add a new move to the planning queue
      * @param targetPos Target position for all axes
      * @param feedrate Feedrate in mm/min
      * @param movementType Type of movement (RAPID or LINEAR)
      * @return True if successful
      */
     bool addMove(const std::vector<float>& targetPos, float feedrate, MovementType movementType);
 
     /**
      * @brief Execute the next move in the queue
      * @return True if a move was executed
      */
     bool executeMove();
 
     /**
      * @brief Clear the planning queue
      */
     void clear();
 
     /**
      * @brief Check if the planner has moves to execute
      * @return True if planner has moves
      */
     bool hasMove() const;
 
     /**
      * @brief Get the number of moves in the queue
      * @return Number of moves
      */
     size_t queueSize() const;
 
     /**
      * @brief Set junction deviation
      * @param junctionDeviation Junction deviation in mm
      */
     void setJunctionDeviation(float junctionDeviation);
 
     /**
      * @brief Set the current position
      * @param position Current position for all axes
      */
     void setCurrentPosition(const std::vector<float>& position);

     /**
     * @brief Check if the motion planning queue is full
     * @return True if the queue is full
     */
    bool isFull() const;
 
 private:
     // Constants
     static const size_t MAX_QUEUE_SIZE = 32;        ///< Maximum segments in planning queue
     static const size_t MIN_SEGMENTS_PLANNING = 3;  ///< Minimum segments for planning
 
     MotorManager* motorManager;                ///< Reference to motor manager
     ConfigManager* configManager;              ///< Reference to configuration manager
     std::deque<MotionSegment> planningQueue;   ///< Queue of motion segments for planning
     std::vector<float> currentPosition;        ///< Current machine position
     float junctionDeviation;                   ///< Junction deviation setting (mm)
     float junctionVelocitySquared;             ///< Cached computation for junction velocity
     bool running;                              ///< Flag to indicate if planner is active
 
     /**
      * @brief Calculate the entry and exit velocities for all segments in the planning queue
      */
     void planVelocityProfile();
 
     /**
      * @brief Calculate the junction velocity between two segments
      * @param prev Previous segment
      * @param current Current segment
      * @return Junction velocity in mm/s
      */
     float calculateJunctionVelocity(const MotionSegment& prev, const MotionSegment& current);
 
     /**
      * @brief Calculate the unit vector of a segment
      * @param segment Motion segment
      */
     void calculateUnitVector(MotionSegment& segment);
 
     /**
      * @brief Calculate the Euclidean distance of a segment
      * @param segment Motion segment
      * @return Distance in mm
      */
     float calculateDistance(const MotionSegment& segment);
 
     /**
      * @brief Calculate the dot product of two unit vectors
      * @param v1 First unit vector
      * @param v2 Second unit vector
      * @return Dot product
      */
     float dotProduct(const std::vector<float>& v1, const std::vector<float>& v2);
 
     /**
      * @brief Execute a single motion segment
      * @param segment Motion segment to execute
      * @return True if execution was successful
      */
     bool executeSegment(const MotionSegment& segment);
 
     /**
      * @brief Calculate trapezoid velocity profile for a segment
      * @param segment Motion segment
      */
     void calculateTrapezoidProfile(MotionSegment& segment);
 
     /**
      * @brief Forward pass - calculate exit velocities based on maximum acceleration
      */
     void forwardPass();
 
     /**
      * @brief Backward pass - adjust entry velocities based on exit velocities
      */
     void backwardPass();
 };
 
 #endif // MOTION_PLANNER_H