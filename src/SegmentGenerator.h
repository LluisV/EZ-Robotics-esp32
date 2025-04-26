/**
 * @file SegmentGenerator.h
 * @brief Motion segment generator for breaking moves into constant-velocity segments
 */

 #ifndef SEGMENT_GENERATOR_H
 #define SEGMENT_GENERATOR_H
 
 #include <Arduino.h>
 #include <vector>
 #include <deque>
 #include "CommonTypes.h"
 #include "Debug.h"
 
 // Forward declarations
 class MotorManager;
 class Planner;
 class MachineController;
 
 /**
  * @brief Segment generator for breaking moves into constant-velocity segments
  * 
  * The segment generator takes planned moves from the Planner and breaks them
  * down into small constant-velocity segments that can be executed by the motors.
  * It handles acceleration and deceleration profiles within each move.
  */
 class SegmentGenerator {
 public:
     /**
      * @brief Construct a new Segment Generator
      * 
      * @param machineController Reference to the machine controller
      * @param motorManager Reference to the motor manager
      */
     SegmentGenerator(MachineController* machineController, MotorManager* motorManager);
     
     /**
      * @brief Initialize the segment generator
      * 
      * @return true if initialization successful
      * @return false if initialization failed
      */
     bool initialize();
     
     /**
      * @brief Set the planner reference
      * 
      * @param planner Pointer to the planner
      */
     void setPlanner(Planner* planner) { this->planner = planner; }
     
     /**
      * @brief Generate segments for the current move
      * 
      * Processes the current move from the planner and generates
      * appropriate segments for execution.
      */
     void generateSegments();
     
     /**
      * @brief Execute the next segment
      * 
      * @return true if a segment was executed
      * @return false if no segment was available to execute
      */
     bool executeNextSegment();
     
     /**
      * @brief Check if the segment buffer is empty
      * 
      * @return true if no segments are available
      * @return false if segments are available
      */
     bool isEmpty() const;
     
     /**
      * @brief Check if the segment buffer is full
      * 
      * @return true if buffer is full
      * @return false if buffer has space
      */
     bool isFull() const;
     
     /**
      * @brief Clear all segments
      */
     void clear();
     
     /**
      * @brief Process the pipeline
      * 
      * This function checks the planner for new moves, generates segments
      * if needed, and executes the next segment if motors are ready.
      * 
      * @return true if any action was taken
      * @return false if no action was taken
      */
     bool update();
 
 private:
     MachineController* machineController;  ///< Reference to the machine controller
     MotorManager* motorManager;            ///< Reference to the motor manager
     Planner* planner;                      ///< Reference to the planner
     std::deque<Segment> segmentBuffer;     ///< Buffer of generated segments
     std::vector<int32_t> currentSteps;     ///< Current position in steps
     
     const int SEGMENT_BUFFER_SIZE = 32;            ///< Maximum number of segments in buffer
     const float SEGMENT_MAX_LENGTH = 0.5f;         ///< Maximum segment length in mm
     const float SEGMENT_MIN_LENGTH = 0.1f;         ///< Minimum segment length in mm
     const float MIN_SEGMENT_TIME = 0.01f;          ///< Minimum time per segment in seconds
     
     /**
      * @brief Break a move into segments based on acceleration profile
      * 
      * @param startPos Starting position in machine units
      * @param endPos Ending position in machine units
      * @param startSpeed Starting speed in mm/min
      * @param endSpeed Ending speed in mm/min
      * @param maxSpeed Maximum speed in mm/min
      * @param acceleration Acceleration in mm/s^2
      * @param moveVector Unit vector in the direction of motion
      */
     void segmentMove(const std::vector<float>& startPos, const std::vector<float>& endPos,
                     float startSpeed, float endSpeed, float maxSpeed, float acceleration,
                     const std::vector<float>& moveVector);
     
     /**
      * @brief Calculate the number of segments needed for a move
      * 
      * @param moveDistance Total move distance
      * @param startSpeed Starting speed
      * @param endSpeed Ending speed
      * @param maxSpeed Maximum speed
      * @param acceleration Acceleration
      * @return Number of segments
      */
     int calculateSegmentCount(float moveDistance, float startSpeed, float endSpeed, 
                              float maxSpeed, float acceleration);
     
     /**
      * @brief Add a segment to the buffer
      * 
      * @param segment Segment to add
      * @return true if segment was added successfully
      * @return false if segment could not be added
      */
     bool addSegment(const Segment& segment);
     
     /**
      * @brief Calculate the speed at a particular distance in an acceleration profile
      * 
      * @param initialSpeed Initial speed in mm/min
      * @param distance Distance traveled in mm
      * @param acceleration Acceleration in mm/s^2
      * @return Speed at the specified distance in mm/min
      */
     float calculateSpeed(float initialSpeed, float distance, float acceleration);
     
     /**
      * @brief Calculate the distance needed to change speed
      * 
      * @param initialSpeed Initial speed in mm/min
      * @param targetSpeed Target speed in mm/min
      * @param acceleration Acceleration in mm/s^2
      * @return Distance required in mm
      */
     float calculateDistance(float initialSpeed, float targetSpeed, float acceleration);
 };
 
 #endif // SEGMENT_GENERATOR_H