/**
 * @file Planner.h
 * @brief Motion planner for coordinated movement with acceleration planning
 */

 #ifndef PLANNER_H
 #define PLANNER_H
 
 #include <Arduino.h>
 #include <vector>
 #include <deque>
 #include "CommonTypes.h"
 #include "Debug.h"
 
 // Forward declarations
 class SegmentGenerator;
 
 /**
  * @brief Motion move structure for the planner
  */
 struct PlannerMove {
     std::vector<float> targetPosition;  ///< Target position in machine coordinates
     MovementType type;                  ///< Type of movement (RAPID_MOVE, LINEAR_MOVE, etc.)
     float feedrate;                     ///< Feedrate in mm/min
     float distance;                     ///< Total distance of the move
     
     float entrySpeed;                   ///< Entry speed (mm/min)
     float exitSpeed;                    ///< Exit speed (mm/min)
     float maxEntrySpeed;                ///< Maximum allowable entry speed (mm/min)
     float maxExitSpeed;                 ///< Maximum allowable exit speed (mm/min)
     float maxSpeed;                     ///< Maximum speed for this move (mm/min)
     float acceleration;                 ///< Acceleration for this move (mm/s^2)
     
     std::vector<float> unitVector;      ///< Unit vector in the direction of travel
     int32_t lineNumber;                 ///< Original GCode line number (if available)
     bool recalculate;                   ///< Flag indicating this move needs recalculation
 };
 
 /**
  * @brief Motion planner for CNC controller
  * 
  * Handles movement planning with proper acceleration and deceleration
  * to ensure smooth coordinated motion between axes.
  */
 class Planner {
 public:
     /**
      * @brief Construct a new Planner object
      * 
      * @param segmentGenerator Reference to the segment generator
      */
     Planner(SegmentGenerator* segmentGenerator);
     
     /**
      * @brief Initialize the planner
      * 
      * @return true if initialization successful
      * @return false if initialization failed
      */
     bool initialize();
     
     /**
      * @brief Add a move to the planner queue
      * 
      * @param move Move to add
      * @return true if move was added successfully
      * @return false if move could not be added
      */
     bool addMove(const PlannerMove& move);
     
     /**
      * @brief Add a rapid move to the planner queue
      * 
      * @param targetPos Target position in machine coordinates
      * @return true if move was added successfully
      * @return false if move could not be added
      */
     bool addRapidMove(const std::vector<float>& targetPos);
     
     /**
      * @brief Add a linear move to the planner queue
      * 
      * @param targetPos Target position in machine coordinates
      * @param feedrate Feedrate in mm/min
      * @return true if move was added successfully
      * @return false if move could not be added
      */
     bool addLinearMove(const std::vector<float>& targetPos, float feedrate);
     
     /**
      * @brief Get the next planned move
      * 
      * @return Next move or nullptr if no moves available
      */
     const PlannerMove* getNextMove();
     
     /**
      * @brief Remove the current move from the queue
      * 
      * @return true if successful
      * @return false if no move to remove
      */
     bool removeCurrentMove();
     
     /**
      * @brief Check if the planner queue is empty
      * 
      * @return true if queue is empty
      * @return false if queue has moves
      */
     bool isEmpty() const;
     
     /**
      * @brief Check if the planner queue is full
      * 
      * @return true if queue is full
      * @return false if queue has space
      */
     bool isFull() const;
     
     /**
      * @brief Clear all planned moves
      */
     void clear();
     
     /**
      * @brief Wait for the planner to complete all moves
      * 
      * @param dwell_time Additional dwell time in seconds (0 for no dwell)
      * @return true if all moves completed normally
      * @return false if waiting was interrupted
      */
     bool waitForCompletion(float dwell_time = 0.0f);
     
     /**
      * @brief Get the number of moves in the queue
      * 
      * @return Move count
      */
     size_t moveCount() const;
     
     /**
      * @brief Process the planning queue, updating speeds and accelerations
      * 
      * This is called when a new move is added or when a move is completed
      * to recalculate the entry and exit speeds for proper junction planning.
      */
     void recalculatePlannerMoves();
 
 private:
     SegmentGenerator* segmentGenerator; ///< Reference to the segment generator
     std::deque<PlannerMove> moveQueue;  ///< Queue of planned moves
     
     const size_t MAX_MOVE_QUEUE_SIZE = 32;  ///< Maximum number of moves in the queue
     float junctionDeviation;               ///< Junction deviation setting (controls cornering)
     float maxVelocity;                     ///< Maximum allowed velocity in mm/min
     float maxAcceleration;                 ///< Maximum allowed acceleration in mm/s^2
     
     /**
      * @brief Calculate the maximum junction speed between two moves
      * 
      * @param previous Previous move
      * @param current Current move
      * @return Maximum allowed junction speed in mm/min
      */
     float calculateJunctionSpeed(const PlannerMove& previous, const PlannerMove& current);
     
     /**
      * @brief Calculate a unit vector for a move
      * 
      * @param move Move to calculate vector for
      * @return true if calculation successful
      * @return false if calculation failed
      */
     bool calculateUnitVector(PlannerMove& move);
     
     /**
      * @brief Calculate maximum allowable speed for a move
      * 
      * @param move Move to calculate speed for
      */
     void calculateMaxSpeed(PlannerMove& move);
     
     /**
      * @brief Perform forward planning pass
      * 
      * This propagates speed constraints forward through the queue,
      * setting maximum exit speeds based on junction limits.
      */
     void forwardPass();
     
     /**
      * @brief Perform backward planning pass
      * 
      * This propagates speed constraints backward through the queue,
      * ensuring proper deceleration between moves.
      */
     void backwardPass();
 };
 
 #endif // PLANNER_H