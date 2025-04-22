#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <vector>
#include "MotorManager.h"
#include "ConfigManager.h"
#include "CommonTypes.h"
#include "Debug.h"
class MachineController;
#include <deque> 


/**
 * @brief Motion scheduler class with velocity look-ahead and progressive segmentation
 */
class Scheduler {
public:
    /**
     * @brief Constructor
     * @param machineController Reference to machine controller
     * @param motorManager Reference to motor manager
     * @param configManager Reference to config manager
     */
    Scheduler(MachineController* machineController, MotorManager* motorManager, ConfigManager* configManager);
    
    /**
     * @brief Initialize the scheduler
     * @return True if successful
     */
    bool initialize();
    
    /**
     * @brief Add a linear move to the queue
     * @param targetPos Target position
     * @param feedrate Feedrate in mm/min
     * @return True if successful
     */
    bool addLinearMove(const std::vector<float>& targetPos, float feedrate);
    
    /**
     * @brief Add a rapid move to the queue
     * @param targetPos Target position
     * @return True if successful
     */
    bool addRapidMove(const std::vector<float>& targetPos);
    
    /**
     * @brief Process moves and generate segments progressively
     */
    void processQueue();
    
    /**
     * @brief Execute the next segment
     * @return True if a segment was executed
     */
    bool executeNextSegment();
    
    /**
     * @brief Clear all queues
     */
    void clear();
    
    /**
     * @brief Check if scheduler has moves to process
     * @return True if there are moves or segments
     */
    bool hasMove() const;

    /**
     * @brief Check if the scheduler queue is full
     * @return True if either move queue or segment buffer is full
     */
    bool isFull() const;

private:
    MachineController* machineController;      // Reference to machine controller
    MotorManager* motorManager;                // Reference to motor manager
    ConfigManager* configManager;              // Reference to config manager
    std::vector<int32_t> currentSteps;         // Current steps for each motor
    std::deque<ScheduledMove> moveQueue;       // Queue of planned moves
    std::deque<Segment> segmentBuffer;         // Buffer of segments

    const int MOVE_QUEUE_SIZE = 128;           // Maximum number of moves in queue
    const int SEGMENT_BUFFER_SIZE = 1000;       // Maximum segments in buffer
    const float SEGMENT_MAX_LENGTH = 10.0f;     // 0.15mm max segment length
    const float SEGMENT_BUFFER_THRESHOLD = 0.5f;   // When to start generating more segments (50%)
    
    
    /**
     * @brief Generate segments progressively for a move
     * @param move Move to segment
     * @return True if segments were generated successfully
     */
    bool generateSegmentsProgressive(ScheduledMove& move);
    
    /**
     * @brief Apply velocity adjustments using look-ahead algorithm
     * Performs forward and backward passes to adjust segment velocities
     * respecting acceleration and deceleration constraints
     */
    void applyVelocityAdjustments();
    
    /**
     * @brief Execute a segment
     * @param segment Segment to execute
     * @return True if execution started
     */
    bool executeSegment(const Segment& segment);
};

#endif // SCHEDULER_H