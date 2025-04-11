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
 * @brief Motion scheduler class
 */
class Scheduler {
public:
    /**
     * @brief Constructor
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
     * @brief Process moves and generate segments
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
     * @brief Set current position
     * @param position Current position
     */
    void setCurrentPosition(const std::vector<float>& position);

    /**
     * @brief Check if the scheduler queue is full
     * @return True if either move queue or segment buffer is full
     */
    bool isFull() const;

private:
    MachineController* machineController;
    MotorManager* motorManager;                // Reference to motor manager
    ConfigManager* configManager;              // Reference to config manager
    std::vector<int32_t> currentSteps;         // Current steps for each motor
    std::deque<ScheduledMove> moveQueue;       // Queue of planned moves
    std::deque<Segment> segmentBuffer;         // Buffer of segments
    bool executing;                            // Flag for execution status

    const int MOVE_QUEUE_SIZE = 128;  // Maximum number of moves in queue
    const int SEGMENT_BUFFER_SIZE = 1024;
    const float SEGMENT_MAX_LENGTH = 0.25f; // 0.25mm max segment length
    const float LOOK_AHEAD_DISTANCE = 20; //20mm


    
    /**
     * @brief Generate segments for a move
     * @param move Move to segment
     * @return Number of segments generated
     */
    bool generateSegments(ScheduledMove& move);
    
    /**
     * @brief Calculate move direction vector
     * @param start Start position
     * @param end End position
     * @return Unit vector of direction
     */
    std::vector<float> calculateDirectionVector(const std::vector<float>& start, 
                                                const std::vector<float>& end);
    
    /**
     * @brief Calculate dot product of two vectors
     * @param v1 First vector
     * @param v2 Second vector
     * @return Dot product
     */
    float dotProduct(const std::vector<float>& v1, const std::vector<float>& v2);
    
    /**
     * @brief Calculate magnitude of direction change
     * @param prevDir Previous direction
     * @param newDir New direction
     * @return Value from 0 (no change) to 1 (180° change)
     */
    float calculateDirectionChange(const std::vector<float>& prevDir, 
                                  const std::vector<float>& newDir);
    
    /**
     * @brief Execute a segment
     * @param segment Segment to execute
     * @return True if execution started
     */
    bool executeSegment(const Segment& segment);
    
    /**
     * @brief Update execution status
     * @return True if status changed
     */
    bool updateExecutionStatus();
};

#endif // SCHEDULER_H