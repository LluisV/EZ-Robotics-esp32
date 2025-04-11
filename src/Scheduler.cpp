#include "Scheduler.h"
#include <cmath>
#include <algorithm>
#include "MachineController.h"

Scheduler::Scheduler(MachineController *machineController, MotorManager *motorManager, ConfigManager *configManager)
    : machineController(machineController),
      motorManager(motorManager),
      configManager(configManager),
      executing(false)
{
    // Initialize position vectors
    if (motorManager)
    {
        int numMotors = motorManager->getNumMotors();
        currentSteps.resize(numMotors, 0);
    }
}

bool Scheduler::initialize()
{
    if (!motorManager || !configManager)
    {
        Debug::error("Scheduler", "Invalid motor or config manager");
        return false;
    }

    // Initialize current position and steps
    int numMotors = motorManager->getNumMotors();
    currentSteps.resize(numMotors, 0);

    // Get current position from motors
    for (int i = 0; i < numMotors; i++)
    {
        Motor *motor = motorManager->getMotor(i);
        if (motor)
        {
            currentSteps[i] = motor->getPosition();
        }
    }

    return true;
}

bool Scheduler::addLinearMove(const std::vector<float> &targetPos, float feedrate)
{
    // Validate target position
    if (targetPos.size() != motorManager->getNumMotors())
    {
        Debug::error("Scheduler", "Target position size mismatch");
        return false;
    }

    // Check if this is a zero-distance move
    bool hasMovement = false;
    for (size_t i = 0; i < targetPos.size(); i++)
    {
        if (fabs(targetPos[i] - machineController->getCurrentWorkPosition()[i]) > 0.001f)
        {
            hasMovement = true;
            break;
        }
    }

    if (!hasMovement)
    {
        Debug::verbose("Scheduler", "Skipping zero-distance move");
        return true; // Consider this successful but skip it
    }

    // Create new scheduled move
    ScheduledMove move;
    move.targetPosition = targetPos;
    move.type = LINEAR_MOVE;
    move.feedrate = feedrate;

    // Add to queue
    moveQueue.push_back(move);

    Debug::verbose("Scheduler", "Added linear move at "
                                    + String(feedrate, 0) + " mm/min");
    return true;
}

bool Scheduler::addRapidMove(const std::vector<float> &targetPos)
{
    // Get max feedrate from config
    const MachineConfig &config = configManager->getMachineConfig();
    float rapidFeedrate = config.maxFeedrate;

    // Use linear move with rapid feedrate
    return addLinearMove(targetPos, rapidFeedrate);
}

void Scheduler::processQueue()
{
    if (moveQueue.empty())
        return;

    // Process any unprocessed moves in queue
    for (auto &move : moveQueue)
    {
        // Generate segments for this move
        bool segmentsGenerated = generateSegments(move);
        if (segmentsGenerated)
        {
            // Apply velocity adjustments after generating segments
            applyVelocityAdjustments();
            moveQueue.pop_front();
        }

        // Only process one move at a time to avoid filling buffer
        break;
    }
}

void Scheduler::applyVelocityAdjustments()
{
    if (segmentBuffer.empty())
        return;

    const MachineConfig &config = configManager->getMachineConfig();
    float maxAcceleration = config.maxAcceleration; // mm/s^2
    int numMotors = motorManager->getNumMotors();

    // Calculate how many segments correspond to look-ahead distance
    float totalDistance = 0.0f;
    int lookAheadSegments = 0;
    
    for (size_t i = 0; i < segmentBuffer.size(); i++)
    {
        totalDistance += segmentBuffer[i].distance;
        lookAheadSegments++;
        if (totalDistance >= LOOK_AHEAD_DISTANCE)
            break;
    }

    // Get current machine velocity
    float currentVelocity = 0.0f;
    if (machineController)
    {
        // Use the scalar velocity method
        currentVelocity = machineController->getCurrentVelocity();
        
        // If needed, we could also access the full velocity vector
        // std::vector<float> currentVelocityVector = machineController->getCurrentVelocityVector();
    }

    // Forward pass (acceleration constraint)
    for (size_t i = 0; i < segmentBuffer.size(); i++)
    {
        Segment &segment = segmentBuffer[i];
        
        // Find limiting axis and its velocity
        float maxDesiredVelocity = 0.0f;
        for (int j = 0; j < numMotors; j++)
        {
            if (segment.desiredVelocities[j] > maxDesiredVelocity)
                maxDesiredVelocity = segment.desiredVelocities[j];
        }
        
        // Convert to mm/s using motor 0 as reference (assuming uniform step/mm)
        Motor *referenceMotor = motorManager->getMotor(0);
        float stepsPerUnit = (referenceMotor) ? abs(referenceMotor->unitsToSteps(1)) : 1.0f;
        float desiredVelocity = maxDesiredVelocity / stepsPerUnit;
        
        // Apply acceleration constraint
        float maxPossibleVelocity = sqrtf(currentVelocity * currentVelocity + 
                                         2.0f * maxAcceleration * segment.distance);
        
        // Limit velocity by acceleration constraint
        float forwardVelocity = std::min(desiredVelocity, maxPossibleVelocity);
        
        // Store for backward pass
        segment.forwardVelocity = forwardVelocity;
        
        // Update current velocity for next segment
        currentVelocity = forwardVelocity;
    }
    
    // Backward pass (deceleration constraint)
    // Start with zero final velocity or connect to existing adjusted segments
    currentVelocity = 0.0f;
    
    for (int i = segmentBuffer.size() - 1; i >= 0; i--)
    {
        Segment &segment = segmentBuffer[i];
        
        // Apply deceleration constraint
        float maxPossibleVelocity = sqrtf(currentVelocity * currentVelocity + 
                                         2.0f * maxAcceleration * segment.distance);
        
        // Final velocity is minimum of forward-calculated and backward-calculated
        segment.adjustedVelocity = std::min(segment.forwardVelocity, maxPossibleVelocity);
        
        // Update current velocity for previous segment
        currentVelocity = segment.adjustedVelocity;
        
        // Scale individual axis velocities proportionally
        float scaleFactor = (segment.forwardVelocity > 0.001f) ? 
                           (segment.adjustedVelocity / segment.forwardVelocity) : 0.0f;
                           
        for (int j = 0; j < numMotors; j++)
        {
            segment.adjustedVelocities[j] = segment.desiredVelocities[j] * scaleFactor;
        }
    }
    
    Debug::verbose("Scheduler", "Applied velocity adjustments to " + 
                  String(segmentBuffer.size()) + " segments");
}

bool Scheduler::generateSegments(ScheduledMove &move)
{
    int numMotors = motorManager->getNumMotors();

    // Calculate start position from last move or current position
    std::vector<float> currentPosition(numMotors, 0.0f);
    if (!segmentBuffer.empty())
    {
        // Use last segment's target position
        currentPosition = segmentBuffer.back().jointPositions;
    }
    else
    {
        // Use current machine position
        currentPosition = machineController->getCurrentWorkPosition();
    }

    // Calculate movement vector and total distance
    std::vector<float> moveVector(numMotors, 0.0f);
    float totalDistanceSquared = 0.0f;
    for (int i = 0; i < numMotors; i++)
    {
        float startPos = (i < currentPosition.size()) ? currentPosition[i] : 0.0f;
        float endPos = (i < move.targetPosition.size()) ? move.targetPosition[i] : 0.0f;
        moveVector[i] = endPos - startPos;
        totalDistanceSquared += moveVector[i] * moveVector[i];
    }
    float totalDistance = sqrt(totalDistanceSquared);
    
    // Determine number of segments
    int numSegments = std::max(1, (int)ceil(totalDistance / SEGMENT_MAX_LENGTH));
    
    // Check available buffer space
    int availableSpace = SEGMENT_BUFFER_SIZE - segmentBuffer.size();
    if (numSegments > availableSpace)
    {
        Debug::warning("Scheduler", "Segment buffer limited");
        numSegments = availableSpace;
        if (numSegments <= 0)
            return false;
    }

    // Base velocity (mm/s)
    float baseVelocity = move.feedrate / 60.0f;
    float segmentTime = (totalDistance > 0.000001f) ? (totalDistance / baseVelocity / numSegments) : 0.0f;

    // Generate segments
    for (int s = 0; s < numSegments; s++)
    {
        float t0 = (float)s / numSegments;
        float t1 = (float)(s + 1) / numSegments;

        Segment segment;
        segment.jointPositions.resize(numMotors);
        segment.desiredVelocities.resize(numMotors);
        segment.adjustedVelocities.resize(numMotors);
        segment.distance = totalDistance / numSegments;
        segment.forwardVelocity = 0.0f;
        segment.adjustedVelocity = 0.0f;

        // Interpolated positions
        for (int i = 0; i < numMotors; i++)
        {
            float start = currentPosition[i] + moveVector[i] * t0;
            float end = currentPosition[i] + moveVector[i] * t1;
            segment.jointPositions[i] = end;

            Motor *motor = motorManager->getMotor(i);
            if (motor && segmentTime > 0.000001f)
            {
                // Proportional velocity per axis
                float segmentDistance = fabs(end - start);
                float axisVelocity = (segmentTime > 0.0f) ? (segmentDistance / segmentTime) : 0.0f;

                // Convert to steps/s
                float stepsPerUnit = abs(motor->unitsToSteps(1));
                segment.desiredVelocities[i] = axisVelocity * stepsPerUnit;
                
                // Initialize adjusted velocity to desired velocity
                segment.adjustedVelocities[i] = segment.desiredVelocities[i];
            }
            else
            {
                segment.desiredVelocities[i] = 0.0f;
                segment.adjustedVelocities[i] = 0.0f;
            }
        }
        // Add to buffer
        segmentBuffer.push_back(segment);
    }

    return true;
}

bool Scheduler::executeSegment(const Segment &segment)
{
    // Don't execute new segment if motors are still moving
    if (motorManager->isAnyMotorMoving())
    {
        return false;
    }

    // Get number of motors
    int numMotors = motorManager->getNumMotors();
    if (numMotors != segment.jointPositions.size())
    {
        Debug::error("Scheduler", "Motor count mismatch");
        return false;
    }

    // Configure and start each motor
    for (int i = 0; i < numMotors; i++)
    {
        Motor *motor = motorManager->getMotor(i);
        if (!motor)
            continue;

        // Convert joint position to steps
        int32_t targetSteps = motor->unitsToSteps(segment.jointPositions[i]);

        // Calculate steps to move
        int32_t stepsToMove = targetSteps - currentSteps[i];

        // Skip motors that don't need to move
        if (stepsToMove == 0)
            continue;

        // Use adjusted velocity instead of original velocity
        int32_t speedInHz = segment.adjustedVelocities[i];

        // Start the move
        motor->moveTo(targetSteps, speedInHz);

        // Update current steps
        currentSteps[i] = targetSteps;
    }

    return true;
}

bool Scheduler::executeNextSegment()
{
    // Update execution status first
    updateExecutionStatus();

    // If we're currently executing a segment, don't start a new one
    if (executing)
    {
        return false;
    }

    // Process the queue to generate segments if needed
    if (segmentBuffer.empty())
    {
        processQueue();
    }

    // If we have segments, execute the next one
    if (!segmentBuffer.empty())
    {
        Segment &segment = segmentBuffer.front();
        bool result = executeSegment(segment);

        // If execution was successful, mark as executing and pop from buffer
        if (result)
        {
            executing = true;
            segmentBuffer.pop_front();
            return true;
        }
    }

    return false;
}

bool Scheduler::updateExecutionStatus()
{
    // If not executing, nothing to update
    if (!executing)
    {
        return false;
    }

    // Check if motors are still moving
    if (motorManager->isAnyMotorMoving())
    {
        return false; // Still executing
    }

    // All motors have stopped - segment complete
    executing = false;
    return true;
}

void Scheduler::clear()
{
    moveQueue.clear();
    segmentBuffer.clear();
    executing = false;
    Debug::info("Scheduler", "All queues cleared");
}

bool Scheduler::hasMove() const
{
    return !moveQueue.empty() || !segmentBuffer.empty();
}

bool Scheduler::isFull() const
{
    // Check if move queue is full
    if (moveQueue.size() >= MOVE_QUEUE_SIZE)
    {
        return true;
    }

    // Check if segment buffer is nearly full (leave some space for processing)
    if (segmentBuffer.size() >= SEGMENT_BUFFER_SIZE - 5)
    {
        return true;
    }

    return false;
}

void Scheduler::setCurrentPosition(const std::vector<float>& position)
{
    int numMotors = motorManager->getNumMotors();
    
    // Validate position size
    if (position.size() != numMotors)
    {
        Debug::error("Scheduler", "Position vector size mismatch");
        return;
    }
    
    // Update steps for each motor
    for (int i = 0; i < numMotors; i++)
    {
        Motor *motor = motorManager->getMotor(i);
        if (motor)
        {
            int32_t steps = motor->unitsToSteps(position[i]);
            currentSteps[i] = steps;
            motor->setPosition(steps);
        }
    }
    
    Debug::verbose("Scheduler", "Current position updated");
}
