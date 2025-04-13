#include "Scheduler.h"
#include <cmath>
#include <algorithm>
#include "MachineController.h"

Scheduler::Scheduler(MachineController *machineController, MotorManager *motorManager, ConfigManager *configManager)
    : machineController(machineController),
      motorManager(motorManager),
      configManager(configManager)
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

    auto &move = moveQueue.front();

    if (generateSegments(move))
    {
        moveQueue.pop_front();
        Debug::warning("Scheduler", "Move segmented. Total segments: " + String(segmentBuffer.size()));
    }
}

void Scheduler::applyVelocityAdjustments()
{
    if (segmentBuffer.empty())
        return;

    const MachineConfig &config = configManager->getMachineConfig();
    float maxAcceleration = config.maxAcceleration; // mm/s^2
    int numMotors = motorManager->getNumMotors();

    // Get current machine velocity
    float currentVelocity = 0.0f;
    if (machineController)
    {
        currentVelocity = machineController->getCurrentVelocity();
    }

    // Forward pass (acceleration constraint)
    for (size_t i = 0; i < segmentBuffer.size(); i++)
    {
        Segment &segment = segmentBuffer[i];

        // Find the maximum desired velocity in mm/s across all motors
        float maxDesiredVelocity_mm_s = 0.0f;
        int maxVelocityIndex = 0;
        for (int j = 0; j < numMotors; j++)
        {
            Motor *motor = motorManager->getMotor(j);
            if (!motor) continue;

            float stepsPerUnit = abs(motor->unitsToSteps(1)); // steps/mm or steps/deg
            if (stepsPerUnit == 0.0f) continue;

            float velocity_mm_s = segment.desiredVelocities[j] / stepsPerUnit;
            if (velocity_mm_s > maxDesiredVelocity_mm_s)
            {
                maxVelocityIndex = j;
                maxDesiredVelocity_mm_s = velocity_mm_s;
            }
        }

        // Apply acceleration constraint
        float maxPossibleVelocity = sqrtf(currentVelocity * currentVelocity +
                                          2.0f * maxAcceleration * segment.distance);

        float forwardVelocity = std::min(maxDesiredVelocity_mm_s, maxPossibleVelocity);
        currentVelocity = forwardVelocity;

        // Store this for backward pass
        segment.adjustedVelocities.resize(numMotors);
        for (int j = 0; j < numMotors; j++)
        {
            // Just store calculated forward velocities temporarily
            segment.adjustedVelocities[j] = segment.desiredVelocities[j] * 
                                          (forwardVelocity / maxDesiredVelocity_mm_s);
        }
    }

    // Backward pass (deceleration constraint)
    float exitVelocity = 0.0f; // Final velocity should be zero
    for (int i = segmentBuffer.size() - 1; i >= 0; i--)
    {
        Segment &segment = segmentBuffer[i];

        // Find the maximum forward-pass velocity across all motors
        float maxForwardVelocity_mm_s = 0.0f;
        int maxVelocityIndex = 0;
        for (int j = 0; j < numMotors; j++)
        {
            Motor *motor = motorManager->getMotor(j);
            if (!motor) continue;

            float stepsPerUnit = abs(motor->unitsToSteps(1)); // steps/mm or steps/deg
            if (stepsPerUnit == 0.0f) continue;

            float velocity_mm_s = segment.adjustedVelocities[j] / stepsPerUnit;
            if (velocity_mm_s > maxForwardVelocity_mm_s)
            {
                maxVelocityIndex = j;
                maxForwardVelocity_mm_s = velocity_mm_s;
            }
        }

        // Apply deceleration constraint (how fast we can go and still decelerate to exit velocity)
        float maxPossibleVelocity = sqrtf(exitVelocity * exitVelocity +
                                         2.0f * maxAcceleration * segment.distance);

        // Take minimum of forward-calculated velocity and backward-calculated velocity
        float finalVelocity = std::min(maxForwardVelocity_mm_s, maxPossibleVelocity);
        exitVelocity = finalVelocity; // For next segment

        // Calculate scaling factor for all joint velocities
        float scaleFactor = (maxForwardVelocity_mm_s > 0.0f) ?
                           (finalVelocity / maxForwardVelocity_mm_s) : 0.0f;

        // Apply the scaling factor to all joints
        for (int j = 0; j < numMotors; j++)
        {
            segment.adjustedVelocities[j] *= scaleFactor;
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
    
    // Calculate initial number of segments using default max length
    int numSegments = std::max(1, (int)ceil(totalDistance / SEGMENT_MAX_LENGTH));
    
    // Check available buffer space
    int availableSpace = SEGMENT_BUFFER_SIZE - segmentBuffer.size();
    
    // If segments won't fit but buffer is empty, adapt segment size
    if (numSegments > availableSpace && segmentBuffer.empty())
    {
        // Recalculate segment size to fit within available buffer
        float adaptedSegmentLength = totalDistance / availableSpace;
        numSegments = availableSpace;
        
        Debug::warning("Scheduler", "Adapting segment size from " + 
                      String(SEGMENT_MAX_LENGTH, 2) + " to " + 
                      String(adaptedSegmentLength, 2) + " mm for long move");
    }
    else if (numSegments > availableSpace)
    {
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

    applyVelocityAdjustments(); 

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
    // If we're currently executing a segment, don't start a new one
    if (motorManager->isAnyMotorMoving())
    {
        return false;
    }

    // Process the moves queue to generate segments if needed
    processQueue();
    

    // If we have segments, execute the next one
    if (!segmentBuffer.empty())
    {
        Segment &segment = segmentBuffer.front();
        bool result = executeSegment(segment);

        // If execution was successful, mark as executing and pop from buffer
        if (result)
            segmentBuffer.pop_front();

        return result;
    }

    return false;
}

void Scheduler::clear()
{
    moveQueue.clear();
    segmentBuffer.clear();
    Debug::info("Scheduler", "All queues cleared");
}

bool Scheduler::hasMove() const
{
    return !moveQueue.empty();
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
