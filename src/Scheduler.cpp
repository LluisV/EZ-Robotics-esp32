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
    move.segmentationProgress = 0.0f; // Initialize segmentation progress

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

void Scheduler::processQueue() {
    if (moveQueue.empty())
        return;

    if (generateSegmentsProgressive(moveQueue.front())) {
        applyVelocityAdjustments();
    }
}

void Scheduler::applyVelocityAdjustments()
{
    if (segmentBuffer.empty())
        return;

    const MachineConfig &config = configManager->getMachineConfig();
    float maxAcceleration_mm_min2 = config.maxAcceleration * 60.0f * 60.0f; // mm/min^2
    int numMotors = motorManager->getNumMotors();

    // Get current machine velocity
    float currentVelocity_mm_min = 0.0f;
    if (machineController)
    {
        currentVelocity_mm_min = machineController->getCurrentDesiredVelocity();
        Serial.println("Current velocity: " + String(currentVelocity_mm_min));
    }

    // Forward pass (acceleration constraint)
    for (size_t i = 0; i < segmentBuffer.size(); i++)
    {
        Segment &segment = segmentBuffer[i];

        // Find the maximum desired velocity in mm/min across all motors
        float maxDesiredVelocity_mm_min = 0.0f;
        int maxVelocityIndex = 0;
        for (int j = 0; j < numMotors; j++)
        {
            Motor *motor = motorManager->getMotor(j);
            if (!motor) continue;

            float stepsPerUnit = abs(motor->unitsToSteps(1)); // steps/mm or steps/deg
            if (stepsPerUnit == 0.0f) continue;

            float velocity_mm_min = (segment.desiredVelocities[j] / stepsPerUnit) * 60.0f;
            if (velocity_mm_min > maxDesiredVelocity_mm_min)
            {
                maxVelocityIndex = j;
                maxDesiredVelocity_mm_min = velocity_mm_min;
            }
        }

        // Apply acceleration constraint
        float maxPossibleVelocity_mm_min = sqrtf(currentVelocity_mm_min * currentVelocity_mm_min +
            2.0f * maxAcceleration_mm_min2 * segment.distance);

        float forwardVelocity = std::min(maxDesiredVelocity_mm_min, maxPossibleVelocity_mm_min);
        currentVelocity_mm_min = forwardVelocity;

        // Store this for backward pass
        segment.adjustedVelocities.resize(numMotors);
        for (int j = 0; j < numMotors; j++)
        {
            // Just store calculated forward velocities temporarily
            segment.adjustedVelocities[j] = segment.desiredVelocities[j] * 
                                          (forwardVelocity / maxDesiredVelocity_mm_min);
        }
    }

    // Backward pass (deceleration constraint)
    // Determine exit velocity based on whether this is the final move
    float exitVelocity = 0.0f; // Default to zero for the final move
    
    for (int i = segmentBuffer.size() - 1; i >= 0; i--)
    {
        Segment &segment = segmentBuffer[i];

        // Find the maximum forward-pass velocity across all motors
        float maxForwardVelocity_mm_min = 0.0f;
        int maxVelocityIndex = 0;
        for (int j = 0; j < numMotors; j++)
        {
            Motor *motor = motorManager->getMotor(j);
            if (!motor) continue;

            float stepsPerUnit = abs(motor->unitsToSteps(1)); // steps/mm or steps/deg
            if (stepsPerUnit == 0.0f) continue;

            float velocity_mm_min = (segment.adjustedVelocities[j] / stepsPerUnit) * 60.0f;
            if (velocity_mm_min > maxForwardVelocity_mm_min)
            {
                maxVelocityIndex = j;
                maxForwardVelocity_mm_min = velocity_mm_min;
            }
        }

        // Apply deceleration constraint (how fast we can go and still decelerate to exit velocity)
        float maxPossibleVelocity_mm_min = sqrtf(exitVelocity * exitVelocity +
                                                 2.0f * maxAcceleration_mm_min2 * segment.distance);

        // Take minimum of forward-calculated velocity and backward-calculated velocity
        float finalVelocity = std::min(maxForwardVelocity_mm_min, maxPossibleVelocity_mm_min);
        exitVelocity = finalVelocity; // For next segment

        // Calculate scaling factor for all joint velocities
        float scaleFactor = (maxForwardVelocity_mm_min > 0.0f) ?
                           (finalVelocity / maxForwardVelocity_mm_min) : 0.0f;

        // Apply the scaling factor to all joints
        for (int j = 0; j < numMotors; j++)
        {
            segment.adjustedVelocities[j] *= scaleFactor;
        }
    }

    Debug::verbose("Scheduler", "Applied velocity adjustments to " +
    String(segmentBuffer.size()) + " segments");
}

bool Scheduler::generateSegmentsProgressive(ScheduledMove &move)
{
    // Not empty enough to add new segments
    if(segmentBuffer.size() >= SEGMENT_BUFFER_SIZE * SEGMENT_BUFFER_THRESHOLD)
        return false;

    int numMotors = motorManager->getNumMotors();
    int availableSpace = SEGMENT_BUFFER_SIZE - segmentBuffer.size();
    
    if (availableSpace <= 0)
        return false;
    
    // Calculate start position for this batch of segments
    std::vector<float> startPosition(numMotors, 0.0f);
    
    if (move.segmentationProgress == 0.0f)
    {
        // First segment batch - use current machine position or last segment position
        if (segmentBuffer.empty())
        {
            startPosition = machineController->getCurrentWorkPosition();
        }
        else
        {
            startPosition = segmentBuffer.back().jointPositions;
        }
        
        // Store this as the move's start position for future batches
        move.startPosition = startPosition;
        
        // Calculate movement vector and total distance
        move.moveVector.resize(numMotors);
        float totalDistanceSquared = 0.0f;
        for (int i = 0; i < numMotors; i++)
        {
            move.moveVector[i] = move.targetPosition[i] - startPosition[i];
            totalDistanceSquared += move.moveVector[i] * move.moveVector[i];
        }
        move.totalDistance = sqrt(totalDistanceSquared);
        
        // Calculate total number of segments needed
        move.totalSegments = std::max(1, (int)ceil(move.totalDistance / SEGMENT_MAX_LENGTH));
    }
    else
    {
        // Continue from where we left off
        startPosition = move.startPosition;
    }
    
    // Base velocity (mm/s)
    float baseVelocity = move.feedrate / 60.0f;
    
    // Calculate the range of segments to generate in this batch
    int startSegment = (int)(move.segmentationProgress * move.totalSegments);
    int endSegment = std::min(startSegment + availableSpace, move.totalSegments);
    
    // Generate segments for this batch
    for (int s = startSegment; s < endSegment; s++)
    {
        float t0 = (float)s / move.totalSegments;
        float t1 = (float)(s + 1) / move.totalSegments;
        
        Segment segment;
        segment.jointPositions.resize(numMotors);
        segment.desiredVelocities.resize(numMotors);
        segment.adjustedVelocities.resize(numMotors);
        segment.distance = move.totalDistance / move.totalSegments;
        
        float segmentTime = (move.totalDistance > 0.000001f) ? 
                            (segment.distance / baseVelocity) : 0.0f;
        
        // Interpolated positions
        for (int i = 0; i < numMotors; i++)
        {
            float start = move.startPosition[i] + move.moveVector[i] * t0;
            float end = move.startPosition[i] + move.moveVector[i] * t1;
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
    
    // Update segmentation progress
    move.segmentationProgress = (float)endSegment / move.totalSegments;
    
    // If we've completed segmentation for this move, remove it from the queue
    if (move.segmentationProgress >= 1.0f)
    {
        moveQueue.pop_front();
        Debug::verbose("Scheduler", "Move fully segmented into " + 
                      String(move.totalSegments) + " segments");
    }
    else
    {
        Debug::verbose("Scheduler", "Move partially segmented: " + 
                      String((int)(move.segmentationProgress * 100)) + "% complete");
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

        // Conversión de adjustedVelocities (steps/s) a mm/min
        std::vector<float> velocities_mm_min(numMotors);

        for (int j = 0; j < numMotors; j++)
        {
            Motor *motor = motorManager->getMotor(j);
            if (!motor) continue;

            float stepsPerUnit = abs(motor->unitsToSteps(1)); // steps/mm
            if (stepsPerUnit == 0.0f) continue;

            // Convertir de steps/s a mm/s, luego de mm/s a mm/min
            float velocity_mm_min = (segment.adjustedVelocities[j] / stepsPerUnit) * 60.0f;
            velocities_mm_min[j] = velocity_mm_min;
        }

        // Ahora puedes enviar las velocidades ajustadas en mm/min
        machineController->setCurrentDesiredVelocityVector(velocities_mm_min);

        
        // Update current steps
        currentSteps[i] = targetSteps;
    }

    return true;
}

bool Scheduler::executeNextSegment()
{
    // Process the moves queue to generate segments if needed
    processQueue();
    
    // For smooth motion, we need to check if we're near the end of current moves
    // and start the next one before completely stopping
    bool motorsNearlyDone = false;
    int numMotors = motorManager->getNumMotors();
    
    if (motorManager->isAnyMotorMoving())
    {
        motorsNearlyDone = true;
        // Check if motors are close to finishing (less than 10% of steps remaining)
        for (int i = 0; i < numMotors; i++)
        {
            Motor *motor = motorManager->getMotor(i);
            if (motor)
            {
                if (motor->isMoving())
                {
                    long stepsLeft = abs(motor->getTargetPosition() - motor->getPosition());
                    long totalSteps = abs(motor->getTargetPosition() - currentSteps[i]);
                    if (totalSteps > 0 && stepsLeft > 4)
                    {
                        motorsNearlyDone = false;
                        break;
                    }
                }
            }
        }
    }

    // Execute next segment if motors are idle or nearly done
    if (!motorManager->isAnyMotorMoving() || motorsNearlyDone)
    {
        if (!segmentBuffer.empty())
        {
            Segment &segment = segmentBuffer.front();
            bool result = executeSegment(segment);

            // If execution was successful, pop from buffer
            if (result)
                segmentBuffer.pop_front();

            return result;
        }
        else if(!motorManager->isAnyMotorMoving())
        {
            machineController->setCurrentDesiredVelocityVector({0.0f,0.0f,0.0f});
        }
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
    return !moveQueue.empty() || !segmentBuffer.empty();
}

bool Scheduler::isFull() const
{
    // Check if move queue is full
    if (moveQueue.size() >= MOVE_QUEUE_SIZE)
    {
        return true;
    }

    if (segmentBuffer.size() >= SEGMENT_BUFFER_SIZE)
    {
        return true;
    }

    return false;
}

