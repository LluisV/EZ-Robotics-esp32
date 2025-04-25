#include "Scheduler.h"
#include <cmath>
#include <algorithm>
#include "MachineController.h"

Scheduler::Scheduler(MachineController *machineController, MotorManager *motorManager, ConfigManager *configManager)
    : machineController(machineController),
      motorManager(motorManager),
      configManager(configManager)
{
    // Pre-allocate memory for position vectors
    if (motorManager) {
        int numMotors = motorManager->getNumMotors();
        currentSteps.reserve(numMotors);
        currentSteps.resize(numMotors, 0);
    }
}

bool Scheduler::initialize()
{
    if (!motorManager || !configManager) {
        Debug::error("Scheduler", "Invalid motor or config manager");
        return false;
    }

    // Initialize current position and steps
    int numMotors = motorManager->getNumMotors();
    currentSteps.resize(numMotors, 0);

    // Get current position from motors
    for (int i = 0; i < numMotors; i++) {
        Motor *motor = motorManager->getMotor(i);
        if (motor) {
            currentSteps[i] = motor->getPosition();
        }
    }

    return true;
}

bool Scheduler::addLinearMove(const std::vector<float> &targetPos, float feedrate)
{
    // Quick size check
    if (targetPos.size() != motorManager->getNumMotors()) {
        Debug::error("Scheduler", "Target position size mismatch");
        return false;
    }

    // Queue full check
    if (moveQueue.size() >= MOVE_QUEUE_SIZE) {
        return false;
    }

    // Check if this is a zero-distance move - use squared distance for efficiency
    const std::vector<float>& currentPos = machineController->getCurrentWorkPosition();
    float distSquared = 0.0f;
    for (size_t i = 0; i < targetPos.size(); i++) {
        float diff = targetPos[i] - currentPos[i];
        distSquared += diff * diff;
        if (distSquared > 0.000001f) { // Break early if we detect movement
            break;
        }
    }

    // Skip zero moves
    if (distSquared <= 0.000001f) {
        return true;
    }

    // Create and add new move with minimal initialization
    ScheduledMove move;
    move.targetPosition = targetPos;
    move.type = LINEAR_MOVE;
    move.feedrate = feedrate;
    move.segmentationProgress = 0.0f;
    moveQueue.push_back(std::move(move)); // Use move semantics

    return true;
}

bool Scheduler::addRapidMove(const std::vector<float> &targetPos)
{
    // Use linear move with rapid feedrate - inline for efficiency
    const MachineConfig &config = configManager->getMachineConfig();
    return addLinearMove(targetPos, config.maxFeedrate);
}

void Scheduler::processQueue() {
    if (moveQueue.empty() || segmentBuffer.size() >= SEGMENT_BUFFER_SIZE * SEGMENT_BUFFER_THRESHOLD) {
        return;
    }

    // Process the front move without function call overhead
    generateSegmentsProgressive(moveQueue.front());
}

bool Scheduler::generateSegmentsProgressive(ScheduledMove &move)
{
    // Quick bail if buffer is too full
    if(segmentBuffer.size() >= SEGMENT_BUFFER_SIZE * SEGMENT_BUFFER_THRESHOLD)
        return false;

    const int numMotors = motorManager->getNumMotors();
    const int availableSpace = SEGMENT_BUFFER_SIZE - segmentBuffer.size();
    
    if (availableSpace <= 0)
        return false;
    
    // Calculate start position for this batch of segments
    static std::vector<float> startPosition; // Static to avoid reallocations
    startPosition.resize(numMotors);
    
    if (move.segmentationProgress == 0.0f) {
        // First segment batch - use current machine position or last segment position
        if (segmentBuffer.empty()) {
            const std::vector<float>& currentPos = machineController->getCurrentWorldPosition();
            std::copy(currentPos.begin(), currentPos.end(), startPosition.begin());
        } else {
            const std::vector<float>& lastPos = segmentBuffer.back().jointPositions;
            std::copy(lastPos.begin(), lastPos.end(), startPosition.begin());
        }
        
        // Store this as the move's start position for future batches
        move.startPosition = startPosition;
        
        // Calculate movement vector and total distance - more efficiently
        move.moveVector.resize(numMotors);
        float totalDistanceSquared = 0.0f;
        for (int i = 0; i < numMotors; i++) {
            move.moveVector[i] = move.targetPosition[i] - startPosition[i];
            totalDistanceSquared += move.moveVector[i] * move.moveVector[i];
        }
        move.totalDistance = std::sqrt(totalDistanceSquared);
        
        // Calculate total number of segments needed
        move.totalSegments = std::max(1, static_cast<int>(std::ceil(move.totalDistance / SEGMENT_MAX_LENGTH)));
    } else {
        // Continue from where we left off
        std::copy(move.startPosition.begin(), move.startPosition.end(), startPosition.begin());
    }
    
    // Base velocity (mm/s)
    const float baseVelocity = move.feedrate / 60.0f;
    
    // Calculate the range of segments to generate in this batch
    const int startSegment = static_cast<int>(move.segmentationProgress * move.totalSegments);
    const int endSegment = std::min(startSegment + availableSpace, move.totalSegments);
    
    // Pre-calculate inverse total segments for efficiency
    const float invTotalSegments = 1.0f / move.totalSegments;
    const float segmentDistance = move.totalDistance / move.totalSegments;
    const float segmentTime = (move.totalDistance > 0.000001f) ? (segmentDistance / baseVelocity) : 0.0f;
    
    // No need to reserve for std::deque since it doesn't have reserve/capacity methods
    // Pre-checking space is sufficient with the availableSpace calculation above
    
    // Generate segments for this batch - use a single segment template to avoid reallocations
    Segment templateSegment;
    templateSegment.jointPositions.resize(numMotors);
    templateSegment.desiredVelocities.resize(numMotors);
    templateSegment.adjustedVelocities.resize(numMotors);
    templateSegment.distance = segmentDistance;
    
    // Cache motor pointers and steps-per-unit values
    std::vector<Motor*> motors(numMotors);
    std::vector<float> stepsPerUnit(numMotors);
    for (int i = 0; i < numMotors; i++) {
        motors[i] = motorManager->getMotor(i);
        if (motors[i]) {
            stepsPerUnit[i] = std::abs(motors[i]->unitsToSteps(1));
        } else {
            stepsPerUnit[i] = 0.0f;
        }
    }
        
    for (int s = startSegment; s < endSegment; s++) {
        const float t0 = s * invTotalSegments;
        const float t1 = (s + 1) * invTotalSegments;
        
        // Deep copy the template segment
        Segment segment = templateSegment;
        
        // Calculate interpolated positions and velocities
        for (int i = 0; i < numMotors; i++) {
            const float start = move.startPosition[i] + move.moveVector[i] * t0;
            const float end = move.startPosition[i] + move.moveVector[i] * t1;
            segment.jointPositions[i] = end;
            
            if (motors[i] && segmentTime > 0.000001f && stepsPerUnit[i] > 0.0f) {
                // Proportional velocity per axis - calculate once
                const float segmentDistance = std::fabs(end - start);
                const float axisVelocity = segmentDistance / segmentTime;
                
                // Convert to steps/s
                const float velocity = axisVelocity * stepsPerUnit[i];
                segment.desiredVelocities[i] = velocity;
                segment.adjustedVelocities[i] = velocity;
            } else {
                segment.desiredVelocities[i] = 0.0f;
                segment.adjustedVelocities[i] = 0.0f;
            }
        }
        
        // Add to buffer
        segmentBuffer.push_back(std::move(segment));
    }
    
    // Update segmentation progress
    move.segmentationProgress = static_cast<float>(endSegment) * invTotalSegments;
    
    // If we've completed segmentation for this move, remove it from the queue
    if (move.segmentationProgress >= 1.0f) {
        moveQueue.pop_front();
    }
    
    return true;
}

bool Scheduler::executeSegment(const Segment &segment)
{
    // Get number of motors - early return for mismatched sizes
    const int numMotors = motorManager->getNumMotors();
    if (numMotors != segment.jointPositions.size()) {
        Debug::error("Scheduler", "Motor count mismatch");
        return false;
    }

    // Prepare a velocity vector for sending to machineController - only once
    std::vector<float> velocities_mm_min(numMotors, 0.0f);
    bool anyMotorMoving = false;

    // Configure and start each motor
    for (int i = 0; i < numMotors; i++) {
        Motor *motor = motorManager->getMotor(i);
        if (!motor)
            continue;

        // Convert joint position to steps
        const int32_t targetSteps = motor->unitsToSteps(segment.jointPositions[i]);

        // Calculate steps to move
        const int32_t stepsToMove = targetSteps - currentSteps[i];

        // Skip motors that don't need to move
        if (stepsToMove == 0)
            continue;

        // Use adjusted velocity instead of original velocity
        const int32_t speedInHz = segment.adjustedVelocities[i];

        // Start the move
        motor->moveTo(targetSteps, speedInHz);
        anyMotorMoving = true;

        // Convert adjustedVelocities (steps/s) to mm/min
        const float stepsPerUnit = std::abs(motor->unitsToSteps(1)); // steps/mm
        if (stepsPerUnit > 0.0f) {
            // Convert from steps/s to mm/s, then from mm/s to mm/min
            velocities_mm_min[i] = (segment.adjustedVelocities[i] / stepsPerUnit) * 60.0f;
        }
        
        // Update current steps
        currentSteps[i] = targetSteps;
    }

    // Only update velocity vector if any motor is moving
    if (anyMotorMoving) {
        machineController->setCurrentDesiredVelocityVector(velocities_mm_min);
    }

    return true;
}

bool Scheduler::executeNextSegment()
{
    // Process the moves queue to generate segments if needed
    processQueue();
    
    // Execute next segment if motors are idle
    if (!motorManager->isAnyMotorMoving()) {
        if (!segmentBuffer.empty()) {
            const Segment &segment = segmentBuffer.front();
            bool result = executeSegment(segment);

            // If execution was successful, pop from buffer
            if (result) {
                segmentBuffer.pop_front();
            }
            return result;
        } else {
            // Set zero velocity when no segments and no motor movement
            static const std::vector<float> zeroVelocity = {0.0f, 0.0f, 0.0f}; // Static to avoid reallocation
            machineController->setCurrentDesiredVelocityVector(zeroVelocity);
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
    // Quick checks for full queues
    return (moveQueue.size() >= MOVE_QUEUE_SIZE) || (segmentBuffer.size() >= SEGMENT_BUFFER_SIZE);
}