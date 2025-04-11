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
        int segmentsGenerated = generateSegments(move);
        if (segmentsGenerated > 0)
        {
            moveQueue.pop_front();
        }

        // Only process one move at a time to avoid filling buffer
        break;
    }
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

bool Scheduler::generateSegments(ScheduledMove &move)
{
    int numMotors = motorManager->getNumMotors();

    // Calcular posición inicial según el último movimiento o la posición actual
    std::vector<float> currentPosition(numMotors, 0.0f);
    if (moveQueue.size() <= 1)
        currentPosition = machineController->getCurrentWorkPosition();
    else
        currentPosition = moveQueue.back().targetPosition;

    // Calcular vector de movimiento y distancia total
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
    Serial.println("TOTAL DISTANCE: " + String(totalDistance));

    // Determinar número de segmentos
    int numSegments = std::max(1, (int)ceil(totalDistance / SEGMENT_MAX_LENGTH));
    Serial.println("NUM SEGMENTS: " + String(numSegments));

    // Verificar espacio en el buffer
    int availableSpace = SEGMENT_BUFFER_SIZE - segmentBuffer.size();
    if (numSegments > availableSpace)
    {
        Debug::warning("Scheduler", "Segment buffer full");
        return false;
    }
    numSegments = std::min(numSegments, availableSpace);

    // Velocidad base (mm/s)
    float baseVelocity = move.feedrate / 60.0f;
    float moveTime = (totalDistance > 0.000001f) ? (totalDistance / baseVelocity) : 0.0f;
    int segmentsGenerated = 0;

    // Generar segmentos
    for (int s = 0; s < numSegments; s++)
    {
        float t0 = (float)s / numSegments;
        float t1 = (float)(s + 1) / numSegments;

        Segment segment;
        segment.jointPositions.resize(numMotors);
        segment.velocities.resize(numMotors);

        // Posiciones interpoladas
        for (int i = 0; i < numMotors; i++)
        {
            float start = currentPosition[i] + moveVector[i] * t0;
            float end = currentPosition[i] + moveVector[i] * t1;
            segment.jointPositions[i] = end; 

            Motor *motor = motorManager->getMotor(i);
            if (motor && moveTime > 0.000001f)
            {
                // Velocidad proporcional por eje
                float segmentDistance = fabs(end - start);
                float segmentTime = (segmentDistance > 0.000001f) ? (segmentDistance / baseVelocity) : 0.0f;

                // Convertir a pasos/s
                float stepsPerUnit = abs(motor->unitsToSteps(1));
                segment.velocities[i] = (segmentTime > 0.0f) ? (segmentDistance / segmentTime) * stepsPerUnit : 0.0f;
            }
            else
            {
                segment.velocities[i] = 0.0f;
            }
        }
        // Añadir al buffer
        segmentBuffer.push_back(segment);
        segmentsGenerated++;
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

        // Get speed in Hz
        int32_t speedInHz = segment.velocities[i];

        // Start the move
        motor->moveTo(targetSteps, speedInHz);

        // Update current steps
        currentSteps[i] = targetSteps;
    }

    return true;
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