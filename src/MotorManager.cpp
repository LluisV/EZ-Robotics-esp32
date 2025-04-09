/**
 * @file MotorManager.cpp
 * @brief Implementation of the MotorManager class
 */

#include "MotorManager.h"
#include "Debug.h"

MotorManager::MotorManager() : configManager(nullptr)
{
}

MotorManager::~MotorManager()
{
  // Clean up motors
  for (Motor *motor : motors)
  {
    delete motor;
  }
  motors.clear();
}

bool MotorManager::initialize(ConfigManager *configManager)
{
  Debug::info("MotorManager", "Starting motor initialization");

  this->configManager = configManager;
  if (!configManager)
  {
    Debug::error("MotorManager", "Invalid configuration manager");
    return false;
  }

  // Clean up existing motors
  Debug::verbose("MotorManager", "Cleaning up existing motors");
  for (Motor *motor : motors)
  {
    Debug::verbose("MotorManager", "Deleting motor: " + (motor ? motor->getName() : "Unknown"));
    delete motor;
  }
  motors.clear();

  // Initialize the engine
  Debug::info("MotorManager", "Initializing stepper engine");
  engine.init();

  // Create motors based on configuration
  int numMotors = configManager->getNumMotors();
  Debug::verbose("MotorManager", "Total motors in configuration: " + String(numMotors));

  int successfulInitCount = 0;
  int failedInitCount = 0;

  for (int i = 0; i < numMotors; i++)
  {
    const MotorConfig *motorConfig = configManager->getMotorConfig(i);
    if (motorConfig)
    {
      Debug::verbose("MotorManager", "Attempting to initialize motor: " + motorConfig->name);

      Motor *motor = new Motor(motorConfig);
      if (motor->initialize(&engine))
      {
        motors.push_back(motor);
        successfulInitCount++;
        Debug::info("MotorManager", "Successfully initialized motor: " + motorConfig->name);
      }
      else
      {
        Debug::error("MotorManager", "Failed to initialize motor: " + motorConfig->name);
        delete motor;
        failedInitCount++;
      }
    }
    else
    {
      Debug::warning("MotorManager", "Null motor configuration at index " + String(i));
      failedInitCount++;
    }
  }

  Debug::info("MotorManager", "Motor initialization summary: " +
                                  String(successfulInitCount) + " successful, " +
                                  String(failedInitCount) + " failed");

  return motors.size() > 0;
}

Motor *MotorManager::getMotor(int index)
{
  if (index >= 0 && index < motors.size())
  {
    return motors[index];
  }

  Debug::warning("MotorManager", "Invalid motor index requested: " + String(index) +
                                     ". Total motors: " + String(motors.size()));
  return nullptr;
}

Motor *MotorManager::getMotorByName(const String &name)
{
  for (Motor *motor : motors)
  {
    if (motor->getName() == name)
    {
      return motor;
    }
  }

  Debug::warning("MotorManager", "Motor not found with name: " + name);
  return nullptr;
}

int MotorManager::getNumMotors() const
{
  return motors.size();
}

bool MotorManager::update()
{
  for (Motor *motor : motors)
  {
    motor->update();
  }
  return true;
}

bool MotorManager::isAnyMotorMoving() const
{
  for (const Motor *motor : motors)
  {
    if (motor->isMoving())
    {
      return true;
    }
  }

  return false;
}

bool MotorManager::moveToFeedrate(const std::vector<float> &positions, float feedrate, float nominalAcceleration)
{
    // Para movimientos feedrate (G1)
    Debug::verbose("MotorManager", "Starting synchronized feedrate move with feedrate: " +
                                       String(feedrate) + " mm/min, nominal acceleration: " +
                                       String(nominalAcceleration));

    if (positions.size() != motors.size())
    {
        Debug::error("MotorManager", "Position array size mismatch in feedrate move");
        return false;
    }

    // Obtener posiciones actuales y calcular distancias para cada eje
    std::vector<float> currentPositions(motors.size());
    std::vector<float> distances(motors.size());
    float totalDistanceSquared = 0.0f;

    for (size_t i = 0; i < motors.size(); i++)
    {
        currentPositions[i] = motors[i]->getPositionInUnits();
        distances[i] = positions[i] - currentPositions[i];
        totalDistanceSquared += distances[i] * distances[i];
        Debug::verbose("MotorManager", motors[i]->getName() +
                                       " move distance: " + String(distances[i]) + " units");
    }
    float totalDistance = sqrt(totalDistanceSquared);

    // Convertir feedrate (mm/min) a unidades/segundo y calcular la duración ideal
    float feedrateInUnitsPerSec = feedrate / 60.0f;
    float moveDuration = (totalDistance > 0.0001f) ? (totalDistance / feedrateInUnitsPerSec) : 0.0f;

    Debug::verbose("MotorManager", "Feedrate move: total distance=" + String(totalDistance) +
                                     " units, feedrate=" + String(feedrateInUnitsPerSec) +
                                     " units/sec, ideal duration=" + String(moveDuration) + " sec");

    // Calcular la velocidad requerida para cada motor para ejecutar el movimiento de forma sincronizada
    std::vector<float> requiredSpeeds(motors.size());
    bool needToScaleFeedrate = false;
    float maxSpeedRatio = 0.0f;

    for (size_t i = 0; i < motors.size(); i++)
    {
        if (moveDuration > 0.0001f)
        {
            requiredSpeeds[i] = fabs(distances[i]) / moveDuration; // unidades/segundo
        }
        else
        {
            requiredSpeeds[i] = 0.0f;
        }

        // Verificar que la velocidad no exceda la capacidad del motor
        const MotorConfig *config = motors[i]->getConfig();
        int speedInSteps = motors[i]->unitsToSteps(requiredSpeeds[i]);
        if (config->maxSpeed > 0 && speedInSteps > 0)
        {
            float speedRatio = (float)speedInSteps / (float)config->maxSpeed;
            if (speedRatio > maxSpeedRatio)
            {
                maxSpeedRatio = speedRatio;
                if (speedRatio > 1.0f)
                {
                    needToScaleFeedrate = true;
                }
            }
        }

        Debug::verbose("MotorManager", motors[i]->getName() + " requires " +
                                        String(requiredSpeeds[i]) + " units/sec (" +
                                        String(speedInSteps) + " steps/sec), max: " +
                                        String(config->maxSpeed) + " steps/sec");
    }

    // Si algún motor requiere velocidad mayor a la que puede alcanzar, se escala la velocidad
    float speedScaleFactor = 1.0f;
    if (needToScaleFeedrate && maxSpeedRatio > 1.0f)
    {
        speedScaleFactor = 1.0f / maxSpeedRatio;
        moveDuration *= maxSpeedRatio; // Extiende la duración proporcionalmente
        Debug::warning("MotorManager", "Feedrate exceeds machine capabilities by factor of " +
                                       String(maxSpeedRatio) + ". Scaling down speeds by " +
                                       String(speedScaleFactor));
    }

    // Calcular tiempos de aceleración para sincronizar todos los ejes
    bool success = true;
    float maxAccelTime = 0;
    std::vector<float> accelTimes(motors.size());
    for (size_t i = 0; i < motors.size(); i++)
    {
        if (fabs(distances[i]) < 0.0001f)
        {
            accelTimes[i] = 0;
            continue;
        }

        const MotorConfig *config = motors[i]->getConfig();
        // La aceleración efectiva es la mínima entre la nominal y la configurada para el motor
        float effectiveAcceleration = std::min(nominalAcceleration, (float)config->acceleration);
        float accelInUnits = motors[i]->stepsToUnits(effectiveAcceleration);
        accelTimes[i] = (requiredSpeeds[i] * speedScaleFactor) / accelInUnits;

        if (accelTimes[i] > maxAccelTime)
        {
            maxAccelTime = accelTimes[i];
        }
    }

    // Ajustar aceleración y velocidad para cada motor
    for (size_t i = 0; i < motors.size(); i++)
    {
        if (fabs(distances[i]) < 0.0001f)
        {
            continue; // Saltar motores que no se mueven
        }

        const MotorConfig *config = motors[i]->getConfig();
        float adjustedSpeed = requiredSpeeds[i] * speedScaleFactor;
        int speedInSteps = motors[i]->unitsToSteps(adjustedSpeed);

        // Ajustar la aceleración para que todos alcancen la velocidad al mismo tiempo
        float accelRatio = 1.0f;
        if (accelTimes[i] > 0.0001f)
        {
            accelRatio = accelTimes[i] / maxAccelTime;
        }
        int adjustedAccel = config->acceleration * accelRatio;
        // Garantizar un mínimo de aceleración
        adjustedAccel = max(adjustedAccel, 100);

        // Configurar parámetros en el motor
        motors[i]->setAcceleration(adjustedAccel);
        motors[i]->setSpeed(speedInSteps);

        Debug::verbose("MotorManager", motors[i]->getName() + " config: " +
                                           "speed=" + String(adjustedSpeed) + " units/sec, " +
                                           "accel=" + String(adjustedAccel) + " steps/sec² " +
                                           "(ratio: " + String(accelRatio) + ")");
        // Inicia el movimiento
        success &= motors[i]->moveToUnits(positions[i], adjustedSpeed);
    }

    return success;
}

bool MotorManager::moveToRapid(const std::vector<float> &positions, float nominalAcceleration)
{
    // Para movimientos rapid (G0) se busca la sincronización a velocidades máximas
    Debug::verbose("MotorManager", "Starting rapid synchronized move with nominal acceleration: " +
                                       String(nominalAcceleration));

    if (positions.size() != motors.size())
    {
        Debug::error("MotorManager", "Position array size mismatch in rapid move");
        return false;
    }

    // Calcular las distancias a recorrer y el tiempo que tomaría cada motor a velocidad máxima
    std::vector<float> currentPositions(motors.size());
    std::vector<float> distances(motors.size());
    std::vector<float> moveTimes(motors.size());
    float maxMoveTime = 0.0f;

    for (size_t i = 0; i < motors.size(); i++)
    {
        currentPositions[i] = motors[i]->getPositionInUnits();
        distances[i] = positions[i] - currentPositions[i];

        if (fabs(distances[i]) < 0.0001f)
        {
            moveTimes[i] = 0.0f;
            continue;
        }

        const MotorConfig *config = motors[i]->getConfig();
        float maxSpeedInUnits = motors[i]->stepsToUnits(config->maxSpeed);
        moveTimes[i] = fabs(distances[i]) / maxSpeedInUnits;

        if (moveTimes[i] > maxMoveTime)
        {
            maxMoveTime = moveTimes[i];
        }

        Debug::verbose("MotorManager", motors[i]->getName() + " rapid move: " +
                                           "distance=" + String(distances[i]) + " units, " +
                                           "max speed=" + String(maxSpeedInUnits) + " units/sec, " +
                                           "time=" + String(moveTimes[i]) + " sec");
    }

    // Ajustar las velocidades para que todos los motores terminen simultáneamente
    bool success = true;
    float maxAccelTime = 0;
    std::vector<float> maxSpeedTimes(motors.size());
    std::vector<float> adjustedSpeeds(motors.size());

    for (size_t i = 0; i < motors.size(); i++)
    {
        if (fabs(distances[i]) < 0.0001f)
        {
            continue;
        }

        // Calcular la velocidad ajustada para igualar el movimiento del motor más lento
        adjustedSpeeds[i] = fabs(distances[i]) / maxMoveTime;
        const MotorConfig *config = motors[i]->getConfig();
        // La aceleración efectiva se define como la mínima entre la nominal y la aceleración del motor
        float effectiveAcceleration = std::min(nominalAcceleration, (float)config->acceleration);
        float accelInUnits = motors[i]->stepsToUnits(effectiveAcceleration);
        maxSpeedTimes[i] = adjustedSpeeds[i] / accelInUnits;

        if (maxSpeedTimes[i] > maxAccelTime)
        {
            maxAccelTime = maxSpeedTimes[i];
        }
    }

    // Ajustar la aceleración y velocidad de cada motor para que todos alcancen la velocidad deseada en sincronía
    for (size_t i = 0; i < motors.size(); i++)
    {
        if (fabs(distances[i]) < 0.0001f)
        {
            continue;
        }

        const MotorConfig *config = motors[i]->getConfig();
        int speedInSteps = motors[i]->unitsToSteps(adjustedSpeeds[i]);
        float accelRatio = 1.0f;
        if (maxSpeedTimes[i] > 0.0001f)
        {
            accelRatio = maxSpeedTimes[i] / maxAccelTime;
        }
        int adjustedAccel = config->acceleration * accelRatio;
        adjustedAccel = max(adjustedAccel, 100);

        motors[i]->setAcceleration(adjustedAccel);
        motors[i]->setSpeed(speedInSteps);

        Debug::verbose("MotorManager", motors[i]->getName() + " rapid config: " +
                                           "adjusted speed=" + String(adjustedSpeeds[i]) + " units/sec, " +
                                           "accel=" + String(adjustedAccel) + " steps/sec² " +
                                           "(ratio: " + String(accelRatio) + ")");

        success &= motors[i]->moveToUnits(positions[i], adjustedSpeeds[i]);
    }

    return success;
}


void MotorManager::stopAll(bool immediate)
{
  Debug::info("MotorManager", "Stopping all motors" +
                                  String(immediate ? " (IMMEDIATE)" : " (DECELERATE)"));

  for (Motor *motor : motors)
  {
    Debug::verbose("MotorManager", "Stopping motor " + motor->getName() +
                                       String(immediate ? " immediately" : " with deceleration"));
    motor->stop(immediate);
  }
}

bool MotorManager::homeAll()
{
  Debug::info("MotorManager", "Starting home all sequence");
  bool success = true;
  unsigned long startTime = millis();

  // Home sequence prioritized order
  std::vector<String> homeOrder = {"Z", "X", "Y"};

  for (const String &axisName : homeOrder)
  {
    Motor *motor = getMotorByName(axisName);
    if (motor)
    {
      Debug::verbose("MotorManager", "Homing " + axisName + " axis");
      bool axisHomeSuccess = motor->startHoming();

      if (!axisHomeSuccess)
      {
        Debug::error("MotorManager", "Failed to start homing for " + axisName + " axis");
        success = false;
        continue;
      }

      // Wait for homing to complete
      unsigned long axisStartTime = millis();
      while (motor->getStatus() == HOMING)
      {
        update();
        delay(10);

        // Timeout check (30 seconds)
        if (millis() - axisStartTime > 30000)
        {
          Debug::error("MotorManager", axisName + " axis homing timed out");
          success = false;
          break;
        }
      }

      Debug::info("MotorManager", axisName + " axis homing " +
                                      String(motor->getStatus() != HOMING ? "COMPLETED" : "FAILED"));
    }
    else
    {
      Debug::warning("MotorManager", axisName + " axis motor not found");
    }
  }

  // Home any remaining motors not in the main axes
  Debug::verbose("MotorManager", "Checking for additional motors to home");
  for (Motor *motor : motors)
  {
    if (motor->getName() != "X" && motor->getName() != "Y" && motor->getName() != "Z")
    {
      Debug::info("MotorManager", "Homing additional motor: " + motor->getName());

      bool additionalAxisSuccess = motor->startHoming();

      if (!additionalAxisSuccess)
      {
        Debug::error("MotorManager", "Failed to start homing for additional motor: " + motor->getName());
        success = false;
        continue;
      }

      // Wait for homing to complete
      unsigned long additionalAxisStartTime = millis();
      while (motor->getStatus() == HOMING)
      {
        update();
        delay(10);

        // Timeout check (30 seconds)
        if (millis() - additionalAxisStartTime > 30000)
        {
          Debug::error("MotorManager", motor->getName() + " additional axis homing timed out");
          success = false;
          break;
        }
      }

      Debug::info("MotorManager", motor->getName() + " additional axis homing " +
                                      String(motor->getStatus() != HOMING ? "COMPLETED" : "FAILED"));
    }
  }

  // Overall homing process summary
  unsigned long totalTime = millis() - startTime;
  Debug::info("MotorManager", "Home all sequence " +
                                  String(success ? "COMPLETED SUCCESSFULLY" : "FAILED") +
                                  " in " + String(totalTime) + " ms");

  return success;
}

bool MotorManager::homeMotor(int motorIndex)
{
  Motor *motor = getMotor(motorIndex);
  if (motor)
  {
    Debug::info("MotorManager", "Starting homing for motor at index " + String(motorIndex));
    return motor->startHoming();
  }

  Debug::error("MotorManager", "Cannot home motor at index " + String(motorIndex) +
                                   ". Motor not found.");
  return false;
}

bool MotorManager::homeMotorByName(const String &motorName)
{
  Motor *motor = getMotorByName(motorName);
  if (motor)
  {
    Debug::info("MotorManager", "Starting homing for motor: " + motorName);
    return motor->startHoming();
  }

  Debug::error("MotorManager", "Cannot home motor: " + motorName +
                                   ". Motor not found.");
  return false;
}