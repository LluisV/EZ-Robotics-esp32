/**
 * @file MotorManager.h
 * @brief Motor manager for the CNC controller
 */

#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <vector>
#include "ConfigManager.h"
#include "Motor.h"

/**
 * @brief Manager class for multiple motors
 */
class MotorManager
{
public:
  MotorManager();
  ~MotorManager();

  /**
   * @brief Initialize all motors based on configuration
   * @param configManager Configuration manager
   * @return True if all motors were initialized, false otherwise
   */
  bool initialize(ConfigManager *configManager);

  /**
   * @brief Get motor by index
   * @param index Motor index
   * @return Motor pointer or nullptr if not found
   */
  Motor *getMotor(int index);

  /**
   * @brief Get motor by name
   * @param name Motor name
   * @return Motor pointer or nullptr if not found
   */
  Motor *getMotorByName(const String &name);

  /**
   * @brief Get number of motors
   * @return Number of motors
   */
  int getNumMotors() const;

  /**
   * @brief Update all motors
   * @return True if all motors were updated, false otherwise
   */
  bool update();

  /**
   * @brief Check if any motor is moving
   * @return True if any motor is moving, false otherwise
   */
  bool isAnyMotorMoving() const;

  /**
   * @brief Perform rapid (maximum speed) synchronized move of multiple motors
   * @param positions Array of target positions in user units
   * @return True if the move was started, false otherwise
   */
  bool moveToRapid(const std::vector<float> &positions);

  /**
   * @brief Perform synchronized move of multiple motors at specified feedrate
   * @param positions Array of target positions in user units
   * @param feedrate Feedrate in mm/min
   * @return True if the move was started, false otherwise
   */
  bool moveToFeedrate(const std::vector<float> &positions, float feedrate);

  /**
   * @brief Stop all motors
   * @param immediate If true, stop immediately without deceleration
   */
  void stopAll(bool immediate = false);

  /**
   * @brief Start homing sequence for all motors
   * @return True if homing was started for all motors, false otherwise
   */
  bool homeAll();

  /**
   * @brief Home a specific motor
   * @param motorIndex Motor index
   * @return True if homing was started, false otherwise
   */
  bool homeMotor(int motorIndex);

  /**
   * @brief Home a specific motor by name
   * @param motorName Motor name
   * @return True if homing was started, false otherwise
   */
  bool homeMotorByName(const String &motorName);

private:
  FastAccelStepperEngine engine; ///< FastAccelStepper engine
  std::vector<Motor *> motors;   ///< Motors
  ConfigManager *configManager;  ///< Configuration manager
};

#endif // MOTOR_MANAGER_H