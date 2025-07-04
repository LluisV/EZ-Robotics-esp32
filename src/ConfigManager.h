/**
 * @file ConfigManager.h
 * @brief Configuration manager for the CNC controller with kinematics support
 */

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <vector>
#include "RoboticsUtils.h"
#include "DH.h"

/**
 * @brief Axis type enumeration
 */
enum AxisType
{
  LINEAR_AXIS, ///< Linear axis (mm)
  ANGULAR_AXIS ///< Angular axis (degrees)
};

/**
 * @brief Motor configuration structure
 */
struct MotorConfig
{
  String name;           ///< Axis name (X, Y, Z, A, etc.)
  int stepPin;           ///< Step pin
  int dirPin;            ///< Direction pin
  int endstopPin;        ///< Endstop pin
  bool endstopInverted;  ///< True if endstop is active HIGH
  int stepsPerRev;       ///< Steps per revolution
  float reduction;       ///< Mechanical reduction (for direct drive = 1.0)
  float leadScrewPitch;  ///< Lead screw pitch in mm (only for LINEAR type)
  AxisType type;         ///< Axis type (LINEAR or ANGULAR)
  float maxSpeed;          ///< Maximum speed in steps/second
  float homeSpeed;         ///< Homing speed in steps/second
  float maxAcceleration;   ///< Acceleration in steps/second²
  int homingDirection;   ///< 1 or -1
  float backoffDistance; ///< Backoff distance after homing in mm or degrees
  float minPosition;     ///< Minimum allowed position
  float maxPosition;     ///< Maximum allowed position
  float endstopPosition; ///< Position value at endstop
  float homePosition;    ///< Position after homing
  bool invertStep;       ///< Invert step pulse
  bool invertDirection;  ///< Invert direction signal
};

/**
 * @brief DH parameters for a single joint
 */
struct DHParameters {
  double theta;    ///< Joint angle offset (for prismatic joints)
  double d;        ///< Link offset (for revolute joints) 
  double a;        ///< Link length
  double alpha;    ///< Link twist
};


/**
 * @brief Kinematics configuration
 */
struct KinematicsConfig {
  bool useKinematics;              ///< Enable kinematics transformation
  RobotType robotType;             ///< Robot type (CARTESIAN, SCARA, ARTICULATED_2R, etc.)
  std::vector<DHParameters> dhParams; ///< DH parameters for each joint
};

/**
 * @brief Machine configuration structure
 */
struct MachineConfig
{
  String machineName;      ///< Machine name
  float defaultFeedrate;   ///< Default feedrate in mm/min
  float maxFeedrate;       ///< Maximum feedrate in mm/min
  float maxAcceleration;   ///< Acceleration in mm/sec²
  float junctionDeviation; ///< Junction deviation for path planning in mm
  float arcTolerance;      ///< Arc tolerance in mm
  
  // Stepping parameters
  uint32_t stepPulseUsecs;         ///< Step pulse duration in microseconds
  uint32_t directionDelayUsecs;    ///< Direction signal setup time in microseconds
  uint32_t idleTimeoutMsecs;       ///< Idle timeout before disabling steppers in milliseconds
  uint32_t steppingFrequency;      ///< Stepping timer frequency in Hz
  
  struct
  {
    bool enabled;                ///< Whether telemetry is enabled
    int updatePositionFrequency; ///< Telemetry update frequency in Hz
  } telemetry;
  
  KinematicsConfig kinematics;     ///< Kinematics configuration
};

/**
 * @brief Manager class for configuration
 */
class ConfigManager
{
public:
  ConfigManager();
  ~ConfigManager();

  /**
   * @brief Initialize the configuration manager
   * @return True if successful, false otherwise
   */
  bool init();

  /**
   * @brief Load configuration from file
   * @return True if successful, false otherwise
   */
  bool loadConfig();

  /**
   * @brief Save configuration to file
   * @return True if successful, false otherwise
   */
  bool saveConfig();

  /**
   * @brief Use default configuration values
   */
  void useDefaultConfig();

  /**
   * @brief Get motor configuration by index
   * @param index Motor index
   * @return Motor configuration
   */
  const MotorConfig *getMotorConfig(int index) const;

  /**
   * @brief Get motor configuration by name
   * @param name Motor name (e.g., "X", "Y", "Z")
   * @return Motor configuration or nullptr if not found
   */
  const MotorConfig *getMotorConfigByName(const String &name) const;

  /**
   * @brief Get the machine configuration
   * @return Machine configuration
   */
  const MachineConfig &getMachineConfig() const;

  /**
   * @brief Get the number of configured motors
   * @return Number of motors
   */
  int getNumMotors() const;

  /**
   * @brief Add a new motor configuration
   * @param config Motor configuration
   */
  void addMotorConfig(const MotorConfig &config);

  /**
   * @brief Update machine configuration
   * @param config Machine configuration
   */
  void setMachineConfig(const MachineConfig &config);

private:
  static const char *CONFIG_FILE; ///< Configuration file path

  MachineConfig machineConfig;     ///< Machine configuration
  std::vector<MotorConfig> motors; ///< Motor configurations
  bool spiffsInitialized = false;  ///< Flag to track if SPIFFS is initialized

  /**
   * @brief Parse motor configuration from JSON
   * @param json JSON object
   * @param config Motor configuration to fill
   * @return True if successful, false otherwise
   */
  bool parseMotorConfig(const JsonObject &json, MotorConfig &config);

  /**
   * @brief Parse machine configuration from JSON
   * @param json JSON object
   * @return True if successful, false otherwise
   */
  bool parseMachineConfig(const JsonObject &json);
};

#endif // CONFIG_MANAGER_H