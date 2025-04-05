/**
 * @file ConfigManager.h
 * @brief Configuration manager for the CNC controller
 */

 #ifndef CONFIG_MANAGER_H
 #define CONFIG_MANAGER_H
 
 #include <Arduino.h>
 #include <ArduinoJson.h>
 #include <SPIFFS.h>
 #include <vector>
 
 /**
  * @brief Axis type enumeration
  */
 enum AxisType {
   LINEAR_AXIS,   ///< Linear axis (mm)
   ANGULAR_AXIS   ///< Angular axis (degrees)
 };
 
 /**
  * @brief Motor configuration structure
  */
 struct MotorConfig {
   String name;            ///< Axis name (X, Y, Z, A, etc.)
   int stepPin;            ///< Step pin
   int dirPin;             ///< Direction pin
   int endstopPin;         ///< Endstop pin
   bool endstopInverted;   ///< True if endstop is active HIGH
   int stepsPerRev;        ///< Steps per revolution
   float reduction;        ///< Mechanical reduction (for direct drive = 1.0)
   float leadScrewPitch;   ///< Lead screw pitch in mm (only for LINEAR type)
   AxisType type;          ///< Axis type (LINEAR or ANGULAR)
   int maxSpeed;           ///< Maximum speed in steps/second
   int homeSpeed;          ///< Homing speed in steps/second
   int acceleration;       ///< Acceleration in steps/second²
   int homingDirection;    ///< 1 or -1
   float backoffDistance;  ///< Backoff distance after homing in mm or degrees
   float maxTravel;        ///< Maximum travel distance in mm or degrees
 };
 
 /**
  * @brief Machine configuration structure
  */
 struct MachineConfig {
   String machineName;     ///< Machine name
   float defaultFeedrate;  ///< Default feedrate in mm/min
   float maxFeedrate;      ///< Maximum feedrate in mm/min
   float junctionDeviation; ///< Junction deviation for path planning
   float arcTolerance;     ///< Arc tolerance in mm
 };
 
 /**
  * @brief Manager class for configuration
  */
 class ConfigManager {
 public:
   ConfigManager();
   ~ConfigManager();
   
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
   const MotorConfig* getMotorConfig(int index) const;
   
   /**
    * @brief Get motor configuration by name
    * @param name Motor name (e.g., "X", "Y", "Z")
    * @return Motor configuration or nullptr if not found
    */
   const MotorConfig* getMotorConfigByName(const String& name) const;
   
   /**
    * @brief Get the machine configuration
    * @return Machine configuration
    */
   const MachineConfig& getMachineConfig() const;
   
   /**
    * @brief Get the number of configured motors
    * @return Number of motors
    */
   int getNumMotors() const;
   
   /**
    * @brief Add a new motor configuration
    * @param config Motor configuration
    */
   void addMotorConfig(const MotorConfig& config);
   
   /**
    * @brief Update machine configuration
    * @param config Machine configuration
    */
   void setMachineConfig(const MachineConfig& config);
 
 private:
   static const char* CONFIG_FILE;  ///< Configuration file path
   
   MachineConfig machineConfig;     ///< Machine configuration
   std::vector<MotorConfig> motors; ///< Motor configurations
   
   /**
    * @brief Parse motor configuration from JSON
    * @param json JSON object
    * @param config Motor configuration to fill
    * @return True if successful, false otherwise
    */
   bool parseMotorConfig(const JsonObject& json, MotorConfig& config);
   
   /**
    * @brief Parse machine configuration from JSON
    * @param json JSON object
    * @return True if successful, false otherwise
    */
   bool parseMachineConfig(const JsonObject& json);
 };
 
 #endif // CONFIG_MANAGER_H