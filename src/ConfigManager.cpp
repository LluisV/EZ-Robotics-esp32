/**
 * @file ConfigManager.cpp
 * @brief Implementation of the ConfigManager class
 */

 #include <vector>
 #include "ConfigManager.h"
 #include "Debug.h"
 
 const char *ConfigManager::CONFIG_FILE = "/config.json";
 
 ConfigManager::ConfigManager()
 {
   // Do not initialize SPIFFS in constructor
   spiffsInitialized = false;
 }
 
 ConfigManager::~ConfigManager()
 {
   // Only end SPIFFS if it was initialized
   if (spiffsInitialized)
   {
     SPIFFS.end();
   }
 }
 
 bool ConfigManager::init()
 {
   Debug::info("ConfigManager", "Initializing SPIFFS...");
 
   // First try formatting SPIFFS explicitly
   if (!SPIFFS.begin(false))
   {
     Debug::warning("ConfigManager", "SPIFFS mount failed, trying to format...");
 
     if (SPIFFS.format())
     {
       Debug::info("ConfigManager", "SPIFFS formatted successfully");
     }
     else
     {
       Debug::error("ConfigManager", "SPIFFS format failed");
       return false;
     }
 
     // Try mounting again after format
     if (!SPIFFS.begin(false))
     {
       Debug::error("ConfigManager", "SPIFFS mount failed even after formatting");
       return false;
     }
   }
 
   Debug::info("ConfigManager", "SPIFFS initialized successfully");
   spiffsInitialized = true;
   return true;
 }
 
 bool ConfigManager::loadConfig()
 {
   // Check if SPIFFS is initialized
   if (!spiffsInitialized)
   {
     Debug::error("ConfigManager", "SPIFFS not initialized");
     return false;
   }
 
   // Clear existing configuration
   motors.clear();
 
   // Check if config file exists
   if (!SPIFFS.exists(CONFIG_FILE))
   {
     Debug::warning("ConfigManager", "Config file not found");
     return false;
   }
 
   // Open config file
   File file = SPIFFS.open(CONFIG_FILE, "r");
   if (!file)
   {
     Debug::error("ConfigManager", "Failed to open config file");
     return false;
   }
 
   // Parse Json to JsonDocument
   JsonDocument doc;
   DeserializationError error = deserializeJson(doc, file);
   if (error)
   {
     Debug::error("ConfigManager", "Failed to parse config file: " + String(error.c_str()));
     file.close();
     return false;
   }
 
   // Close the file
   file.close();
 
   // Parse machine configuration
   if (!parseMachineConfig(doc["machine"]))
   {
     Debug::error("ConfigManager", "Invalid machine configuration");
     return false;
   }
 
   // Parse motor configurations
   JsonArray motorsArray = doc["motors"].as<JsonArray>();
   if (motorsArray.isNull() || motorsArray.size() == 0)
   {
     Debug::error("ConfigManager", "No motors configured");
     return false;
   }
 
   for (JsonObject motorJson : motorsArray)
   {
     MotorConfig config;
     if (parseMotorConfig(motorJson, config))
     {
       motors.push_back(config);
     }
     else
     {
       Debug::warning("ConfigManager", "Failed to parse motor configuration for " + motorJson["name"].as<String>());
       // Continue parsing other motors
     }
   }
 
   Debug::info("ConfigManager", "Loaded configuration: " + String(motors.size()) + " motors");
   return motors.size() > 0;
 }
 
 bool ConfigManager::saveConfig()
 {
   // Check if SPIFFS is initialized
   if (!spiffsInitialized)
   {
     Debug::error("ConfigManager", "SPIFFS not initialized");
     return false;
   }
 
   Debug::info("ConfigManager", "Saving configuration to " + String(CONFIG_FILE));
 
   // Create a JSON document
   JsonDocument doc;
 
   // Add machine configuration
   JsonObject machineJson = doc["machine"].to<JsonObject>();
   machineJson["machineName"] = machineConfig.machineName;
   machineJson["defaultFeedrate"] = machineConfig.defaultFeedrate;
   machineJson["maxFeedrate"] = machineConfig.maxFeedrate;
   machineJson["maxAcceleration"] = machineConfig.maxAcceleration;
   machineJson["junctionDeviation"] = machineConfig.junctionDeviation;
   machineJson["arcTolerance"] = machineConfig.arcTolerance;
   
   // Add stepping parameters
   machineJson["stepPulseUsecs"] = machineConfig.stepPulseUsecs;
   machineJson["directionDelayUsecs"] = machineConfig.directionDelayUsecs;
   machineJson["idleTimeoutMsecs"] = machineConfig.idleTimeoutMsecs;
   machineJson["steppingFrequency"] = machineConfig.steppingFrequency;
 
   // Add telemetry configuration
   JsonObject telemetryJson = machineJson["telemetry"].to<JsonObject>();
   telemetryJson["enabled"] = machineConfig.telemetry.enabled;
   telemetryJson["updateFrequency"] = machineConfig.telemetry.updatePositionFrequency;
 
   // Add motor configurations
   JsonArray motorsArray = doc["motors"].to<JsonArray>();
   for (const MotorConfig &motor : motors)
   {
     JsonObject motorJson = motorsArray.add<JsonObject>();
     motorJson["name"] = motor.name;
     motorJson["stepPin"] = motor.stepPin;
     motorJson["dirPin"] = motor.dirPin;
     motorJson["endstopPin"] = motor.endstopPin;
     motorJson["endstopInverted"] = motor.endstopInverted;
     motorJson["stepsPerRev"] = motor.stepsPerRev;
     motorJson["reduction"] = motor.reduction;
     motorJson["leadScrewPitch"] = motor.leadScrewPitch;
     motorJson["type"] = motor.type == LINEAR_AXIS ? "LINEAR_AXIS" : "ANGULAR_AXIS";
     motorJson["maxSpeed"] = motor.maxSpeed;
     motorJson["homeSpeed"] = motor.homeSpeed;
     motorJson["maxAcceleration"] = motor.maxAcceleration;
     motorJson["homingDirection"] = motor.homingDirection;
     motorJson["backoffDistance"] = motor.backoffDistance;
     motorJson["minPosition"] = motor.minPosition;
     motorJson["maxPosition"] = motor.maxPosition;
     motorJson["endstopPosition"] = motor.endstopPosition;
     motorJson["homePosition"] = motor.homePosition;
     motorJson["invertStep"] = motor.invertStep;
     motorJson["invertDirection"] = motor.invertDirection;
   }
 
   // Open file for writing
   File file = SPIFFS.open(CONFIG_FILE, "w");
   if (!file)
   {
     Debug::error("ConfigManager", "Failed to open config file for writing");
     return false;
   }
 
   // Serialize JSON to file
   if (serializeJson(doc, file) == 0)
   {
     Debug::error("ConfigManager", "Failed to write config file");
     file.close();
     return false;
   }
 
   // Close the file
   file.close();
   Debug::info("ConfigManager", "Configuration saved successfully");
   return true;
 }
 
 void ConfigManager::useDefaultConfig()
 {
   // Clear existing configuration
   motors.clear();
 
   // Setup default machine configuration
   machineConfig.machineName = "Default CNC";
   machineConfig.defaultFeedrate = 1000.0; //  mm/min
   machineConfig.maxFeedrate = 2000.0;     //  mm/min
   machineConfig.maxAcceleration = 30.0;   // mm/s^2
   machineConfig.junctionDeviation = 0.05; // 0.05 mm
   machineConfig.arcTolerance = 0.002;     // 0.002 mm
   
   // Default stepping parameters
   machineConfig.stepPulseUsecs = 4;         // 4 μs step pulse
   machineConfig.directionDelayUsecs = 0;    // 0 μs direction delay
   machineConfig.idleTimeoutMsecs = 255;     // 255 ms idle timeout
   machineConfig.steppingFrequency = 20000000; // 20 MHz stepping timer frequency
 
   machineConfig.telemetry.enabled = true;               // Enable telemetry by default
   machineConfig.telemetry.updatePositionFrequency = 20; // 20 Hz update rate
 
   // Setup default X axis
   MotorConfig xConfig;
   xConfig.name = "X";
   xConfig.stepPin = 13;
   xConfig.dirPin = 12;
   xConfig.endstopPin = 14;
   xConfig.endstopInverted = false;
   xConfig.stepsPerRev = 1600;
   xConfig.reduction = 1.0;
   xConfig.leadScrewPitch = 5.0;
   xConfig.type = LINEAR_AXIS;
   xConfig.maxSpeed = 2000;
   xConfig.homeSpeed = 1000;
   xConfig.maxAcceleration = 8000;
   xConfig.homingDirection = -1;
   xConfig.backoffDistance = 5.0;
   xConfig.minPosition = 0.0;
   xConfig.maxPosition = 240.0;
   xConfig.endstopPosition = 0.0;
   xConfig.homePosition = 0.0;
   xConfig.invertDirection = false;
   xConfig.invertStep = false;
   motors.push_back(xConfig);
 
   // Setup default Y axis
   MotorConfig yConfig;
   yConfig.name = "Y";
   yConfig.stepPin = 33;
   yConfig.dirPin = 32;
   yConfig.endstopPin = 15;
   yConfig.endstopInverted = false;
   yConfig.stepsPerRev = 1600;
   yConfig.reduction = 1.0;
   yConfig.leadScrewPitch = 5.0;
   yConfig.type = LINEAR_AXIS;
   yConfig.maxSpeed = 2000;
   yConfig.homeSpeed = 1000;
   yConfig.maxAcceleration = 8000;
   yConfig.homingDirection = -1;
   yConfig.backoffDistance = 5.0;
   yConfig.minPosition = 0.0;
   yConfig.maxPosition = 350.0;
   yConfig.endstopPosition = 0.0;
   yConfig.homePosition = 0.0;
   yConfig.invertDirection = false;
   yConfig.invertStep = false;
   motors.push_back(yConfig);
 
   // Setup default Z axis
   MotorConfig zConfig;
   zConfig.name = "Z";
   zConfig.stepPin = 26;
   zConfig.dirPin = 25;
   zConfig.endstopPin = 4;
   zConfig.endstopInverted = false;
   zConfig.stepsPerRev = 1600;
   zConfig.reduction = 1.0;
   zConfig.leadScrewPitch = 5.0;
   zConfig.type = LINEAR_AXIS;
   zConfig.maxSpeed = 2000;
   zConfig.homeSpeed = 1000;
   zConfig.maxAcceleration = 8000;
   zConfig.homingDirection = -1;
   zConfig.backoffDistance = 5.0;
   zConfig.minPosition = 0.0;
   zConfig.maxPosition = 125.0;
   zConfig.endstopPosition = 125.0;
   zConfig.homePosition = 125.0;
   zConfig.invertDirection = true;
   zConfig.invertStep = false;
   motors.push_back(zConfig);
 
   Debug::info("ConfigManager", "Using default configuration with 3 axes");
 }
 
 const MotorConfig *ConfigManager::getMotorConfig(int index) const
 {
   if (index >= 0 && index < motors.size())
   {
     return &motors[index];
   }
   return nullptr;
 }
 
 const MotorConfig *ConfigManager::getMotorConfigByName(const String &name) const
 {
   for (const MotorConfig &motor : motors)
   {
     if (motor.name == name)
     {
       return &motor;
     }
   }
   return nullptr;
 }
 
 const MachineConfig &ConfigManager::getMachineConfig() const
 {
   return machineConfig;
 }
 
 int ConfigManager::getNumMotors() const
 {
   return motors.size();
 }
 
 void ConfigManager::addMotorConfig(const MotorConfig &config)
 {
   // Check if motor with same name already exists
   for (size_t i = 0; i < motors.size(); i++)
   {
     if (motors[i].name == config.name)
     {
       motors[i] = config; // Update existing config
       return;
     }
   }
 
   // Add new config
   motors.push_back(config);
 }
 
 void ConfigManager::setMachineConfig(const MachineConfig &config)
 {
   machineConfig = config;
 }
 
 bool ConfigManager::parseMotorConfig(const JsonObject &json, MotorConfig &config)
 {
   if (!json["name"].is<String>() ||
       !json["stepPin"].is<int>() ||
       !json["dirPin"].is<int>())
   {
     return false;
   }
 
   config.name = json["name"].as<String>();
   config.stepPin = json["stepPin"].as<int>();
   config.dirPin = json["dirPin"].as<int>();
 
   // Parse with defaults for optional fields
   config.endstopPin = json["endstopPin"].is<int>() ? json["endstopPin"].as<int>() : -1;
   config.endstopInverted = json["endstopInverted"].is<bool>() ? json["endstopInverted"].as<bool>() : false;
   config.stepsPerRev = json["stepsPerRev"].is<int>() ? json["stepsPerRev"].as<int>() : 1600;
   config.reduction = json["reduction"].is<float>() ? json["reduction"].as<float>() : 1.0f;
   config.leadScrewPitch = json["leadScrewPitch"].is<float>() ? json["leadScrewPitch"].as<float>() : 5.0f;
 
   // Parse axis type
   if (json["type"].is<String>())
   {
     String typeStr = json["type"].as<String>();
     config.type = (typeStr == "ANGULAR" || typeStr == "ANGULAR_AXIS") ? ANGULAR_AXIS : LINEAR_AXIS;
   }
   else
   {
     config.type = LINEAR_AXIS;
   }
 
   config.maxSpeed = json["maxSpeed"].is<int>() ? json["maxSpeed"].as<int>() : 2000;
   config.homeSpeed = json["homeSpeed"].is<int>() ? json["homeSpeed"].as<int>() : 1000;
   config.maxAcceleration = json["maxAcceleration"].is<int>() ? json["maxAcceleration"].as<int>() : 1000;
   config.homingDirection = json["homingDirection"].is<int>() ? json["homingDirection"].as<int>() : -1;
   config.backoffDistance = json["backoffDistance"].is<float>() ? json["backoffDistance"].as<float>() : 5.0f;
   config.minPosition = json["minPosition"].is<float>() ? json["minPosition"].as<float>() : 0.0f;
   config.maxPosition = json["maxPosition"].is<float>() ? json["maxPosition"].as<float>() : 200.0f;
   config.endstopPosition = json["endstopPosition"].is<float>() ? json["endstopPosition"].as<float>() : 0.0f;
   config.homePosition = json["homePosition"].is<float>() ? json["homePosition"].as<float>() : 0.0f;
   config.invertStep = json["invertStep"].is<bool>() ? json["invertStep"].as<bool>() : false;
   config.invertDirection = json["invertDirection"].is<bool>() ? json["invertDirection"].as<bool>() : false;
 
   return true;
 }
 
 bool ConfigManager::parseMachineConfig(const JsonObject &json)
 {
   if (json.isNull())
   {
     // Use defaults if no machine config is provided
     machineConfig.machineName = "Default CNC";
     machineConfig.defaultFeedrate = 1000.0;
     machineConfig.maxFeedrate = 5000.0;
     machineConfig.maxAcceleration = 30.0;
     machineConfig.junctionDeviation = 0.05;
     machineConfig.arcTolerance = 0.002;
     
     // Default stepping parameters
     machineConfig.stepPulseUsecs = 4;
     machineConfig.directionDelayUsecs = 0;
     machineConfig.idleTimeoutMsecs = 255;
     machineConfig.steppingFrequency = 20000000; // 20 MHz
     
     // Default telemetry configuration
     machineConfig.telemetry.enabled = true;
     machineConfig.telemetry.updatePositionFrequency = 20;
 
     return true;
   }
 
   // Parse machine configuration
   machineConfig.machineName = json["machineName"].is<String>() ? json["machineName"].as<String>() : "Default CNC";
   machineConfig.defaultFeedrate = json["defaultFeedrate"].is<float>() ? json["defaultFeedrate"].as<float>() : 1000.0f;
   machineConfig.maxFeedrate = json["maxFeedrate"].is<float>() ? json["maxFeedrate"].as<float>() : 2000.0f;
   machineConfig.maxAcceleration = json["maxAcceleration"].is<float>() ? json["maxAcceleration"].as<float>() : 30.0f;
   machineConfig.junctionDeviation = json["junctionDeviation"].is<float>() ? json["junctionDeviation"].as<float>() : 0.05f;
   machineConfig.arcTolerance = json["arcTolerance"].is<float>() ? json["arcTolerance"].as<float>() : 0.002f;
   
   // Parse stepping parameters
   machineConfig.stepPulseUsecs = json["stepPulseUsecs"].is<uint32_t>() ? json["stepPulseUsecs"].as<uint32_t>() : 4;
   machineConfig.directionDelayUsecs = json["directionDelayUsecs"].is<uint32_t>() ? json["directionDelayUsecs"].as<uint32_t>() : 0;
   machineConfig.idleTimeoutMsecs = json["idleTimeoutMsecs"].is<uint32_t>() ? json["idleTimeoutMsecs"].as<uint32_t>() : 255;
   machineConfig.steppingFrequency = json["steppingFrequency"].is<uint32_t>() ? json["steppingFrequency"].as<uint32_t>() : 20000000;
 
   // Parse telemetry configuration
   JsonObject telemetryJson = json["telemetry"].as<JsonObject>();
   machineConfig.telemetry.enabled = telemetryJson["enabled"].is<bool>() ? telemetryJson["enabled"].as<bool>() : true;
   machineConfig.telemetry.updatePositionFrequency = telemetryJson["updateFrequency"].is<int>() ? telemetryJson["updateFrequency"].as<int>() : 20;
 
   return true;
 }