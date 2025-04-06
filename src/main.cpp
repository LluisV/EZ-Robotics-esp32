/**
 * @file main.cpp
 * @brief Main application with full functionality, debug support, and dual-core FreeRTOS
 */

 #include <Arduino.h>
 #include "Debug.h"
 #include "ConfigManager.h"
 #include "MotorManager.h"
 #include "MachineController.h"
 #include "GCodeParser.h"
 #include "CommandQueue.h"
 #include "CommandProcessor.h"
 
 // Debug configuration
 #define DEBUG_ENABLED true
 #define DEBUG_LEVEL DEBUG_VERBOSE
 
 // Task handles for dual-core operation
 TaskHandle_t commTaskHandle = NULL;
 TaskHandle_t motionTaskHandle = NULL;
 
 // Global objects
 ConfigManager configManager;
 MotorManager motorManager;
 MachineController* machineController = NULL;
 GCodeParser* gCodeParser = NULL;
 CommandQueue commandQueue;
 CommandProcessor* commandProcessor = NULL;
 
 /**
  * @brief Communication task that runs on Core 0
  * Handles serial communication and G-code parsing with command priority
  */
 void communicationTask(void *parameter) {
   Debug::info("CommunicationTask", "Task started on Core " + String(xPortGetCoreID()));
   
   while (true) {
     // Check for incoming G-code commands
     if (Serial.available()) {
       String command = Serial.readStringUntil('\n');
       command.trim();
       
       if (command.length() > 0) {
         Debug::verbose("CommunicationTask", "Received command: " + command);
         
         // Check for special command prefixes
         if (command.startsWith("!")) {
           // Emergency command - highest priority
           commandQueue.push(command.substring(1), IMMEDIATE);
           Debug::info("CommunicationTask", "Emergency command queued: " + command);
           Serial.println("Emergency command queued: " + command);
         } 
         else if (command.startsWith("?")) {
           // Information request - handle directly without queuing
           String response = commandProcessor->processInfoCommand(command);
           if (!response.isEmpty()) {
             Debug::info("CommunicationTask", "Sending info response directly: " + response);
             Serial.println(response);
           }
         }
         else if (command.startsWith("$")) {
           // Setting command - medium priority
           commandQueue.push(command.substring(1), SETTING);
           Debug::verbose("CommunicationTask", "Setting command queued: " + command.substring(1));
         } 
         else {
           // Regular G-code - lowest priority
           commandQueue.push(command, MOTION);
           Debug::verbose("CommunicationTask", "Motion command queued: " + command);
         }
         
         Debug::verbose("CommunicationTask", "Queue status: " + String(commandQueue.size()) + " commands in queue");
       }
     }
     
     // Let the CPU breathe
     vTaskDelay(1);
   }
 }
 
 /**
  * @brief Motion control task that runs on Core 1
  * Handles motion planning and motor control with priority command handling
  */
 void motionTask(void *parameter) {
   Debug::info("MotionTask", "Task started on Core " + String(xPortGetCoreID()));
   
   while (true) {
     // Add a null pointer check for safety
     if (machineController != NULL && gCodeParser != NULL && commandProcessor != NULL) {
       // First check for immediate commands
       String immediateCmd = commandQueue.getNextImmediate();
       immediateCmd.toUpperCase();

       if (immediateCmd.length() > 0) {
         Debug::info("MotionTask", "Processing immediate command: " + immediateCmd);
         // Process the immediate command
         if (immediateCmd == "STOP" || immediateCmd == "M112") {
           // Emergency stop
           machineController->emergencyStop();
           Debug::error("MotionTask", "EMERGENCY STOP EXECUTED");
           Serial.println("EMERGENCY STOP EXECUTED");
         } else {
           // Other immediate commands
           Debug::verbose("MotionTask", "Parsing immediate G-code: " + immediateCmd);
           gCodeParser->parse(immediateCmd);
         }
       }
       // Then process regular commands if not busy with immediate commands
       else if (!commandQueue.isEmpty()) {
         String command = commandQueue.pop();
         Debug::verbose("MotionTask", "Processing command from queue: " + command);
         
         // Directly parse as G-code
         gCodeParser->parse(command);
       }
       
       // Check and update motor states
       motorManager.update();
     } else {
       Debug::error("MotionTask", "Critical components not initialized!");
       Serial.println("Warning: Core components not initialized!");
     }
     
     // Let the CPU breathe
     vTaskDelay(1);
   }
 }
 
 void setup() {
   // Initialize serial communication
   Serial.begin(115200);
   delay(2000); // Give time to connect serial monitor
   
   // Initialize debug system
   Debug::begin(DEBUG_ENABLED, 115200);
   Debug::setLevel(DEBUG_LEVEL);
   
   Serial.println("CNC Controller starting...");
   Debug::info("Main", "CNC Controller starting up");
   
   // Initialize the ConfigManager
   Debug::timerStart("Main", "ConfigManager Init");
   int configInitAttempts = 0;
   bool configInitSuccess = false;
   
   // Try a few times to initialize SPIFFS
   while (!configInitSuccess && configInitAttempts < 3) {
     configInitAttempts++;
     Debug::info("Main", "ConfigManager init attempt " + String(configInitAttempts));
     
     if (configManager.init()) {
       configInitSuccess = true;
       Debug::info("Main", "ConfigManager initialized successfully on attempt " + String(configInitAttempts));
     } else {
       Debug::warning("Main", "ConfigManager init failed on attempt " + String(configInitAttempts));
       delay(500); // Wait a bit before retrying
     }
   }
   
   if (!configInitSuccess) {
     Debug::error("Main", "Failed to initialize ConfigManager after multiple attempts");
     Serial.println("WARNING: ConfigManager initialization failed. Running without configuration.");
     // Don't halt - continue with default configs
   }
   
   Debug::timerEnd("Main", "ConfigManager Init");
   
   // Load configuration or use defaults
   Debug::timerStart("Main", "Config Load");
   bool configLoaded = false;
   
   if (configInitSuccess) {
     configLoaded = configManager.loadConfig();
   }
   
   if (!configLoaded) {
     Debug::warning("Main", "Failed to load configuration, using defaults");
     Serial.println("Failed to load configuration. Using defaults.");
     configManager.useDefaultConfig();
     
     // Try to save the default configuration if SPIFFS is available
     if (configInitSuccess) {
       Debug::info("Main", "Attempting to save default configuration");
       if (configManager.saveConfig()) {
         Debug::info("Main", "Default configuration saved successfully");
         Serial.println("Default configuration saved.");
       } else {
         Debug::warning("Main", "Failed to save default configuration");
       }
     }
   } else {
     Debug::info("Main", "Configuration loaded successfully");
   }
   Debug::timerEnd("Main", "Config Load");
   
   // Initialize motors based on configuration with error handling
   Serial.println("Initializing motors...");
   Debug::timerStart("Main", "Motor Init");
   if (!motorManager.initialize(&configManager)) {
     Debug::error("Main", "Motor initialization failed");
     Serial.println("Failed to initialize motors! Using safer defaults.");
   } else {
     Debug::info("Main", "Motors initialized successfully");
   }
   Debug::timerEnd("Main", "Motor Init");
   
   // Create objects using dynamic memory allocation to prevent stack issues
   Serial.println("Creating controllers...");
   Debug::info("Main", "Creating machine controller");
   machineController = new MachineController(&motorManager, &configManager);
   if (machineController) {
     Debug::info("Main", "Machine controller created, initializing...");
     machineController->initialize();
     Debug::info("Main", "Machine controller initialized");
   } else {
     Debug::error("Main", "Failed to allocate memory for machine controller");
     Serial.println("Failed to create machine controller!");
   }
   
   Debug::info("Main", "Creating G-code parser");
   gCodeParser = new GCodeParser(machineController);
   if (!gCodeParser) {
     Debug::error("Main", "Failed to allocate memory for G-code parser");
     Serial.println("Failed to create G-code parser!");
   } else {
     Debug::info("Main", "G-code parser created successfully");
   }
   
   Debug::info("Main", "Creating command processor");
   commandProcessor = new CommandProcessor(machineController, &configManager);
   if (!commandProcessor) {
     Debug::error("Main", "Failed to allocate memory for command processor");
     Serial.println("Failed to create command processor!");
   } else {
     Debug::info("Main", "Command processor created successfully");
   }
   
   Serial.println("System initialized. Starting tasks...");
   Debug::info("Main", "Starting FreeRTOS tasks");
   
   // Create communication task on Core 0
   Debug::info("Main", "Creating communication task on Core 0");
   xTaskCreatePinnedToCore(
     communicationTask,    // Function to implement the task
     "communicationTask",  // Name of the task
     4096,                 // Stack size in words
     NULL,                 // Task input parameter
     1,                    // Priority of the task
     &commTaskHandle,      // Task handle
     0                     // Core where the task should run
   );
   
   if (commTaskHandle == NULL) {
     Debug::error("Main", "Failed to create communication task");
   } else {
     Debug::info("Main", "Communication task created successfully");
   }
   
   // Add delay between task creations
   delay(500);
   
   // Create motion task on Core 1
   Debug::info("Main", "Creating motion task on Core 1");
   xTaskCreatePinnedToCore(
     motionTask,           // Function to implement the task
     "motionTask",         // Name of the task
     4096,                 // Stack size in words
     NULL,                 // Task input parameter
     1,                    // Priority of the task
     &motionTaskHandle,    // Task handle
     1                     // Core where the task should run
   );
   
   if (motionTaskHandle == NULL) {
     Debug::error("Main", "Failed to create motion task");
   } else {
     Debug::info("Main", "Motion task created successfully");
   }
   
   Debug::info("Main", "System initialization complete");
   Serial.println("System ready. Send G-code commands to begin.");
   
   // Print initial diagnostics
   Debug::printDiagnostics();
 }
 
 void loop() {
   // Everything is handled by the FreeRTOS tasks
   delay(1000);
   
   // Add periodic diagnostics
   static unsigned long lastDiagnosticTime = 0;
   if (Debug::isEnabled() && millis() - lastDiagnosticTime > 30000) { // Every 30 seconds
     lastDiagnosticTime = millis();
     Debug::printDiagnostics();
   }
 }