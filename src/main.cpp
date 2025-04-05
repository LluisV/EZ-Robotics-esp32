/**
 * @file main.cpp
 * @brief Final version with full functionality, added safeguards, and debug messages
 */

 #include <Arduino.h>
 #include "ConfigManager.h"
 #include "MotorManager.h"
 #include "MachineController.h"
 #include "GCodeParser.h"
 #include "CommandQueue.h"
 #include "CommandProcessor.h"
 
 // Debug flag - set to true to enable debug messages
 #define DEBUG true
 
 // Macro for debug printing
 #define DEBUG_PRINT(x) if(DEBUG) { Serial.println(String("DEBUG: ") + (x)); }
 
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
   DEBUG_PRINT("Communication task started on Core " + String(xPortGetCoreID()));
   
   while (true) {
     // Check for incoming G-code commands
     if (Serial.available()) {
       String command = Serial.readStringUntil('\n');
       command.trim();
       
       if (command.length() > 0) {
         DEBUG_PRINT("Received command: " + command);
         
         // Check for special command prefixes
         if (command.startsWith("!")) {
           // Emergency command - highest priority
           commandQueue.push(command.substring(1), IMMEDIATE);
           DEBUG_PRINT("Emergency command queued: " + command);
           Serial.println("Emergency command queued: " + command);
         } 
         else if (command.startsWith("?")) {
           // Information request - high priority
           commandQueue.push(command.substring(1), INFO);
           DEBUG_PRINT("Info command queued: " + command.substring(1));
         }
         else if (command.startsWith("$")) {
           // Setting command - medium priority
           commandQueue.push(command.substring(1), SETTING);
           DEBUG_PRINT("Setting command queued: " + command.substring(1));
         } 
         else {
           // Regular G-code - lowest priority
           commandQueue.push(command, MOTION);
           DEBUG_PRINT("Motion command queued: " + command);
         }
         
         DEBUG_PRINT("Queue status: " + String(commandQueue.size()) + " commands in queue");
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
   DEBUG_PRINT("Motion task started on Core " + String(xPortGetCoreID()));
   
   while (true) {
     // Add a null pointer check for safety
     if (machineController != NULL && gCodeParser != NULL && commandProcessor != NULL) {
       // First check for immediate commands
       String immediateCmd = commandQueue.getNextImmediate();
       
       if (immediateCmd.length() > 0) {
         DEBUG_PRINT("Processing immediate command: " + immediateCmd);
         // Process the immediate command
         if (immediateCmd == "STOP" || immediateCmd == "M112") {
           // Emergency stop
           machineController->emergencyStop();
           DEBUG_PRINT("EMERGENCY STOP EXECUTED");
           Serial.println("EMERGENCY STOP EXECUTED");
         } else {
           // Other immediate commands
           DEBUG_PRINT("Parsing immediate G-code: " + immediateCmd);
           gCodeParser->parse(immediateCmd);
         }
       }
       // Then process regular commands if not busy with immediate commands
       else if (!commandQueue.isEmpty()) {
         String command = commandQueue.pop();
         DEBUG_PRINT("Processing command from queue: " + command);
         
         // Handle information requests specially
         if (command.startsWith("POS")) {
           // Report current position
           DEBUG_PRINT("Position request received");
           std::vector<float> positions = machineController->getCurrentPosition();
           String posStr = "Position:";
           for (size_t i = 0; i < positions.size(); i++) {
             Motor* motor = motorManager.getMotor(i);
             if (motor) {
               posStr += " " + motor->getName() + ":" + String(positions[i], 3);
             }
           }
           DEBUG_PRINT("Sending position data: " + posStr);
           Serial.println(posStr);
         }
         else if (command.startsWith("STATUS")) {
           // Report machine status
           DEBUG_PRINT("Status request received");
           String status = "Status: ";
           status += machineController->isMoving() ? "MOVING" : "IDLE";
           DEBUG_PRINT("Sending status: " + status);
           Serial.println(status);
         }
         else {
           // Parse and execute regular G-code command
           DEBUG_PRINT("Parsing G-code: " + command);
           gCodeParser->parse(command);
         }
       }
       
       // Check and update motor states
       motorManager.update();
     } else {
       DEBUG_PRINT("Warning: Core components not initialized!");
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
   Serial.println("CNC Controller starting...");
   
   if (DEBUG) {
     Serial.println("DEBUG MODE ENABLED");
   }
   
   // Mount SPIFFS with format option if needed
   if (!SPIFFS.begin(true)) {
     Serial.println("SPIFFS Mount Failed even with formatting. Halting...");
     DEBUG_PRINT("Critical error: SPIFFS mount failure");
     while (1) { delay(1000); }
   }
   
   DEBUG_PRINT("SPIFFS mounted successfully");
   
   // Load configuration
   DEBUG_PRINT("Loading configuration...");
   if (!configManager.loadConfig()) {
     Serial.println("Failed to load configuration. Using defaults.");
     DEBUG_PRINT("Config load failed, using defaults");
     configManager.useDefaultConfig();
     
     // Save the default configuration
     DEBUG_PRINT("Attempting to save default configuration");
     if (configManager.saveConfig()) {
       Serial.println("Default configuration saved.");
       DEBUG_PRINT("Default configuration saved successfully");
     } else {
       DEBUG_PRINT("Failed to save default configuration");
     }
   } else {
     DEBUG_PRINT("Configuration loaded successfully");
   }
   
   // Initialize motors based on configuration with error handling
   Serial.println("Initializing motors...");
   DEBUG_PRINT("Beginning motor initialization");
   if (!motorManager.initialize(&configManager)) {
     Serial.println("Failed to initialize motors! Using safer defaults.");
     DEBUG_PRINT("Motor initialization failed, using safer defaults");
     // Continue anyway, but with limited functionality
   } else {
     DEBUG_PRINT("Motors initialized successfully");
   }
   
   // Create objects using dynamic memory allocation to prevent stack issues
   Serial.println("Creating controllers...");
   DEBUG_PRINT("Creating machine controller");
   machineController = new MachineController(&motorManager);
   if (machineController) {
     DEBUG_PRINT("Machine controller created, initializing...");
     machineController->initialize();
     DEBUG_PRINT("Machine controller initialized");
   } else {
     Serial.println("Failed to create machine controller!");
     DEBUG_PRINT("CRITICAL: Failed to allocate memory for machine controller");
   }
   
   DEBUG_PRINT("Creating G-code parser");
   gCodeParser = new GCodeParser(machineController);
   if (!gCodeParser) {
     Serial.println("Failed to create G-code parser!");
     DEBUG_PRINT("CRITICAL: Failed to allocate memory for G-code parser");
   } else {
     DEBUG_PRINT("G-code parser created successfully");
   }
   
   DEBUG_PRINT("Creating command processor");
   commandProcessor = new CommandProcessor(machineController);
   if (!commandProcessor) {
     Serial.println("Failed to create command processor!");
     DEBUG_PRINT("CRITICAL: Failed to allocate memory for command processor");
   } else {
     DEBUG_PRINT("Command processor created successfully");
   }
   
   Serial.println("System initialized. Starting tasks...");
   DEBUG_PRINT("Starting FreeRTOS tasks");
   
   // Create communication task on Core 0
   DEBUG_PRINT("Creating communication task on Core 0");
   xTaskCreatePinnedToCore(
     communicationTask,    // Function to implement the task
     "communicationTask",  // Name of the task
     4096,                 // Stack size in words (reduced)
     NULL,                 // Task input parameter
     1,                    // Priority of the task
     &commTaskHandle,      // Task handle
     0                     // Core where the task should run
   );
   
   if (commTaskHandle == NULL) {
     DEBUG_PRINT("CRITICAL: Failed to create communication task");
   } else {
     DEBUG_PRINT("Communication task created successfully");
   }
   
   // Add delay between task creations
   delay(500);
   
   // Create motion task on Core 1
   DEBUG_PRINT("Creating motion task on Core 1");
   xTaskCreatePinnedToCore(
     motionTask,           // Function to implement the task
     "motionTask",         // Name of the task
     4096,                 // Stack size in words (reduced)
     NULL,                 // Task input parameter
     1,                    // Priority of the task
     &motionTaskHandle,    // Task handle
     1                     // Core where the task should run
   );
   
   if (motionTaskHandle == NULL) {
     DEBUG_PRINT("CRITICAL: Failed to create motion task");
   } else {
     DEBUG_PRINT("Motion task created successfully");
   }
   
   DEBUG_PRINT("System initialization complete");
   Serial.println("System ready. Send G-code commands to begin.");
 }
 
 void loop() {
   // Everything is handled by the FreeRTOS tasks
   delay(1000);
   
   // Add periodic debug heartbeat
   if (DEBUG) {
     static unsigned long lastHeartbeat = 0;
     if (millis() - lastHeartbeat > 10000) { // Every 10 seconds
       lastHeartbeat = millis();
       DEBUG_PRINT("Heartbeat - System running for " + String(lastHeartbeat / 1000) + " seconds");
       DEBUG_PRINT("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
     }
   }
 }