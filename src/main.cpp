/**
 * @file main.cpp
 * @brief Main application with communication layer separation and file handling
 */

 #include <Arduino.h>
 #include "Debug.h"
 #include "ConfigManager.h"
 #include "MotorManager.h"
 #include "MachineController.h"
 #include "GCodeParser.h"
 #include "CommandQueue.h"
 #include "CommandProcessor.h"
 #include "FileManager.h"
 #include "JobManager.h"
 #include "CommunicationManager.h"
 
 // Debug configuration
 #define DEBUG_ENABLED true
 #define DEBUG_LEVEL DEBUG_VERBOSE
 
 // Task handles for dual-core operation
 TaskHandle_t commTaskHandle = NULL;
 TaskHandle_t motionTaskHandle = NULL;
 
 // Command queue parameters
 #define COMMAND_QUEUE_SIZE 30
 
 // Global objects
 ConfigManager configManager;
 MotorManager motorManager;
 MachineController* machineController = NULL;
 GCodeParser* gCodeParser = NULL;
 CommandQueue* commandQueue = NULL;
 CommandProcessor* commandProcessor = NULL;
 FileManager* fileManager = NULL;
 JobManager* jobManager = NULL;
 CommunicationManager* communicationManager = NULL;
 
 /**
  * @brief Communication task that runs on Core 0
  * Handles serial communication, file operations, and command preprocessing
  */
 void communicationTask(void *parameter) {
   Debug::info("CommunicationTask", "Task started on Core " + String(xPortGetCoreID()));
   
   // Wait for system initialization
   delay(500);
   
   unsigned long lastStatusTime = 0;
   const unsigned long STATUS_INTERVAL = 1000; // Send status update every second
   
   while (true) {
     // Update communication manager
     if (communicationManager) {
       communicationManager->update();
     }
     
     // Update job manager
     if (jobManager) {
       jobManager->update();
     }
     
     // Periodic status updates
     unsigned long currentTime = millis();
     if (currentTime - lastStatusTime >= STATUS_INTERVAL) {
       lastStatusTime = currentTime;
       
       if (communicationManager && machineController && machineController->isMoving()) {
         // Send brief status update when machine is moving
         communicationManager->sendStatusUpdate(false);
       }
     }
     
     // Let the CPU breathe
     vTaskDelay(1);
   }
 }
 
 /**
  * @brief Motion control task that runs on Core 1
  * Handles motion planning and motor control with priority command handling
  * This task only processes commands from the queue and has no direct communication with serial
  */
 void motionTask(void *parameter) {
   Debug::info("MotionTask", "Task started on Core " + String(xPortGetCoreID()));
   
   while (true) {
     // Process commands from queue
     if (machineController && gCodeParser && commandQueue && !commandQueue->isEmpty()) {
       // First check for immediate commands (already placed in the queue by CommunicationManager)
       String immediateCmd = commandQueue->getNextImmediate();
       if (immediateCmd.length() > 0) {
         Debug::info("MotionTask", "Processing immediate command: " + immediateCmd);
         
         // Process immediate command - all interpretation should be done here
         // Emergency stop commands are already properly formatted as M112 by CommunicationManager
         gCodeParser->parse(immediateCmd);
       }
       // Then process regular commands if not busy with immediate commands
       else {
         String command = commandQueue->pop();
         Debug::verbose("MotionTask", "Processing command from queue: " + command);
         
         // Parse as G-code
         gCodeParser->parse(command);
       }
     }
     
     // Update motor states
     motorManager.update();
     
     // Let the CPU breathe
     vTaskDelay(1);
   }
 }
 
 void setup() {
   // Initialize serial communication
   Serial.begin(115200);
   delay(1000); // Give time to connect serial monitor
   
   // Initialize debug system
   Debug::begin(DEBUG_ENABLED, 115200);
   Debug::setLevel(DEBUG_LEVEL);
   
   Serial.println("CNC Controller starting...");
   Debug::info("Main", "CNC Controller starting up");
   
   // Initialize components in sequence with error checks
   
   // 1. Initialize ConfigManager
   Debug::info("Main", "Initializing ConfigManager");
   int configInitAttempts = 0;
   bool configInitSuccess = false;
   
   // Try a few times to initialize SPIFFS for configuration
   while (!configInitSuccess && configInitAttempts < 3) {
     configInitAttempts++;
     if (configManager.init()) {
       configInitSuccess = true;
       Debug::info("Main", "ConfigManager initialized successfully");
     } else {
       Debug::warning("Main", "ConfigManager init failed on attempt " + String(configInitAttempts));
       delay(500);
     }
   }
   
   // Load configuration or use defaults
   bool configLoaded = false;
   if (configInitSuccess) {
     configLoaded = configManager.loadConfig();
   }
   
   if (!configLoaded) {
     Debug::warning("Main", "Failed to load configuration, using defaults");
     Serial.println("Failed to load configuration. Using defaults.");
     configManager.useDefaultConfig();
   }
   
   // 2. Initialize MotorManager
   Debug::info("Main", "Initializing MotorManager");
   Serial.println("Initializing motors...");
   if (!motorManager.initialize(&configManager)) {
     Debug::error("Main", "Motor initialization failed - using safer defaults");
     Serial.println("Failed to initialize motors! Using safer defaults.");
   }
   
   // 3. Initialize CommandQueue
   Debug::info("Main", "Creating CommandQueue");
   commandQueue = new CommandQueue(COMMAND_QUEUE_SIZE);
   if (!commandQueue) {
     Debug::error("Main", "Failed to create CommandQueue");
     Serial.println("Failed to create command queue!");
   }
   
   // 4. Initialize MachineController
   Debug::info("Main", "Creating MachineController");
   machineController = new MachineController(&motorManager, &configManager);
   if (machineController) {
     machineController->initialize();
   } else {
     Debug::error("Main", "Failed to create MachineController");
     Serial.println("Failed to create machine controller!");
   }
   
   // 5. Initialize GCodeParser
   Debug::info("Main", "Creating GCodeParser");
   gCodeParser = new GCodeParser(machineController);
   if (!gCodeParser) {
     Debug::error("Main", "Failed to create GCodeParser");
     Serial.println("Failed to create G-code parser!");
   }
   
   // 6. Initialize CommandProcessor
   Debug::info("Main", "Creating CommandProcessor");
   commandProcessor = new CommandProcessor(machineController, &configManager);
   if (!commandProcessor) {
     Debug::error("Main", "Failed to create CommandProcessor");
     Serial.println("Failed to create command processor!");
   }
   
   // 7. Initialize FileManager
   Debug::info("Main", "Creating FileManager");
   fileManager = new FileManager();
   if (fileManager) {
     if (!fileManager->initialize(true)) {
       Debug::error("Main", "Failed to initialize FileManager");
       Serial.println("Failed to initialize file system!");
     }
   } else {
     Debug::error("Main", "Failed to create FileManager");
     Serial.println("Failed to create file manager!");
   }
   
   // 8. Initialize JobManager
   Debug::info("Main", "Creating JobManager");
   jobManager = new JobManager(commandQueue, fileManager);
   if (!jobManager) {
     Debug::error("Main", "Failed to create JobManager");
     Serial.println("Failed to create job manager!");
   }
   
   // 9. Initialize CommunicationManager
   Debug::info("Main", "Creating CommunicationManager");
   communicationManager = new CommunicationManager(commandQueue, commandProcessor, fileManager, jobManager);
   if (communicationManager) {
     communicationManager->initialize(115200);
   } else {
     Debug::error("Main", "Failed to create CommunicationManager");
     Serial.println("Failed to create communication manager!");
   }
   
   // Create FreeRTOS tasks
   Debug::info("Main", "Starting FreeRTOS tasks");
   
   // Create communication task on Core 0
   xTaskCreatePinnedToCore(
     communicationTask,    // Function to implement the task
     "communicationTask",  // Name of the task
     8192,                 // Stack size in words (increased for file operations)
     NULL,                 // Task input parameter
     1,                    // Priority of the task
     &commTaskHandle,      // Task handle
     0                     // Core where the task should run
   );
   
   if (commTaskHandle == NULL) {
     Debug::error("Main", "Failed to create communication task");
   }
   
   // Add delay between task creations
   delay(500);
   
   // Create motion task on Core 1
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
   }
   
   Debug::info("Main", "System initialization complete");
   Serial.println("System ready. Send G-code commands to begin.");
   
   // Print initial diagnostics
   Debug::printDiagnostics();
 }
 
 void loop() {
   // Everything is handled by the FreeRTOS tasks
   delay(1000);
   
   // Periodic diagnostics
   static unsigned long lastDiagnosticTime = 0;
   if (Debug::isEnabled() && millis() - lastDiagnosticTime > 60000) { // Every 60 seconds
     lastDiagnosticTime = millis();
     Debug::printDiagnostics();
   }
 }