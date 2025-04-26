/**
 * @file main.cpp
 * @brief Main application with GRBL-inspired pipeline architecture
 * 
 * Implements a pipeline architecture similar to GRBL:
 * Line Buffer → GCode Parser → Motion Control → Planner → Segment Generator → Motors
 */

 #include <Arduino.h>
 #include "Debug.h"
 #include "ConfigManager.h"
 #include "MotorManager.h"
 #include "MachineController.h"
 #include "MotionControl.h"
 #include "Planner.h"
 #include "SegmentGenerator.h"
 #include "GCodeParser.h"
 #include "LineBuffer.h"
 #include "CommunicationManager.h"
 
 // Debug configuration
 #define DEBUG_ENABLED true
 #define DEBUG_LEVEL DEBUG_WARNING
 
 // Task handles for dual-core operation
 TaskHandle_t commTaskHandle = NULL;
 TaskHandle_t motionTaskHandle = NULL;
 
 // Global objects
 ConfigManager configManager;
 MotorManager motorManager;
 MachineController* machineController = NULL;
 MotionControl* motionControl = NULL;
 Planner* planner = NULL;
 SegmentGenerator* segmentGenerator = NULL;
 GCodeParser* gCodeParser = NULL;
 LineBuffer* lineBuffer = NULL;
 CommunicationManager* communicationManager = NULL;
 
 /**
  * @brief Communication task that runs on Core 0
  * Handles serial communication and buffering incoming lines
  */
 void communicationTask(void *parameter)
 {
   Debug::info("CommunicationTask", "Task started on Core " + String(xPortGetCoreID()));
 
   // Wait for system initialization
   delay(500);
 
   while (true)
   {
     try
     {
       // Update communication manager
       if (communicationManager)
       {
         communicationManager->update();
       }
     }
     catch (const std::exception &e)
     {
       // Log the error
       Debug::error("CommunicationTask", "Exception caught: " + String(e.what()));
 
       // Report the error
       if (communicationManager)
       {
         communicationManager->sendMessage("error:1 System error: " + String(e.what()));
       }
 
       // Emergency stop on error
       if (machineController)
       {
         machineController->emergencyStop();
       }
     }
     catch (...)
     {
       // Log the unknown error
       Debug::error("CommunicationTask", "Unknown exception caught");
 
       // Report the error
       if (communicationManager)
       {
         communicationManager->sendMessage("error:1 System error: Unknown exception");
       }
 
       // Emergency stop on error
       if (machineController)
       {
         machineController->emergencyStop();
       }
     }
 
     // Let the CPU breathe
     vTaskDelay(1);
   }
 }
 
 /**
  * @brief Motion control task that runs on Core 1
  * Processes the execution pipeline from GCode parsing to motor control
  */
 void motionTask(void *parameter)
 {
   Debug::info("MotionTask", "Task started on Core " + String(xPortGetCoreID()));
   
   unsigned long lastCommandTime = 0;
   const unsigned long COMMAND_INTERVAL = 10; // Process commands every 10ms
   
   // Keep track of the current command being processed
   String currentCommand = "";
   bool retryingCommand = false;
 
   while (true)
   {
     try
     {
       // Step 1: Update segment generator and execute segments
       if (segmentGenerator) { 
         segmentGenerator->update();
       }
 
       // Step 2: Process GCode from the line buffer less frequently
       unsigned long currentTime = millis();
       if (currentTime - lastCommandTime >= COMMAND_INTERVAL) {
         lastCommandTime = currentTime;
         
         // Only parse new GCode if motion control has space
         if (gCodeParser && lineBuffer && motionControl && motionControl->hasSpace()) {
           
           // If we're not retrying a command, get the next one
           if (!retryingCommand && currentCommand.isEmpty() && !lineBuffer->isEmpty()) {
             currentCommand = lineBuffer->getLine();
           }
           
           // Process the command if we have one
           if (currentCommand.length() > 0) {
             // Process the line with the GCode parser
             GCodeParseResult parseResult = gCodeParser->parse(currentCommand);
             
             switch (parseResult)
             {
               case GCodeParseResult::PARSE_ERROR:
                 Debug::error("MotionTask", "Failed to execute command: " + currentCommand);
                 // Error is already reported by the GCode parser
                 // Clear the command since it failed
                 currentCommand = "";
                 retryingCommand = false;
                 break;
               
               case GCodeParseResult::QUEUE_FULL:
                 // Hold onto the command to retry later
                 retryingCommand = true;
                 // Wait for more space before retrying
                 Debug::verbose("MotionTask", "Queue full, waiting to retry: " + currentCommand);
                 break;
               
               case GCodeParseResult::SUCCESS:
                 // Command successfully processed
                 currentCommand = "";
                 retryingCommand = false;
                 break;
             }
           }
         }
       }
 
       // Step 3: Update motor states - this should be called frequently
       motorManager.update();
 
       // Only add a delay if nothing important is happening
       if ((!planner || planner->isEmpty()) && 
           (!segmentGenerator || segmentGenerator->isEmpty()) && 
           !motorManager.isAnyMotorMoving() &&
           !retryingCommand) {
         vTaskDelay(1); // Only delay when idle
       }
     }
     catch (const std::exception &e)
     {
       Debug::error("MotionTask", "Exception caught: " + String(e.what()));
       
       // Emergency stop on error
       if (machineController) {
         machineController->emergencyStop();
       }
     }
     catch (...)
     {
       Debug::error("MotionTask", "Unknown exception caught");
       
       // Emergency stop on error
       if (machineController) {
         machineController->emergencyStop();
       }
     }
   }
 }
 
 void setup()
 {
   // Initialize serial communication
   Serial.begin(115200);
   delay(1000); // Give time to connect serial monitor
 
   // Initialize debug system
   Debug::begin(DEBUG_ENABLED, 115200);
   Debug::setLevel(DEBUG_LEVEL);
 
   Serial.println("CNC Controller starting...");
   Debug::info("Main", "CNC Controller starting up with GRBL-inspired pipeline");
 
   // Initialize components in sequence with error checks
 
   // 1. Initialize ConfigManager
   Debug::info("Main", "Initializing ConfigManager");
   int configInitAttempts = 0;
   bool configInitSuccess = false;
 
   // Try a few times to initialize SPIFFS for configuration
   while (!configInitSuccess && configInitAttempts < 3)
   {
     configInitAttempts++;
     if (configManager.init())
     {
       configInitSuccess = true;
       Debug::info("Main", "ConfigManager initialized successfully");
     }
     else
     {
       delay(500);
     }
   }
 
   // Load configuration or use defaults
   bool configLoaded = false;
   if (configInitSuccess)
   {
     configLoaded = configManager.loadConfig();
   }
 
   if (!configLoaded)
   {
     Serial.println("Failed to load configuration. Using defaults.");
     configManager.useDefaultConfig();
   }
 
   // 2. Initialize MotorManager
   Debug::info("Main", "Initializing MotorManager");
   Serial.println("Initializing motors...");
   if (!motorManager.initialize(&configManager))
   {
     Debug::error("Main", "Motor initialization failed - using safer defaults");
     Serial.println("Failed to initialize motors! Using safer defaults.");
   }
 
   // 3. Initialize LineBuffer (receives incoming GCode lines)
   Debug::info("Main", "Creating LineBuffer");
   lineBuffer = new LineBuffer();
   if (!lineBuffer)
   {
     Debug::error("Main", "Failed to create LineBuffer");
     Serial.println("Failed to create line buffer!");
   }
 
   // 4. Initialize MachineController (basic machine operations)
   Debug::info("Main", "Creating MachineController");
   machineController = new MachineController(&motorManager, &configManager);
   if (machineController)
   {
     machineController->initialize();
   }
   else
   {
     Debug::error("Main", "Failed to create MachineController");
     Serial.println("Failed to create machine controller!");
   }
 
   // 5. Initialize SegmentGenerator (generates constant-velocity segments)
   Debug::info("Main", "Creating SegmentGenerator");
   segmentGenerator = new SegmentGenerator(machineController, &motorManager);
   if (segmentGenerator)
   {
     segmentGenerator->initialize();
     Debug::info("Main", "SegmentGenerator initialized");
   }
   else
   {
     Debug::error("Main", "Failed to create segment generator");
     Serial.println("Failed to create segment generator!");
   }
 
   // 6. Initialize Planner (plans movement paths)
   Debug::info("Main", "Creating Planner");
   planner = new Planner(segmentGenerator);
   if (planner)
   {
     planner->initialize();
     Debug::info("Main", "Planner initialized");
     
     // Connect segment generator to planner
     if (segmentGenerator) {
       segmentGenerator->setPlanner(planner);
     }
   }
   else
   {
     Debug::error("Main", "Failed to create motion planner");
     Serial.println("Failed to create motion planner!");
   }
 
   // 7. Initialize MotionControl (handles GCode-to-movement transformation)
   Debug::info("Main", "Creating MotionControl");
   motionControl = new MotionControl(machineController, planner);
   if (motionControl)
   {
     motionControl->initialize();
     Debug::info("Main", "MotionControl initialized");
   }
   else
   {
     Debug::error("Main", "Failed to create motion control");
     Serial.println("Failed to create motion control!");
   }
 
   // 8. Initialize GCodeParser (parses GCode into machine commands)
   Debug::info("Main", "Creating GCodeParser");
   gCodeParser = new GCodeParser(motionControl);
   if (!gCodeParser)
   {
     Debug::error("Main", "Failed to create GCodeParser");
     Serial.println("Failed to create G-code parser!");
   }
 
   // 9. Initialize CommunicationManager (handles serial communication)
   Debug::info("Main", "Creating CommunicationManager");
   communicationManager = new CommunicationManager(lineBuffer, machineController);
   if (communicationManager)
   {
     communicationManager->initialize(115200);
     
     // Configure telemetry from machine config if available
     const MachineConfig &machineConfig = configManager.getMachineConfig();
     if (machineConfig.telemetry.enabled) {
       communicationManager->setTelemetryEnabled(true);
       communicationManager->setTelemetryFrequency(machineConfig.telemetry.updatePositionFrequency);
       Debug::info("Main", "Telemetry enabled at " + String(machineConfig.telemetry.updatePositionFrequency) + "Hz");
     } else {
       communicationManager->setTelemetryEnabled(false);
       Debug::info("Main", "Telemetry disabled by configuration");
     }
     
     // Set status report interval to 100ms (10Hz) for GCode senders
     communicationManager->setStatusReportInterval(100);
   }
   else
   {
     Debug::error("Main", "Failed to create CommunicationManager");
     Serial.println("Failed to create communication manager!");
   }
 
   // Create FreeRTOS tasks
   Debug::info("Main", "Starting FreeRTOS tasks");
 
   // Create communication task on Core 0
   xTaskCreatePinnedToCore(
       communicationTask,   // Function to implement the task
       "communicationTask", // Name of the task
       4096,                // Stack size in words
       NULL,                // Task input parameter
       1,                   // Priority of the task
       &commTaskHandle,     // Task handle
       0                    // Core where the task should run
   );
 
   if (commTaskHandle == NULL)
   {
     Debug::error("Main", "Failed to create communication task");
   }
 
   // Add delay between task creations
   delay(500);
 
   // Create motion task on Core 1
   xTaskCreatePinnedToCore(
       motionTask,        // Function to implement the task
       "motionTask",      // Name of the task
       8192,              // Stack size in words
       NULL,              // Task input parameter
       3,                 // Priority of the task
       &motionTaskHandle, // Task handle
       1                  // Core where the task should run
   );
 
   if (motionTaskHandle == NULL)
   {
     Debug::error("Main", "Failed to create motion task");
   }
 
   Debug::info("Main", "Pipeline initialization complete:");
   Debug::info("Main", "LineBuffer → GCodeParser → MotionControl → Planner → SegmentGenerator → Motors");
   Serial.println("System ready. Send G-code commands to begin.");
 
   // Print initial diagnostics
   Debug::printDiagnostics();
 }
 
 void loop()
 {
   // Everything is handled by the FreeRTOS tasks
   delay(1000);
 }