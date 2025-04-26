/**
 * @file main.cpp
 * @brief Main application with GRBL protocol support and JSON telemetry
 */

 #include <Arduino.h>
 #include "Debug.h"
 #include "ConfigManager.h"
 #include "MotorManager.h"
 #include "MachineController.h"
 #include "GCodeParser.h"
 #include "CommandQueue.h"
 #include "CommandProcessor.h"
 #include "CommunicationManager.h" // GRBL communication manager with telemetry
 #include "Scheduler.h"
 
 // Debug configuration
 #define DEBUG_ENABLED true
 #define DEBUG_LEVEL DEBUG_WARNING
 
 // Task handles for dual-core operation
 TaskHandle_t commTaskHandle = NULL;
 TaskHandle_t motionTaskHandle = NULL;
 
 // Command queue parameters
 #define COMMAND_QUEUE_SIZE 500
 
 // Global objects
 ConfigManager configManager;
 MotorManager motorManager;
 MachineController *machineController = NULL;
 GCodeParser *gCodeParser = NULL;
 CommandQueue *commandQueue = NULL;
 CommandProcessor *commandProcessor = NULL;
 CommunicationManager *communicationManager = NULL;
 Scheduler *scheduler = NULL;
 
 /**
  * @brief Communication task that runs on Core 0
  * Handles serial communication and command preprocessing
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
  * Handles motion planning and motor control with priority command handling
  * This task only processes commands from the queue and has no direct communication with serial
  */
 void motionTask(void *parameter)
 {
   Debug::info("MotionTask", "Task started on Core " + String(xPortGetCoreID()));
   
   String pendingCommand = "";
   unsigned long lastCommandTime = 0;
   const unsigned long COMMAND_INTERVAL = 10; // Process commands every 10ms
 
   while (true)
   {
     try
     {
       // Process segments from the scheduler - HIGHEST PRIORITY
       if (scheduler) { 
         // Execute the next movement segment
         scheduler->executeNextSegment();
       }
 
       // Process commands from queue less frequently
       unsigned long currentTime = millis();
       if (currentTime - lastCommandTime >= COMMAND_INTERVAL) {
         lastCommandTime = currentTime;
         
         // Process commands from queue
         if (machineController && gCodeParser && commandQueue && !commandQueue->isEmpty())
         {
           // First check for immediate commands
           String immediateCmd = commandQueue->getNextImmediate();
           if (immediateCmd.length() > 0)
           {
             Debug::info("MotionTask", "Processing immediate command: " + immediateCmd);
 
             // Process immediate command
             GCodeParseResult parseResult = gCodeParser->parse(immediateCmd);
             switch (parseResult)
             {
               case GCodeParseResult::PARSE_ERROR:
                 Debug::error("MotionTask", "Failed to execute immediate command: " + immediateCmd);
 
                 if (immediateCmd.startsWith("M112") || immediateCmd.startsWith("STOP"))
                 {
                   machineController->emergencyStop();
                 }
                 break;
 
               case GCodeParseResult::QUEUE_FULL:
                 if (immediateCmd.startsWith("M112") || immediateCmd.startsWith("STOP"))
                 {
                   machineController->emergencyStop();
                 }
                 else
                 {
                   commandQueue->push(immediateCmd, IMMEDIATE);
                 }
                 break;
 
               case GCodeParseResult::SUCCESS:
                 // Command successfully processed
                 break;
             }
           }
           // Process regular commands
           else
           {
             if(commandQueue->isEmpty())
               continue;
             String command = commandQueue->pop();
 
             // Parse as G-code
             GCodeParseResult parseResult = gCodeParser->parse(command);
             switch (parseResult)
             {
               case GCodeParseResult::PARSE_ERROR:
                 Debug::error("MotionTask", "Failed to execute command: " + command);
                 break;
 
               case GCodeParseResult::QUEUE_FULL:
                 commandQueue->push(command, IMMEDIATE);
                 break;
 
               case GCodeParseResult::SUCCESS:
                 // Command successfully processed
                 break;
             }
           }
         }
       }
 
       // Update motor states - this should be called frequently
       motorManager.update();
 
       // Only add a delay if nothing important is happening
       if (!scheduler->hasMove() && !motorManager.isAnyMotorMoving()) {
         vTaskDelay(1); // Only delay when idle
       }
     }
     catch (const std::exception &e)
     {
       Debug::error("MotionTask", "Exception caught: " + String(e.what()));
       // Error handling code...
     }
     catch (...)
     {
       Debug::error("MotionTask", "Unknown exception caught");
       // Error handling code...
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
   Debug::info("Main", "CNC Controller starting up");
 
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
 
   // 3. Initialize CommandQueue
   Debug::info("Main", "Creating CommandQueue");
   commandQueue = new CommandQueue(COMMAND_QUEUE_SIZE);
   if (!commandQueue)
   {
     Debug::error("Main", "Failed to create CommandQueue");
     Serial.println("Failed to create command queue!");
   }
 
   // 4. Initialize MachineController
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
 
   // 5. Initialize segmented motion planner
   Debug::info("Main", "Creating MotionPlanner");
   scheduler = new Scheduler(machineController, &motorManager, &configManager);
   if (scheduler)
   {
     scheduler->initialize();
     if (machineController)
     {
       machineController->setMotionPlanner(scheduler);
       Debug::info("Main", "Segmented motion planner connected to machine controller");
     }
   }
   else
   {
     Debug::error("Main", "Failed to create segmented motion planner");
     Serial.println("Failed to create segmented motion planner!");
   }
 
   // 6. Initialize GCodeParser
   Debug::info("Main", "Creating GCodeParser");
   gCodeParser = new GCodeParser(machineController);
   if (!gCodeParser)
   {
     Debug::error("Main", "Failed to create GCodeParser");
     Serial.println("Failed to create G-code parser!");
   }
 
   // 7. Initialize CommandProcessor
   Debug::info("Main", "Creating CommandProcessor");
   commandProcessor = new CommandProcessor(machineController, &configManager);
   if (!commandProcessor)
   {
     Debug::error("Main", "Failed to create CommandProcessor");
     Serial.println("Failed to create command processor!");
   }
 
   // 8. Initialize GRBL CommunicationManager with telemetry
   Debug::info("Main", "Creating GRBLCommunicationManager");
   communicationManager = new CommunicationManager(commandQueue, commandProcessor);
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
     Debug::error("Main", "Failed to create GRBLCommunicationManager");
     Serial.println("Failed to create GRBL communication manager!");
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
 
   Debug::info("Main", "System initialization complete");
   Serial.println("Grbl system ready. Send G-code commands to begin.");
 
   // Print initial diagnostics
   Debug::printDiagnostics();
 }
 
 void loop()
 {
   // Everything is handled by the FreeRTOS tasks
   delay(1000);
 }