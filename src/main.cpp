/**
 * @file main.cpp
 * @brief Main application with FluidNC-style motion planning
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
 #include "GCodeValidator.h"
 #include "Scheduler.h"
 #include "MotionSystem.h"
 #include "Stepping.h"
 
 // Debug configuration
 #define DEBUG_ENABLED true
 #define DEBUG_LEVEL DEBUG_INFO
 
 // Task handles for dual-core operation
 TaskHandle_t commTaskHandle = NULL;
 TaskHandle_t motionTaskHandle = NULL;
 TaskHandle_t diagnosticTaskHandle = NULL;
 
 // Command queue parameters
 #define COMMAND_QUEUE_SIZE 500
 
 // Global objects
 ConfigManager configManager;
 MotorManager motorManager;
 MachineController *machineController = NULL;
 GCodeParser *gCodeParser = NULL;
 CommandQueue *commandQueue = NULL;
 CommandProcessor *commandProcessor = NULL;
 FileManager *fileManager = NULL;
 JobManager *jobManager = NULL;
 CommunicationManager *communicationManager = NULL;
 GCodeValidator *gCodeValidator = NULL;
 Scheduler *scheduler = NULL;
 
 // Global pointer to motorManager for external modules like Planner
 MotorManager* g_motorManager = nullptr;
 
 /**
  * @brief Diagnostic task that runs on Core 0
  * Periodically logs system state and diagnostics
  */
 void diagnosticTask(void *parameter)
 {
   unsigned long lastCheckTime = 0;
   const unsigned long CHECK_INTERVAL = 1000; // Check every second
   
   Debug::info("DiagnosticTask", "Task started on Core " + String(xPortGetCoreID()));
   
   while (true)
   {
     unsigned long currentTime = millis();
     
     if (currentTime - lastCheckTime >= CHECK_INTERVAL)
     {
       lastCheckTime = currentTime;
       
       // Check stepper activity
       Debug::info("Diagnostic", "Stepper total steps: " + String(Stepping::getTotalStepCount()));
       Debug::info("Diagnostic", "Stepper timer running: " + String(Stepping::isTimerRunning() ? "YES" : "NO"));
       Debug::info("Diagnostic", "Stepper timer start pending: " + String(Stepping::isTimerStartPending() ? "YES" : "NO"));
       
       // Check for active steppers
       if (Stepper::get_current_block() != nullptr) {
         Debug::info("Diagnostic", "Active stepper block detected!");
         float velocity = Stepper::get_realtime_rate();
         Debug::info("Diagnostic", "Current stepper velocity: " + String(velocity) + " mm/min");
       } else {
         Debug::info("Diagnostic", "No active stepper block");
       }
       
       // Check motion status
       if (machineController) {
         bool isMoving = machineController->isMoving();
         Debug::info("Diagnostic", "Machine controller reports: " + String(isMoving ? "MOVING" : "NOT MOVING"));
         
         if (isMoving) {
           // Get current velocity
           float currentVelocity = machineController->getCurrentVelocity();
           Debug::info("Diagnostic", "Machine velocity: " + String(currentVelocity) + " mm/min");
           
           // Get desired velocity
           float desiredVelocity = machineController->getCurrentDesiredVelocity();
           Debug::info("Diagnostic", "Desired velocity: " + String(desiredVelocity) + " mm/min");
           
           // Get position
           std::vector<float> pos = machineController->getCurrentWorkPosition();
           String posStr = "Position:";
           for (size_t i = 0; i < pos.size(); i++) {
             posStr += " " + String(i) + "=" + String(pos[i]);
           }
           Debug::info("Diagnostic", posStr);
         }
       }
       
       // Check queue state
       if (commandQueue) {
         Debug::info("Diagnostic", "Command queue size: " + String(commandQueue->size()));
       }
       
       // Check scheduler state
       if (scheduler) {
         Debug::info("Diagnostic", "Scheduler has moves: " + String(scheduler->hasMove() ? "YES" : "NO"));
         Debug::info("Diagnostic", "Scheduler queue full: " + String(scheduler->isFull() ? "YES" : "NO"));
       }
       
       // Print memory info
       Debug::info("Diagnostic", "Free heap: " + String(ESP.getFreeHeap()) + " bytes");
     }
     
     // Let the CPU breathe
     vTaskDelay(100);
   }
 }
 
 /**
  * @brief Communication task that runs on Core 0
  * Handles serial communication, file operations, and command preprocessing
  */
 void communicationTask(void *parameter)
 {
   Debug::info("CommunicationTask", "Task started on Core " + String(xPortGetCoreID()));
 
   // Wait for system initialization
   delay(500);
 
   unsigned long lastTelemetryTime = 0;
   unsigned long telemetryInterval = 50; // Default to 20 Hz, will be updated by config
 
   // Get telemetry settings from configuration
   if (communicationManager && communicationManager->getTelemetryFrequency() > 0)
   {
     telemetryInterval = 1000 / communicationManager->getTelemetryFrequency();
   }
 
   while (true)
   {
     try
     {
       // Update communication manager
       if (communicationManager)
       {
         communicationManager->update();
       }
 
       // Update job manager
       if (jobManager)
       {
         jobManager->update();
       }
 
       // Periodic telemetry updates
       unsigned long currentTime = millis();
       if (communicationManager)
       {
         // Check if it's time for a telemetry update
         if (currentTime - lastTelemetryTime >= telemetryInterval)
         {
           communicationManager->sendPositionTelemetry(true);
           lastTelemetryTime = currentTime;
         }
       }
     }
     catch (const std::exception &e)
     {
       Debug::error("CommunicationTask", "Exception caught: " + String(e.what()));
       
       // Report error to user
       if (communicationManager)
       {
         communicationManager->sendMessage("System error: " + String(e.what()));
       }
 
       // Emergency abort any running job
       if (jobManager)
       {
         jobManager->emergencyAbortJob("Communication task exception: " + String(e.what()));
       }
     }
     catch (...)
     {
       Debug::error("CommunicationTask", "Unknown exception caught");
       
       if (communicationManager)
       {
         communicationManager->sendMessage("System error: Unknown exception");
       }
 
       if (jobManager)
       {
         jobManager->emergencyAbortJob("Communication task unknown exception");
       }
     }
 
     // Let the CPU breathe
     vTaskDelay(1);
   }
 }
 
 /**
  * @brief Motion control task that runs on Core 1
  * Handles motion planning and motor control
  */
 void motionTask(void *parameter)
 {
   Debug::info("MotionTask", "Task started on Core " + String(xPortGetCoreID()));
   
   // For regular motion processing
   const unsigned long MOTION_UPDATE_INTERVAL = 1; // 1ms for motion control
   unsigned long lastMotionUpdateTime = 0;
 
   // For command processing
   String pendingCommand = "";
   unsigned long lastCommandTime = 0;
   const unsigned long COMMAND_INTERVAL = 5; // Process commands every 5ms
 
   while (true)
   {
     try
     {
       unsigned long currentTime = millis();
       
       // Regular motion system update
       if (currentTime - lastMotionUpdateTime >= MOTION_UPDATE_INTERVAL) {
         lastMotionUpdateTime = currentTime;
         
         // Update motion system - processes segments and steps
         MotionSystem::update();
         
         // Update motor states
         motorManager.update();
       }
       
       // Process commands from queue less frequently
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
                 Debug::warning("MotionTask", "Scheduler queue full for immediate command, requeueing: " + immediateCmd);
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
             if (commandQueue->isEmpty())
               continue;
               
             String command = commandQueue->pop();
             Debug::verbose("MotionTask", "Processing command from queue: " + command);
 
             // Parse as G-code
             GCodeParseResult parseResult = gCodeParser->parse(command);
             switch (parseResult)
             {
               case GCodeParseResult::PARSE_ERROR:
                 Debug::error("MotionTask", "Failed to execute command: " + command);
 
                 if (jobManager && jobManager->isJobRunning())
                 {
                   Debug::warning("MotionTask", "Command failed during job execution");
                 }
                 break;
 
               case GCodeParseResult::QUEUE_FULL:
                 Debug::warning("MotionTask", "Scheduler queue full, requeueing command: " + command);
                 commandQueue->push(command, MOTION);
                 break;
 
               case GCodeParseResult::SUCCESS:
                 // Command successfully processed
                 break;
             }
           }
         }
       }
 
       // Only add a delay if nothing important is happening
       if (scheduler && !scheduler->hasMove()) {
         vTaskDelay(1); // Only delay when idle
       }
     }
     catch (const std::exception &e)
     {
       Debug::error("MotionTask", "Exception caught: " + String(e.what()));
       
       // Handle error - emergency stop in case of serious issues
       if (machineController) {
         machineController->emergencyStop();
       }
     }
     catch (...)
     {
       Debug::error("MotionTask", "Unknown exception caught");
       
       // Handle error - emergency stop in case of serious issues
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
   Debug::info("Main", "CNC Controller starting up with FluidNC-style motion system");
 
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
       Debug::warning("Main", "ConfigManager init failed on attempt " + String(configInitAttempts));
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
     Debug::warning("Main", "Failed to load configuration, using defaults");
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
   
   // Set global pointer for external modules
   g_motorManager = &motorManager;
 
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
 
   // 5. Initialize FluidNC-style motion system
   Debug::info("Main", "Initializing motion system");
   if (!MotionSystem::init(machineController, &motorManager, &configManager))
   {
     Debug::error("Main", "Failed to initialize motion system");
     Serial.println("Failed to initialize motion system!");
   }
   else
   {
     Debug::info("Main", "Motion system initialized successfully");
     // Explicitly register motors with the stepping system again to ensure proper setup
     Debug::info("Main", "Explicitly registering motors");
     MotionSystem::registerMotors(&motorManager);
   }
 
   // 6. Initialize scheduler
   Debug::info("Main", "Creating Scheduler");
   scheduler = new Scheduler(machineController, &motorManager, &configManager);
   if (scheduler)
   {
     scheduler->initialize();
     if (machineController)
     {
       machineController->setMotionPlanner(scheduler);
       Debug::info("Main", "Motion scheduler connected to machine controller");
     }
   }
   else
   {
     Debug::error("Main", "Failed to create scheduler");
     Serial.println("Failed to create scheduler!");
   }
 
   // 7. Initialize GCodeParser
   Debug::info("Main", "Creating GCodeParser");
   gCodeParser = new GCodeParser(machineController);
   if (!gCodeParser)
   {
     Debug::error("Main", "Failed to create GCodeParser");
     Serial.println("Failed to create G-code parser!");
   }
 
   // 8. Initialize CommandProcessor
   Debug::info("Main", "Creating CommandProcessor");
   commandProcessor = new CommandProcessor(machineController, &configManager);
   if (!commandProcessor)
   {
     Debug::error("Main", "Failed to create CommandProcessor");
     Serial.println("Failed to create command processor!");
   }
 
   // 9. Initialize FileManager
   Debug::info("Main", "Creating FileManager");
   fileManager = new FileManager();
   if (fileManager)
   {
     if (!fileManager->initialize(true))
     {
       Debug::error("Main", "Failed to initialize FileManager");
       Serial.println("Failed to initialize file system!");
     }
   }
   else
   {
     Debug::error("Main", "Failed to create FileManager");
     Serial.println("Failed to create file manager!");
   }
 
   // 10. Initialize JobManager
   Debug::info("Main", "Creating JobManager");
   jobManager = new JobManager(commandQueue, fileManager);
   if (!jobManager)
   {
     Debug::error("Main", "Failed to create JobManager");
     Serial.println("Failed to create job manager!");
   }
 
   // Initialize GCodeValidator
   Debug::info("Main", "Creating GCodeValidator");
   gCodeValidator = new GCodeValidator(fileManager, gCodeParser);
   if (!gCodeValidator)
   {
     Debug::error("Main", "Failed to create GCodeValidator");
     Serial.println("Failed to create G-code validator!");
   }
   else
   {
     // Connect validator to JobManager
     if (jobManager)
     {
       jobManager->setGCodeValidator(gCodeValidator);
       Debug::info("Main", "GCodeValidator connected to JobManager");
     }
   }
 
   // Connect CommandProcessor to JobManager
   if (jobManager && commandProcessor)
   {
     Debug::info("Main", "Connecting CommandProcessor to JobManager");
     jobManager->setCommandProcessor(commandProcessor);
   }
 
   // 11. Initialize CommunicationManager
   Debug::info("Main", "Creating CommunicationManager");
   communicationManager = new CommunicationManager(commandQueue, commandProcessor, fileManager, jobManager);
   if (communicationManager)
   {
     communicationManager->initialize(115200);
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
       8192,                // Stack size in words (increased for file operations)
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
   delay(200);
 
   // Create motion task on Core 1
   xTaskCreatePinnedToCore(
       motionTask,        // Function to implement the task
       "motionTask",      // Name of the task
       4096,              // Stack size in words
       NULL,              // Task input parameter
       10,                // Priority of the task (increased for better timing)
       &motionTaskHandle, // Task handle
       1                  // Core where the task should run
   );
 
   if (motionTaskHandle == NULL)
   {
     Debug::error("Main", "Failed to create motion task");
   }
   
   // Add delay between task creations
   delay(200);
   
   // Create diagnostic task on Core 0 with lower priority
   xTaskCreatePinnedToCore(
       diagnosticTask,     // Function to implement the task
       "diagnosticTask",   // Name of the task
       4096,               // Stack size in words
       NULL,               // Task input parameter
       1,                  // Low priority (lower than communication task)
       &diagnosticTaskHandle, // Task handle
       0                   // Core where the task should run
   );
   
   if (diagnosticTaskHandle == NULL)
   {
     Debug::error("Main", "Failed to create diagnostic task");
   }
 
   Debug::info("Main", "System initialization complete with FluidNC-style motion system");
   Serial.println("System ready with FluidNC-style motion system. Send G-code commands to begin.");
   
   // Send some additional diagnostic information
   Debug::info("Main", "Motor count: " + String(motorManager.getNumMotors()));
   Debug::info("Main", "Command queue size: " + String(COMMAND_QUEUE_SIZE));
   
   // DO NOT immediately wake the stepper system - let it be handled by the commands
   // This follows the FluidNC pattern of only activating when needed
   Debug::info("Main", "Stepper system will be activated when needed");
 }
 
 void loop()
{
  // Main loop doesn't need to do much as the work is done in the FreeRTOS tasks
  // But we'll add a heartbeat check to ensure system is responsive
  static unsigned long lastHeartbeat = 0;
  static int heartbeatCounter = 0;
  static unsigned long timerStartRequestTime = 0;
  
  // Process any pending timer starts from the main loop context
  // This is critical to avoid watchdog issues
  Debug::verbose("Main", "Checking for pending timer starts...");
  Stepping::processDelayedStart();
  Debug::verbose("Main", "Processed pending timer starts");
  
  // Check if timer start has been pending for too long
  if (Stepping::isTimerStartPending()) {
    if (timerStartRequestTime == 0) {
      timerStartRequestTime = millis();
      Debug::info("Main", "Timer start request detected at " + String(timerStartRequestTime));
    } else if (millis() - timerStartRequestTime > 5000) {
      // Timer has been pending for too long, force it
      Debug::warning("Main", "Timer start has been pending for too long, forcing start");
      Stepping::forceTimerStart();
      timerStartRequestTime = 0;
    }
  } else {
    timerStartRequestTime = 0;
  }
  
  unsigned long currentTime = millis();
  if (currentTime - lastHeartbeat >= 5000) {  // Every 5 seconds
    lastHeartbeat = currentTime;
    heartbeatCounter++;
    
    // Print basic system status
    Debug::info("Main", "Heartbeat #" + String(heartbeatCounter) + 
                      " - Free heap: " + String(ESP.getFreeHeap()) + " bytes");
    
    // Check for stepper activity
    bool stepsActive = Stepping::getTotalStepCount() > 0;
    Debug::info("Main", "Steps executed: " + String(Stepping::getTotalStepCount()) + 
                      " - Stepper timer running: " + String(Stepping::isTimerRunning() ? "YES" : "NO"));
    
    // Force step counts to be printed
    Stepping::printStepCounts();
    
    // Check if scheduler has moves but stepper isn't active - could indicate a problem
    if (scheduler && scheduler->hasMove() && !Stepping::isTimerRunning() && !Stepping::isTimerStartPending()) {
      Debug::warning("Main", "Scheduler has moves but stepper timer is not running or pending!");
      
      // Try to wake up planner instead of directly activating stepper
      // This follows the FluidNC pattern
      Planner::update_velocity_profile_parameters();
      
      // Also try to force wake up the stepper
      Stepper::wake_up();
      
      Debug::info("Main", "Attempted to trigger planner recalculation and wake up stepper");
    }
  }
  
  // Give other tasks time to run - keep this short to process delayed starts quickly
  delay(10);
}