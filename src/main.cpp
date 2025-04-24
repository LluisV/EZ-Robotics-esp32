/**
 * @file main.cpp
 * @brief Main application with segmented motion planning for precision movement
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
FileManager *fileManager = NULL;
JobManager *jobManager = NULL;
CommunicationManager *communicationManager = NULL;
GCodeValidator *gCodeValidator = NULL;
Scheduler *scheduler = NULL;

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
          // Update last telemetry time
          lastTelemetryTime = currentTime;
        }
      }
    }
    catch (const std::exception &e)
    {
      // Log the error
      Debug::error("CommunicationTask", "Exception caught: " + String(e.what()));

      // Report the error
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
      // Log the unknown error
      Debug::error("CommunicationTask", "Unknown exception caught");

      // Report the error
      if (communicationManager)
      {
        communicationManager->sendMessage("System error: Unknown exception");
      }

      // Emergency abort any running job
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
 * Handles motion planning and motor control with priority command handling
 * This task only processes commands from the queue and has no direct communication with serial
 */
// In main.cpp, modify the motion task to minimize delays:
void motionTask(void *parameter)
{
  // Configure this task for maximum performance
  btStop();  // Disable Bluetooth to free resources
  setCpuFrequencyMhz(240);  // Use maximum CPU frequency
  
  // Pin this task to Core 1 with high priority and disable watchdog
  TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
  esp_task_wdt_delete(taskHandle);  // Disable watchdog for this task
  vTaskPrioritySet(taskHandle, configMAX_PRIORITIES - 1);  // Set to maximum priority
  
  Debug::info("MotionTask", "Task started on Core " + String(xPortGetCoreID()) + 
              " with priority " + String(uxTaskPriorityGet(taskHandle)));

  // Setup high resolution timing
  const TickType_t xFrequency = 1;  // 1ms default task frequency
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  // Command processing timing
  const unsigned long COMMAND_INTERVAL_DURING_MOTION = 20;  // Process commands less frequently during motion
  const unsigned long COMMAND_INTERVAL_IDLE = 5;  // Process commands more frequently when idle
  unsigned long lastCommandTime = 0;
  unsigned long currentTime;

  // Allocate buffers once for reuse
  String command;
  command.reserve(128);  // Pre-allocate memory for command string

  while (true)
  {
    try
    {
      // Always prioritize motor updates
      currentTime = millis();
      motorManager.update();
      
      // Process segments from the scheduler
      if (scheduler)
      {
        scheduler->executeNextSegment();
      }
      
      // Process commands from queue less frequently and when appropriate
      unsigned long commandInterval = motorManager.isAnyMotorMoving() ? 
                                     COMMAND_INTERVAL_DURING_MOTION : COMMAND_INTERVAL_IDLE;
                                     
      if (currentTime - lastCommandTime >= commandInterval)
      {
        lastCommandTime = currentTime;
        
        // Process commands from queue
        if (machineController && gCodeParser && commandQueue && !commandQueue->isEmpty())
        {
          // First check for immediate commands - these have highest priority
          String immediateCmd = commandQueue->getNextImmediate();
          if (immediateCmd.length() > 0)
          {
            // Process emergency stop immediately without parsing
            if (immediateCmd.startsWith("M112") || immediateCmd.startsWith("STOP"))
            {
              Debug::info("MotionTask", "Emergency stop command received");
              machineController->emergencyStop();
            }
            else
            {
              // Process other immediate commands
              GCodeParseResult parseResult = gCodeParser->parse(immediateCmd);
              
              // For queue full case, requeue with immediate priority
              if (parseResult == GCodeParseResult::QUEUE_FULL)
              {
                commandQueue->push(immediateCmd, IMMEDIATE);
              }
            }
          }
          // Process regular commands
          else if (!commandQueue->isEmpty() && !motorManager.isAnyMotorMoving())
          {
            command = commandQueue->pop();
            
            // Parse as G-code
            GCodeParseResult parseResult = gCodeParser->parse(command);
            
            // Handle queue full case by requeueing
            if (parseResult == GCodeParseResult::QUEUE_FULL)
            {
              commandQueue->push(command, MOTION);
            }
          }
        }
      }

      // Efficient sleep strategy based on motion state
      if (motorManager.isAnyMotorMoving())
      {
        // During motion, use vTaskDelayUntil for precise timing
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
      }
      else
      {
        // When idle, use regular delay to save power
        vTaskDelay(1);
      }
    }
    catch (const std::exception &e)
    {
      Debug::error("MotionTask", "Exception caught: " + String(e.what()));
      
      // Emergency stop on exceptions
      if (machineController) {
        machineController->emergencyStop();
      }
    }
    catch (...)
    {
      Debug::error("MotionTask", "Unknown exception caught");
      
      // Emergency stop on exceptions
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

  // 8. Initialize FileManager
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

  // 9. Initialize JobManager
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

  // 10. Initialize CommunicationManager
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
      4096,                // Stack size in words (increased for file operations)
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
  Serial.println("System ready with segmented motion planner. Send G-code commands to begin.");

  // Print initial diagnostics
  Debug::printDiagnostics();
}

void loop()
{
  // Everything is handled by the FreeRTOS tasks
  delay(1000);

  // Periodic diagnostics
  /* static unsigned long lastDiagnosticTime = 0;
   if (Debug::isEnabled() && millis() - lastDiagnosticTime > 60000)
   { // Every 60 seconds
     lastDiagnosticTime = millis();
     Debug::printDiagnostics();
   }*/
}