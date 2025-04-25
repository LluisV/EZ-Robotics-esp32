/**
 * @file GRBLCommunicationManager.cpp
 * @brief Implementation of the GRBLCommunicationManager class
 */

#include "CommunicationManager.h"
#include "MachineController.h"
#include <ArduinoJson.h>

// GRBL protocol defines
#define GRBL_VERSION "1.1f.20230925"
#define GRBL_LINE_FEED_CHAR '\n'
#define GRBL_CARRIAGE_RETURN_CHAR '\r'
#define GRBL_REALTIME_STATUS '?'
#define GRBL_REALTIME_FEEDHOLD '!'
#define GRBL_REALTIME_RESUME '~'
#define GRBL_REALTIME_RESET 0x18 // Ctrl-X
#define GRBL_REALTIME_SAFETY_DOOR 0x84

// Response strings
#define GRBL_RESPONSE_OK "ok"
#define GRBL_RESPONSE_ERROR "error:"
#define GRBL_RESPONSE_ALARM "ALARM:"
#define GRBL_RESPONSE_INITIALIZED "Grbl " GRBL_VERSION " ['$' for help]"

GRBLCommunicationManager::GRBLCommunicationManager(CommandQueue *commandQueue, CommandProcessor *commandProcessor,
                                                   FileManager *fileManager, JobManager *jobManager)
    : commandQueue(commandQueue),
      commandProcessor(commandProcessor),
      fileManager(fileManager),
      jobManager(jobManager),
      lineBufferIndex(0),
      autoReportEnabled(false),
      autoReportIntervalMs(1000), // Default to 1 second
      lastAutoReportTime(0),
      feedHoldActive(false),
      lineNumber(0),
      awaitingAck(false)
{
  // Initialize machine controller from command processor
  if (commandProcessor)
  {
    machineController = commandProcessor->getMachineController();
  }
  else
  {
    machineController = nullptr;
  }

  // Initialize line buffer
  resetLineBuffer();

  // Get CPU monitor instance
  cpuMonitor = CPUMonitor::getInstance();
  if (cpuMonitor)
  {
    cpuMonitor->begin();
  }
}

bool GRBLCommunicationManager::initialize(unsigned long baudRate)
{
  Serial.begin(baudRate);
  delay(100); // Give serial a moment to initialize

  Debug::info("GRBLCommunicationManager", "Serial initialized at " + String(baudRate) + " baud");

  // Send GRBL welcome message to let GCode senders know we're ready
  Serial.println(GRBL_RESPONSE_INITIALIZED);

  return true;
}

bool GRBLCommunicationManager::update()
{
  // Check for CPU usage update
  if (cpuMonitor)
  {
    cpuMonitor->update();
  }

  // Handle incoming data
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    // Check for realtime commands first
    if (c == GRBL_REALTIME_STATUS || c == GRBL_REALTIME_FEEDHOLD || c == GRBL_REALTIME_RESUME ||
        c == GRBL_REALTIME_RESET || c == GRBL_REALTIME_SAFETY_DOOR)
    {
      processRealtimeCommand(c);
      continue; // Skip normal line processing for realtime commands
    }

    // Normal line processing
    if (c == GRBL_LINE_FEED_CHAR || c == GRBL_CARRIAGE_RETURN_CHAR)
    {
      // End of line detected
      if (lineBufferIndex > 0)
      {
        // Null terminate the string
        lineBuffer[lineBufferIndex] = '\0';

        // Process the line
        String line = String(lineBuffer);
        processLine(line);

        // Reset buffer for next line
        resetLineBuffer();
      }
    }
    else if (lineBufferIndex < GRBL_LINE_BUFFER_SIZE - 1)
    {
      // Add character to buffer
      lineBuffer[lineBufferIndex++] = c;
    }
    else
    {
      // Line too long, discard and reset
      Debug::warning("GRBLCommunicationManager", "Line too long, discarding");
      sendMessage("error:1"); // Line exceeds GRBL buffer
      resetLineBuffer();
    }
  }

  // Handle automatic status reporting if enabled
  if (autoReportEnabled && autoReportIntervalMs > 0)
  {
    unsigned long currentTime = millis();
    if (currentTime - lastAutoReportTime >= autoReportIntervalMs)
    {
      sendStatusReport(false);
      lastAutoReportTime = currentTime;
    }
  }

  // Handle command acknowledgment
  if (awaitingAck)
  {
    // In a real implementation, check the state of command execution
    // and send acknowledgment when appropriate.
    // For now, we'll just send it immediately for basic functionality.
    sendAcknowledgment(true);
    awaitingAck = false;
  }

  return true;
}

void GRBLCommunicationManager::sendMessage(const String &message)
{
  Serial.println(message);
}

void GRBLCommunicationManager::processLine(const String &line)
{
  // Skip empty lines
  if (line.length() == 0)
  {
    return;
  }

  Debug::verbose("GRBLCommunicationManager", "Processing line: " + line);

  // Check for system commands (starting with $)
  if (line.startsWith("$"))
  {
    if (handleSystemCommand(line))
    {
      return; // Command was handled
    }
  }

  // Queue as a regular G-code command
  if (commandQueue->push(line, MOTION))
  {
    Debug::verbose("GRBLCommunicationManager", "Queued G-code command: " + line);
    awaitingAck = true; // Mark that we need to send an acknowledgment
  }
  else
  {
    // Queue is full
    sendAcknowledgment(false, 4); // Cannot be queued, buffer full
  }
}

void GRBLCommunicationManager::processRealtimeCommand(char c)
{
  switch (c)
  {
  case GRBL_REALTIME_STATUS:
    // Send status report
    sendStatusReport();
    break;

  case GRBL_REALTIME_FEEDHOLD:
    // Pause job
    Debug::info("GRBLCommunicationManager", "Feedhold requested");
    if (jobManager && jobManager->isJobRunning())
    {
      jobManager->pauseJob();
      feedHoldActive = true;
    }
    else if (machineController)
    {
      machineController->pauseMovement();
      feedHoldActive = true;
    }
    break;

  case GRBL_REALTIME_RESUME:
    // Resume job
    Debug::info("GRBLCommunicationManager", "Resume requested");
    if (feedHoldActive)
    {
      if (jobManager && jobManager->getJobStatus() == JOB_PAUSED)
      {
        jobManager->resumeJob();
      }
      else if (machineController)
      {
        machineController->resumeMovement();
      }
      feedHoldActive = false;
    }
    break;

  case GRBL_REALTIME_RESET:
    // Soft reset
    Debug::info("GRBLCommunicationManager", "Soft reset requested");

    // Stop job and clear queues
    if (jobManager)
    {
      jobManager->stopJob();
    }

    if (commandQueue)
    {
      commandQueue->clear();
    }

    // Stop all motors
    if (commandProcessor && commandProcessor->getMachineController())
    {
      commandProcessor->getMachineController()->emergencyStop();
    }

    // Send reset confirmation
    sendMessage(GRBL_RESPONSE_INITIALIZED);
    break;

  case GRBL_REALTIME_SAFETY_DOOR:
    // Safety door - same as feedhold for now
    Debug::info("GRBLCommunicationManager", "Safety door triggered");
    if (jobManager && jobManager->isJobRunning())
    {
      jobManager->pauseJob();
      feedHoldActive = true;
    }
    break;

  default:
    // Unhandled realtime command
    Debug::warning("GRBLCommunicationManager", "Unhandled realtime command: " + String(c));
    break;
  }
}

void GRBLCommunicationManager::resetLineBuffer()
{
  memset(lineBuffer, 0, GRBL_LINE_BUFFER_SIZE);
  lineBufferIndex = 0;
}

void GRBLCommunicationManager::sendStatusReport(bool detailed)
{
  String status = composeStatusReport();
  sendMessage(status);
}

void GRBLCommunicationManager::setStatusReportInterval(unsigned long intervalMs)
{
  autoReportEnabled = (intervalMs > 0);
  autoReportIntervalMs = intervalMs;

  Debug::info("GRBLCommunicationManager", "Auto reporting " +
                                              String(autoReportEnabled ? "enabled" : "disabled") +
                                              " with interval " + String(autoReportIntervalMs) + "ms");
}

void GRBLCommunicationManager::sendAcknowledgment(bool success, int errorCode)
{
  if (success)
  {
    Serial.println(GRBL_RESPONSE_OK);
  }
  else
  {
    Serial.println(GRBL_RESPONSE_ERROR + String(errorCode));
  }
}

String GRBLCommunicationManager::composeStatusReport()
{
  // Get machine controller
  MachineController *machine = nullptr;
  if (commandProcessor)
  {
    machine = commandProcessor->getMachineController();
  }

  // Default status if no machine controller
  String machineStatus = "Idle";
  String machinePosition = "0.000,0.000,0.000";
  String workPosition = "0.000,0.000,0.000";
  String feedRate = "0";
  String spindleSpeed = "0";

  // Get job status if applicable
  if (jobManager && jobManager->isJobRunning())
  {
    machineStatus = "Run";
    if (feedHoldActive || jobManager->getJobStatus() == JOB_PAUSED)
    {
      machineStatus = "Hold";
    }
  }
  else if (machine && machine->isMoving())
  {
    machineStatus = "Run";
    if (feedHoldActive)
    {
      machineStatus = "Hold";
    }
  }

  // Get position information if available
  if (machine)
  {
    std::vector<float> worldPos = machine->getCurrentWorldPosition();
    std::vector<float> workPos = machine->getCurrentWorkPosition();

    // Format position strings with 3 decimal places
    machinePosition = "";
    workPosition = "";

    for (size_t i = 0; i < worldPos.size() && i < 3; i++)
    {
      machinePosition += String(worldPos[i], 3);
      if (i < 2)
        machinePosition += ",";
    }

    for (size_t i = 0; i < workPos.size() && i < 3; i++)
    {
      workPosition += String(workPos[i], 3);
      if (i < 2)
        workPosition += ",";
    }

    // Get feedrate
    feedRate = String(machine->getCurrentFeedrate(), 0);
  }

  // Format GRBL status report
  String report = "<" + machineStatus + "|MPos:" + machinePosition + "|WPos:" + workPosition;

  // Add feedrate if running
  if (machineStatus == "Run" || machineStatus == "Hold")
  {
    report += "|FS:" + feedRate + "," + spindleSpeed;
  }

  // Add buffer information (how many blocks available in planner)
  if (machine && machine->getMotionPlanner())
  {
    int bufferState = 15; // Approximate value, replace with actual planner free blocks
    report += "|Bf:" + String(bufferState);
  }

  // Add line number if job is running
  if (jobManager && jobManager->isJobRunning())
  {
    report += "|Ln:" + String(lineNumber);
  }

  // Close status report
  report += ">";

  return report;
}

String GRBLCommunicationManager::composeAlarmMessage(int alarmCode)
{
  return GRBL_RESPONSE_ALARM + String(alarmCode);
}

bool GRBLCommunicationManager::handleSystemCommand(const String &line)
{
  // Handle $ commands for GRBL settings and information
  if (line == "$")
  {
    // Display help
    sendMessage("GRBL ESP32 Compatible Controller");
    sendMessage("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H ~]");
    return true;
  }
  else if (line == "$$")
  {
    // Display settings
    sendMessage("$0=10 (Step pulse time, microseconds)");
    sendMessage("$1=25 (Step idle delay, milliseconds)");
    sendMessage("$2=0 (Step pulse invert, mask)");
    sendMessage("$3=0 (Step direction invert, mask)");
    sendMessage("$4=0 (Invert step enable pin, boolean)");
    sendMessage("$5=0 (Invert limit pins, boolean)");
    sendMessage("$6=0 (Invert probe pin, boolean)");
    sendMessage("$10=1 (Status report options, mask)");
    sendMessage("$11=0.010 (Junction deviation, millimeters)");
    sendMessage("$12=0.002 (Arc tolerance, millimeters)");
    sendMessage("$13=0 (Report in inches, boolean)");
    sendMessage("$20=0 (Soft limits enable, boolean)");
    sendMessage("$21=0 (Hard limits enable, boolean)");
    sendMessage("$22=1 (Homing cycle enable, boolean)");
    sendMessage("$23=3 (Homing direction invert, mask)");
    sendMessage("$24=25.000 (Homing locate feed rate, mm/min)");
    sendMessage("$25=500.000 (Homing search seek rate, mm/min)");
    sendMessage("$26=250 (Homing switch debounce delay, milliseconds)");
    sendMessage("$27=1.000 (Homing switch pull-off distance, millimeters)");
    sendMessage("$30=1000 (Maximum spindle speed, RPM)");
    sendMessage("$31=0 (Minimum spindle speed, RPM)");
    sendMessage("$32=0 (Laser mode enable, boolean)");
    sendMessage("$100=250.000 (X-axis travel resolution, step/mm)");
    sendMessage("$101=250.000 (Y-axis travel resolution, step/mm)");
    sendMessage("$102=250.000 (Z-axis travel resolution, step/mm)");
    sendMessage("$110=500.000 (X-axis maximum rate, mm/min)");
    sendMessage("$111=500.000 (Y-axis maximum rate, mm/min)");
    sendMessage("$112=500.000 (Z-axis maximum rate, mm/min)");
    sendMessage("$120=10.000 (X-axis acceleration, mm/sec^2)");
    sendMessage("$121=10.000 (Y-axis acceleration, mm/sec^2)");
    sendMessage("$122=10.000 (Z-axis acceleration, mm/sec^2)");
    sendMessage("$130=200.000 (X-axis maximum travel, millimeters)");
    sendMessage("$131=200.000 (Y-axis maximum travel, millimeters)");
    sendMessage("$132=200.000 (Z-axis maximum travel, millimeters)");
    return true;
  }
  else if (line == "$#")
  {
    // Display coordinate parameters
    sendMessage("[G54:0.000,0.000,0.000]");
    sendMessage("[G55:0.000,0.000,0.000]");
    sendMessage("[G56:0.000,0.000,0.000]");
    sendMessage("[G57:0.000,0.000,0.000]");
    sendMessage("[G58:0.000,0.000,0.000]");
    sendMessage("[G59:0.000,0.000,0.000]");
    sendMessage("[G28:0.000,0.000,0.000]");
    sendMessage("[G30:0.000,0.000,0.000]");
    sendMessage("[G92:0.000,0.000,0.000]");
    sendMessage("[TLO:0.000]");
    sendMessage("[PRB:0.000,0.000,0.000:0]");
    return true;
  }
  else if (line == "$G")
  {
    // Display active G-code state
    sendMessage("[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]");
    return true;
  }
  else if (line == "$I")
  {
    // Display build info
    sendMessage("[VER:" GRBL_VERSION ":ESP32]");
    sendMessage("[OPT:V,15,128]");
    return true;
  }
  else if (line == "$N")
  {
    // Display startup blocks
    sendMessage("[MSG:No startup line saved]");
    return true;
  }
  else if (line.startsWith("$H"))
  {
    // Home axes
    Debug::info("GRBLCommunicationManager", "Homing command received");

    MachineController *machine = nullptr;
    if (commandProcessor)
    {
      machine = commandProcessor->getMachineController();
    }

    if (machine)
    {
      machine->homeAll();
      sendAcknowledgment(true);
    }
    else
    {
      sendAcknowledgment(false, 5); // Homing failed
    }
    return true;
  }
  else if (line.startsWith("$X"))
  {
    // Kill alarm lock
    Debug::info("GRBLCommunicationManager", "Kill alarm lock received");

    // Just acknowledge, used to reset after alarms
    sendAcknowledgment(true);
    return true;
  }
  else if (line.startsWith("$10="))
  {
    // Set status report options
    int value = line.substring(4).toInt();
    Debug::info("GRBLCommunicationManager", "Status report options set to: " + String(value));
    sendAcknowledgment(true);
    return true;
  }
  else if (line.startsWith("$110="))
  {
    // Set status report interval
    float value = line.substring(5).toFloat();
    setStatusReportInterval(value * 1000); // Convert to milliseconds
    sendAcknowledgment(true);
    return true;
  }

  // Handle other $ commands as needed

  // If we get here, the command was not recognized
  Debug::warning("GRBLCommunicationManager", "Unknown system command: " + line);
  sendAcknowledgment(false, 20); // Unsupported or invalid command
  return true;
}