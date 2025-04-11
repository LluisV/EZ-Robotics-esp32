/**
 * @file CommandProcessor.cpp
 * @brief Implementation of the CommandProcessor class with debug functionality
 */

#include "CommandProcessor.h"
#include "MotorManager.h"
#include "Debug.h"

CommandProcessor::CommandProcessor(MachineController *machineController, ConfigManager *configManager)
    : machineController(machineController), configManager(configManager)
{
  Debug::info("CommandProcessor", "Initialized");
}

String CommandProcessor::processInfoCommand(const String &command)
{
  Debug::verbose("CommandProcessor", "Processing info command: " + command);

  if (!machineController)
  {
    Debug::error("CommandProcessor", "Machine controller not available");
    return "Error: Machine controller not available";
  }

  // Remove the leading '?' if present
  String cleanCmd = command;
  if (cleanCmd.startsWith("?"))
  {
    cleanCmd = cleanCmd.substring(1);
  }
  cleanCmd.trim();
  cleanCmd.toUpperCase();

  Debug::verbose("CommandProcessor", "Cleaned command: " + cleanCmd);

  if (cleanCmd.startsWith("POS"))
  {
    Debug::timerStart("CommandProcessor", "GetPosition");
    std::vector<float> workPositions = machineController->getCurrentWorkPosition();
    std::vector<float> worldPositions = machineController->getCurrentWorldPosition();
    std::vector<float> workOffset = machineController->getWorkOffset();
    Debug::timerEnd("CommandProcessor", "GetPosition");

    String posStr = "";

    for (size_t i = 0; i < workPositions.size(); i++)
    {
      Motor *motor = machineController->getMotorManager()->getMotor(i);
      if (motor)
      {
        posStr += " " + motor->getName() + ":" + String(workPositions[i], 3) +
                  " (" + motor->getName() + "_world:" + String(worldPositions[i], 3) + ")";
      }
    }

    return "<RESPONSE:POS>" + posStr;
  }
  else if (cleanCmd.startsWith("STATUS"))
  {
    // Report machine status
    String status = "";
    status += machineController->isMoving() ? "MOVING" : "IDLE";

    // Add more status information as needed
    status += " | Absolute mode: " + String(machineController->isAbsoluteMode() ? "ON" : "OFF");

    return "<RESPONSE:STATUS>" + status;
  }
  else if (cleanCmd.startsWith("ENDSTOPS"))
  {
    // Report endstop status for all motors
    String endstopStatus = "";

    MotorManager *motorManager = machineController->getMotorManager();
    for (int i = 0; i < motorManager->getNumMotors(); i++)
    {
      Motor *motor = motorManager->getMotor(i);
      if (motor)
      {
        bool triggered = motor->isEndstopTriggered();
        endstopStatus += " " + motor->getName() + ":" +
                         (triggered ? "TRIGGERED" : "OPEN");

        // Log individual endstop states for debugging
        Debug::verbose("CommandProcessor", "Endstop " + motor->getName() +
                                               " is " + (triggered ? "TRIGGERED" : "OPEN"));
      }
    }

    Debug::info("CommandProcessor", "Endstop report: " + endstopStatus);
    return "<RESPONSE:ENDSTOPS>" + endstopStatus;
  }
  else if (cleanCmd.startsWith("HELP"))
  {
    // Show available commands
    Debug::info("CommandProcessor", "Help command requested");
    String helpText = "Available commands:\n";
    helpText += "Regular G-codes: G0, G1, G28, G90, G91, G92, etc.\n";
    helpText += "Special commands:\n";
    helpText += "  !STOP or !M112 - Emergency stop\n";
    helpText += "  ?POS - Current position\n";
    helpText += "  ?STATUS - Machine status\n";
    helpText += "  ?ENDSTOPS - Endstop status\n";
    helpText += "  ?CONFIG - Machine configuration\n";
    helpText += "  ?HELP - This help text\n";
    helpText += "  $G90/$G91 - Set absolute/relative mode\n";
    helpText += "File commands:\n";
    helpText += "  @LIST - List all G-code files\n";
    helpText += "  @SEND filename - Send a file to client\n";
    helpText += "  @RECEIVE filename size - Receive a file from client\n";
    helpText += "  @DELETE filename - Delete a file\n";
    helpText += "  @RUN filename - Run a G-code file\n";
    helpText += "  @PAUSE - Pause current job\n";
    helpText += "  @RESUME - Resume current job\n";
    helpText += "  @STOP - Stop current job\n";

    return "<RESPONSE:HELP>" + helpText;
  }
  else if (cleanCmd.startsWith("DEBUG"))
  {
    // Process debug commands
    if (cleanCmd == "DEBUG ON")
    {
      Debug::begin(true);
      Debug::info("CommandProcessor", "Debug enabled via command");
      return "<RESPONSE:DEBUG_MODE_CHANGE> Debug mode enabled";
    }
    else if (cleanCmd == "DEBUG OFF")
    {
      Debug::info("CommandProcessor", "Debug disabled via command");
      Debug::begin(false);
      return "<RESPONSE:DEBUG_MODE_CHANGE> Debug mode disabled";
    }
    else if (cleanCmd.startsWith("DEBUG LEVEL"))
    {
      // Set debug level
      int level = cleanCmd.substring(11).toInt();
      level = constrain(level, 0, 3);
      Debug::setLevel(level);
      return "<RESPONSE:DEBUG_MODE_CHANGE> Debug level set to " + String(level);
    }
    else if (cleanCmd == "DEBUG DIAG")
    {
      Debug::printDiagnostics();
      return "<RESPONSE:DEBUG_MODE_CHANGE> Diagnostics printed to debug output";
    }
  }
  else if (cleanCmd.startsWith("CONFIG"))
  {
    // Report current machine configuration
    if (!machineController)
    {
      Debug::error("CommandProcessor", "Machine controller not available for config report");
      return "<RESPONSE:DEBUG_MODE_CHANGE> Error: Machine controller not available";
    }

    ConfigManager *configManager = machineController->getConfigManager();
    if (!configManager)
    {
      Debug::error("CommandProcessor", "Config manager not available");
      return "<RESPONSE:DEBUG_MODE_CHANGE> Error: Config manager not available";
    }

    // Get machine configuration
    const MachineConfig &machineConfig = configManager->getMachineConfig();

    String configStr = "Machine Configuration:\n";
    configStr += "  Name: " + machineConfig.machineName + "\n";
    configStr += "  Default Feedrate: " + String(machineConfig.defaultFeedrate) + " mm/min\n";
    configStr += "  Max Feedrate: " + String(machineConfig.maxFeedrate) + " mm/min\n";
    configStr += "  Junction Deviation: " + String(machineConfig.junctionDeviation, 4) + " mm\n";
    configStr += "  Arc Tolerance: " + String(machineConfig.arcTolerance, 4) + " mm\n\n";

    configStr += "Motor Configurations:\n";

    // Get motor configurations
    int numMotors = configManager->getNumMotors();
    for (int i = 0; i < numMotors; i++)
    {
      const MotorConfig *motorConfig = configManager->getMotorConfig(i);
      if (motorConfig)
      {
        configStr += "  Motor " + motorConfig->name + ":\n";
        configStr += "    Type: " + String(motorConfig->type == LINEAR_AXIS ? "Linear" : "Angular") + "\n";
        configStr += "    Steps/Rev: " + String(motorConfig->stepsPerRev) + "\n";
        configStr += "    Max Speed: " + String(motorConfig->maxSpeed) + " steps/sec\n";
        configStr += "    Max Acceleration: " + String(motorConfig->maxAcceleration) + " steps/sec²\n";
        configStr += "    Min Position: " + String(motorConfig->minPosition) + " mm\n";
        configStr += "    Max Position: " + String(motorConfig->maxPosition) + " mm\n";
        configStr += "    Endstop Position: " + String(motorConfig->endstopPosition) + " mm\n";
        configStr += "    Home Position: " + String(motorConfig->homePosition) + " mm\n";
        configStr += "    Position Inverted: " + String(motorConfig->invertPosition ? "Yes" : "No") + "\n";
        configStr += "    Home Speed: " + String(motorConfig->homeSpeed) + " steps/sec\n";
        configStr += "    Homing Direction: " + String(motorConfig->homingDirection > 0 ? "Positive" : "Negative") + "\n";
        configStr += "    Backoff Distance: " + String(motorConfig->backoffDistance) + " mm\n\n";
      }
    }

    Debug::info("CommandProcessor", "Configuration report requested");
    return "<RESPONSE:DEBUG_MODE_CHANGE>" + configStr;
  }

  Debug::warning("CommandProcessor", "Unknown info command: " + command);
  return "<RESPONSE:DEBUG_MODE_CHANGE> Unknown info command: " + command;
}

bool CommandProcessor::processImmediateCommand(const String &command)
{
  Debug::info("CommandProcessor", "Processing immediate command: " + command);

  if (!machineController)
  {
    Debug::error("CommandProcessor", "Machine controller not available for immediate command");
    return false;
  }

  // Convert to uppercase for case-insensitive comparison
  String cmdUpper = command;
  cmdUpper.trim();
  cmdUpper.toUpperCase();

  if (cmdUpper == "M112" || cmdUpper == "STOP")
  {
    // Emergency stop
    Debug::error("CommandProcessor", "EMERGENCY STOP REQUESTED");
    machineController->emergencyStop();
    return true;
  }

  // For other immediate commands, parse as G-code
  // This is handled by the G-code parser in the motion task
  return true;
}

bool CommandProcessor::processSettingCommand(const String &command)
{
  Debug::info("CommandProcessor", "Processing setting command: " + command);

  if (!machineController)
  {
    Debug::error("CommandProcessor", "Machine controller not available for setting command");
    return false;
  }

  if (command.startsWith("G90"))
  {
    // Set absolute mode
    Debug::info("CommandProcessor", "Setting absolute mode");
    machineController->setAbsoluteMode(true);
    return true;
  }
  else if (command.startsWith("G91"))
  {
    // Set relative mode
    Debug::info("CommandProcessor", "Setting relative mode");
    machineController->setAbsoluteMode(false);
    return true;
  }

  // Other setting commands could be added here
  Debug::warning("CommandProcessor", "Unknown setting command: " + command);
  return false; // Unknown setting command
}