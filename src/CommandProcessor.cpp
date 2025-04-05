/**
 * @file CommandProcessor.cpp
 * @brief Implementation of the CommandProcessor class with debug functionality
 */

 #include "CommandProcessor.h"
 #include "MotorManager.h"
 #include "Debug.h"
 
 CommandProcessor::CommandProcessor(MachineController* machineController)
   : machineController(machineController)
 {
   Debug::info("CommandProcessor", "Initialized");
 }
 
 String CommandProcessor::processInfoCommand(const String& command) {
  Debug::verbose("CommandProcessor", "Processing info command: " + command);
  
  if (!machineController) {
    Debug::error("CommandProcessor", "Machine controller not available");
    return "Error: Machine controller not available";
  }
  
  // Remove the leading '?' if present
  String cleanCmd = command;
  if (cleanCmd.startsWith("?")) {
    cleanCmd = cleanCmd.substring(1);
  }
  cleanCmd.trim();
  cleanCmd.toUpperCase();
  
  Debug::verbose("CommandProcessor", "Cleaned command: " + cleanCmd);
  
  if (cleanCmd.startsWith("POS")) {
    // Report current position
    Debug::timerStart("CommandProcessor", "GetPosition");
    std::vector<float> positions = machineController->getCurrentPosition();
    Debug::timerEnd("CommandProcessor", "GetPosition");
    
    String posStr = "Position:";
    
    for (size_t i = 0; i < positions.size(); i++) {
      Motor* motor = machineController->getMotorManager()->getMotor(i);
      if (motor) {
        posStr += " " + motor->getName() + ":" + String(positions[i], 3);
      }
    }
    
    Debug::info("CommandProcessor", "Position report: " + posStr);
    return posStr;
  } 
  else if (cleanCmd.startsWith("STATUS")) {
    // Report machine status
    String status = "Status: ";
    status += machineController->isMoving() ? "MOVING" : "IDLE";
    
    // Add more status information as needed
    status += " | Absolute mode: " + String(machineController->isAbsoluteMode() ? "ON" : "OFF");
    
    Debug::info("CommandProcessor", "Status report: " + status);
    return status;
  }
  else if (cleanCmd.startsWith("ENDSTOPS")) {
    // Report endstop status for all motors
    String endstopStatus = "Endstop Status:";
    
    MotorManager* motorManager = machineController->getMotorManager();
    for (int i = 0; i < motorManager->getNumMotors(); i++) {
      Motor* motor = motorManager->getMotor(i);
      if (motor) {
        bool triggered = motor->isEndstopTriggered();
        endstopStatus += " " + motor->getName() + ":" + 
                       (triggered ? "TRIGGERED" : "OPEN");
        
        // Log individual endstop states for debugging
        Debug::verbose("CommandProcessor", "Endstop " + motor->getName() + 
                      " is " + (triggered ? "TRIGGERED" : "OPEN"));
      }
    }
    
    Debug::info("CommandProcessor", "Endstop report: " + endstopStatus);
    return endstopStatus;
  }
  else if (cleanCmd.startsWith("HELP")) {
    // Show available commands
    Debug::info("CommandProcessor", "Help command requested");
    String helpText = "Available commands:\n";
    helpText += "Regular G-codes: G0, G1, G28, G90, G91, G92, etc.\n";
    helpText += "Special commands:\n";
    helpText += "  !STOP or !M112 - Emergency stop\n";
    helpText += "  ?POS - Current position\n";
    helpText += "  ?STATUS - Machine status\n";
    helpText += "  ?ENDSTOPS - Endstop status\n";
    helpText += "  ?HELP - This help text\n";
    helpText += "  $G90/$G91 - Set absolute/relative mode\n";
    
    return helpText;
  }
  else if (cleanCmd.startsWith("DEBUG")) {
    // Process debug commands
    if (cleanCmd == "DEBUG ON") {
      Debug::begin(true);
      Debug::info("CommandProcessor", "Debug enabled via command");
      return "Debug mode enabled";
    }
    else if (cleanCmd == "DEBUG OFF") {
      Debug::info("CommandProcessor", "Debug disabled via command");
      Debug::begin(false);
      return "Debug mode disabled";
    }
    else if (cleanCmd.startsWith("DEBUG LEVEL")) {
      // Set debug level
      int level = cleanCmd.substring(11).toInt();
      level = constrain(level, 0, 3);
      Debug::setLevel(level);
      return "Debug level set to " + String(level);
    }
    else if (cleanCmd == "DEBUG DIAG") {
      Debug::printDiagnostics();
      return "Diagnostics printed to debug output";
    }
  }
  
  Debug::warning("CommandProcessor", "Unknown info command: " + command);
  return "Unknown info command: " + command;
}
 
 bool CommandProcessor::processImmediateCommand(const String& command) {
   Debug::info("CommandProcessor", "Processing immediate command: " + command);
   
   if (!machineController) {
     Debug::error("CommandProcessor", "Machine controller not available for immediate command");
     return false;
   }
   
   if (command == "STOP" || command == "M112") {
     // Emergency stop
     Debug::error("CommandProcessor", "EMERGENCY STOP REQUESTED");
     machineController->emergencyStop();
     return true;
   }
   
   // Other immediate commands could be added here
   Debug::warning("CommandProcessor", "Unknown immediate command: " + command);
   return false; // Unknown immediate command
 }
 
 bool CommandProcessor::processSettingCommand(const String& command) {
   Debug::info("CommandProcessor", "Processing setting command: " + command);
   
   if (!machineController) {
     Debug::error("CommandProcessor", "Machine controller not available for setting command");
     return false;
   }
   
   if (command.startsWith("G90")) {
     // Set absolute mode
     Debug::info("CommandProcessor", "Setting absolute mode");
     machineController->setAbsoluteMode(true);
     return true;
   }
   else if (command.startsWith("G91")) {
     // Set relative mode
     Debug::info("CommandProcessor", "Setting relative mode");
     machineController->setAbsoluteMode(false);
     return true;
   }
   
   // Other setting commands could be added here
   Debug::warning("CommandProcessor", "Unknown setting command: " + command);
   return false; // Unknown setting command
 }