/**
 * @file CommandProcessor.cpp
 * @brief Implementation of the CommandProcessor class
 */

 #include "CommandProcessor.h"
 #include "MotorManager.h"
 
 CommandProcessor::CommandProcessor(MachineController* machineController)
   : machineController(machineController)
 {
 }
 
 String CommandProcessor::processInfoCommand(const String& command) {
   if (!machineController) {
     return "Error: Machine controller not available";
   }
   
   if (command.startsWith("POS")) {
     // Report current position
     std::vector<float> positions = machineController->getCurrentPosition();
     String posStr = "Position:";
     
     for (size_t i = 0; i < positions.size(); i++) {
       Motor* motor = machineController->getMotorManager()->getMotor(i);
       if (motor) {
         posStr += " " + motor->getName() + ":" + String(positions[i], 3);
       }
     }
     
     return posStr;
   } 
   else if (command.startsWith("STATUS")) {
     // Report machine status
     String status = "Status: ";
     status += machineController->isMoving() ? "MOVING" : "IDLE";
     
     // Add more status information as needed
     status += " | Absolute mode: " + String(machineController->isAbsoluteMode() ? "ON" : "OFF");
     
     return status;
   }
   else if (command.startsWith("ENDSTOPS")) {
     // Report endstop status for all motors
     String endstopStatus = "Endstop Status:";
     
     MotorManager* motorManager = machineController->getMotorManager();
     for (int i = 0; i < motorManager->getNumMotors(); i++) {
       Motor* motor = motorManager->getMotor(i);
       if (motor) {
         endstopStatus += " " + motor->getName() + ":" + 
                          (motor->isEndstopTriggered() ? "TRIGGERED" : "OPEN");
       }
     }
     
     return endstopStatus;
   }
   else if (command.startsWith("HELP")) {
     // Show available commands
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
   
   return "Unknown info command: " + command;
 }
 
 bool CommandProcessor::processImmediateCommand(const String& command) {
   if (!machineController) {
     return false;
   }
   
   if (command == "STOP" || command == "M112") {
     // Emergency stop
     machineController->emergencyStop();
     return true;
   }
   
   // Other immediate commands could be added here
   
   return false; // Unknown immediate command
 }
 
 bool CommandProcessor::processSettingCommand(const String& command) {
   if (!machineController) {
     return false;
   }
   
   if (command.startsWith("G90")) {
     // Set absolute mode
     machineController->setAbsoluteMode(true);
     return true;
   }
   else if (command.startsWith("G91")) {
     // Set relative mode
     machineController->setAbsoluteMode(false);
     return true;
   }
   
   // Other setting commands could be added here
   
   return false; // Unknown setting command
 }