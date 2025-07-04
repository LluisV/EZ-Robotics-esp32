/**
 * @file CommunicationManager.cpp
 * @brief Implementation of the communication manager
 */

 #include "CommunicationManager.h"
 #include <ArduinoJson.h>
 
 CommunicationManager::CommunicationManager(LineBuffer *lineBuffer, MachineController *machineController) :
     lineBuffer(lineBuffer),
     machineController(machineController),
     serialLineBufferIndex(0),
     telemetryEnabled(true),
     telemetryInterval(100),
     lastTelemetryTime(0),
     lastStatusReportTime(0),
     statusReportInterval(250)
 {
     // Initialize line buffer
     memset(serialLineBuffer, 0, SERIAL_LINE_BUFFER_SIZE);
     
     // Get CPU monitor instance
     cpuMonitor = CPUMonitor::getInstance();
     if (cpuMonitor) {
         cpuMonitor->begin();
     }
     
     // Pre-allocate buffer to avoid memory fragmentation
     telemetryBuffer.reserve(512);
     lastReportedPosition.reserve(3);
     
     Debug::info("CommunicationManager", "Initialized");
 }
 
 bool CommunicationManager::initialize(unsigned long baudRate)
 {
     Serial.begin(baudRate);
     delay(100); // Give serial time to initialize
     Debug::info("CommunicationManager", "Serial initialized at " + String(baudRate) + " baud");
     return true;
 }
 
 bool CommunicationManager::update()
 {
     // Update CPU monitor
     if (cpuMonitor) {
         cpuMonitor->update();
     }
     
     // Process incoming data
     while (Serial.available() > 0) {
         char c = Serial.read();
         
         // Check for realtime commands first (like '?' for status report)
         if (processRealtimeCharacter(c)) {
             continue; // Skip further processing for realtime characters
         }
         
         // Handle end of line
         if (c == '\n' || c == '\r') {
             if (serialLineBufferIndex > 0) {
                 // Null terminate and process the line
                 serialLineBuffer[serialLineBufferIndex] = '\0';
                 processLine(String(serialLineBuffer));
                 
                 // Reset buffer for next line
                 serialLineBufferIndex = 0;
                 memset(serialLineBuffer, 0, SERIAL_LINE_BUFFER_SIZE);
             }
         }
         // Add character to buffer if space available
         else if (serialLineBufferIndex < SERIAL_LINE_BUFFER_SIZE - 1) {
             serialLineBuffer[serialLineBufferIndex++] = c;
         }
         // Line too long, discard and reset
         else {
             sendError(1, "Line exceeds buffer"); 
             serialLineBufferIndex = 0;
             memset(serialLineBuffer, 0, SERIAL_LINE_BUFFER_SIZE);
         }
     }
     
     
     // Send telemetry data if enabled
     updateTelemetry();
     
     return true;
 }
 
 void CommunicationManager::sendMessage(const String &message)
 {
     Serial.println(message);
 }
 
 bool CommunicationManager::processRealtimeCharacter(char c)
 {
     // Check for realtime command characters
     switch (c) {
         case '!': // Feed hold (pause)
             if (machineController) {
                 //machineController->pauseMovement();
                 Debug::info("CommunicationManager", "Feed hold triggered");
             }
             return true;
             
         case '~': // Cycle start/resume
             if (machineController) {
                 //machineController->resumeMovement();
                 Debug::info("CommunicationManager", "Resume triggered");
             }
             return true;
             
         case 0x18: // Ctrl-X (Reset/Emergency stop)
             if (machineController) {
                 machineController->emergencyStop();
                 Debug::info("CommunicationManager", "Emergency stop triggered");
             }
             return true;
     }
     
     return false; // Not a realtime character
 }
 
 void CommunicationManager::sendAcknowledgment()
 {
     sendMessage("ok");
 }
 
 void CommunicationManager::sendError(int errorCode, const String &errorMessage)
 {
     String message = "error:" + String(errorCode);
     if (errorMessage.length() > 0) {
         message += " (" + errorMessage + ")";
     }
     sendMessage(message);
 }
 
 void CommunicationManager::setTelemetryEnabled(bool enabled)
 {
     telemetryEnabled = enabled;
     Debug::info("CommunicationManager", "Telemetry " + String(enabled ? "enabled" : "disabled"));
 }
 
 void CommunicationManager::setTelemetryInterval(unsigned long intervalMs)
 {
     telemetryInterval = intervalMs > 0 ? intervalMs : 100;
     Debug::info("CommunicationManager", "Telemetry interval set to " + String(telemetryInterval) + "ms");
 }
 
 void CommunicationManager::setTelemetryFrequency(int frequency)
 {
     if (frequency > 0) {
         telemetryInterval = 1000 / frequency;
         Debug::info("CommunicationManager", "Telemetry frequency set to " + String(frequency) + 
                  "Hz (interval: " + String(telemetryInterval) + "ms)");
     }
 }
 
 void CommunicationManager::setStatusReportInterval(unsigned long intervalMs)
 {
     statusReportInterval = intervalMs;
     Debug::info("CommunicationManager", "Status report interval set to " + String(statusReportInterval) + "ms");
 }
 
 void CommunicationManager::processLine(const String &line)
 {
     // Skip empty lines
     if (line.length() == 0) {
         return;
     }
     
     Debug::verbose("CommunicationManager", "Processing line: " + line);
     
     // Check for special commands (info queries)
     if (line.startsWith("?")) {
         String response = processInfoCommand(line);
         sendMessage(response);
         return;
     }
 
     // Check for emergency stop command
     if (line.startsWith("!") || line.equalsIgnoreCase("M112")) {
         // Handle emergency stop directly
         if (machineController) {
             machineController->emergencyStop();
             Debug::info("CommunicationManager", "Emergency stop triggered by command: " + line);
         }
         sendAcknowledgment();
         return;
     }
     
     // For all other commands, add to the line buffer for processing by the GCode parser
     if (lineBuffer->addLine(line)) {
         // Only send acknowledgment once the command is queued
         sendAcknowledgment();
     } else {
         // Line buffer is full
         sendError(4, "Buffer full");
     }
 }
 
 void CommunicationManager::updateTelemetry()
 {
     // Skip if telemetry is disabled or no machine controller
     if (!telemetryEnabled || !machineController) {
         return;
     }
     
     // Check if it's time to send telemetry
     unsigned long currentTime = millis();
     if (currentTime - lastTelemetryTime < telemetryInterval) {
         return;
     }
     
     // Get current positions
     const std::vector<float>& workPosition = machineController->getCurrentWorkPosition();
     const std::vector<float>& worldPosition = machineController->getCurrentWorldPosition();
     const std::vector<float>& velocityVector = machineController->getCurrentDesiredVelocityVector();
     
     // Only send if position has changed
     bool positionChanged = lastReportedPosition.empty() || 
                          workPosition.size() != lastReportedPosition.size();
     
     if (!positionChanged) {
         for (size_t i = 0; i < workPosition.size(); i++) {
             if (abs(workPosition[i] - lastReportedPosition[i]) > 0.001f) {
                 positionChanged = true;
                 break;
             }
         }
     }
     
     if (positionChanged || machineController->isMoving()) {
         // Clear buffer and build telemetry JSON
         telemetryBuffer = "[TELEMETRY]{";
         
         // Add position data
         telemetryBuffer += "\"work\":{";
         if (workPosition.size() >= 3) {
             telemetryBuffer += "\"X\":" + String(workPosition[0], 3) + ",";
             telemetryBuffer += "\"Y\":" + String(workPosition[1], 3) + ",";
             telemetryBuffer += "\"Z\":" + String(workPosition[2], 3);
         }
         telemetryBuffer += "},\"world\":{";
         if (worldPosition.size() >= 3) {
             telemetryBuffer += "\"X\":" + String(worldPosition[0], 3) + ",";
             telemetryBuffer += "\"Y\":" + String(worldPosition[1], 3) + ",";
             telemetryBuffer += "\"Z\":" + String(worldPosition[2], 3);
         }
         telemetryBuffer += "},";
         
         // Add current velocity
         float currentVelocity = machineController->getCurrentVelocity();
         telemetryBuffer += "\"velocity\":" + String(currentVelocity, 3) + ",";
         
         // Add velocity vector if available
         telemetryBuffer += "\"velocityVector\":{";
         if (velocityVector.size() >= 3) {
             telemetryBuffer += "\"X\":" + String(velocityVector[0], 3) + ",";
             telemetryBuffer += "\"Y\":" + String(velocityVector[1], 3) + ",";
             telemetryBuffer += "\"Z\":" + String(velocityVector[2], 3);
         }
         telemetryBuffer += "},";
         
         // Add ESP32 temperature
         float temp = temperatureRead();
         telemetryBuffer += "\"temperature\":" + String(temp, 1);
         
         // Add CPU usage if available
         if (cpuMonitor) {
             telemetryBuffer += ",\"cpuUsage\":" + String(cpuMonitor->getTotalUsage(), 1);
             telemetryBuffer += ",\"cpuCore0\":" + String(cpuMonitor->getCore0Usage(), 1);
             telemetryBuffer += ",\"cpuCore1\":" + String(cpuMonitor->getCore1Usage(), 1);
         }
         
         // Add buffer levels
         telemetryBuffer += ",\"bufferFree\":" + String(lineBuffer->availableSpace());
         
         telemetryBuffer += "}";
         
         // Send the message
         sendMessage(telemetryBuffer);
         
         // Update tracking variables
         if (lastReportedPosition.size() != workPosition.size()) {
             lastReportedPosition.resize(workPosition.size());
         }
         for (size_t i = 0; i < workPosition.size(); i++) {
             lastReportedPosition[i] = workPosition[i];
         }
         lastTelemetryTime = currentTime;
     }
 }
 
 String CommunicationManager::processInfoCommand(const String &command)
 {
     if (!machineController) {
         return "Error: Machine controller not available";
     }
 
     // Remove the leading '?' if present
     String cleanCmd = command;
     if (cleanCmd.startsWith("?")) {
         cleanCmd = cleanCmd.substring(1);
     }
     cleanCmd.trim();
     cleanCmd.toUpperCase();
 
     if (cleanCmd.startsWith("POS")) {
         std::vector<float> workPositions = machineController->getCurrentWorkPosition();
         std::vector<float> worldPositions = machineController->getCurrentWorldPosition();
         std::vector<float> workOffset = machineController->getWorkOffset();
 
         String posStr = "";
 
         for (size_t i = 0; i < workPositions.size(); i++) {
             Motor *motor = machineController->getMotorManager()->getMotor(i);
             if (motor) {
                 posStr += " " + motor->getName() + ":" + String(workPositions[i], 3) +
                           " (" + motor->getName() + "_world:" + String(worldPositions[i], 3) + ")";
             }
         }
 
         return "<RESPONSE:POS>" + posStr;
     }
     else if (cleanCmd.startsWith("STATUS")) {
         // Report machine status
         String status = "";
         status += machineController->isMoving() ? "MOVING" : "IDLE";
 
         // Add more status information
         status += " | Absolute mode: " + String(machineController->isAbsoluteMode() ? "ON" : "OFF");
         status += " | Buffer free: " + String(lineBuffer->availableSpace());
 
         return "<RESPONSE:STATUS>" + status;
     }
     else if (cleanCmd.startsWith("ENDSTOPS")) {
         // Report endstop status for all motors
         String endstopStatus = "";
 
         MotorManager *motorManager = machineController->getMotorManager();
         for (int i = 0; i < motorManager->getNumMotors(); i++) {
             Motor *motor = motorManager->getMotor(i);
             if (motor) {
                 bool triggered = motor->isEndstopTriggered();
                 endstopStatus += " " + motor->getName() + ":" +
                          (triggered ? "TRIGGERED" : "OPEN");
             }
         }
 
         return "<RESPONSE:ENDSTOPS>" + endstopStatus;
     }
     else if (cleanCmd.startsWith("HELP")) {
         // Show available commands
         String helpText = "Available commands:\n";
         helpText += "Regular G-codes: G0, G1, G28, G90, G91, G92, etc.\n";
         helpText += "Special commands:\n";
         helpText += "  !STOP or !M112 - Emergency stop\n";
         helpText += "  ?POS - Current position\n";
         helpText += "  ?STATUS - Machine status\n";
         helpText += "  ?ENDSTOPS - Endstop status\n";
         helpText += "  ?HELP - This help text\n";
         helpText += "Realtime characters:\n";
         helpText += "  ? - Status report\n";
         helpText += "  ! - Feed hold (pause)\n";
         helpText += "  ~ - Cycle start (resume)\n";
         helpText += "  Ctrl-X - Reset/Emergency stop\n";
 
         return "<RESPONSE:HELP>" + helpText;
     }
     else if (cleanCmd.startsWith("DEBUG")) {
         // Process debug commands
         if (cleanCmd == "DEBUG ON") {
             Debug::begin(true);
             return "<RESPONSE:DEBUG> Debug mode enabled";
         }
         else if (cleanCmd == "DEBUG OFF") {
             Debug::begin(false);
             return "<RESPONSE:DEBUG> Debug mode disabled";
         }
         else if (cleanCmd.startsWith("DEBUG LEVEL")) {
             // Set debug level
             int level = cleanCmd.substring(11).toInt();
             level = constrain(level, 0, 3);
             Debug::setLevel(level);
             return "<RESPONSE:DEBUG> Debug level set to " + String(level);
         }
         else if (cleanCmd == "DEBUG DIAG") {
             Debug::printDiagnostics();
             return "<RESPONSE:DEBUG> Diagnostics printed to debug output";
         }
     }
 
     return "<RESPONSE:UNKNOWN> Unknown info command: " + command;
 }