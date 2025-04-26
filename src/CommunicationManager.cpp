/**
 * @file CommunicationManager.cpp
 * @brief Implementation of the simplified communication manager
 */

 #include "CommunicationManager.h"
 #include <ArduinoJson.h>
 
 CommunicationManager::CommunicationManager(CommandQueue *commandQueue, CommandProcessor *commandProcessor) :
   commandQueue(commandQueue),
   commandProcessor(commandProcessor),
   lineBufferIndex(0),
   telemetryEnabled(true),
   telemetryInterval(100),
   lastTelemetryTime(0)
 {
   // Initialize machine controller from command processor
   machineController = commandProcessor ? commandProcessor->getMachineController() : nullptr;
   
   // Initialize line buffer
   memset(lineBuffer, 0, LINE_BUFFER_SIZE);
   
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
     
     // Handle end of line
     if (c == '\n' || c == '\r') {
       if (lineBufferIndex > 0) {
         // Null terminate and process the line
         lineBuffer[lineBufferIndex] = '\0';
         processLine(String(lineBuffer));
         
         // Reset buffer for next line
         lineBufferIndex = 0;
         memset(lineBuffer, 0, LINE_BUFFER_SIZE);
       }
     }
     // Add character to buffer if space available
     else if (lineBufferIndex < LINE_BUFFER_SIZE - 1) {
       lineBuffer[lineBufferIndex++] = c;
     }
     // Line too long, discard and reset
     else {
       sendMessage("error:1"); // Line exceeds buffer
       lineBufferIndex = 0;
       memset(lineBuffer, 0, LINE_BUFFER_SIZE);
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
   // For compatibility with original API
   setTelemetryInterval(intervalMs);
 }
 
 void CommunicationManager::processLine(const String &line)
 {
   // Skip empty lines
   if (line.length() == 0) {
     return;
   }
   
   Debug::verbose("CommunicationManager", "Processing line: " + line);
   
   // Check for special commands (immediate, info, or setting)
   if (line.startsWith("!") || line.startsWith("M112")) {
     // Push as immediate command
     if (commandQueue->push(line, IMMEDIATE)) {
       Debug::info("CommunicationManager", "Queued immediate command: " + line);
     } else {
       sendMessage("error:4"); // Cannot queue immediate command
       Debug::warning("CommunicationManager", "Failed to queue immediate command: " + line);
     }
   }
   // Queue as regular command
   else if (commandQueue->push(line, NORMAL)) {
     // Normal G-code, acknowledge
     sendMessage("ok");
     Debug::verbose("CommunicationManager", "Queued normal command: " + line);
   } else {
     // Queue is full
     sendMessage("error:4"); // Cannot queue command, buffer full
     Debug::warning("CommunicationManager", "Command queue full, rejected: " + line);
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