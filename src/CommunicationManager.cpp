/**
 * @file GRBLCommunicationManager.cpp
 * @brief Implementation of the GRBLCommunicationManager class with JSON telemetry
 */

 #include "CommunicationManager.h"
 #include <ArduinoJson.h>
 
 #define LINE_FEED_CHAR '\n'
 #define CARRIAGE_RETURN_CHAR '\r'

 #define GRBL_RESPONSE_OK "ok"
 #define GRBL_RESPONSE_ERROR "error:"
 
 GRBLCommunicationManager::GRBLCommunicationManager(CommandQueue *commandQueue, CommandProcessor *commandProcessor)
     : commandQueue(commandQueue),
       commandProcessor(commandProcessor),
       lineBufferIndex(0),
       autoReportEnabled(false),
       autoReportIntervalMs(1000), // Default to 1 second
       lastAutoReportTime(0),
       feedHoldActive(false),
       lineNumber(0),
       awaitingAck(false),
       telemetryEnabled(true),      // Enable telemetry by default
       telemetryFrequency(5),       // Default to 5Hz
       lastTelemetryTime(0)
 {
     // Initialize machine controller from command processor
     if (commandProcessor) {
         machineController = commandProcessor->getMachineController();
     } else {
         machineController = nullptr;
     }
     
     // Initialize line buffer
     resetLineBuffer();
     
     // Get CPU monitor instance
     cpuMonitor = CPUMonitor::getInstance();
     if (cpuMonitor) {
         cpuMonitor->begin();
     }
     
     // Pre-allocate buffers to avoid memory fragmentation
     telemetryMsgBuffer.reserve(512);
     lastReportedPosition.reserve(3);
 }
 
 bool GRBLCommunicationManager::initialize(unsigned long baudRate)
 {
     Serial.begin(baudRate);
     delay(100); // Give serial a moment to initialize
 
     Debug::info("GRBLCommunicationManager", "Serial initialized at " + String(baudRate) + " baud");
     
     return true;
 }
 
 bool GRBLCommunicationManager::update()
 {
     // Check for CPU usage update
     if (cpuMonitor) {
         //cpuMonitor->update();
     }
     
     // Handle incoming data
     while (Serial.available() > 0) {
         char c = Serial.read();
         

         // Normal line processing
         if (c == LINE_FEED_CHAR || c == CARRIAGE_RETURN_CHAR) {
             // End of line detected
             if (lineBufferIndex > 0) {
                 // Null terminate the string
                 lineBuffer[lineBufferIndex] = '\0';
                 
                 // Process the line
                 String line = String(lineBuffer);
                 processLine(line);
                 
                 // Reset buffer for next line
                 resetLineBuffer();
             }
         }
         else if (lineBufferIndex < GRBL_LINE_BUFFER_SIZE - 1) {
             // Add character to buffer
             lineBuffer[lineBufferIndex++] = c;
         }
         else {
             // Line too long, discard and reset
             Debug::warning("GRBLCommunicationManager", "Line too long, discarding");
             sendMessage("error:1"); // Line exceeds GRBL buffer
             resetLineBuffer();
         }
     }
     
     
     // Handle command acknowledgment
     if (awaitingAck) {
         // In a real implementation, check the state of command execution
         // and send acknowledgment when appropriate.
         // For now, we'll just send it immediately for basic functionality.
         sendAcknowledgment(true);
         awaitingAck = false;
     }
     
     // Handle telemetry updates
     sendPositionTelemetry(false);

     return true;
 }
 
 void GRBLCommunicationManager::sendMessage(const String &message)
 {
     Serial.println(message);
 }
 
 void GRBLCommunicationManager::processLine(const String &line)
 {
     // Skip empty lines
     if (line.length() == 0) {
         return;
     }
 
     Debug::verbose("GRBLCommunicationManager", "Processing line: " + line);

     // Queue as a regular G-code command
     if (commandQueue->push(line, MOTION)) {
         Debug::verbose("GRBLCommunicationManager", "Queued G-code command: " + line);
         awaitingAck = true; // Mark that we need to send an acknowledgment
     } else {
         // Queue is full
         sendAcknowledgment(false, 4); // Cannot be queued, buffer full
     }
 }
 
 void GRBLCommunicationManager::resetLineBuffer()
 {
     memset(lineBuffer, 0, GRBL_LINE_BUFFER_SIZE);
     lineBufferIndex = 0;
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
     if (success) {
         Serial.println(GRBL_RESPONSE_OK);
     } else {
         Serial.println(GRBL_RESPONSE_ERROR + String(errorCode));
     }
 }
 
 void GRBLCommunicationManager::sendPositionTelemetry(bool force)
 {
     // Skip if telemetry is disabled
     if (!telemetryEnabled)
         return;
 
     // Get current time
     unsigned long currentTime = millis();
 
     // Check if it's time to send telemetry
     unsigned long telemetryInterval = 1000 / telemetryFrequency;
     if (!force && currentTime - lastTelemetryTime < telemetryInterval)
         return;
 
     // Ensure machine controller exists
     if (!machineController)
         return;
 
     // Get current positions
     const std::vector<float>& workPosition = machineController->getCurrentWorkPosition();
     const std::vector<float>& worldPosition = machineController->getCurrentWorldPosition();
     const std::vector<float>& velocityVector = machineController->getCurrentDesiredVelocityVector();
 
     // Only send if position has changed or force is true
     if (force ||
         lastReportedPosition.empty() ||
         workPosition != lastReportedPosition)
     {
         // Clear and reuse the string buffer
         telemetryMsgBuffer = "[TELEMETRY]{";
         telemetryMsgBuffer += "\"work\":{";
 
         // Make sure we have at least 3 axes
         if (workPosition.size() >= 3) {
             telemetryMsgBuffer += "\"X\":" + String(workPosition[0], 3) + ",";
             telemetryMsgBuffer += "\"Y\":" + String(workPosition[1], 3) + ",";
             telemetryMsgBuffer += "\"Z\":" + String(workPosition[2], 3);
         }
 
         telemetryMsgBuffer += "},\"world\":{";
 
         if (worldPosition.size() >= 3) {
             telemetryMsgBuffer += "\"X\":" + String(worldPosition[0], 3) + ",";
             telemetryMsgBuffer += "\"Y\":" + String(worldPosition[1], 3) + ",";
             telemetryMsgBuffer += "\"Z\":" + String(worldPosition[2], 3);
         }
 
         telemetryMsgBuffer += "},";
 
         // Add scalar velocity information
         float currentVelocity = machineController->getCurrentDesiredVelocity();
         telemetryMsgBuffer += "\"velocity\":" + String(currentVelocity, 3) + ",";
 
         // Add velocity vector
         telemetryMsgBuffer += "\"velocityVector\":{";
 
         // Make sure we have at least 3 axes
         if (velocityVector.size() >= 3) {
             telemetryMsgBuffer += "\"X\":" + String(velocityVector[0], 3) + ",";
             telemetryMsgBuffer += "\"Y\":" + String(velocityVector[1], 3) + ",";
             telemetryMsgBuffer += "\"Z\":" + String(velocityVector[2], 3);
         }
 
         telemetryMsgBuffer += "},";
 
         // Add ESP32 temperature
         float temp = temperatureRead();
         telemetryMsgBuffer += "\"temperature\":" + String(temp, 1);
 
         // Add CPU usage if available
         if (cpuMonitor) {
             telemetryMsgBuffer += ",\"cpuUsage\":" + String(cpuMonitor->getTotalUsage(), 1);
             telemetryMsgBuffer += ",\"cpuCore0\":" + String(cpuMonitor->getCore0Usage(), 1);
             telemetryMsgBuffer += ",\"cpuCore1\":" + String(cpuMonitor->getCore1Usage(), 1);
         }
 
         telemetryMsgBuffer += "}";
 
         // Send the message
         sendMessage(telemetryMsgBuffer);
 
         // Update tracking variables - copy data to avoid new allocations
         if (lastReportedPosition.size() != workPosition.size()) {
             lastReportedPosition.resize(workPosition.size());
         }
         std::copy(workPosition.begin(), workPosition.end(), lastReportedPosition.begin());
         lastTelemetryTime = currentTime;
     }
 }
 