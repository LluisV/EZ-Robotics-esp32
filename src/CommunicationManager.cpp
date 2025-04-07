/**
 * @file CommunicationManager.cpp
 * @brief Implementation of the CommunicationManager class
 */

 #include "CommunicationManager.h"
 #include "MachineController.h"
 #include <SPIFFS.h>
 
 // Timeout for file transfers in milliseconds (30 seconds)
 #define FILE_TRANSFER_TIMEOUT 30000
 
 // Maximum time between progress updates in milliseconds (1 second)
 #define PROGRESS_UPDATE_INTERVAL 1000
 
 // Escape sequence for file transfer start/end
 #define FILE_ESCAPE_SEQUENCE "<FILE>"
 #define FILE_END_SEQUENCE "</FILE>"
 
 CommunicationManager::CommunicationManager(CommandQueue* commandQueue, CommandProcessor* commandProcessor,
                                          FileManager* fileManager, JobManager* jobManager)
   : commandQueue(commandQueue),
     commandProcessor(commandProcessor),
     fileManager(fileManager),
     jobManager(jobManager),
     lineBufferIndex(0)
 {
   // Initialize file transfer state
   fileTransfer.mode = TRANSFER_IDLE;
   fileTransfer.bytesTransferred = 0;
   fileTransfer.fileSize = 0;
   fileTransfer.error = false;
   fileTransfer.errorMessage = "";
   
   // Clear line buffer
   resetLineBuffer();
 }
 
 bool CommunicationManager::initialize(unsigned long baudRate) {
   Serial.begin(baudRate);
   delay(100); // Give serial a moment to initialize
   
   Debug::info("CommunicationManager", "Serial initialized at " + String(baudRate) + " baud");
   
   return true;
 }
 
 bool CommunicationManager::update() {
   // Check for serial data
   while (Serial.available() > 0) {
    char c = Serial.read();
     
     // Check if we're in file receive mode
     if (fileTransfer.mode == TRANSFER_RECEIVING) {
       return handleFileReceiveData();
     }
     
     // Normal command processing
     if (c == '\n' || c == '\r') {
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
     } else if (lineBufferIndex < MAX_LINE_LENGTH - 1) {
       // Add character to buffer
       lineBuffer[lineBufferIndex++] = c;
     } else {
       // Line too long, discard and reset
       Debug::warning("CommunicationManager", "Line too long, discarding");
       resetLineBuffer();
     }
   }
   
   // Check if we should send a file
   if (fileTransfer.mode == TRANSFER_SENDING) {
     return handleFileSendData();
   }
   
   // Check for file transfer timeout
   if (fileTransfer.mode != TRANSFER_IDLE && checkFileTransferTimeout()) {
     cancelFileTransfer("Timeout");
     return false;
   }
   
   return true;
 }
 
 void CommunicationManager::sendMessage(const String& message) {
   Serial.println(message);
   Debug::verbose("CommunicationManager", "Sent message: " + message);
 }
 
 void CommunicationManager::sendStatusUpdate(bool detailed) {
   if (!commandProcessor || !jobManager) {
     return;
   }
   
   MachineController* machineController = commandProcessor->getMachineController();
   if (!machineController) {
     return;
   }
   
   // Basic status
   String status = "Status: ";
   status += machineController->isMoving() ? "MOVING" : "IDLE";
   
   if (jobManager->isJobRunning()) {
     status += ", Job: " + jobManager->getCurrentJobName() + " (" + 
               String(jobManager->getJobProgress()) + "%)";
   }
   
   // Add more detailed status information if requested
   if (detailed) {
     // Add endstop status
     MotorManager* motorManager = machineController->getMotorManager();
     if (motorManager) {
       status += "\nEndstops: ";
       for (int i = 0; i < motorManager->getNumMotors(); i++) {
         Motor* motor = motorManager->getMotor(i);
         if (motor) {
           bool triggered = motor->isEndstopTriggered();
           status += motor->getName() + ":" + (triggered ? "TRIGGERED" : "OPEN") + " ";
         }
       }
     }
     
     // Add position information
     status += "\nPosition: ";
     std::vector<float> positions = machineController->getCurrentPosition();
     for (size_t i = 0; i < positions.size(); i++) {
       Motor* motor = machineController->getMotorManager()->getMotor(i);
       if (motor) {
         status += motor->getName() + ":" + String(positions[i], 3) + " ";
       }
     }
     
     // Add buffer status
     status += "\nBuffer: " + String(commandQueue->size()) + " commands";
   }
   
   sendMessage(status);
 }
 
 bool CommunicationManager::startFileReceive(const String& filename, size_t fileSize) {
   // Check if already in transfer mode
   if (fileTransfer.mode != TRANSFER_IDLE) {
     Debug::error("CommunicationManager", "File transfer already in progress");
     return false;
   }
   
   // Validate filename
   if (filename.length() == 0 || filename.indexOf('/') != 0) {
     Debug::error("CommunicationManager", "Invalid filename: " + filename);
     return false;
   }
   
   // Check if file already exists
   if (SPIFFS.exists(filename)) {
     // For now, we'll overwrite the file
     Debug::warning("CommunicationManager", "File already exists, overwriting: " + filename);
   }
   
   // Initialize file transfer state
   fileTransfer.mode = TRANSFER_RECEIVING;
   fileTransfer.filename = filename;
   fileTransfer.fileSize = fileSize;
   fileTransfer.bytesTransferred = 0;
   fileTransfer.startTime = millis();
   fileTransfer.lastUpdate = millis();
   fileTransfer.error = false;
   fileTransfer.errorMessage = "";
   
   // Send acknowledgment
   sendMessage("Receiving file: " + filename + ", size: " + String(fileSize) + " bytes");
   sendMessage(FILE_ESCAPE_SEQUENCE);
   
   Debug::info("CommunicationManager", "Started receiving file: " + filename + ", size: " + String(fileSize));
   
   return true;
 }
 
 bool CommunicationManager::startFileSend(const String& filename) {
   // Check if already in transfer mode
   if (fileTransfer.mode != TRANSFER_IDLE) {
     Debug::error("CommunicationManager", "File transfer already in progress");
     return false;
   }
   
   // Check if file exists
   if (!SPIFFS.exists(filename)) {
     Debug::error("CommunicationManager", "File not found: " + filename);
     return false;
   }
   
   // Get file size
   File file = SPIFFS.open(filename, "r");
   if (!file) {
     Debug::error("CommunicationManager", "Failed to open file: " + filename);
     return false;
   }
   
   size_t fileSize = file.size();
   file.close();
   
   // Initialize file transfer state
   fileTransfer.mode = TRANSFER_SENDING;
   fileTransfer.filename = filename;
   fileTransfer.fileSize = fileSize;
   fileTransfer.bytesTransferred = 0;
   fileTransfer.startTime = millis();
   fileTransfer.lastUpdate = millis();
   fileTransfer.error = false;
   fileTransfer.errorMessage = "";
   
   // Send start message
   sendMessage("Sending file: " + filename + ", size: " + String(fileSize) + " bytes");
   sendMessage(FILE_ESCAPE_SEQUENCE);
   
   Debug::info("CommunicationManager", "Started sending file: " + filename + ", size: " + String(fileSize));
   
   return true;
 }
 
 void CommunicationManager::cancelFileTransfer(const String& reason) {
   if (fileTransfer.mode == TRANSFER_IDLE) {
     return;
   }
   
   Debug::warning("CommunicationManager", "File transfer cancelled: " + reason);
   
   // Close file if open
   if (fileTransfer.mode == TRANSFER_RECEIVING) {
     // Remove incomplete file
     if (SPIFFS.exists(fileTransfer.filename)) {
       SPIFFS.remove(fileTransfer.filename);
     }
     
     sendMessage(FILE_END_SEQUENCE);
     sendMessage("File receive cancelled: " + reason);
   } else if (fileTransfer.mode == TRANSFER_SENDING) {
     sendMessage(FILE_END_SEQUENCE);
     sendMessage("File send cancelled: " + reason);
   }
   
   // Reset file transfer state
   fileTransfer.mode = TRANSFER_IDLE;
   fileTransfer.bytesTransferred = 0;
   fileTransfer.fileSize = 0;
   fileTransfer.error = true;
   fileTransfer.errorMessage = reason;
 }
 
 void CommunicationManager::processLine(const String& line) {
   // Skip empty lines
   if (line.length() == 0) {
     return;
   }
   
   Debug::verbose("CommunicationManager", "Processing line: " + line);
   
   // Check for special command prefixes
   if (line.startsWith("!") || line.startsWith("?")) {
     if (processSpecialCommand(line)) {
       return;
     }
   }
   
   // Check for file commands
   if (line.startsWith("@")) {
     if (processFileCommand(line)) {
       return;
     }
   }
   
   // Check if this is a file transfer escape sequence
   if (line == FILE_END_SEQUENCE) {
     if (fileTransfer.mode == TRANSFER_SENDING) {
       // End of file transfer
       fileTransfer.mode = TRANSFER_IDLE;
       sendMessage("File send completed: " + fileTransfer.filename);
       Debug::info("CommunicationManager", "File send completed: " + fileTransfer.filename);
     }
     return;
   }
   
   // Otherwise, queue as a regular G-code command
   commandQueue->push(line, MOTION);
   Debug::verbose("CommunicationManager", "Queued G-code command: " + line);
 }
 
 bool CommunicationManager::processSpecialCommand(const String& command) {
   if (command.startsWith("!")) {
     // Emergency command - highest priority
     String cmd = command.substring(1);
     cmd.trim();
     
     if (cmd.length() > 0) {
       // Special case for emergency stop - normalize to M112
       if (cmd.equalsIgnoreCase("STOP") || cmd.equalsIgnoreCase("EMERGENCY") || 
           cmd.equalsIgnoreCase("ESTOP")) {
         cmd = "M112";
       }
 
       // Process emergency stop immediately before queuing
       if (cmd == "M112" && commandProcessor) {
         commandProcessor->processImmediateCommand(cmd);
         sendMessage("EMERGENCY STOP ACTIVATED");
       }
 
       // Push to immediate queue for processing by motion task
       commandQueue->push(cmd, IMMEDIATE);
       Debug::info("CommunicationManager", "Emergency command queued: " + cmd);
       sendMessage("Emergency command queued: " + cmd);
       
       return true;
     }
   } 
   else if (command.startsWith("?")) {
     // Information request - handle directly
     String response = commandProcessor->processInfoCommand(command);
     if (response.length() > 0) {
       Debug::info("CommunicationManager", "Sending info response: " + response);
       sendMessage(response);
       return true;
     }
   }
   
   return false;
 }
 
 bool CommunicationManager::processFileCommand(const String& command) {
   // File command format: @COMMAND [PARAMETERS]
   String cmd = command.substring(1);
   cmd.trim();
   
   // Split into command and parameters
   int spaceIndex = cmd.indexOf(' ');
   String fileCmd = (spaceIndex > 0) ? cmd.substring(0, spaceIndex) : cmd;
   String params = (spaceIndex > 0) ? cmd.substring(spaceIndex + 1) : "";
   
   fileCmd.toUpperCase();
   
   if (fileCmd == "LIST") {
     // List files
     if (!fileManager) {
       sendMessage("Error: File manager not available");
       return true;
     }
     
     String fileList = fileManager->listFiles();
     sendMessage("File list:\n" + fileList);
     return true;
   }
   else if (fileCmd == "SEND") {
     // Send file to client
     if (!fileManager) {
       sendMessage("Error: File manager not available");
       return true;
     }
     
     params.trim();
     if (params.length() == 0) {
       sendMessage("Error: No filename specified");
       return true;
     }
     
     // Make sure filename starts with /
     if (!params.startsWith("/")) {
       params = "/" + params;
     }
     
     // Start file send
     if (!startFileSend(params)) {
       sendMessage("Error starting file send: " + params);
     }
     
     return true;
   }
   else if (fileCmd == "RECEIVE") {
     // Receive file from client
     if (!fileManager) {
       sendMessage("Error: File manager not available");
       return true;
     }
     
     // Parse parameters: FILENAME SIZE
     int spaceIdx = params.indexOf(' ');
     if (spaceIdx <= 0) {
       sendMessage("Error: Invalid parameters. Format: @RECEIVE filename size");
       return true;
     }
     
     String filename = params.substring(0, spaceIdx);
     filename.trim();
     
     // Make sure filename starts with /
     if (!filename.startsWith("/")) {
       filename = "/" + filename;
     }
     
     String sizeStr = params.substring(spaceIdx + 1);
     sizeStr.trim();
     size_t fileSize = sizeStr.toInt();
     
     if (fileSize <= 0) {
       sendMessage("Error: Invalid file size: " + sizeStr);
       return true;
     }
     
     // Start file receive
     if (!startFileReceive(filename, fileSize)) {
       sendMessage("Error starting file receive: " + filename);
     }
     
     return true;
   }
   else if (fileCmd == "DELETE") {
     // Delete file
     if (!fileManager) {
       sendMessage("Error: File manager not available");
       return true;
     }
     
     params.trim();
     if (params.length() == 0) {
       sendMessage("Error: No filename specified");
       return true;
     }
     
     // Make sure filename starts with /
     if (!params.startsWith("/")) {
       params = "/" + params;
     }
     
     // Delete file
     if (fileManager->deleteFile(params)) {
       sendMessage("File deleted: " + params);
     } else {
       sendMessage("Error deleting file: " + params);
     }
     
     return true;
   }
   else if (fileCmd == "RUN") {
     // Run a G-code file
     if (!fileManager || !jobManager) {
       sendMessage("Error: File or job manager not available");
       return true;
     }
     
     params.trim();
     if (params.length() == 0) {
       sendMessage("Error: No filename specified");
       return true;
     }
     
     // Make sure filename starts with /
     if (!params.startsWith("/")) {
       params = "/" + params;
     }
     
     // Start job
     if (jobManager->startJob(params)) {
       sendMessage("Job started: " + params);
     } else {
       sendMessage("Error starting job: " + params);
     }
     
     return true;
   }
   else if (fileCmd == "PAUSE") {
     // Pause current job
     if (!jobManager) {
       sendMessage("Error: Job manager not available");
       return true;
     }
     
     if (jobManager->pauseJob()) {
       sendMessage("Job paused");
     } else {
       sendMessage("Error pausing job: No active job");
     }
     
     return true;
   }
   else if (fileCmd == "RESUME") {
     // Resume current job
     if (!jobManager) {
       sendMessage("Error: Job manager not available");
       return true;
     }
     
     if (jobManager->resumeJob()) {
       sendMessage("Job resumed");
     } else {
       sendMessage("Error resuming job: No paused job");
     }
     
     return true;
   }
   else if (fileCmd == "STOP") {
     // Stop current job
     if (!jobManager) {
       sendMessage("Error: Job manager not available");
       return true;
     }
     
     if (jobManager->stopJob()) {
       sendMessage("Job stopped");
     } else {
       sendMessage("Error stopping job: No active job");
     }
     
     return true;
   }
   
   // Unknown file command
   sendMessage("Unknown file command: " + fileCmd);
   return true;
 }
 
 bool CommunicationManager::handleFileReceiveData() {
  // Extended debugging for file receive process
  if (fileTransfer.mode != TRANSFER_RECEIVING) {
    Debug::error("CommunicationManager", "File Receive: Not in receive mode");
    return false;
  }

  // Log current transfer state
  Debug::verbose("CommunicationManager", 
    "File Receive Status - File: " + fileTransfer.filename + 
    ", Size: " + String(fileTransfer.fileSize) + 
    ", Transferred: " + String(fileTransfer.bytesTransferred) + 
    ", Available: " + String(Serial.available())
  );

  // Check for transfer timeout
  unsigned long currentTime = millis();
  if (currentTime - fileTransfer.lastUpdate > FILE_TRANSFER_TIMEOUT) {
    Debug::error("CommunicationManager", 
      "File Receive Timeout - Transferred " + 
      String(fileTransfer.bytesTransferred) + 
      " of " + String(fileTransfer.fileSize) + " bytes"
    );
    cancelFileTransfer("Transfer timeout");
    return false;
  }

  // Check if we have data to read
  if (Serial.available() <= 0) {
    return true;
  }
  
  // Open the file for writing 
  File file = SPIFFS.open(fileTransfer.filename, "a");
  if (!file) {
    Debug::error("CommunicationManager", 
      "Failed to open file for writing: " + fileTransfer.filename);
    cancelFileTransfer("File open error");
    return false;
  }
  
  // Read data in chunks and write to file
  const size_t bufferSize = 512;
  uint8_t buffer[bufferSize];
  size_t bytesAvailable = Serial.available();
  size_t bytesToRead = min(bytesAvailable, 
    min(bufferSize, fileTransfer.fileSize - fileTransfer.bytesTransferred));
  
  size_t bytesRead = Serial.readBytes(buffer, bytesToRead);
  
  // Debug read operation
  Debug::verbose("CommunicationManager", 
    "File Receive - Reading " + String(bytesRead) + 
    " bytes (available: " + String(bytesAvailable) + ")"
  );

  size_t bytesWritten = file.write(buffer, bytesRead);
  
  if (bytesWritten != bytesRead) {
    Debug::error("CommunicationManager", 
      "Failed to write bytes: " + String(bytesWritten) + 
      " of " + String(bytesRead)
    );
    file.close();
    cancelFileTransfer("Write error");
    return false;
  }
  
  // Update transfer state
  fileTransfer.bytesTransferred += bytesWritten;
  fileTransfer.lastUpdate = currentTime;
  
  // Always send progress for debugging
  int progress = (fileTransfer.bytesTransferred * 100) / fileTransfer.fileSize;
  sendMessage("Progress: " + String(progress) + "% (" + 
    String(fileTransfer.bytesTransferred) + "/" + 
    String(fileTransfer.fileSize) + " bytes)");
  
  // Check if transfer is complete
  if (fileTransfer.bytesTransferred >= fileTransfer.fileSize) {
    file.close();
    
    // Send completion message with verification
    sendMessage(FILE_END_SEQUENCE);
    sendMessage("File receive completed: " + fileTransfer.filename + 
                " (Verified " + String(fileTransfer.bytesTransferred) + " bytes)");
    
    Debug::info("CommunicationManager", 
      "File receive fully completed: " + fileTransfer.filename + 
      " (" + String(fileTransfer.bytesTransferred) + " bytes)"
    );
    
    // Reset file transfer state
    fileTransfer.mode = TRANSFER_IDLE;
  } else {
    file.close();
  }
  
  return true;
}
 
 bool CommunicationManager::handleFileSendData() {
   // Make sure we're in send mode
   if (fileTransfer.mode != TRANSFER_SENDING) {
     Debug::error("CommunicationManager", "Not in file send mode");
     return false;
   }
   
   // Open the file for reading if not already open
   File file = SPIFFS.open(fileTransfer.filename, "r");
   if (!file) {
     Debug::error("CommunicationManager", "Failed to open file for reading: " + fileTransfer.filename);
     cancelFileTransfer("Failed to open file for reading");
     return false;
   }
   
   // Seek to the current position
   if (file.seek(fileTransfer.bytesTransferred)) {
     // Read and send data in chunks
     const size_t bufferSize = 64;  // Smaller chunks to avoid overwhelming serial
     uint8_t buffer[bufferSize];
     
     size_t bytesToRead = min(file.available(), (int)bufferSize);
     size_t bytesRead = file.read(buffer, bytesToRead);
     
     if (bytesRead > 0) {
       // Write directly to Serial
       size_t bytesWritten = Serial.write(buffer, bytesRead);
       
       if (bytesWritten != bytesRead) {
         Debug::error("CommunicationManager", "Failed to write all bytes to serial");
         file.close();
         cancelFileTransfer("Serial write error");
         return false;
       }
       
       // Flush the output
       Serial.flush();
       
       // Update transfer state
       fileTransfer.bytesTransferred += bytesWritten;
       fileTransfer.lastUpdate = millis();
       
       // Send progress update periodically
       if (millis() - fileTransfer.lastUpdate >= PROGRESS_UPDATE_INTERVAL) {
         int progress = (fileTransfer.bytesTransferred * 100) / fileTransfer.fileSize;
         sendMessage("\nProgress: " + String(progress) + "%");
       }
     }
     
     // Check if transfer is complete
     if (fileTransfer.bytesTransferred >= fileTransfer.fileSize) {
       file.close();
       
       // Send completion message
       sendMessage(FILE_END_SEQUENCE);
       sendMessage("File send completed: " + fileTransfer.filename);
       
       Debug::info("CommunicationManager", "File send completed: " + fileTransfer.filename);
       
       // Reset file transfer state
       fileTransfer.mode = TRANSFER_IDLE;
     } else {
       file.close();
       // Allow some time between chunks to avoid buffer overflows
       delay(10);
     }
   } else {
     Debug::error("CommunicationManager", "Failed to seek in file");
     file.close();
     cancelFileTransfer("File seek error");
     return false;
   }
   
   return true;
 }
 
 bool CommunicationManager::checkFileTransferTimeout() {
   if (fileTransfer.mode == TRANSFER_IDLE) {
     return false;
   }
   
   // Check if timeout has occurred
   if (millis() - fileTransfer.lastUpdate > FILE_TRANSFER_TIMEOUT) {
     Debug::warning("CommunicationManager", "File transfer timeout");
     return true;
   }
   
   return false;
 }
 
 void CommunicationManager::resetLineBuffer() {
   memset(lineBuffer, 0, MAX_LINE_LENGTH);
   lineBufferIndex = 0;
 }