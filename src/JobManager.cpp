/**
 * @file JobManager.cpp
 * @brief Implementation of the JobManager class, optimized for smooth motor operation
 */

 #include "JobManager.h"

 JobManager::JobManager(CommandQueue *commandQueue, FileManager *fileManager)
     : commandQueue(commandQueue),
       fileManager(fileManager),
       jobStatus(JOB_IDLE),
       currentLineNumber(0),
       totalJobLines(0),
       validCommandCount(0),
       bufferThreshold(10),  // Increased from 5 to 10 to reduce update frequency
       linesPerBatch(20),    // Increased from 10 to 20 to load more at once
       emergencyStopRequested(false),
       jobCompletionAcknowledged(true),
       lastLoadAttemptTime(0),
       loadThrottleInterval(50) // Wait at least 50ms between load attempts
 {
   // Pre-allocate the command buffer to avoid memory allocations during processing
   commandBuffer.reserve(MAX_GCODE_LINE_LENGTH);
 }
 
 bool JobManager::update()
 {
   // Only check for loading more commands periodically to reduce CPU load
   unsigned long currentTime = millis();
   if (jobStatus == JOB_RUNNING && 
       commandQueue->size() <= bufferThreshold && 
       (currentTime - lastLoadAttemptTime >= loadThrottleInterval))
   {
     lastLoadAttemptTime = currentTime;
     int loaded = loadNextCommandBatch();
 
     if (loaded < 0)
     {
       // Error loading commands
       Debug::error("JobManager", "Error loading commands from job file");
       jobStatus = JOB_ERROR;
       return false;
     }
     else if (loaded == 0 && !currentJobFile)
     {
       // Job complete (no more commands and file is closed)
       Debug::info("JobManager", "Job completed: " + currentJobFilename);
       jobStatus = JOB_COMPLETED;
       jobCompletionAcknowledged = false;
       return true;
     }
   }
 
   return true;
 }
 
 bool JobManager::startJob(const String &filename)
 {
   // Check if system can accept a new job
   if (!canStartNewJob())
   {
     Debug::error("JobManager", "Cannot start new job, system busy");
     return false;
   }
 
   // Check if we already have the file open from validation
   if (currentJobFile && currentJobFilename == filename && currentLineNumber == 0)
   {
     Debug::info("JobManager", "Using already open file from validation: " + filename);
     // File is already open and positioned at the beginning
   }
   else
   {
     // Check if file exists
     if (!fileManager->fileExists(filename))
     {
       Debug::error("JobManager", "Job file not found: " + filename);
       return false;
     }
 
     // Close any existing file
     if (currentJobFile)
     {
       currentJobFile.close();
     }
 
     // Validate the G-code file first
     Debug::info("JobManager", "Validating G-code file before execution: " + filename);
 
     if (gCodeValidator)
     {
       ValidationResult result = validateGCodeFile(filename);
 
       if (!result.valid)
       {
         Debug::error("JobManager", "G-code validation failed. Job not started.");
         Debug::error("JobManager", "Found " + String(result.errors.size()) + " errors in file.");
 
         // Format and log the first few errors
         int errorLimit = min(5, (int)result.errors.size());
         for (int i = 0; i < errorLimit; i++)
         {
           const GCodeError &error = result.errors[i];
           Debug::error("JobManager", "Line " + String(error.lineNumber) + ": " +
                                         error.errorDescription);
         }
 
         return false;
       }
 
       // Use validated line count
       totalJobLines = result.lineCount;
       Debug::info("JobManager", "G-code validation passed. Lines: " + String(totalJobLines));
       
       // The file should already be open from validateGCodeFile if validation passed
     }
     else
     {
       // No validator available, open the file directly
       currentJobFile = fileManager->openFile(filename);
       if (!currentJobFile)
       {
         Debug::error("JobManager", "Failed to open job file: " + filename);
         return false;
       }
       
       // Count lines if no validation was done
       totalJobLines = countJobLines(filename);
     }
 
     if (totalJobLines <= 0)
     {
       Debug::error("JobManager", "Empty or invalid job file: " + filename);
       if (currentJobFile) currentJobFile.close();
       return false;
     }
     
     // Reset job state
     currentJobFilename = filename;
     currentLineNumber = 0;
     validCommandCount = 0;
   }
 
   Debug::info("JobManager", "Starting job: " + filename + ", estimated lines: " + String(totalJobLines));
 
   // Set job status to running
   jobStatus = JOB_RUNNING;
   jobCompletionAcknowledged = true;
   lastLoadAttemptTime = 0; // Reset load timer
 
   // Prefill the command queue with a substantial batch to avoid empty queue issues
   int loaded = loadNextCommandBatch();
   if (loaded <= 0)
   {
     Debug::error("JobManager", "Failed to load initial command batch");
     currentJobFile.close();
     jobStatus = JOB_ERROR;
     return false;
   }
 
   return true;
 }
 
 bool JobManager::pauseJob()
 {
   if (jobStatus != JOB_RUNNING)
   {
     return false;
   }
 
   Debug::info("JobManager", "Pausing job: " + currentJobFilename);
   jobStatus = JOB_PAUSED;
 
   // Save job state for possible resume after power loss
   return saveJobState();
 }
 
 bool JobManager::resumeJob()
 {
   if (jobStatus != JOB_PAUSED)
   {
     return false;
   }
 
   if (!currentJobFile)
   {
     Debug::error("JobManager", "Cannot resume: Job file not open");
     return false;
   }
 
   Debug::info("JobManager", "Resuming job: " + currentJobFilename);
   jobStatus = JOB_RUNNING;
   lastLoadAttemptTime = 0; // Reset load timer to allow immediate loading
 
   return true;
 }
 
 bool JobManager::stopJob()
 {
   if (jobStatus == JOB_IDLE)
   {
     return false;
   }
 
   Debug::info("JobManager", "Stopping job: " + currentJobFilename);
 
   // Close the file
   if (currentJobFile)
   {
     currentJobFile.close();
   }
 
   // Clear command queue
   commandQueue->clear();
 
   // Reset job state
   jobStatus = JOB_IDLE;
   currentJobFilename = "";
   currentLineNumber = 0;
   validCommandCount = 0;
   totalJobLines = 0;
 
   return true;
 }
 
 bool JobManager::isJobRunning() const
 {
   return (jobStatus == JOB_RUNNING || jobStatus == JOB_PAUSED);
 }
 
 String JobManager::getCurrentJobName() const
 {
   return currentJobFilename;
 }
 
 int JobManager::getJobProgress() const
 {
   if (totalJobLines <= 0 || currentLineNumber <= 0)
   {
     return 0;
   }
 
   int progress = (currentLineNumber * 100) / totalJobLines;
 
   // Cap progress at 99% until job is marked completed
   if (progress >= 100 && jobStatus != JOB_COMPLETED)
   {
     progress = 99;
   }
 
   return progress;
 }
 
 JobStatus JobManager::getJobStatus() const
 {
   return jobStatus;
 }
 
 void JobManager::setBufferThreshold(int threshold)
 {
   bufferThreshold = max(5, threshold); // Ensure at least 5 to prevent queue starvation
 }
 
 void JobManager::setLinesPerBatch(int lines)
 {
   linesPerBatch = max(5, lines); // Ensure at least 5 for efficient loading
 }
 
 bool JobManager::canStartNewJob() const
 {
   // Can't start a new job if one is already running
   if (jobStatus != JOB_IDLE && jobStatus != JOB_COMPLETED)
   {
     return false;
   }
 
   // Only start a new job if command queue is empty or nearly empty
   return (commandQueue->size() <= bufferThreshold);
 }
 
 int JobManager::countJobLines(const String &filename)
 {
   if (!fileManager)
   {
     Debug::error("JobManager", "File manager not available");
     return -1;
   }
 
   File file = fileManager->openFile(filename);
   if (!file)
   {
     Debug::error("JobManager", "Failed to open file for counting lines: " + filename);
     return -1;
   }
 
   // Use a fixed buffer for line reading to avoid String allocations
   char lineBuf[MAX_GCODE_LINE_LENGTH];
   int lineCount = 0;
   int validCommands = 0;
 
   // Count total lines and valid commands
   while (readLineToBuffer(file, lineBuf, sizeof(lineBuf)))
   {
     lineCount++;
 
     // Skip empty lines and comments to count valid commands
     if (!isEmptyLine(lineBuf) && !isCommentLine(lineBuf))
     {
       validCommands++;
     }
 
     // Limit line counting to avoid long delays
     if (lineCount >= 10000)
     {
       break;
     }
   }
 
   file.close();
 
   Debug::info("JobManager", "Job file has " + String(lineCount) + " lines, " +
                                String(validCommands) + " valid commands");
 
   return lineCount;
 }
 
 int JobManager::loadNextCommandBatch()
 {
   // First, ensure there's actually space in the command queue
   if (commandQueue->size() > bufferThreshold)
   {
     return 0; // Queue is still adequately filled, no need to load more
   }
 
   // Verify file is open and readable
   if (!currentJobFile || !currentJobFile.available()) {
     if (currentJobFile) {
       currentJobFile.close();
     }
     
     // Try reopening the file if it was closed prematurely
     if (currentLineNumber < totalJobLines) {
       Debug::info("JobManager", "Attempting to reopen file at line " + String(currentLineNumber));
       currentJobFile = fileManager->openFile(currentJobFilename);
       if (currentJobFile) {
         // Skip to current line efficiently with a fixed buffer
         char lineBuf[MAX_GCODE_LINE_LENGTH];
         for (int i = 0; i < currentLineNumber; i++) {
           if (!readLineToBuffer(currentJobFile, lineBuf, sizeof(lineBuf))) {
             break;
           }
         }
       }
     }
   }
 
   if (jobStatus != JOB_RUNNING)
   {
     // Don't load commands when paused or stopped
     return 0;
   }
 
   int loadedCount = 0;
   char lineBuf[MAX_GCODE_LINE_LENGTH];
   
   // Pre-allocate a small batch of commands (to avoid dynamic allocation during processing)
   validCommandBuffer.clear();
   validCommandBuffer.reserve(linesPerBatch);
 
   // Load up to linesPerBatch valid commands, but limit processing time per batch
   unsigned long startTime = millis();
   unsigned long maxProcessingTime = 5; // Maximum milliseconds to spend loading
   
   for (int i = 0; i < linesPerBatch; i++)
   {
     // Check if we've reached EOF
     if (!readLineToBuffer(currentJobFile, lineBuf, sizeof(lineBuf)))
     {
       // EOF reached
       currentJobFile.close();
       Debug::info("JobManager", "End of job file reached");
       break;
     }
 
     // Update line number
     currentLineNumber++;
 
     // Process the line without creating new String objects
     // Skip empty lines and comments
     if (isEmptyLine(lineBuf) || isCommentLine(lineBuf)) {
       continue;
     }
     
     // Process the line in-place (remove comments and trim)
     processLineInPlace(lineBuf);
     
     // Skip if the processing resulted in an empty line
     if (lineBuf[0] == '\0') {
       continue;
     }
     
     // Store valid command in buffer
     validCommandBuffer.push_back(String(lineBuf));
     loadedCount++;
     
     // Check if we've spent too much time processing lines
     if ((millis() - startTime) > maxProcessingTime) {
       break; // Limit processing time to avoid motor stutters
     }
   }
   
   // Push all collected commands to the queue in one batch - reduces overhead
   for (const String& cmd : validCommandBuffer) {
     if (!commandQueue->push(cmd, MOTION)) {
       break;
     }
     validCommandCount++;
   }
 
   // Save job state periodically - but only when we have enough time
   // and not too frequently to avoid SD card overhead
   if (currentLineNumber % 500 == 0)
   {
     // Instead of calling saveJobState() here, just track the need to save
     // and do the actual save during a period when motors aren't busy
     needsStateSave = true;
   }
 
   return loadedCount;
 }
 
 String JobManager::processGCodeLine(const String &line)
 {
   // Use our pre-allocated buffer to avoid new String allocations
   commandBuffer = line;
 
   // Remove comments
   int commentIdx = commandBuffer.indexOf(';');
   if (commentIdx >= 0)
   {
     commandBuffer.remove(commentIdx);
   }
 
   // Trim whitespace
   commandBuffer.trim();
 
   // Skip empty lines
   if (commandBuffer.length() == 0)
   {
     return "";
   }
 
   return commandBuffer;
 }
 
 bool JobManager::readLineToBuffer(File &file, char *buffer, size_t bufferSize)
 {
   if (!file || !file.available() || !buffer || bufferSize < 2) {
     return false;
   }
   
   size_t idx = 0;
   buffer[0] = '\0'; // Start with empty string
   
   // Read char by char for compatibility with Arduino's File implementation
   while (idx < bufferSize - 1 && file.available()) {
     char c = file.read();
     
     if (c == '\n' || c == '\r') {
       // End of line found
       buffer[idx] = '\0'; // Null terminate the string
       
       // Skip additional line break characters
       if (file.available()) {
         char next = file.peek();
         if (next == '\n' || next == '\r') {
           file.read(); // Skip the next character
         }
       }
       
       return true;
     } else {
       buffer[idx++] = c;
     }
   }
   
   // If we get here, either buffer is full or EOF is reached
   buffer[idx] = '\0'; // Ensure null termination
   return idx > 0; // Return true if we read anything
 }
 
 bool JobManager::isEmptyLine(const char *line)
 {
   if (!line) return true;
   
   while (*line) {
     if (!isspace((unsigned char)*line)) {
       return false;
     }
     line++;
   }
   return true;
 }
 
 bool JobManager::isCommentLine(const char *line)
 {
   if (!line) return false;
   
   // Skip leading whitespace
   while (*line && isspace((unsigned char)*line)) {
     line++;
   }
   
   // Check if first non-whitespace char is a comment marker
   return *line == ';';
 }
 
 void JobManager::processLineInPlace(char *line)
 {
   if (!line) return;
   
   // Remove comments (anything after ';')
   char *commentStart = strchr(line, ';');
   if (commentStart) {
     *commentStart = '\0';
   }
   
   // Trim trailing whitespace
   size_t len = strlen(line);
   while (len > 0 && isspace((unsigned char)line[len - 1])) {
     line[--len] = '\0';
   }
   
   // Trim leading whitespace by moving the content
   char *start = line;
   while (*start && isspace((unsigned char)*start)) {
     start++;
   }
   
   if (start != line) {
     // Move the trimmed string to the beginning
     memmove(line, start, strlen(start) + 1); // +1 for the null terminator
   }
 }
 
 bool JobManager::saveJobState()
 {
   // Reset save flag
   needsStateSave = false;
   
   if (currentJobFilename.length() == 0)
   {
     return false;
   }
 
   return fileManager->createResumeFile(currentJobFilename, currentLineNumber);
 }
 
 ValidationResult JobManager::validateGCodeFile(const String &filename)
 {
   ValidationResult emptyResult;
   emptyResult.valid = false;
   emptyResult.lineCount = 0;
   emptyResult.validCommandCount = 0;
 
   // Check if validator is available
   if (!gCodeValidator)
   {
     Debug::error("JobManager", "G-code validator not available");
     GCodeError error;
     error.lineNumber = 0;
     error.line = "";
     error.errorDescription = "G-code validator not available";
     emptyResult.errors.push_back(error);
     lastValidationResult = emptyResult;
     return emptyResult;
   }
 
   // Check if file manager is available
   if (!fileManager)
   {
     Debug::error("JobManager", "File manager not available");
     GCodeError error;
     error.lineNumber = 0;
     error.line = "";
     error.errorDescription = "File manager not available";
     emptyResult.errors.push_back(error);
     lastValidationResult = emptyResult;
     return emptyResult;
   }
 
   // Open the file once and use it for validation and later operations
   File file = fileManager->openFile(filename);
   if (!file)
   {
     Debug::error("JobManager", "Failed to open file: " + filename);
     GCodeError error;
     error.lineNumber = 0;
     error.line = "";
     error.errorDescription = "Failed to open file: " + filename;
     emptyResult.errors.push_back(error);
     lastValidationResult = emptyResult;
     return emptyResult;
   }
 
   // Validate the open file
   lastValidationResult = gCodeValidator->validateOpenFile(file, filename);
   
   // If validation passed and we're going to start a job, keep the file open
   // Otherwise, close it
   if (lastValidationResult.valid && jobStatus == JOB_IDLE) 
   {
     // Store the file for future use in startJob
     currentJobFile = file;
     currentJobFilename = filename;
     // Reset position to beginning of file
     currentJobFile.seek(0);
     // Other initialization for job
     currentLineNumber = 0;
     validCommandCount = 0;
     totalJobLines = lastValidationResult.lineCount;
   }
   else
   {
     // Close the file if we're not going to use it
     file.close();
   }
 
   return lastValidationResult;
 }
 
 void JobManager::emergencyAbortJob(const String &errorMessage)
 {
   Debug::error("JobManager", "EMERGENCY ABORT: " + errorMessage);
 
   // Stop current job immediately
   stopJob();
 
   // Reset machine state
   resetMachineState();
 }
 
 void JobManager::resetMachineState()
 {
   // Reset command queue
   if (commandQueue)
   {
     Debug::info("JobManager", "Clearing command queue");
     commandQueue->clear();
   }
 
   // Reset machine controller status if available
   MachineController *machineController = nullptr;
   if (commandProcessor)
   {
     machineController = commandProcessor->getMachineController();
   }
 
   if (machineController)
   {
     Debug::info("JobManager", "Stopping all motors and resetting machine state");
     machineController->emergencyStop();
   }
 
   // Reset job status
   jobStatus = JOB_IDLE;
 
   Debug::info("JobManager", "Machine state reset completed");
 }
 
 // Called during the main loop's "idle" time when motors aren't as active
 void JobManager::performMaintenanceTasks()
 {
   // Check if we need to save state and system is not busy with critical operations
   if (needsStateSave && (jobStatus == JOB_RUNNING || jobStatus == JOB_PAUSED)) {
     saveJobState();
   }
   
   // Other maintenance tasks that should happen during idle time
   // can be added here in the future
 }