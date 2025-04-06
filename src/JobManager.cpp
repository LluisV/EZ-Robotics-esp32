/**
 * @file JobManager.cpp
 * @brief Implementation of the JobManager class
 */

 #include "JobManager.h"

 JobManager::JobManager(CommandQueue* commandQueue, FileManager* fileManager)
   : commandQueue(commandQueue),
     fileManager(fileManager),
     jobStatus(JOB_IDLE),
     currentLineNumber(0),
     totalJobLines(0),
     validCommandCount(0),
     bufferThreshold(5),   // Default threshold: load more when queue has 5 or fewer commands
     linesPerBatch(10),    // Default batch size: load 10 lines at a time
     emergencyStopRequested(false),
     jobCompletionAcknowledged(true)
 {
 }
 
 bool JobManager::update() {
   // Check for emergency stop
   if (emergencyStopRequested) {
     // Emergency stop handling 
     stopJob();
     emergencyStopRequested = false;
     return false;
   }
   
   // If job is running and command queue is below threshold, load more commands
   if (jobStatus == JOB_RUNNING && commandQueue->size() <= bufferThreshold) {
     int loaded = loadNextCommandBatch();
     
     if (loaded < 0) {
       // Error loading commands
       Debug::error("JobManager", "Error loading commands from job file");
       jobStatus = JOB_ERROR;
       return false;
     } else if (loaded == 0 && !currentJobFile) {
       // Job complete (no more commands and file is closed)
       Debug::info("JobManager", "Job completed: " + currentJobFilename);
       jobStatus = JOB_COMPLETED;
       jobCompletionAcknowledged = false;
       return true;
     }
   }
   
   return true;
 }
 
 bool JobManager::startJob(const String& filename) {
   // Check if system can accept a new job
   if (!canStartNewJob()) {
     Debug::error("JobManager", "Cannot start new job, system busy");
     return false;
   }
   
   // Check if file exists
   if (!fileManager->fileExists(filename)) {
     Debug::error("JobManager", "Job file not found: " + filename);
     return false;
   }
   
   // Count lines to estimate job size
   totalJobLines = countJobLines(filename);
   if (totalJobLines <= 0) {
     Debug::error("JobManager", "Empty or invalid job file: " + filename);
     return false;
   }
   
   Debug::info("JobManager", "Starting job: " + filename + ", estimated lines: " + String(totalJobLines));
   
   // Open the file for reading
   currentJobFile = fileManager->openFile(filename);
   if (!currentJobFile) {
     Debug::error("JobManager", "Failed to open job file: " + filename);
     return false;
   }
   
   // Reset job state
   currentJobFilename = filename;
   currentLineNumber = 0;
   validCommandCount = 0;
   jobStatus = JOB_RUNNING;
   jobCompletionAcknowledged = true;
   
   // Load initial batch of commands
   int loaded = loadNextCommandBatch();
   if (loaded <= 0) {
     Debug::error("JobManager", "Failed to load initial command batch");
     currentJobFile.close();
     jobStatus = JOB_ERROR;
     return false;
   }
   
   return true;
 }
 
 bool JobManager::pauseJob() {
   if (jobStatus != JOB_RUNNING) {
     Debug::warning("JobManager", "Cannot pause: No job running");
     return false;
   }
   
   Debug::info("JobManager", "Pausing job: " + currentJobFilename);
   jobStatus = JOB_PAUSED;
   
   // Save job state for possible resume after power loss
   return saveJobState();
 }
 
 bool JobManager::resumeJob() {
   if (jobStatus != JOB_PAUSED) {
     Debug::warning("JobManager", "Cannot resume: No paused job");
     return false;
   }
   
   if (!currentJobFile) {
     Debug::error("JobManager", "Cannot resume: Job file not open");
     return false;
   }
   
   Debug::info("JobManager", "Resuming job: " + currentJobFilename);
   jobStatus = JOB_RUNNING;
   
   return true;
 }
 
 bool JobManager::stopJob() {
   if (jobStatus == JOB_IDLE) {
     Debug::warning("JobManager", "Cannot stop: No job running");
     return false;
   }
   
   Debug::info("JobManager", "Stopping job: " + currentJobFilename);
   
   // Close the file
   if (currentJobFile) {
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
 
 bool JobManager::isJobRunning() const {
   return (jobStatus == JOB_RUNNING || jobStatus == JOB_PAUSED);
 }
 
 String JobManager::getCurrentJobName() const {
   return currentJobFilename;
 }
 
 int JobManager::getJobProgress() const {
   if (totalJobLines <= 0 || currentLineNumber <= 0) {
     return 0;
   }
   
   int progress = (currentLineNumber * 100) / totalJobLines;
   
   // Cap progress at 99% until job is marked completed
   if (progress >= 100 && jobStatus != JOB_COMPLETED) {
     progress = 99;
   }
   
   return progress;
 }
 
 JobStatus JobManager::getJobStatus() const {
   return jobStatus;
 }
 
 void JobManager::setBufferThreshold(int threshold) {
   bufferThreshold = max(1, threshold);
   Debug::verbose("JobManager", "Buffer threshold set to " + String(bufferThreshold));
 }
 
 void JobManager::setLinesPerBatch(int lines) {
   linesPerBatch = max(1, lines);
   Debug::verbose("JobManager", "Lines per batch set to " + String(linesPerBatch));
 }
 
 bool JobManager::canStartNewJob() const {
   // Can't start a new job if one is already running
   if (jobStatus != JOB_IDLE && jobStatus != JOB_COMPLETED) {
     return false;
   }
   
   // Only start a new job if command queue is empty or nearly empty
   return (commandQueue->size() <= bufferThreshold);
 }
 
 int JobManager::countJobLines(const String& filename) {
   if (!fileManager) {
     Debug::error("JobManager", "File manager not available");
     return -1;
   }
   
   File file = fileManager->openFile(filename);
   if (!file) {
     Debug::error("JobManager", "Failed to open file for counting lines: " + filename);
     return -1;
   }
   
   int lineCount = 0;
   int validCommands = 0;
   String line;
   
   // Count total lines and valid commands
   while (fileManager->readLine(file, line)) {
     lineCount++;
     
     // Process line to check if it's a valid command
     String processed = processGCodeLine(line);
     if (processed.length() > 0) {
       validCommands++;
     }
     
     // Limit line counting to avoid long delays
     if (lineCount >= 10000) {
       Debug::warning("JobManager", "Job file has more than 10000 lines, estimation stopped");
       break;
     }
   }
   
   file.close();
   
   Debug::info("JobManager", "Job file has " + String(lineCount) + " lines, " + 
               String(validCommands) + " valid commands");
   
   return lineCount;
 }
 
 int JobManager::loadNextCommandBatch() {
   if (!currentJobFile || !commandQueue) {
     Debug::error("JobManager", "Cannot load commands: File or queue not available");
     return -1;
   }
   
   if (jobStatus != JOB_RUNNING) {
     // Don't load commands when paused or stopped
     return 0;
   }
   
   int loadedCount = 0;
   String line;
   
   // Load up to linesPerBatch valid commands
   for (int i = 0; i < linesPerBatch; i++) {
     // Check if we've reached EOF
     if (!fileManager->readLine(currentJobFile, line)) {
       // EOF reached
       currentJobFile.close();
       Debug::info("JobManager", "End of job file reached");
       break;
     }
     
     // Update line number
     currentLineNumber++;
     
     // Process the line to remove comments and check if it's a valid command
     String processedLine = processGCodeLine(line);
     if (processedLine.length() > 0) {
       // Queue the command
       if (!commandQueue->push(processedLine, MOTION)) {
         Debug::warning("JobManager", "Command queue full, will retry later");
         break;
       }
       
       validCommandCount++;
       loadedCount++;
     }
   }
   
   // Save job state periodically
   if (currentLineNumber % 100 == 0) {
     saveJobState();
   }
   
   return loadedCount;
 }
 
 String JobManager::processGCodeLine(const String& line) {
   String processedLine = line;
   
   // Remove comments
   int commentIdx = processedLine.indexOf(';');
   if (commentIdx >= 0) {
     processedLine = processedLine.substring(0, commentIdx);
   }
   
   // Trim whitespace
   processedLine.trim();
   
   // Skip empty lines
   if (processedLine.length() == 0) {
     return "";
   }
   
   return processedLine;
 }
 
 bool JobManager::saveJobState() {
   if (currentJobFilename.length() == 0) {
     return false;
   }
   
   return fileManager->createResumeFile(currentJobFilename, currentLineNumber);
 }