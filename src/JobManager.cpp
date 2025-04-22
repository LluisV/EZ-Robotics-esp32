/**
 * @file JobManager.cpp
 * @brief Implementation of the JobManager class
 */

#include "JobManager.h"

JobManager::JobManager(CommandQueue *commandQueue, FileManager *fileManager)
    : commandQueue(commandQueue),
      fileManager(fileManager),
      jobStatus(JOB_IDLE),
      currentLineNumber(0),
      totalJobLines(0),
      validCommandCount(0),
      bufferThreshold(5), // Default threshold: load more when queue has 5 or fewer commands
      linesPerBatch(10),  // Default batch size: load 10 lines at a time
      emergencyStopRequested(false),
      jobCompletionAcknowledged(true)
{
}

bool JobManager::update()
{
  // Check if job is running and command queue is below threshold, load more commands
  if (jobStatus == JOB_RUNNING && commandQueue->size() <= bufferThreshold)
  {
    int loaded = loadNextCommandBatch();
    Debug::verbose("JobManager", "Loaded " + String(loaded) + " commands. Total progress: " 
                               + String(currentLineNumber) + "/" + String(totalJobLines) 
                               + " (" + String(getJobProgress()) + "%)");

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

  // Check if file exists
  if (!fileManager->fileExists(filename))
  {
    Debug::error("JobManager", "Job file not found: " + filename);
    return false;
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
  }
  else
  {
    // No validator available, fall back to counting lines
    totalJobLines = countJobLines(filename);
    Debug::warning("JobManager", "G-code validator not available. Proceeding without validation.");
  }

  if (totalJobLines <= 0)
  {
    Debug::error("JobManager", "Empty or invalid job file: " + filename);
    return false;
  }

  Debug::info("JobManager", "Starting job: " + filename + ", estimated lines: " + String(totalJobLines));

  // Open the file for reading
  currentJobFile = fileManager->openFile(filename);
  if (!currentJobFile)
  {
    Debug::error("JobManager", "Failed to reopen job file after validation: " + filename);
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
    Debug::warning("JobManager", "Cannot pause: No job running");
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
    Debug::warning("JobManager", "Cannot resume: No paused job");
    return false;
  }

  if (!currentJobFile)
  {
    Debug::error("JobManager", "Cannot resume: Job file not open");
    return false;
  }

  Debug::info("JobManager", "Resuming job: " + currentJobFilename);
  jobStatus = JOB_RUNNING;

  return true;
}

bool JobManager::stopJob()
{
  if (jobStatus == JOB_IDLE)
  {
    Debug::warning("JobManager", "Cannot stop: No job running");
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
  bufferThreshold = max(1, threshold);
  Debug::verbose("JobManager", "Buffer threshold set to " + String(bufferThreshold));
}

void JobManager::setLinesPerBatch(int lines)
{
  linesPerBatch = max(1, lines);
  Debug::verbose("JobManager", "Lines per batch set to " + String(linesPerBatch));
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

  int lineCount = 0;
  int validCommands = 0;
  String line;

  // Count total lines and valid commands
  while (fileManager->readLine(file, line))
  {
    lineCount++;

    // Process line to check if it's a valid command
    String processed = processGCodeLine(line);
    if (processed.length() > 0)
    {
      validCommands++;
    }

    // Limit line counting to avoid long delays
    if (lineCount >= 10000)
    {
      Debug::warning("JobManager", "Job file has more than 10000 lines, estimation stopped");
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
  // Verify file is open and readable
  if (!currentJobFile || !currentJobFile.available()) {
    Debug::error("JobManager", "File not open or no data available");
    if (currentJobFile) {
      currentJobFile.close();
    }
    // Try reopening the file if it was closed prematurely
    if (currentLineNumber < totalJobLines) {
      Debug::info("JobManager", "Attempting to reopen file at line " + String(currentLineNumber));
      currentJobFile = fileManager->openFile(currentJobFilename);
      if (currentJobFile) {
        // Skip to current line
        String line;
        for (int i = 0; i < currentLineNumber; i++) {
          if (!fileManager->readLine(currentJobFile, line)) {
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
  String line;

  // Load up to linesPerBatch valid commands
  for (int i = 0; i < linesPerBatch; i++)
  {
    // Check if we've reached EOF
    if (!fileManager->readLine(currentJobFile, line))
    {
      // EOF reached
      currentJobFile.close();
      Debug::info("JobManager", "End of job file reached");
      break;
    }

    // Update line number
    currentLineNumber++;

    // Process the line to remove comments and check if it's a valid command
    String processedLine = processGCodeLine(line);
    if (processedLine.length() > 0)
    {
      // Attempt to queue the command
      if (!commandQueue->push(processedLine, MOTION))
      {
        Debug::warning("JobManager", "Command queue full, will retry later");
        break;
      }

      validCommandCount++;
      loadedCount++;
    }
  }

  // Save job state periodically
  if (currentLineNumber % 100 == 0)
  {
    saveJobState();
  }

  return loadedCount;
}

String JobManager::processGCodeLine(const String &line)
{
  String processedLine = line;

  // Remove comments
  int commentIdx = processedLine.indexOf(';');
  if (commentIdx >= 0)
  {
    processedLine = processedLine.substring(0, commentIdx);
  }

  // Trim whitespace
  processedLine.trim();

  // Skip empty lines
  if (processedLine.length() == 0)
  {
    return "";
  }

  return processedLine;
}

bool JobManager::saveJobState()
{
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

  // Validate the file
  lastValidationResult = gCodeValidator->validateFile(filename);
  if (fileManager)
  {
    File file = fileManager->openFile(filename);
    if (file)
      file.close(); // Make sure to close any open validation file handle
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