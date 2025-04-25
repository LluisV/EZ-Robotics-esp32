/**
 * @file JobManager.h
 * @brief Job manager for running G-code files
 */

#ifndef JOB_MANAGER_H
#define JOB_MANAGER_H

#include <Arduino.h>
#include "CommandQueue.h"
#include "FileManager.h"
#include "Debug.h"
#include "GCodeValidator.h"
#include "CommandProcessor.h"

// Forward declarations
class FileManager;

// Maximum line length for G-code line
#define MAX_GCODE_LINE_LENGTH 96

// Enumeration for job status
enum JobStatus
{
  JOB_IDLE,      ///< No job running
  JOB_RUNNING,   ///< Job is running
  JOB_PAUSED,    ///< Job is paused
  JOB_COMPLETED, ///< Job completed successfully
  JOB_ERROR      ///< Job error
};

/**
 * @brief Job manager class for running G-code files
 */
class JobManager
{
public:
  /**
   * @brief Construct a new Job Manager
   * @param commandQueue Reference to the command queue
   * @param fileManager Reference to the file manager
   */
  JobManager(CommandQueue *commandQueue, FileManager *fileManager);

  /**
   * @brief Set the G-code validator
   * @param validator G-code validator reference
   */
  void setGCodeValidator(GCodeValidator *validator)
  {
    this->gCodeValidator = validator;
  }

  /**
   * @brief Update function to be called regularly
   * @return True if successful, false otherwise
   */
  bool update();

  /**
   * @brief Start a job from a G-code file
   * @param filename Name of the G-code file
   * @return True if successful, false otherwise
   */
  bool startJob(const String &filename);

  /**
   * @brief Pause the current job
   * @return True if successful, false otherwise
   */
  bool pauseJob();

  /**
   * @brief Resume the current job
   * @return True if successful, false otherwise
   */
  bool resumeJob();

  /**
   * @brief Stop the current job
   * @return True if successful, false otherwise
   */
  bool stopJob();

  /**
   * @brief Check if a job is running
   * @return True if a job is running, false otherwise
   */
  bool isJobRunning() const;

  /**
   * @brief Get the current job name
   * @return Name of the current job file
   */
  String getCurrentJobName() const;

  /**
   * @brief Get the job progress
   * @return Progress in percent (0-100)
   */
  int getJobProgress() const;

  /**
   * @brief Get the job status
   * @return Job status
   */
  JobStatus getJobStatus() const;

  /**
   * @brief Set buffer threshold for getting more lines
   * @param threshold Number of commands in queue before requesting more
   */
  void setBufferThreshold(int threshold);

  /**
   * @brief Set lines to load per batch
   * @param lines Number of lines to load per batch
   */
  void setLinesPerBatch(int lines);

  /**
   * @brief Check if system can accept a new job
   * @return True if system is ready for a new job, false otherwise
   */
  bool canStartNewJob() const;

  /**
   * @brief Validate a G-code file before running
   * @param filename Name of the G-code file
   * @return Validation result
   */
  ValidationResult validateGCodeFile(const String &filename);

  /**
   * @brief Get last validation result
   * @return Last validation result
   */
  ValidationResult getLastValidationResult() const
  {
    return lastValidationResult;
  }

  /**
   * @brief Emergency abort current job and reset machine state
   * @param errorMessage Error message to log
   */
  void emergencyAbortJob(const String &errorMessage);

  /**
   * @brief Set the command processor
   * @param processor Command processor reference
   */
  void setCommandProcessor(CommandProcessor *processor)
  {
    commandProcessor = processor;
  }

  /**
   * @brief Get the G-code validator
   * @return Pointer to G-code validator or nullptr
   */
  GCodeValidator *getGCodeValidator() const
  {
    return gCodeValidator;
  }

private:
  CommandQueue *commandQueue; ///< Reference to the command queue
  FileManager *fileManager;   ///< Reference to the file manager

  File currentJobFile;       ///< Current job file
  String currentJobFilename; ///< Current job filename
  JobStatus jobStatus;       ///< Current job status

  int currentLineNumber; ///< Current line number being processed
  int totalJobLines;     ///< Total number of lines in the job (estimate)
  int validCommandCount; ///< Count of valid commands (not comments or empty lines)

  int bufferThreshold; ///< Threshold for loading more lines
  int linesPerBatch;   ///< Number of lines to load per batch

  bool emergencyStopRequested;    ///< Flag for emergency stop
  bool jobCompletionAcknowledged; ///< Flag for job completion acknowledgment

  GCodeValidator *gCodeValidator = nullptr;     ///< G-code validator reference
  ValidationResult lastValidationResult;        ///< Last validation result
  CommandProcessor *commandProcessor = nullptr; ///< Command processor reference

  /**
   * @brief Reset machine state after an error
   */
  void resetMachineState();

  /**
   * @brief Count total lines and estimate job size
   * @param filename Name of the G-code file
   * @return Total line count or -1 if error
   */
  int countJobLines(const String &filename);

  /**
   * @brief Load next batch of commands into queue
   * @return Number of commands loaded or -1 if error
   */
  int loadNextCommandBatch();

  /**
   * @brief Process a G-code line (filtering comments and empty lines)
   * @param line Line to process
   * @return Processed line or empty string if line should be skipped
   */
  String processGCodeLine(const String &line);

  /**
   * @brief Save job state for possible resume
   * @return True if successful, false otherwise
   */
  bool saveJobState();

  /**
   * @brief Read a line from a file directly into a buffer without using String
   * @param file File reference
   * @param buffer Buffer to store the line
   * @param bufferSize Size of the buffer
   * @return True if a line was read, false if EOF
   */
  bool readLineToBuffer(File &file, char *buffer, size_t bufferSize);
  
  /**
   * @brief Check if a line is empty (contains only whitespace)
   * @param line Null-terminated C-string to check
   * @return True if line is empty or contains only whitespace
   */
  bool isEmptyLine(const char *line);
  
  /**
   * @brief Check if a line is a comment line (begins with ';')
   * @param line Null-terminated C-string to check
   * @return True if line is a comment line
   */
  bool isCommentLine(const char *line);
  
  /**
   * @brief Process a G-code line in-place (remove comments and trim whitespace)
   * @param line Null-terminated C-string to process
   */
  void processLineInPlace(char *line);
};

#endif // JOB_MANAGER_H