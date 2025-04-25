/**
 * @file CommunicationManager.h
 * @brief Enhanced Communication manager with improved file transfer reliability
 */

#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <Arduino.h>
#include "CommandQueue.h"
#include "CommandProcessor.h"
#include "FileManager.h"
#include "JobManager.h"
#include "Debug.h"
#include "CPUMonitor.h"

// Forward declarations
class FileManager;
class JobManager;

// Maximum line length for receiving commands
#define MAX_LINE_LENGTH 128

// Maximum time between progress updates in milliseconds
#define PROGRESS_UPDATE_INTERVAL 1000

// Timeout for file transfers in milliseconds
#define FILE_TRANSFER_TIMEOUT 5000

// Size of file transfer receive buffer
#define FILE_TRANSFER_BUFFER_SIZE 512

// Enumeration for file transfer modes
enum FileTransferMode
{
  TRANSFER_IDLE,      ///< No file transfer in progress
  TRANSFER_RECEIVING, ///< Receiving a file from serial
  TRANSFER_SENDING    ///< Sending a file to serial
};

// Enumeration for file transfer error codes
enum FileTransferError
{
  TRANSFER_NO_ERROR,           ///< No error
  TRANSFER_TIMEOUT,            ///< Transfer timeout
  TRANSFER_FILE_OPEN_ERROR,    ///< Failed to open file
  TRANSFER_WRITE_ERROR,        ///< Failed to write data
  TRANSFER_SIZE_MISMATCH,      ///< File size doesn't match expected size
  TRANSFER_CANCELLED,          ///< Transfer cancelled by user
  TRANSFER_DEVICE_DISCONNECTED ///< Device disconnected during transfer
};

// Structure to hold file transfer state
struct FileTransferState
{
  FileTransferMode mode;            ///< Current file transfer mode
  String filename;                  ///< Name of the file being transferred
  size_t bytesTransferred;          ///< Number of bytes transferred
  size_t fileSize;                  ///< Total file size in bytes
  unsigned long startTime;          ///< Transfer start time
  unsigned long lastUpdate;         ///< Last update time for timeout tracking
  unsigned long lastProgressUpdate; ///< Last time progress was reported
  bool error;                       ///< Error flag
  String errorMessage;              ///< Error message if any
  FileTransferError errorCode;      ///< Error code
  uint8_t retryCount;               ///< Number of retries attempted
  uint8_t maxRetries;               ///< Maximum allowed retries
};

/**
 * @brief Enhanced communication manager class for reliable file transfers
 */
class CommunicationManager
{
public:
  /**
   * @brief Construct a new Communication Manager
   * @param commandQueue Reference to the command queue
   * @param commandProcessor Reference to the command processor
   * @param fileManager Reference to the file manager
   * @param jobManager Reference to the job manager
   */
  CommunicationManager(CommandQueue *commandQueue, CommandProcessor *commandProcessor,
                       FileManager *fileManager, JobManager *jobManager);

  /**
   * @brief Initialize the communication manager
   * @param baudRate Serial baud rate
   * @return True if successful, false otherwise
   */
  bool initialize(unsigned long baudRate = 115200);

  /**
   * @brief Update function to be called regularly
   * @return True if successful, false otherwise
   */
  bool update();

  /**
   * @brief Send a message through serial
   * @param message Message to send
   */
  void sendMessage(const String &message);

  /**
   * @brief Send a status update
   * @param detailed If true, send a detailed status
   */
  void sendStatusUpdate(bool detailed = false);

  /**
   * @brief Start receiving a file
   * @param filename Name of the file to receive
   * @param fileSize Size of the file in bytes
   * @return True if successful, false otherwise
   */
  bool startFileReceive(const String &filename, size_t fileSize);

  /**
   * @brief Start sending a file
   * @param filename Name of the file to send
   * @return True if successful, false otherwise
   */
  bool startFileSend(const String &filename);

  /**
   * @brief Cancel the current file transfer
   * @param reason Reason for cancellation
   */
  void cancelFileTransfer(const String &reason);

  /**
   * @brief Process a complete line from serial
   * @param line Line to process
   */
  void processLine(const String &line);

  /**
   * @brief Process a special command (with ! or ? prefix)
   * @param command Command to process
   * @return True if command was processed, false otherwise
   */
  bool processSpecialCommand(const String &command);

  /**
   * @brief Process a file command (with @ prefix)
   * @param command Command to process
   * @return True if command was processed, false otherwise
   */
  bool processFileCommand(const String &command);

  /**
   * @brief Send machine position telemetry
   * @param force Force sending telemetry even if no change or machine not moving
   */
  void sendPositionTelemetry(bool force = false);

  /**
   * @brief Get telemetry frequency in Hz
   * @return Current telemetry frequency
   */
  int getTelemetryFrequency() const
  {
    return telemetryFrequency;
  }

  /**
   * @brief Finalize a file transfer, ensuring all data is properly written
   */
  void finalizeFileTransfer();

  /**
   * @brief Retry a failed file transfer
   * @return True if retry was successful, false otherwise
   */
  bool retryFileTransfer();

  /**
   * @brief Reset all file transfer state
   */
  void resetFileTransfer();

  /**
   * @brief Get the current file transfer state
   * @return Current file transfer state
   */
  const FileTransferState &getFileTransferState() const
  {
    return fileTransfer;
  }

  /**
   * @brief Set the maximum retries for file transfers
   * @param maxRetries Maximum retry count
   */
  void setMaxFileTransferRetries(uint8_t maxRetries)
  {
    fileTransfer.maxRetries = maxRetries;
  }

private:
  CommandQueue *commandQueue;         ///< Reference to the command queue
  CommandProcessor *commandProcessor; ///< Reference to the command processor
  FileManager *fileManager;           ///< Reference to the file manager
  JobManager *jobManager;             ///< Reference to the job manager

  char lineBuffer[MAX_LINE_LENGTH]; ///< Buffer for receiving characters
  int lineBufferIndex;              ///< Current position in line buffer

  FileTransferState fileTransfer; ///< File transfer state

  unsigned long lastTelemetryTime = 0;     ///< Time of last telemetry update
  std::vector<float> lastReportedPosition; ///< Last reported position
  bool telemetryEnabled = true;            ///< Telemetry global enable flag
  int telemetryFrequency = 5;              ///< Telemetry update frequency in Hz

  // Pre-allocated buffers for telemetry
  std::vector<float> telemetryWorkBuffer;     // Pre-allocated buffer for work coordinates
  std::vector<float> telemetryWorldBuffer;    // Pre-allocated buffer for world coordinates
  std::vector<float> telemetryVelocityBuffer; // Pre-allocated buffer for velocity
  String telemetryMsgBuffer;                  // Pre-allocated string buffer for telemetry messages

  CPUMonitor *cpuMonitor;  // CPU usage monitor

  /**
   * @brief Handle receiving binary data for a file transfer
   * @return True if successful, false otherwise
   */
  bool handleFileReceiveData();

  /**
   * @brief Handle sending binary data for a file transfer
   * @return True if successful, false otherwise
   */
  bool handleFileSendData();

  /**
   * @brief Check for file transfer timeout
   * @return True if timed out, false otherwise
   */
  bool checkFileTransferTimeout();

  /**
   * @brief Reset the line buffer
   */
  void resetLineBuffer();
};

#endif // COMMUNICATION_MANAGER_H