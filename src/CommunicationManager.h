/**
 * @file CommunicationManager.h
 * @brief Communication manager for handling serial and future websocket connections
 */

#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <Arduino.h>
#include "CommandQueue.h"
#include "CommandProcessor.h"
#include "FileManager.h"
#include "JobManager.h"
#include "Debug.h"

// Forward declarations
class FileManager;
class JobManager;

// Maximum line length for receiving commands
#define MAX_LINE_LENGTH 128

// Enumeration for file transfer modes
enum FileTransferMode
{
  TRANSFER_IDLE,      ///< No file transfer in progress
  TRANSFER_RECEIVING, ///< Receiving a file from serial
  TRANSFER_SENDING    ///< Sending a file to serial
};

// Structure to hold file transfer state
struct FileTransferState
{
  FileTransferMode mode;    ///< Current file transfer mode
  String filename;          ///< Name of the file being transferred
  size_t bytesTransferred;  ///< Number of bytes transferred
  size_t fileSize;          ///< Total file size in bytes
  unsigned long startTime;  ///< Transfer start time
  unsigned long lastUpdate; ///< Last update time for timeout tracking
  bool error;               ///< Error flag
  String errorMessage;      ///< Error message if any
};

/**
 * @brief Communication manager class for handling serial and future websocket connections
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

  int getTelemetryFrequency() const
  {
    return telemetryFrequency;
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
  int telemetryFrequency = 30;             ///< Telemetry update frequency in Hz

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