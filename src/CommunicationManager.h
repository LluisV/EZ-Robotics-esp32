/**
 * @file GRBLCommunicationManager.h
 * @brief Communication manager with GRBL protocol support
 */

 #ifndef GRBL_COMMUNICATION_MANAGER_H
 #define GRBL_COMMUNICATION_MANAGER_H
 
 #include <Arduino.h>
 #include "CommandQueue.h"
 #include "CommandProcessor.h"
 #include "MachineController.h"
 #include "FileManager.h"
 #include "JobManager.h"
 #include "Debug.h"
 #include "CPUMonitor.h"
 
 // Maximum line length for receiving commands
 #define GRBL_LINE_BUFFER_SIZE 128
 
 /**
  * @brief Communication manager class for GRBL protocol
  */
 class GRBLCommunicationManager
 {
 public:
     /**
      * @brief Construct a new GRBLCommunicationManager
      * @param commandQueue Reference to the command queue
      * @param commandProcessor Reference to the command processor
      * @param fileManager Reference to the file manager
      * @param jobManager Reference to the job manager
      */
     GRBLCommunicationManager(CommandQueue *commandQueue, CommandProcessor *commandProcessor,
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
      * @brief Send GRBL status report
      * @param detailed If true, send detailed status
      */
     void sendStatusReport(bool detailed = false);
 
     /**
      * @brief Set status report interval in milliseconds
      * @param intervalMs Interval in milliseconds (0 to disable auto-reporting)
      */
     void setStatusReportInterval(unsigned long intervalMs);
 
 private:
     CommandQueue *commandQueue;         ///< Reference to the command queue
     CommandProcessor *commandProcessor; ///< Reference to the command processor
     MachineController *machineController; ///< Reference to the machine controller
     FileManager *fileManager;           ///< Reference to the file manager
     JobManager *jobManager;             ///< Reference to the job manager
 
     char lineBuffer[GRBL_LINE_BUFFER_SIZE]; ///< Buffer for receiving characters
     int lineBufferIndex;                    ///< Current position in line buffer
 
     // GRBL protocol state
     bool autoReportEnabled;                 ///< Status auto-reporting enabled
     unsigned long autoReportIntervalMs;     ///< Status auto-reporting interval
     unsigned long lastAutoReportTime;       ///< Last auto-report time
     bool feedHoldActive;                    ///< Feedhold active flag
     
     // State tracking for command acknowledgment
     int lineNumber;                         ///< Current line number (if supported)
     bool awaitingAck;                       ///< Waiting to send acknowledgment
 
     CPUMonitor *cpuMonitor;                 ///< CPU usage monitor
 
     /**
      * @brief Process a complete line from serial
      * @param line Line to process
      */
     void processLine(const String &line);
 
     /**
      * @brief Process a realtime command character
      * @param c The realtime command character
      */
     void processRealtimeCommand(char c);
 
     /**
      * @brief Reset the line buffer
      */
     void resetLineBuffer();
 
     /**
      * @brief Send an acknowledgment for a processed command
      * @param success True if command was successful, false if error
      * @param errorCode Error code if success is false
      */
     void sendAcknowledgment(bool success, int errorCode = 0);
 
     /**
      * @brief Compose a GRBL-compatible status report
      * @return Formatted status report string
      */
     String composeStatusReport();
 
     /**
      * @brief Compose a GRBL-compatible alarm message
      * @param alarmCode Alarm code
      * @return Formatted alarm message
      */
     String composeAlarmMessage(int alarmCode);
 
     /**
      * @brief Handle GRBL system commands ($ commands)
      * @param line Command line
      * @return True if command was handled
      */
     bool handleSystemCommand(const String &line);
 };
 
 #endif // GRBL_COMMUNICATION_MANAGER_H