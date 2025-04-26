/**
 * @file GRBLCommunicationManager.h
 * @brief Communication manager with GRBL protocol support and JSON telemetry
 */

 #ifndef GRBL_COMMUNICATION_MANAGER_H
 #define GRBL_COMMUNICATION_MANAGER_H
 
 #include <Arduino.h>
 #include "CommandQueue.h"
 #include "CommandProcessor.h"
 #include "MachineController.h"
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
      */
     GRBLCommunicationManager(CommandQueue *commandQueue, CommandProcessor *commandProcessor);
 
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
 
     /**
      * @brief Send machine position telemetry in JSON format
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
      * @brief Set telemetry frequency in Hz
      * @param frequency Frequency in Hz
      */
     void setTelemetryFrequency(int frequency)
     {
         telemetryFrequency = frequency > 0 ? frequency : 1;
     }
 
     /**
      * @brief Enable or disable telemetry
      * @param enabled True to enable, false to disable
      */
     void setTelemetryEnabled(bool enabled)
     {
         telemetryEnabled = enabled;
     }
 
 private:
     CommandQueue *commandQueue;         ///< Reference to the command queue
     CommandProcessor *commandProcessor; ///< Reference to the command processor
     MachineController *machineController; ///< Reference to the machine controller
 
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
 
     // Telemetry settings
     bool telemetryEnabled;                  ///< Telemetry enabled flag
     int telemetryFrequency;                 ///< Telemetry frequency in Hz
     unsigned long lastTelemetryTime;        ///< Last telemetry time
     std::vector<float> lastReportedPosition; ///< Last reported position
     String telemetryMsgBuffer;              ///< Buffer for telemetry messages
 
     CPUMonitor *cpuMonitor;                 ///< CPU usage monitor
 
     /**
      * @brief Process a complete line from serial
      * @param line Line to process
      */
     void processLine(const String &line);
 
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
 
 };
 
 #endif // GRBL_COMMUNICATION_MANAGER_H