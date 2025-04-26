/**
 * @file CommunicationManager.h
 * @brief Communication manager implementing protocol and status reporting
 */

 #ifndef COMMUNICATION_MANAGER_H
 #define COMMUNICATION_MANAGER_H
 
 #include <Arduino.h>
 #include "LineBuffer.h"
 #include "MachineController.h"
 #include "Debug.h"
 #include "CPUMonitor.h"
 
 // Maximum line length for receiving commands
 #define SERIAL_LINE_BUFFER_SIZE 128
 
 class CommunicationManager
 {
 public:
     /**
      * @brief Construct a new CommunicationManager
      * @param lineBuffer Reference to the line buffer
      * @param machineController Reference to the machine controller
      */
     CommunicationManager(LineBuffer *lineBuffer, MachineController *machineController);
 
     /**
      * @brief Initialize the communication manager
      * @param baudRate Serial baud rate
      * @return True if successful
      */
     bool initialize(unsigned long baudRate = 115200);
 
     /**
      * @brief Update function to be called regularly
      * @return True if successful
      */
     bool update();
 
     /**
      * @brief Send a message through serial
      * @param message Message to send
      */
     void sendMessage(const String &message);
 
     /**
      * @brief Process a realtime command character
      * @param c Realtime command character
      * @return True if character was processed as a realtime command
      */
     bool processRealtimeCharacter(char c);
 
     /**
      * @brief Send an acknowledgment for a received line
      */
     void sendAcknowledgment();
 
     /**
      * @brief Send an error message
      * @param errorCode Error code
      * @param errorMessage Optional error message
      */
     void sendError(int errorCode, const String &errorMessage = "");
 
     /**
      * @brief Enable or disable telemetry
      * @param enabled True to enable, false to disable
      */
     void setTelemetryEnabled(bool enabled);
 
     /**
      * @brief Set telemetry update interval
      * @param intervalMs Interval in milliseconds
      */
     void setTelemetryInterval(unsigned long intervalMs);
 
     /**
      * @brief Set telemetry frequency in Hz
      * @param frequency Frequency in Hz
      */
     void setTelemetryFrequency(int frequency);
 
     /**
      * @brief Set status report interval in milliseconds
      * @param intervalMs Interval in milliseconds (0 to disable auto-reporting)
      */
     void setStatusReportInterval(unsigned long intervalMs);
 
 private:
     LineBuffer *lineBuffer;                ///< Reference to the line buffer
     MachineController *machineController;  ///< Reference to the machine controller
     CPUMonitor *cpuMonitor;                ///< CPU monitor reference
     
     char serialLineBuffer[SERIAL_LINE_BUFFER_SIZE]; ///< Buffer for incoming serial data
     int serialLineBufferIndex;                    ///< Current position in serial line buffer
     
     bool telemetryEnabled;                ///< Telemetry enabled flag
     unsigned long telemetryInterval;      ///< Telemetry interval in milliseconds
     unsigned long lastTelemetryTime;      ///< Last telemetry update time
     unsigned long lastStatusReportTime;   ///< Last status report time
     unsigned long statusReportInterval;   ///< Status report interval in milliseconds
     
     String telemetryBuffer;               ///< Buffer for telemetry message
     std::vector<float> lastReportedPosition; ///< Last reported position for change detection
 
     /**
      * @brief Process a complete line from serial
      * @param line Line to process
      */
     void processLine(const String &line);
 
     /**
      * @brief Update and send telemetry data if needed
      */
     void updateTelemetry();
 
     /**
      * @brief Process an information query command
      * @param command Information command
      * @return Response string
      */
     String processInfoCommand(const String &command);
 };
 
 #endif // COMMUNICATION_MANAGER_H