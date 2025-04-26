/**
 * @file CommunicationManager.h
 * @brief Simplified communication manager for GRBL protocol
 */

 #ifndef COMMUNICATION_MANAGER_H
 #define COMMUNICATION_MANAGER_H
 
 #include <Arduino.h>
 #include "CommandQueue.h"
 #include "CommandProcessor.h"
 #include "MachineController.h"
 #include "Debug.h"
 #include "CPUMonitor.h"
 
 // Maximum line length for receiving commands
 #define LINE_BUFFER_SIZE 128
 
 class CommunicationManager
 {
 public:
   /**
    * @brief Construct a new CommunicationManager
    * @param commandQueue Reference to the command queue
    * @param commandProcessor Reference to the command processor
    */
   CommunicationManager(CommandQueue *commandQueue, CommandProcessor *commandProcessor);
 
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
   CommandQueue *commandQueue;         ///< Reference to the command queue
   CommandProcessor *commandProcessor; ///< Reference to the command processor
   MachineController *machineController; ///< Reference to the machine controller
   CPUMonitor *cpuMonitor;             ///< CPU monitor reference
   
   char lineBuffer[LINE_BUFFER_SIZE];  ///< Buffer for incoming data
   int lineBufferIndex;                ///< Current position in line buffer
   
   bool telemetryEnabled;              ///< Telemetry enabled flag
   unsigned long telemetryInterval;    ///< Telemetry interval in milliseconds
   unsigned long lastTelemetryTime;    ///< Last telemetry update time
   
   String telemetryBuffer;             ///< Buffer for telemetry message
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
 };
 
 #endif // COMMUNICATION_MANAGER_H