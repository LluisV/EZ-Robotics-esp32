/**
 * @file CPUMonitor.h
 * @brief Accurate CPU usage monitoring for ESP32 dual-core systems
 */

 #ifndef CPU_MONITOR_H
 #define CPU_MONITOR_H
 
 #include <Arduino.h>
 #include <freertos/FreeRTOS.h>
 #include <freertos/task.h>
 #include "esp_timer.h"
 #include "Debug.h"
 
 class CPUMonitor {
 private:
     // Store previous idle task times for each core
     uint32_t previousIdleTimeCore0;
     uint32_t previousIdleTimeCore1;
     
     // Store previous total times
     uint32_t previousTotalTimeCore0;
     uint32_t previousTotalTimeCore1;
     
     // Store current CPU usage percentages
     float cpuUsageCore0;
     float cpuUsageCore1;
     float totalCpuUsage;
     
     // Timestamp of last update
     uint32_t lastUpdateTime;
     
     // Idle task handles for each core
     TaskHandle_t idleTaskHandleCore0;
     TaskHandle_t idleTaskHandleCore1;
     
     // Update interval in milliseconds
     const uint32_t updateInterval = 1000;
     
     // Singleton instance
     static CPUMonitor* instance;
     
     // Private constructor (singleton)
     CPUMonitor();
     
     /**
      * @brief Estimate CPU load for a specific core
      * @param coreId Core to estimate (0 or 1)
      * @return Estimated CPU usage percentage (0-100)
      */
     float estimateCoreLoad(int coreId);
           
 public:
     // Singleton getter
     static CPUMonitor* getInstance();
     
     /**
      * @brief Initialize the CPU monitor
      * @return True if initialization was successful
      */
     bool begin();
     
     /**
      * @brief Update CPU usage measurements
      * @param forceUpdate Force update regardless of interval
      * @return True if measurements were updated
      */
     bool update(bool forceUpdate = false);
     
     /**
      * @brief Get CPU usage for Core 0
      * @return CPU usage percentage (0-100)
      */
     float getCore0Usage() const;
     
     /**
      * @brief Get CPU usage for Core 1
      * @return CPU usage percentage (0-100)
      */
     float getCore1Usage() const;
     
     /**
      * @brief Get total CPU usage (average of both cores)
      * @return CPU usage percentage (0-100)
      */
     float getTotalUsage() const;
     
     /**
      * @brief Get detailed CPU information as a formatted string
      * @return String with CPU usage details
      */
     String getDetailedInfo() const;
 };
 
 #endif // CPU_MONITOR_H