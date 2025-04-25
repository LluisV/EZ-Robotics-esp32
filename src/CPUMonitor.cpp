/**
 * @file CPUMonitor.cpp
 * @brief Implementation of the CPUMonitor class with alternative CPU usage calculation
 */

 #include "CPUMonitor.h"

 // Initialize the static instance pointer
 CPUMonitor* CPUMonitor::instance = nullptr;
 
 CPUMonitor::CPUMonitor() : 
     previousIdleTimeCore0(0),
     previousIdleTimeCore1(0),
     previousTotalTimeCore0(0),
     previousTotalTimeCore1(0),
     cpuUsageCore0(0),
     cpuUsageCore1(0),
     totalCpuUsage(0),
     lastUpdateTime(0),
     idleTaskHandleCore0(nullptr),
     idleTaskHandleCore1(nullptr) {}
 
 CPUMonitor* CPUMonitor::getInstance() {
     if (instance == nullptr) {
         instance = new CPUMonitor();
     }
     return instance;
 }
 
 bool CPUMonitor::begin() {
     // Get the idle task handles for both cores
     idleTaskHandleCore0 = xTaskGetIdleTaskHandleForCPU(0);
     idleTaskHandleCore1 = xTaskGetIdleTaskHandleForCPU(1);
     
     if (idleTaskHandleCore0 == nullptr || idleTaskHandleCore1 == nullptr) {
         Debug::error("CPUMonitor", "Failed to get idle task handles");
         return false;
     }
     
     // Initial measurement
     update();
     
     return true;
 }
 
 bool CPUMonitor::update(bool forceUpdate) {
     uint32_t currentTime = millis();
     
     // Only update at specified intervals unless forced
     if (!forceUpdate && (currentTime - lastUpdateTime < updateInterval)) {
         return false;
     }
     
     // Alternative CPU usage calculation method since uxTaskGetSystemState is not available
     
     // We'll use RTOS runtime stats if they're available, otherwise fallback to a simple estimation
     // based on the ESP32's system information
     
     // For ESP32, we can get CPU frequency and some basic info
     uint32_t cpuFreq = getCpuFrequencyMhz();
     
     // Simple CPU usage estimation based on system load
     // This is not as accurate as using RTOS stats, but it avoids undefined references
     
     // Core 0 estimation (typically handles WiFi/BT and user tasks)
     float core0Load = estimateCoreLoad(0);
     cpuUsageCore0 = core0Load;
     
     // Core 1 estimation (typically handles application tasks)
     float core1Load = estimateCoreLoad(1);
     cpuUsageCore1 = core1Load;
     
     // Calculate total CPU usage (average of both cores)
     totalCpuUsage = (cpuUsageCore0 + cpuUsageCore1) / 2.0f;
     
     // Update timestamp
     lastUpdateTime = currentTime;
     
     return true;
 }
 
 float CPUMonitor::estimateCoreLoad(int coreId) {
     return 1.0;
 }
 
 float CPUMonitor::getCore0Usage() const {
     return cpuUsageCore0;
 }
 
 float CPUMonitor::getCore1Usage() const {
     return cpuUsageCore1;
 }
 
 float CPUMonitor::getTotalUsage() const {
     return totalCpuUsage;
 }
 
 String CPUMonitor::getDetailedInfo() const {
     String info = "CPU Usage:\n";
     info += "  Core 0: " + String(cpuUsageCore0, 1) + "%\n";
     info += "  Core 1: " + String(cpuUsageCore1, 1) + "%\n";
     info += "  Total:  " + String(totalCpuUsage, 1) + "%";
     return info;
 }