/**
 * @file CPUMonitor.cpp
 * @brief Implementation of the CPUMonitor class
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
     
     // Get current run time stats for all tasks
     TaskStatus_t *taskStats = nullptr;
     uint32_t totalRunTime = 0;
     uint32_t taskCount = uxTaskGetNumberOfTasks();
     
     // Allocate memory for task statistics
     taskStats = (TaskStatus_t *)malloc(sizeof(TaskStatus_t) * taskCount);
     
     if (taskStats == nullptr) {
         Debug::error("CPUMonitor", "Failed to allocate memory for task stats");
         return false;
     }
     
     // Get task statistics
     taskCount = uxTaskGetSystemState(taskStats, taskCount, &totalRunTime);
     
     if (totalRunTime == 0) {
         free(taskStats);
         return false;
     }
     
     // Find idle task run times for each core
     uint32_t currentIdleTimeCore0 = 0;
     uint32_t currentIdleTimeCore1 = 0;
     
     for (uint32_t i = 0; i < taskCount; i++) {
         if (taskStats[i].xHandle == idleTaskHandleCore0) {
             currentIdleTimeCore0 = taskStats[i].ulRunTimeCounter;
         }
         else if (taskStats[i].xHandle == idleTaskHandleCore1) {
             currentIdleTimeCore1 = taskStats[i].ulRunTimeCounter;
         }
     }
     
     // Calculate CPU usage percentage for each core
     if (previousTotalTimeCore0 > 0) {
         uint32_t totalDeltaCore0 = totalRunTime - previousTotalTimeCore0;
         uint32_t idleDeltaCore0 = currentIdleTimeCore0 - previousIdleTimeCore0;
         
         if (totalDeltaCore0 > 0) {
             cpuUsageCore0 = 100.0f - (idleDeltaCore0 * 100.0f / totalDeltaCore0);
         }
     }
     
     if (previousTotalTimeCore1 > 0) {
         uint32_t totalDeltaCore1 = totalRunTime - previousTotalTimeCore1;
         uint32_t idleDeltaCore1 = currentIdleTimeCore1 - previousIdleTimeCore1;
         
         if (totalDeltaCore1 > 0) {
             cpuUsageCore1 = 100.0f - (idleDeltaCore1 * 100.0f / totalDeltaCore1);
         }
     }
     
     // Calculate total CPU usage (average of both cores)
     totalCpuUsage = (cpuUsageCore0 + cpuUsageCore1) / 2.0f;
     
     // Update previous values for next calculation
     previousIdleTimeCore0 = currentIdleTimeCore0;
     previousIdleTimeCore1 = currentIdleTimeCore1;
     previousTotalTimeCore0 = totalRunTime;
     previousTotalTimeCore1 = totalRunTime;
     
     // Update timestamp
     lastUpdateTime = currentTime;
     
     // Free allocated memory
     free(taskStats);
     
     return true;
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