/**
 * @file Debug.cpp
 * @brief Implementation of the Debug class
 */

 #include "Debug.h"

 // Initialize static members
 bool Debug::enabled = false;
 uint8_t Debug::debugLevel = 2;
 Debug::Timer Debug::timers[MAX_TIMERS];
 int Debug::timerCount = 0;
 
 void Debug::begin(bool enable, unsigned long serialBaud) {
     enabled = enable;
     if (enabled) {
         if (!Serial) {
             Serial.begin(serialBaud);
             delay(100); // Give serial a moment to initialize
         }
         Serial.println("DEBUG: Debug system initialized");
     }
 }
 
 void Debug::setLevel(uint8_t level) {
     debugLevel = level;
     if (enabled) {
         Serial.printf("DEBUG: Debug level set to %d\n", debugLevel);
     }
 }
 
 void Debug::print(const String& component, const String& message, uint8_t level) {
     if (!enabled || level > debugLevel) {
         return;
     }
 
     // Get timestamp
     unsigned long ms = millis();
     unsigned long seconds = ms / 1000;
     unsigned int minutes = seconds / 60;
     unsigned int hours = minutes / 60;
     
     seconds %= 60;
     minutes %= 60;
     
     // Get current core
     int core = xPortGetCoreID();
     
     // Level strings
     const char* levelStr;
     switch (level) {
         case 0: levelStr = "ERROR"; break;
         case 1: levelStr = "WARN "; break;
         case 2: levelStr = "INFO "; break;
         case 3: levelStr = "DEBUG"; break;
         default: levelStr = "?????";
     }
     
     // Format: [HH:MM:SS.mmm][CORE n][LEVEL][Component] Message
     char timestamp[20];
     sprintf(timestamp, "[%02u:%02u:%02u.%03lu]", hours, minutes, seconds, ms % 1000);
     
     Serial.printf("%s[CORE %d][%s][%s] %s\n", 
                   timestamp, 
                   core, 
                   levelStr, 
                   component.c_str(), 
                   message.c_str());
 }
 
 void Debug::error(const String& component, const String& message) {
     print(component, message, 0);
 }
 
 void Debug::warning(const String& component, const String& message) {
     print(component, message, 1);
 }
 
 void Debug::info(const String& component, const String& message) {
     print(component, message, 2);
 }
 
 void Debug::verbose(const String& component, const String& message) {
     print(component, message, 3);
 }
 
 void Debug::printDiagnostics() {
    if (!enabled) {
        return;
    }
    
    info("System", "--- Diagnostics ---");
    info("System", "Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
    info("System", "Free PSRAM: " + String(ESP.getFreePsram()) + " bytes");
    info("System", "Uptime: " + String(millis() / 1000) + " seconds");
    
    // Task statistics if using FreeRTOS with TRACE facility enabled
    #if defined(configUSE_TRACE_FACILITY) && configUSE_TRACE_FACILITY == 1
    char taskStats[400] = {0};
    vTaskList(taskStats);
    info("System", "Tasks:");
    Serial.println(taskStats);
    #else
    info("System", "Tasks: (Task listing not available - TRACE facility not enabled)");
    #endif
    
    info("System", "------------------");
}
 
 bool Debug::isEnabled() {
     return enabled;
 }
 
 void Debug::timerStart(const String& component, const String& section) {
     if (!enabled) {
         return;
     }
     
     // Store start time
     String key = component + ":" + section;
     timers[timerCount].key = key;
     timers[timerCount].startTime = micros();
     timerCount = (timerCount + 1) % MAX_TIMERS;
     
     verbose(component, "Timer started: " + section);
 }
 
 void Debug::timerEnd(const String& component, const String& section) {
     if (!enabled) {
         return;
     }
     
     unsigned long endTime = micros();
     String key = component + ":" + section;
     
     // Find the timer
     for (int i = 0; i < MAX_TIMERS; i++) {
         if (timers[i].key == key) {
             unsigned long elapsed = endTime - timers[i].startTime;
             
             // Print timing information
             if (elapsed > 1000000) {
                 info(component, section + " completed in " + String(elapsed / 1000000.0, 3) + " seconds");
             } else if (elapsed > 1000) {
                 info(component, section + " completed in " + String(elapsed / 1000.0, 3) + " milliseconds");
             } else {
                 info(component, section + " completed in " + String(elapsed) + " microseconds");
             }
             
             // Clear the timer
             timers[i].key = "";
             break;
         }
     }
 }