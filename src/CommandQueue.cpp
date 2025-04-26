/**
 * @file CommandQueue.cpp
 * @brief Implementation of the simplified CommandQueue class
 */

 #include "CommandQueue.h"
 #include "Debug.h"
 
 CommandQueue::CommandQueue(size_t size)
     : maxSize(size)
 {
   // Create mutex for thread safety
   mutex = xSemaphoreCreateMutex();
   if (mutex == NULL) {
     Debug::error("CommandQueue", "Failed to create mutex");
   }
 }
 
 bool CommandQueue::push(const String &command, CommandType type)
 {
   if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
     Debug::warning("CommandQueue", "Failed to acquire mutex for push");
     return false;
   }
 
   // Create a queue item
   QueueItem item;
   item.command = command;
   item.type = type;
   item.timestamp = millis();
 
   bool result = false;
   
   // Add to appropriate queue based on type
   if (type == IMMEDIATE) {
     // Always accept immediate commands as long as the queue isn't absurdly full
     if (immediateQueue.size() < maxSize) {
       immediateQueue.push_back(item);
       result = true;
     }
   } else {
     // For normal commands, check if normal queue has space
     if (normalQueue.size() < maxSize) {
       normalQueue.push_back(item);
       result = true;
     }
   }
 
   xSemaphoreGive(mutex);
   return result;
 }
 
 String CommandQueue::pop()
 {
   if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
     Debug::warning("CommandQueue", "Failed to acquire mutex for pop");
     return "";
   }
 
   String result = "";
 
   // Check queues in priority order: IMMEDIATE > NORMAL
   if (!immediateQueue.empty()) {
     result = immediateQueue.front().command;
     immediateQueue.pop_front();
   } else if (!normalQueue.empty()) {
     result = normalQueue.front().command;
     normalQueue.pop_front();
   }
 
   xSemaphoreGive(mutex);
   return result;
 }
 
 String CommandQueue::getNextImmediate()
 {
   if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
     Debug::warning("CommandQueue", "Failed to acquire mutex for getNextImmediate");
     return "";
   }
 
   String result = "";
 
   if (!immediateQueue.empty()) {
     result = immediateQueue.front().command;
     immediateQueue.pop_front();
   }
 
   xSemaphoreGive(mutex);
   return result;
 }
 
 bool CommandQueue::isEmpty() const
 {
   if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
     Debug::warning("CommandQueue", "Failed to acquire mutex for isEmpty");
     return true; // Safer to return empty if we can't verify
   }
 
   bool empty = immediateQueue.empty() && normalQueue.empty();
 
   xSemaphoreGive(mutex);
   return empty;
 }
 
 size_t CommandQueue::size() const
 {
   if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
     Debug::warning("CommandQueue", "Failed to acquire mutex for size");
     return 0;
   }
 
   size_t totalSize = immediateQueue.size() + normalQueue.size();
 
   xSemaphoreGive(mutex);
   return totalSize;
 }
 
 void CommandQueue::clear()
 {
   if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
     Debug::warning("CommandQueue", "Failed to acquire mutex for clear");
     return;
   }
 
   immediateQueue.clear();
   normalQueue.clear();
   Debug::info("CommandQueue", "All queues cleared");
 
   xSemaphoreGive(mutex);
 }