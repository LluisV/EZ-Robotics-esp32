/**
 * @file CommandQueue.cpp
 * @brief Implementation of the CommandQueue class with priority support
 */

 #include "CommandQueue.h"

 CommandQueue::CommandQueue(size_t size)
   : maxSize(size),
     motionHead(0), motionTail(0), motionCount(0),
     settingHead(0), settingTail(0), settingCount(0),
     infoHead(0), infoTail(0), infoCount(0),
     immedHead(0), immedTail(0), immedCount(0)
 {
   // Initialize queues
   motionQueue.resize(maxSize);
   settingQueue.resize(maxSize);
   infoQueue.resize(maxSize);
   immediateQueue.resize(maxSize);
   
   // Create mutex for thread safety
   mutex = xSemaphoreCreateMutex();
 }
 
 bool CommandQueue::push(const String& command, CommandType type) {
   if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
     return false;
   }
   
   bool result = false;
   
   switch (type) {
     case MOTION:
       result = pushToQueue(motionQueue, motionHead, motionTail, motionCount, command);
       break;
     case SETTING:
       result = pushToQueue(settingQueue, settingHead, settingTail, settingCount, command);
       break;
     case INFO:
       result = pushToQueue(infoQueue, infoHead, infoTail, infoCount, command);
       break;
     case IMMEDIATE:
       result = pushToQueue(immediateQueue, immedHead, immedTail, immedCount, command);
       break;
   }
   
   xSemaphoreGive(mutex);
   return result;
 }
 
 String CommandQueue::pop() {
   if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
     return "";
   }
   
   String result = "";
   
   // Check queues in priority order: IMMEDIATE > INFO > SETTING > MOTION
   if (immedCount > 0) {
     result = popFromQueue(immediateQueue, immedHead, immedTail, immedCount);
   } else if (infoCount > 0) {
     result = popFromQueue(infoQueue, infoHead, infoTail, infoCount);
   } else if (settingCount > 0) {
     result = popFromQueue(settingQueue, settingHead, settingTail, settingCount);
   } else if (motionCount > 0) {
     result = popFromQueue(motionQueue, motionHead, motionTail, motionCount);
   }
   
   xSemaphoreGive(mutex);
   return result;
 }
 
 String CommandQueue::peekImmediate() const {
   if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
     return "";
   }
   
   String result = "";
   
   if (immedCount > 0) {
     result = immediateQueue[immedHead].command;
   }
   
   xSemaphoreGive(mutex);
   return result;
 }
 
 String CommandQueue::getNextImmediate() {
   if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
     return "";
   }
   
   String result = "";
   
   if (immedCount > 0) {
     result = popFromQueue(immediateQueue, immedHead, immedTail, immedCount);
   }
   
   xSemaphoreGive(mutex);
   return result;
 }
 
 bool CommandQueue::isEmpty() const {
   if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
     return true;
   }
   
   bool empty = (motionCount == 0 && settingCount == 0 && 
                 infoCount == 0 && immedCount == 0);
   
   xSemaphoreGive(mutex);
   return empty;
 }
 
 bool CommandQueue::isFull(CommandType type) const {
   if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
     return true;
   }
   
   bool full = false;
   
   switch (type) {
     case MOTION:
       full = (motionCount >= maxSize);
       break;
     case SETTING:
       full = (settingCount >= maxSize);
       break;
     case INFO:
       full = (infoCount >= maxSize);
       break;
     case IMMEDIATE:
       full = (immedCount >= maxSize);
       break;
   }
   
   xSemaphoreGive(mutex);
   return full;
 }
 
 size_t CommandQueue::size() const {
   if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
     return 0;
   }
   
   size_t totalSize = motionCount + settingCount + infoCount + immedCount;
   
   xSemaphoreGive(mutex);
   return totalSize;
 }
 
 void CommandQueue::clear() {
   if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
     return;
   }
   
   // Reset all queue pointers
   motionHead = 0; motionTail = 0; motionCount = 0;
   settingHead = 0; settingTail = 0; settingCount = 0;
   infoHead = 0; infoTail = 0; infoCount = 0;
   immedHead = 0; immedTail = 0; immedCount = 0;
   
   xSemaphoreGive(mutex);
 }
 
 bool CommandQueue::pushToQueue(std::vector<QueueItem>& queue, size_t& head, 
                              size_t& tail, size_t& count, const String& command) {
   if (count >= maxSize) {
     return false;
   }
   
   QueueItem item;
   item.command = command;
   item.timestamp = millis(); // Record timestamp for potential timeout features
   
   queue[tail] = item;
   tail = (tail + 1) % maxSize;
   count++;
   
   return true;
 }
 
 String CommandQueue::popFromQueue(std::vector<QueueItem>& queue, size_t& head, 
                                 size_t& tail, size_t& count) {
   if (count == 0) {
     return "";
   }
   
   String result = queue[head].command;
   head = (head + 1) % maxSize;
   count--;
   
   return result;
 }
 
 bool CommandQueue::isQueueEmpty(size_t count) const {
   return count == 0;
 }