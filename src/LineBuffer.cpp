/**
 * @file LineBuffer.cpp
 * @brief Implementation of the LineBuffer class
 */

 #include "LineBuffer.h"

 LineBuffer::LineBuffer() {
     // Initialize mutex for thread safety
     mutex = xSemaphoreCreateMutex();
     if (mutex == NULL) {
         Debug::error("LineBuffer", "Failed to create mutex");
     }
     
     // Reserve space to avoid reallocations
     lines.reserve(LINE_BUFFER_CAPACITY);
 }
 
 bool LineBuffer::addLine(const String &line) {
     if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
         Debug::warning("LineBuffer", "Failed to acquire mutex for addLine");
         return false;
     }
     
     bool result = false;
     
     // Check if there's space in the buffer
     if (lines.size() < LINE_BUFFER_CAPACITY) {
         // Limit line length to prevent buffer overruns
         String trimmedLine = line;
         if (trimmedLine.length() > MAX_LINE_LENGTH) {
             trimmedLine = trimmedLine.substring(0, MAX_LINE_LENGTH);
             Debug::warning("LineBuffer", "Line truncated to " + String(MAX_LINE_LENGTH) + " characters");
         }
         
         lines.push_back(trimmedLine);
         result = true;
     } else {
         Debug::warning("LineBuffer", "Buffer full, line rejected");
     }
     
     xSemaphoreGive(mutex);
     return result;
 }
 
 String LineBuffer::getLine() {
     if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
         Debug::warning("LineBuffer", "Failed to acquire mutex for getLine");
         return "";
     }
     
     String result = "";
     
     if (!lines.empty()) {
         result = lines.front();
         lines.erase(lines.begin());
     }
     
     xSemaphoreGive(mutex);
     return result;
 }
 
 bool LineBuffer::isEmpty() const {
     if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
         Debug::warning("LineBuffer", "Failed to acquire mutex for isEmpty");
         return true;  // Safer to return empty if we can't verify
     }
     
     bool empty = lines.empty();
     
     xSemaphoreGive(mutex);
     return empty;
 }
 
 size_t LineBuffer::size() const {
     if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
         Debug::warning("LineBuffer", "Failed to acquire mutex for size");
         return 0;
     }
     
     size_t count = lines.size();
     
     xSemaphoreGive(mutex);
     return count;
 }
 
 size_t LineBuffer::availableSpace() const {
     if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
         Debug::warning("LineBuffer", "Failed to acquire mutex for availableSpace");
         return 0;
     }
     
     size_t space = LINE_BUFFER_CAPACITY - lines.size();
     
     xSemaphoreGive(mutex);
     return space;
 }
 
 void LineBuffer::clear() {
     if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
         Debug::warning("LineBuffer", "Failed to acquire mutex for clear");
         return;
     }
     
     lines.clear();
     Debug::info("LineBuffer", "Buffer cleared");
     
     xSemaphoreGive(mutex);
 }