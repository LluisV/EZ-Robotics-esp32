/**
 * @file LineBuffer.h
 * @brief Line buffer for receiving and managing GCode commands
 */

 #ifndef LINE_BUFFER_H
 #define LINE_BUFFER_H
 
 #include <Arduino.h>
 #include <vector>
 #include <mutex>
 #include "Debug.h"
 
 // Maximum line length and buffer capacity
 #define MAX_LINE_LENGTH 128
 #define LINE_BUFFER_CAPACITY 32
 
 /**
  * @brief Line buffer for GCode commands
  * 
  * Implements a thread-safe buffer for storing incoming GCode commands
  * from the sender until they can be processed by the parser.
  */
 class LineBuffer {
 public:
     /**
      * @brief Constructor
      */
     LineBuffer();
 
     /**
      * @brief Add a line to the buffer
      * @param line Line to add
      * @return True if line was added, false if buffer is full
      */
     bool addLine(const String &line);
 
     /**
      * @brief Get the next line from the buffer
      * @return Next line or empty string if buffer is empty
      */
     String getLine();
 
     /**
      * @brief Check if the buffer is empty
      * @return True if buffer is empty
      */
     bool isEmpty() const;
 
     /**
      * @brief Get the number of lines in the buffer
      * @return Number of lines
      */
     size_t size() const;
 
     /**
      * @brief Get the available space in the buffer
      * @return Number of available slots
      */
     size_t availableSpace() const;
 
     /**
      * @brief Clear the buffer
      */
     void clear();
 
 private:
     std::vector<String> lines;            ///< Buffer of lines
     mutable SemaphoreHandle_t mutex;      ///< Mutex for thread safety
 };
 
 #endif // LINE_BUFFER_H