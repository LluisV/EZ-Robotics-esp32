/**
 * @file CommandQueue.h
 * @brief Thread-safe command queue for G-code processing with simplified priority handling
 */

 #ifndef COMMAND_QUEUE_H
 #define COMMAND_QUEUE_H
 
 #include <Arduino.h>
 #include <deque>
 
 /**
  * @brief Command types with different priority levels
  */
 enum CommandType
 {
   NORMAL,     ///< Standard commands (G-code motion, settings, etc.)
   IMMEDIATE   ///< Emergency and high-priority commands (M112, etc.)
 };
 
 /**
  * @brief Command structure with priority
  */
 struct QueueItem
 {
   String command;              ///< G-code command
   CommandType type;            ///< Command type/priority
   unsigned long timestamp;     ///< Timestamp when command was added
 };
 
 /**
  * @brief Thread-safe queue for G-code commands with simplified priority support
  */
 class CommandQueue
 {
 public:
   /**
    * @brief Construct a new CommandQueue
    * @param size Maximum queue size
    */
   CommandQueue(size_t size = 32);
   
   /**
    * @brief Push a command to the queue with specified priority
    * @param command G-code command
    * @param type Command type/priority
    * @return True if successful, false if queue is full
    */
   bool push(const String &command, CommandType type = NORMAL);
   
   /**
    * @brief Pop the highest priority command from the queue
    * @return G-code command or empty string if queue is empty
    */
   String pop();
   
   /**
    * @brief Get and remove the next immediate command
    * @return Next immediate command or empty string if none
    */
   String getNextImmediate();
   
   /**
    * @brief Check if the queue is empty
    * @return True if queue is empty, false otherwise
    */
   bool isEmpty() const;
   
   /**
    * @brief Get the total size of the queue
    * @return Total number of commands in the queue
    */
   size_t size() const;
   
   /**
    * @brief Clear the queue
    */
   void clear();
 
 private:
   std::deque<QueueItem> immediateQueue;  ///< Queue for immediate commands
   std::deque<QueueItem> normalQueue;     ///< Queue for normal commands
   size_t maxSize;                        ///< Maximum size of the queue
   SemaphoreHandle_t mutex;               ///< Mutex for thread safety
 };
 
 #endif // COMMAND_QUEUE_H