/**
 * @file CommandQueue.h
 * @brief Thread-safe command queue for G-code processing with priority levels
 */

 #ifndef COMMAND_QUEUE_H
 #define COMMAND_QUEUE_H
 
 #include <Arduino.h>
 #include <vector>
 
 /**
  * @brief Command types with different priority levels
  */
 enum CommandType {
   MOTION,     ///< G-code commands that control motion (G0, G1, etc.)
   SETTING,    ///< Commands that change settings (G90, G91, etc.)
   INFO,       ///< Commands requesting information (position, status, etc.)
   IMMEDIATE   ///< Emergency commands (M112, etc.)
 };
 
 /**
  * @brief Thread-safe queue for G-code commands with priority support
  */
 class CommandQueue {
 public:
   /**
    * @brief Construct a new CommandQueue
    * @param size Maximum queue size for each priority level
    */
   CommandQueue(size_t size = 10);
   
   /**
    * @brief Push a command to the queue with specified priority
    * @param command G-code command
    * @param type Command type/priority
    * @return True if successful, false if queue is full
    */
   bool push(const String& command, CommandType type = MOTION);
   
   /**
    * @brief Pop the highest priority command from the queue
    * @return G-code command or empty string if queue is empty
    */
   String pop();
   
   /**
    * @brief Get the next immediate command without removing it
    * @return Next immediate command or empty string if none
    */
   String peekImmediate() const;
   
   /**
    * @brief Get and remove the next immediate command
    * @return Next immediate command or empty string if none
    */
   String getNextImmediate();
   
   /**
    * @brief Check if all queues are empty
    * @return True if all queues are empty, false otherwise
    */
   bool isEmpty() const;
   
   /**
    * @brief Check if a specific queue is full
    * @param type Command type/priority
    * @return True if the specified queue is full, false otherwise
    */
   bool isFull(CommandType type) const;
   
   /**
    * @brief Get the total size of all queues
    * @return Total number of commands in all queues
    */
   size_t size() const;
   
   /**
    * @brief Clear all queues
    */
   void clear();
 
 private:
   struct QueueItem {
     String command;
     unsigned long timestamp;  // For potential timeout features
   };
   
   // Separate queues for different priority levels
   std::vector<QueueItem> motionQueue;    ///< Queue for motion commands
   std::vector<QueueItem> settingQueue;   ///< Queue for setting commands
   std::vector<QueueItem> infoQueue;      ///< Queue for info requests
   std::vector<QueueItem> immediateQueue; ///< Queue for immediate commands
   
   size_t maxSize;                ///< Maximum size of each queue
   size_t motionHead, motionTail; ///< Head and tail indices for motion queue
   size_t settingHead, settingTail; ///< Head and tail indices for setting queue
   size_t infoHead, infoTail;     ///< Head and tail indices for info queue
   size_t immedHead, immedTail;   ///< Head and tail indices for immediate queue
   size_t motionCount;            ///< Number of elements in motion queue
   size_t settingCount;           ///< Number of elements in setting queue
   size_t infoCount;              ///< Number of elements in info queue
   size_t immedCount;             ///< Number of elements in immediate queue
   
   SemaphoreHandle_t mutex;       ///< Mutex for thread safety
   
   /**
    * @brief Push a command to a specific queue
    * @param queue Queue to push to
    * @param head Head index
    * @param tail Tail index
    * @param count Count reference
    * @param command Command to push
    * @return True if successful, false if queue is full
    */
   bool pushToQueue(std::vector<QueueItem>& queue, size_t& head, size_t& tail, size_t& count, const String& command);
   
   /**
    * @brief Pop a command from a specific queue
    * @param queue Queue to pop from
    * @param head Head index
    * @param tail Tail index
    * @param count Count reference
    * @return Command or empty string if queue is empty
    */
   String popFromQueue(std::vector<QueueItem>& queue, size_t& head, size_t& tail, size_t& count);
   
   /**
    * @brief Check if a specific queue is empty
    * @param count Count of the queue
    * @return True if empty, false otherwise
    */
   bool isQueueEmpty(size_t count) const;
 };
 
 #endif // COMMAND_QUEUE_H