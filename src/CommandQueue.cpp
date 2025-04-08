/**
 * @file CommandQueue.cpp
 * @brief Implementation of the CommandQueue class with priority support and debugging
 */

#include "CommandQueue.h"
#include "Debug.h"

CommandQueue::CommandQueue(size_t size)
    : maxSize(size),
      motionHead(0), motionTail(0), motionCount(0),
      settingHead(0), settingTail(0), settingCount(0),
      infoHead(0), infoTail(0), infoCount(0),
      immedHead(0), immedTail(0), immedCount(0)
{
  Debug::verbose("CommandQueue", "Constructor called with max size: " + String(size));

  // Initialize queues
  motionQueue.resize(maxSize);
  settingQueue.resize(maxSize);
  infoQueue.resize(maxSize);
  immediateQueue.resize(maxSize);

  // Create mutex for thread safety
  mutex = xSemaphoreCreateMutex();

  if (mutex == NULL)
  {
    Debug::error("CommandQueue", "Failed to create mutex");
  }
  else
  {
    Debug::verbose("CommandQueue", "Mutex created successfully");
  }
}

bool CommandQueue::push(const String &command, CommandType type)
{
  Debug::verbose("CommandQueue", "Attempting to push command: " + command +
                                     " of type " + String(type));

  if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
  {
    Debug::warning("CommandQueue", "Failed to acquire mutex for push");
    return false;
  }

  bool result = false;

  switch (type)
  {
  case MOTION:
    result = pushToQueue(motionQueue, motionHead, motionTail, motionCount, command);
    Debug::verbose("CommandQueue", "Motion queue push " +
                                       String(result ? "successful" : "failed"));
    break;
  case SETTING:
    result = pushToQueue(settingQueue, settingHead, settingTail, settingCount, command);
    Debug::verbose("CommandQueue", "Setting queue push " +
                                       String(result ? "successful" : "failed"));
    break;
  case INFO:
    result = pushToQueue(infoQueue, infoHead, infoTail, infoCount, command);
    Debug::verbose("CommandQueue", "Info queue push " +
                                       String(result ? "successful" : "failed"));
    break;
  case IMMEDIATE:
    result = pushToQueue(immediateQueue, immedHead, immedTail, immedCount, command);
    Debug::verbose("CommandQueue", "Immediate queue push " +
                                       String(result ? "successful" : "failed"));
    break;
  }

  xSemaphoreGive(mutex);
  return result;
}

String CommandQueue::pop()
{
  Debug::verbose("CommandQueue", "Attempting to pop command");

  if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
  {
    Debug::warning("CommandQueue", "Failed to acquire mutex for pop");
    return "";
  }

  String result = "";

  // Check queues in priority order: IMMEDIATE > INFO > SETTING > MOTION
  if (immedCount > 0)
  {
    result = popFromQueue(immediateQueue, immedHead, immedTail, immedCount);
    Debug::verbose("CommandQueue", "Popped IMMEDIATE command: " + result);
  }
  else if (infoCount > 0)
  {
    result = popFromQueue(infoQueue, infoHead, infoTail, infoCount);
    Debug::verbose("CommandQueue", "Popped INFO command: " + result);
  }
  else if (settingCount > 0)
  {
    result = popFromQueue(settingQueue, settingHead, settingTail, settingCount);
    Debug::verbose("CommandQueue", "Popped SETTING command: " + result);
  }
  else if (motionCount > 0)
  {
    result = popFromQueue(motionQueue, motionHead, motionTail, motionCount);
    Debug::verbose("CommandQueue", "Popped MOTION command: " + result);
  }

  xSemaphoreGive(mutex);
  return result;
}

String CommandQueue::peekImmediate() const
{
  Debug::verbose("CommandQueue", "Attempting to peek immediate command");

  if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
  {
    Debug::warning("CommandQueue", "Failed to acquire mutex for peek");
    return "";
  }

  String result = "";

  if (immedCount > 0)
  {
    result = immediateQueue[immedHead].command;
    Debug::verbose("CommandQueue", "Peeked immediate command: " + result);
  }

  xSemaphoreGive(mutex);
  return result;
}

String CommandQueue::getNextImmediate()
{

  if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
  {
    Debug::warning("CommandQueue", "Failed to acquire mutex for get next immediate");
    return "";
  }

  String result = "";

  if (immedCount > 0)
  {
    result = popFromQueue(immediateQueue, immedHead, immedTail, immedCount);
    Debug::verbose("CommandQueue", "Got next immediate command: " + result);
  }

  xSemaphoreGive(mutex);
  return result;
}

bool CommandQueue::isEmpty() const
{
  if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
  {
    Debug::warning("CommandQueue", "Failed to acquire mutex for isEmpty");
    return true;
  }

  bool empty = (motionCount == 0 && settingCount == 0 &&
                infoCount == 0 && immedCount == 0);

  xSemaphoreGive(mutex);
  return empty;
}

bool CommandQueue::isFull(CommandType type) const
{
  Debug::verbose("CommandQueue", "Checking if queue type is full: " + String(type));

  if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
  {
    Debug::warning("CommandQueue", "Failed to acquire mutex for isFull");
    return true;
  }

  bool full = false;

  switch (type)
  {
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

  Debug::verbose("CommandQueue", "Queue type " + String(type) +
                                     " is " + String(full ? "full" : "not full"));

  xSemaphoreGive(mutex);
  return full;
}

size_t CommandQueue::size() const
{
  Debug::verbose("CommandQueue", "Getting total queue size");

  if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
  {
    Debug::warning("CommandQueue", "Failed to acquire mutex for size");
    return 0;
  }

  size_t totalSize = motionCount + settingCount + infoCount + immedCount;

  Debug::verbose("CommandQueue", "Total queue size: " + String(totalSize) +
                                     " (Motion: " + String(motionCount) +
                                     ", Setting: " + String(settingCount) +
                                     ", Info: " + String(infoCount) +
                                     ", Immediate: " + String(immedCount) + ")");

  xSemaphoreGive(mutex);
  return totalSize;
}

void CommandQueue::clear()
{
  Debug::info("CommandQueue", "Clearing all queues");

  if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
  {
    Debug::warning("CommandQueue", "Failed to acquire mutex for clear");
    return;
  }

  // Reset all queue pointers
  motionHead = 0;
  motionTail = 0;
  motionCount = 0;
  settingHead = 0;
  settingTail = 0;
  settingCount = 0;
  infoHead = 0;
  infoTail = 0;
  infoCount = 0;
  immedHead = 0;
  immedTail = 0;
  immedCount = 0;

  Debug::verbose("CommandQueue", "All queues reset to initial state");

  xSemaphoreGive(mutex);
}

bool CommandQueue::pushToQueue(std::vector<QueueItem> &queue, size_t &head,
                               size_t &tail, size_t &count, const String &command)
{
  if (count >= maxSize)
  {
    Debug::warning("CommandQueue", "Cannot push to queue. Queue is full");
    return false;
  }

  QueueItem item;
  item.command = command;
  item.timestamp = millis(); // Record timestamp for potential timeout features

  queue[tail] = item;
  tail = (tail + 1) % maxSize;
  count++;

  Debug::verbose("CommandQueue", "Pushed command: " + command +
                                     ", New tail: " + String(tail) +
                                     ", New count: " + String(count));

  return true;
}

String CommandQueue::popFromQueue(std::vector<QueueItem> &queue, size_t &head,
                                  size_t &tail, size_t &count)
{
  if (count == 0)
  {
    Debug::warning("CommandQueue", "Cannot pop from empty queue");
    return "";
  }

  String result = queue[head].command;
  head = (head + 1) % maxSize;
  count--;

  Debug::verbose("CommandQueue", "Popped command: " + result +
                                     ", New head: " + String(head) +
                                     ", New count: " + String(count));

  return result;
}

bool CommandQueue::isQueueEmpty(size_t count) const
{
  return count == 0;
}