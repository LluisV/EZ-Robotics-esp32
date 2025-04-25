/**
 * @file FileManager.h
 * @brief File manager for G-code files
 */

#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <Arduino.h>
#include <LittleFS.h>
#include "Debug.h"

// Maximum number of files to be listed (to avoid OOM)
#define MAX_FILES_LIST 30

/**
 * @brief File manager class for handling G-code files
 */
class FileManager
{
public:
  /**
   * @brief Construct a new File Manager
   */
  FileManager();

  /**
   * @brief Initialize the file manager
   * @param formatIfNeeded If true, format SPIFFS if mounting fails
   * @return True if successful, false otherwise
   */
  bool initialize(bool formatIfNeeded = true);

  /**
   * @brief List all G-code files
   * @return String with file list, one file per line
   */
  String listFiles();

  /**
   * @brief Get file information
   * @param filename Name of the file
   * @return String with file information (size, modification time)
   */
  String getFileInfo(const String &filename);

  /**
   * @brief Delete a file
   * @param filename Name of the file to delete
   * @return True if successful, false otherwise
   */
  bool deleteFile(const String &filename);

  /**
   * @brief Read a line from a file
   * @param file File reference
   * @param line String to store the line
   * @return True if a line was read, false if EOF
   */
  bool readLine(File &file, String &line);

  /**
   * @brief Check if a file exists
   * @param filename Name of the file
   * @return True if file exists, false otherwise
   */
  bool fileExists(const String &filename);

  /**
   * @brief Get file size
   * @param filename Name of the file
   * @return File size in bytes, or -1 if file not found
   */
  size_t getFileSize(const String &filename);

  /**
   * @brief Open a file for reading
   * @param filename Name of the file
   * @return File object
   */
  File openFile(const String &filename);

  /**
   * @brief Create a new file
   * @param filename Name of the file
   * @return File object
   */
  File createFile(const String &filename);

  /**
   * @brief Create a temporary file for resuming job after reset
   * @param filename Original file name
   * @param lineNumber Line number to resume from
   * @return True if successful, false otherwise
   */
  bool createResumeFile(const String &filename, int lineNumber);

  /**
   * @brief Get free space in SPIFFS
   * @return Free space in bytes
   */
  size_t getFreeSpace();

  /**
   * @brief Get total space in SPIFFS
   * @return Total space in bytes
   */
  size_t getTotalSpace();

  /**
   * @brief Format SPIFFS
   * @return True if successful, false otherwise
   */
  bool formatFileSystem();

private:
  bool LittleFSInitialized; ///< Flag to track if SPIFFS is initialized

  /**
   * @brief Check if file is a G-code file
   * @param filename Name of the file
   * @return True if file is a G-code file, false otherwise
   */
  bool isGCodeFile(const String &filename);
};

#endif // FILE_MANAGER_H