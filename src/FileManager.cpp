/**
 * @file FileManager.cpp
 * @brief Implementation of the FileManager class
 */

#include "FileManager.h"
#include <FS.h>

// G-code file extensions to consider
const char *GCODE_EXTENSIONS[] = {".gcode", ".gc", ".g", ".nc", ".cnc", ".ngc"};
const int NUM_GCODE_EXTENSIONS = 6;

FileManager::FileManager() : LittleFSInitialized(false)
{
}

bool FileManager::initialize(bool formatIfNeeded)
{
  Debug::info("FileManager", "Initializing SPIFFS");

  // First try mounting SPIFFS normally
  if (!LittleFS.begin(false))
  {
    //Debug::warning("FileManager", "SPIFFS mount failed");

    if (formatIfNeeded)
    {
      //Debug::warning("FileManager", "Trying to format SPIFFS");

      if (LittleFS.format())
      {
        Debug::info("FileManager", "SPIFFS formatted successfully");

        // Try mounting again after format
        if (!LittleFS.begin(false))
        {
          Debug::error("FileManager", "SPIFFS mount failed even after formatting");
          return false;
        }
      }
      else
      {
        Debug::error("FileManager", "SPIFFS format failed");
        return false;
      }
    }
    else
    {
      Debug::error("FileManager", "SPIFFS mount failed and format not requested");
      return false;
    }
  }

  LittleFSInitialized = true;
  Debug::info("FileManager", "SPIFFS initialized successfully");

  // Print SPIFFS info
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  Debug::info("FileManager", "SPIFFS total: " + String(totalBytes) + " bytes, used: " +
                                 String(usedBytes) + " bytes, free: " + String(totalBytes - usedBytes) + " bytes");

  return true;
}

String FileManager::listFiles()
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return "Error: File system not initialized";
  }

  String fileList = "";
  File root = LittleFS.open("/");
  if (!root)
  {
    Debug::error("FileManager", "Failed to open root directory");
    return "Error: Failed to open root directory";
  }

  if (!root.isDirectory())
  {
    Debug::error("FileManager", "Root is not a directory");
    return "Error: Root is not a directory";
  }

  int fileCount = 0;
  File file = root.openNextFile();

  while (file && fileCount < MAX_FILES_LIST)
  {
    // Skip directories
    if (!file.isDirectory())
    {
      String filename = file.name();

      // Show only G-code files
      if (isGCodeFile(filename))
      {
        // Add file info to list
        fileList += filename;
        fileList += " (";
        fileList += String(file.size());
        fileList += " bytes)\n";
        fileCount++;
      }
    }

    file = root.openNextFile();
  }

  if (fileCount == 0)
  {
    fileList = "No G-code files found";
  }

  return fileList;
}

String FileManager::getFileInfo(const String &filename)
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return "Error: File system not initialized";
  }

  if (!LittleFS.exists(filename))
  {
    Debug::error("FileManager", "File not found: " + filename);
    return "Error: File not found";
  }

  File file = LittleFS.open(filename, "r");
  if (!file)
  {
    Debug::error("FileManager", "Failed to open file: " + filename);
    return "Error: Failed to open file";
  }

  String info = "Filename: " + filename + "\n";
  info += "Size: " + String(file.size()) + " bytes\n";

  // Count number of lines as a rough estimate of command count
  int lineCount = 0;
  String line;

  while (readLine(file, line) && lineCount < 1000)
  {
    // Skip empty lines and comments
    if (line.length() > 0 && !line.startsWith(";"))
    {
      lineCount++;
    }
  }

  if (lineCount >= 1000)
  {
    info += "Lines: 1000+ (estimate)\n";
  }
  else
  {
    info += "Lines: " + String(lineCount) + "\n";
  }

  file.close();

  return info;
}

bool FileManager::deleteFile(const String &filename)
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return false;
  }

  if (!LittleFS.exists(filename))
  {
    Debug::error("FileManager", "File not found: " + filename);
    return false;
  }

  // First, ensure no open file handles by temporarily ending LittleFS
  LittleFS.end();
  
  // Reinitialize the file system
  if (!LittleFS.begin()) {
    Debug::error("FileManager", "Failed to reinitialize file system when trying to delete: " + filename);
    return false;
  }

  // Check again if file exists after reinitialization
  if (!LittleFS.exists(filename)) {
    Debug::warning("FileManager", "File not found after reinitialization: " + filename);
    return true; // File is already gone, so consider deletion successful
  }

  // Now try to delete the file
  if (LittleFS.remove(filename))
  {
    Debug::info("FileManager", "File deleted: " + filename);
    return true;
  }
  else
  {
    Debug::error("FileManager", "Failed to delete file: " + filename);
    
    // If deletion failed, try one more approach: create an empty file to overwrite
    File file = LittleFS.open(filename, "w");
    if (file) {
      file.close();
      Debug::info("FileManager", "Created empty file to replace: " + filename);
      
      // Try deletion again
      if (LittleFS.remove(filename)) {
        Debug::info("FileManager", "File deleted on second attempt: " + filename);
        return true;
      }
    }
    
    return false;
  }
}

bool FileManager::readLine(File &file, String &line)
{
  line = "";

  if (!file || file.available() <= 0)
  {
    return false;
  }

  // Read characters until end of line or end of file
  char buffer[128]; // Define a constant for maximum line length
  int bufferIndex = 0;
  bool lineEndFound = false;
  
  while (file.available() > 0 && bufferIndex < 128 - 1 && !lineEndFound)
  {
    char c = file.read();
    
    if (c == '\n' || c == '\r')
    {
      // Handle CR+LF sequence (Windows-style line endings)
      if (c == '\r' && file.peek() == '\n')
      {
        file.read(); // Skip the LF
      }
      
      lineEndFound = true;
    }
    else
    {
      // Add character to buffer
      buffer[bufferIndex++] = c;
    }
  }
  
  // Null-terminate the buffer
  buffer[bufferIndex] = '\0';
  
  // Create the result string
  line = String(buffer);
  
  // Check if we reached end of file without a newline
  return lineEndFound || bufferIndex > 0;
}

bool FileManager::fileExists(const String &filename)
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return false;
  }

  return LittleFS.exists(filename);
}

size_t FileManager::getFileSize(const String &filename)
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return -1;
  }

  if (!LittleFS.exists(filename))
  {
    Debug::error("FileManager", "File not found: " + filename);
    return -1;
  }

  File file = LittleFS.open(filename, "r");
  if (!file)
  {
    Debug::error("FileManager", "Failed to open file: " + filename);
    return -1;
  }

  size_t size = file.size();
  file.close();

  return size;
}

File FileManager::openFile(const String &filename)
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "LittleFS not initialized");
    return File();
  }

  if (!LittleFS.exists(filename))
  {
    Debug::error("FileManager", "File not found: " + filename);
    return File();
  }

  File file = LittleFS.open(filename, "r");
  if (!file)
  {
    Debug::error("FileManager", "Failed to open file: " + filename);
    return File();
  }

  return file;
}

File FileManager::createFile(const String &filename)
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return File();
  }

  // If file exists, it will be truncated
  File file = LittleFS.open(filename, "w");
  if (!file)
  {
    Debug::error("FileManager", "Failed to create file: " + filename);
    return File();
  }

  return file;
}

bool FileManager::createResumeFile(const String &filename, int lineNumber)
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return false;
  }

  String resumeFilename = "/resume_info.txt";

  File resumeFile = LittleFS.open(resumeFilename, "w");
  if (!resumeFile)
  {
    Debug::error("FileManager", "Failed to create resume file");
    return false;
  }

  // Write resume information
  resumeFile.println(filename);
  resumeFile.println(lineNumber);
  resumeFile.close();

  Debug::info("FileManager", "Resume file created for " + filename + " at line " + String(lineNumber));

  return true;
}

size_t FileManager::getFreeSpace()
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "LittleFS not initialized");
    return 0;
  }

  return LittleFS.totalBytes() - LittleFS.usedBytes();
}

size_t FileManager::getTotalSpace()
{
  if (!LittleFSInitialized)
  {
    Debug::error("FileManager", "LittleFS not initialized");
    return 0;
  }

  return LittleFS.totalBytes();
}

bool FileManager::formatFileSystem()
{
  //Debug::warning("FileManager", "Formatting file system");

  bool result = LittleFS.format();

  if (result)
  {
    Debug::info("FileManager", "File system formatted successfully");

    // Remount SPIFFS
    LittleFS.begin(false);
    LittleFSInitialized = true;
  }
  else
  {
    Debug::error("FileManager", "Failed to format file system");
  }

  return result;
}

bool FileManager::isGCodeFile(const String &filename)
{
  String lowerFilename = filename;
  lowerFilename.toLowerCase();

  for (int i = 0; i < NUM_GCODE_EXTENSIONS; i++)
  {
    if (lowerFilename.endsWith(GCODE_EXTENSIONS[i]))
    {
      return true;
    }
  }

  // If no extension, also check if it contains G-code commands by reading first few lines
  if (lowerFilename.indexOf('.') == -1)
  {
    File file = LittleFS.open(filename, "r");
    if (file)
    {
      // Check first 5 lines for G-code commands
      bool isGCode = false;
      int lineCount = 0;
      String line;

      while (readLine(file, line) && lineCount < 5)
      {
        line.trim();

        // Check for common G-code commands
        if (line.startsWith("G") || line.startsWith("M") || line.startsWith("T"))
        {
          isGCode = true;
          break;
        }

        lineCount++;
      }

      file.close();
      return isGCode;
    }
  }

  return false;
}