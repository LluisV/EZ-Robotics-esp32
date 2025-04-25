/**
 * @file FileManager.cpp
 * @brief Implementation of the FileManager class
 */

#include "FileManager.h"
#include <FS.h>

// G-code file extensions to consider
const char *GCODE_EXTENSIONS[] = {".gcode", ".gc", ".g", ".nc", ".cnc", ".ngc"};
const int NUM_GCODE_EXTENSIONS = 6;

FileManager::FileManager() : spiffsInitialized(false)
{
}

bool FileManager::initialize(bool formatIfNeeded)
{
  Debug::info("FileManager", "Initializing SPIFFS");

  // First try mounting SPIFFS normally
  if (!SPIFFS.begin(false))
  {
    //Debug::warning("FileManager", "SPIFFS mount failed");

    if (formatIfNeeded)
    {
      //Debug::warning("FileManager", "Trying to format SPIFFS");

      if (SPIFFS.format())
      {
        Debug::info("FileManager", "SPIFFS formatted successfully");

        // Try mounting again after format
        if (!SPIFFS.begin(false))
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

  spiffsInitialized = true;
  Debug::info("FileManager", "SPIFFS initialized successfully");

  // Print SPIFFS info
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  Debug::info("FileManager", "SPIFFS total: " + String(totalBytes) + " bytes, used: " +
                                 String(usedBytes) + " bytes, free: " + String(totalBytes - usedBytes) + " bytes");

  return true;
}

String FileManager::listFiles()
{
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return "Error: File system not initialized";
  }

  String fileList = "";
  File root = SPIFFS.open("/");
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
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return "Error: File system not initialized";
  }

  if (!SPIFFS.exists(filename))
  {
    Debug::error("FileManager", "File not found: " + filename);
    return "Error: File not found";
  }

  File file = SPIFFS.open(filename, "r");
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
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return false;
  }

  if (!SPIFFS.exists(filename))
  {
    Debug::error("FileManager", "File not found: " + filename);
    return false;
  }

  if (SPIFFS.remove(filename))
  {
    Debug::info("FileManager", "File deleted: " + filename);
    return true;
  }
  else
  {
    Debug::error("FileManager", "Failed to delete file: " + filename);
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
  while (file.available() > 0)
  {
    char c = file.read();
    if (c == '\n' || c == '\r')
    {
      // Skip CR and LF
      // Check for CRLF sequence
      if (c == '\r' && file.peek() == '\n')
      {
        file.read(); // Skip the LF
      }

      return true;
    }
    else
    {
      line += c;
    }
  }

  // If we get here, we reached EOF
  return line.length() > 0;
}

bool FileManager::fileExists(const String &filename)
{
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return false;
  }

  return SPIFFS.exists(filename);
}

size_t FileManager::getFileSize(const String &filename)
{
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return -1;
  }

  if (!SPIFFS.exists(filename))
  {
    Debug::error("FileManager", "File not found: " + filename);
    return -1;
  }

  File file = SPIFFS.open(filename, "r");
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
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return File();
  }

  if (!SPIFFS.exists(filename))
  {
    Debug::error("FileManager", "File not found: " + filename);
    return File();
  }

  File file = SPIFFS.open(filename, "r");
  if (!file)
  {
    Debug::error("FileManager", "Failed to open file: " + filename);
    return File();
  }

  return file;
}

File FileManager::createFile(const String &filename)
{
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return File();
  }

  // If file exists, it will be truncated
  File file = SPIFFS.open(filename, "w");
  if (!file)
  {
    Debug::error("FileManager", "Failed to create file: " + filename);
    return File();
  }

  return file;
}

bool FileManager::createResumeFile(const String &filename, int lineNumber)
{
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return false;
  }

  String resumeFilename = "/resume_info.txt";

  File resumeFile = SPIFFS.open(resumeFilename, "w");
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
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return 0;
  }

  return SPIFFS.totalBytes() - SPIFFS.usedBytes();
}

size_t FileManager::getTotalSpace()
{
  if (!spiffsInitialized)
  {
    Debug::error("FileManager", "SPIFFS not initialized");
    return 0;
  }

  return SPIFFS.totalBytes();
}

bool FileManager::formatFileSystem()
{
  //Debug::warning("FileManager", "Formatting file system");

  bool result = SPIFFS.format();

  if (result)
  {
    Debug::info("FileManager", "File system formatted successfully");

    // Remount SPIFFS
    SPIFFS.begin(false);
    spiffsInitialized = true;
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
    File file = SPIFFS.open(filename, "r");
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