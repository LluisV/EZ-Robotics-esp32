/**
 * @file GCodeValidator.cpp
 * @brief Implementation of the GCodeValidator class
 */

#include "GCodeValidator.h"

GCodeValidator::GCodeValidator(FileManager *fileManager, GCodeParser *gCodeParser)
    : fileManager(fileManager), gCodeParser(gCodeParser)
{
}

ValidationResult GCodeValidator::validateFile(const String &filename, int maxErrors)
{
  ValidationResult result;
  result.valid = true;
  result.lineCount = 0;
  result.validCommandCount = 0;

  Debug::info("GCodeValidator", "Validating file: " + filename);

  // Check if file exists
  if (!fileManager->fileExists(filename))
  {
    Debug::error("GCodeValidator", "File not found: " + filename);
    GCodeError error;
    error.lineNumber = 0;
    error.line = "";
    error.errorDescription = "File not found: " + filename;
    result.errors.push_back(error);
    result.valid = false;
    return result;
  }

  // Open the file
  File file = fileManager->openFile(filename);
  if (!file)
  {
    Debug::error("GCodeValidator", "Failed to open file: " + filename);
    GCodeError error;
    error.lineNumber = 0;
    error.line = "";
    error.errorDescription = "Failed to open file: " + filename;
    result.errors.push_back(error);
    result.valid = false;
    return result;
  }

  // Read and validate each line
  String line;
  int lineNumber = 0;

  while (fileManager->readLine(file, line))
  {
    lineNumber++;
    result.lineCount++;

    // Preprocess line (strip comments, whitespace)
    String processedLine = preprocessLine(line);

    // Skip empty lines and comments
    if (processedLine.length() == 0)
    {
      continue;
    }

    // Validate the line
    GCodeError error;
    if (!validateLine(processedLine, lineNumber, error))
    {
      result.errors.push_back(error);
      result.valid = false;

      Debug::warning("GCodeValidator", "Error in line " + String(lineNumber) +
                                           ": " + error.errorDescription);

      // Check if we've reached the maximum number of errors to report
      if (maxErrors > 0 && result.errors.size() >= (size_t)maxErrors)
      {
        Debug::warning("GCodeValidator", "Maximum error limit reached. Stopping validation.");
        break;
      }
    }
    else
    {
      result.validCommandCount++;
    }
  }

  file.close();

  Debug::info("GCodeValidator", "Validation complete: " +
                                    String(result.valid ? "VALID" : "INVALID") +
                                    ", Total lines: " + String(result.lineCount) +
                                    ", Valid commands: " + String(result.validCommandCount) +
                                    ", Errors: " + String(result.errors.size()));

  return result;
}

String GCodeValidator::formatValidationErrors(const ValidationResult &result)
{
  String output = "G-code validation " + String(result.valid ? "PASSED" : "FAILED") + "\n";
  output += "Total lines: " + String(result.lineCount) + "\n";
  output += "Valid commands: " + String(result.validCommandCount) + "\n";
  output += "Errors found: " + String(result.errors.size()) + "\n\n";

  if (result.errors.size() > 0)
  {
    output += "Error details:\n";

    for (size_t i = 0; i < result.errors.size(); i++)
    {
      const GCodeError &error = result.errors[i];
      output += "Line " + String(error.lineNumber) + ": " + error.line + "\n";
      output += "    " + error.errorDescription + "\n";
    }
  }

  return output;
}

bool GCodeValidator::validateLine(const String &line, int lineNumber, GCodeError &error)
{
  // Initialize error structure
  error.lineNumber = lineNumber;
  error.line = line;

  // Check for empty lines
  if (line.length() == 0)
  {
    return true; // Empty lines are valid
  }

  // Basic format validation
  char firstChar = line.charAt(0);

  // Basic G-code format check - should start with G, M, or T for most common commands
  if (!isalpha(firstChar))
  {
    error.errorDescription = "Invalid G-code format: Line should start with a letter";
    return false;
  }

  // For G and M codes, there should be a number after the letter
  if ((firstChar == 'G' || firstChar == 'g' || firstChar == 'M' || firstChar == 'm') &&
      (line.length() == 1 || !isdigit(line.charAt(1))))
  {
    error.errorDescription = "Invalid " + String(firstChar) +
                             "-code format: Missing code number";
    return false;
  }

  // Extract G or M code when present
  int codeValue = -1;
  if (firstChar == 'G' || firstChar == 'g' || firstChar == 'M' || firstChar == 'm')
  {
    // Find the end of the number
    int i = 1;
    while (i < line.length() && (isdigit(line.charAt(i)) || line.charAt(i) == '.'))
    {
      i++;
    }

    if (i > 1)
    {
      codeValue = line.substring(1, i).toInt();
    }
  }

  // Check supported G-codes
  if (firstChar == 'G' || firstChar == 'g')
  {
    // List of supported G-codes - add more as needed
    static const int supportedGCodes[] = {0, 1, 4, 20, 21, 28, 90, 91, 92};
    bool supported = false;

    for (size_t i = 0; i < sizeof(supportedGCodes) / sizeof(supportedGCodes[0]); i++)
    {
      if (codeValue == supportedGCodes[i])
      {
        supported = true;
        break;
      }
    }

    if (!supported)
    {
      error.errorDescription = "Unsupported G-code: G" + String(codeValue);
      return false;
    }

    // G-code specific validation
    switch (codeValue)
    {
    case 0: // G0 - Rapid move
    case 1: // G1 - Linear move
      // At least one axis should be specified
      if (line.indexOf('X') < 0 && line.indexOf('Y') < 0 &&
          line.indexOf('Z') < 0 && line.indexOf('x') < 0 &&
          line.indexOf('y') < 0 && line.indexOf('z') < 0)
      {
        error.errorDescription = "G" + String(codeValue) +
                                 " requires at least one axis parameter (X, Y, Z)";
        return false;
      }
      break;

    case 28: // G28 - Home
      // No specific validation needed for G28
      break;

    case 92: // G92 - Set position
      // At least one axis should be specified
      if (line.indexOf('X') < 0 && line.indexOf('Y') < 0 &&
          line.indexOf('Z') < 0 && line.indexOf('x') < 0 &&
          line.indexOf('y') < 0 && line.indexOf('z') < 0)
      {
        error.errorDescription = "G92 requires at least one axis parameter (X, Y, Z)";
        return false;
      }
      break;
    }
  }
  // Check supported M-codes
  else if (firstChar == 'M' || firstChar == 'm')
  {
    // List of supported M-codes - add more as needed
    static const int supportedMCodes[] = {0, 1, 2, 3, 4, 5, 30, 112};
    bool supported = false;

    for (size_t i = 0; i < sizeof(supportedMCodes) / sizeof(supportedMCodes[0]); i++)
    {
      if (codeValue == supportedMCodes[i])
      {
        supported = true;
        break;
      }
    }

    if (!supported)
    {
      error.errorDescription = "Unsupported M-code: M" + String(codeValue);
      return false;
    }
  }

  // Parse parameters for further validation
  for (int i = 0; i < line.length(); i++)
  {
    char c = line.charAt(i);

    // Skip non-parameter letters
    if (!isalpha(c) || c == 'G' || c == 'g' || c == 'M' || c == 'm')
    {
      continue;
    }

    // Find parameter value
    if (i + 1 < line.length())
    {
      int valueStart = i + 1;
      int valueEnd = valueStart;

      // Find end of number
      bool hasDecimal = false;
      bool hasDigit = false;

      while (valueEnd < line.length() &&
             (isdigit(line.charAt(valueEnd)) ||
              line.charAt(valueEnd) == '.' ||
              line.charAt(valueEnd) == '-'))
      {

        if (line.charAt(valueEnd) == '.')
        {
          if (hasDecimal)
          {
            error.errorDescription = "Invalid parameter format: Multiple decimal points";
            return false;
          }
          hasDecimal = true;
        }
        else if (isdigit(line.charAt(valueEnd)))
        {
          hasDigit = true;
        }
        else if (line.charAt(valueEnd) == '-' && valueEnd > valueStart)
        {
          error.errorDescription = "Invalid parameter format: Minus sign not at beginning";
          return false;
        }

        valueEnd++;
      }

      // Check if we found a valid number
      if (valueEnd > valueStart && hasDigit)
      {
        // Parameter is valid, continue to next one
        i = valueEnd - 1; // -1 because loop will increment
      }
      else
      {
        error.errorDescription = "Invalid parameter format for " + String(c);
        return false;
      }
    }
    else
    {
      // Parameter letter at end of line with no value
      error.errorDescription = "Missing value for parameter " + String(c);
      return false;
    }
  }

  // If we get here, the line is valid
  return true;
}

// Helper function to preprocess G-code line
String GCodeValidator::preprocessLine(const String &line)
{
  String processedLine = line;

  // Remove comments
  int commentIdx = processedLine.indexOf(';');
  if (commentIdx >= 0)
  {
    processedLine = processedLine.substring(0, commentIdx);
  }

  // Trim whitespace
  processedLine.trim();

  return processedLine;
}