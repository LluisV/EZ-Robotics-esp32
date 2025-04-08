/**
 * @file GCodeValidator.h
 * @brief Validator for G-code files to detect errors before execution
 */

#ifndef GCODE_VALIDATOR_H
#define GCODE_VALIDATOR_H

#include <Arduino.h>
#include <vector>
#include "FileManager.h"
#include "GCodeParser.h"
#include "Debug.h"

/**
 * @brief Error structure for G-code validation
 */
struct GCodeError
{
  int lineNumber;          ///< Line number where error occurred
  String line;             ///< The actual G-code line with error
  String errorDescription; ///< Description of the error
};

/**
 * @brief Result of G-code validation
 */
struct ValidationResult
{
  bool valid;                     ///< Is the G-code file valid
  std::vector<GCodeError> errors; ///< List of errors if any
  int lineCount;                  ///< Total line count
  int validCommandCount;          ///< Valid command count
};

/**
 * @brief Validator class for G-code files
 */
class GCodeValidator
{
public:
  /**
   * @brief Construct a new GCodeValidator
   * @param fileManager File manager reference
   * @param gCodeParser G-code parser reference for syntax checking
   */
  GCodeValidator(FileManager *fileManager, GCodeParser *gCodeParser);

  /**
   * @brief Validate a G-code file
   * @param filename Name of the G-code file
   * @param maxErrors Maximum number of errors to report (0 = unlimited)
   * @return Validation result
   */
  ValidationResult validateFile(const String &filename, int maxErrors = 10);

  /**
   * @brief Format validation errors as a string
   * @param result Validation result
   * @return Formatted string with error details
   */
  String formatValidationErrors(const ValidationResult &result);

private:
  FileManager *fileManager; ///< File manager reference
  GCodeParser *gCodeParser; ///< G-code parser reference

  /**
   * @brief Validate a single G-code line
   * @param line G-code line
   * @param lineNumber Line number for error reporting
   * @param error Output parameter for error details
   * @return True if valid, false if error
   */
  bool validateLine(const String &line, int lineNumber, GCodeError &error);
  String preprocessLine(const String &line);
};

#endif // GCODE_VALIDATOR_H