/**
 * @file GCodeParser.cpp
 * @brief Implementation of the GCodeParser class
 */

#include "GCodeParser.h"
#include <map>
#include "Debug.h"
#include "Scheduler.h"

GCodeParser::GCodeParser(MachineController *machineController)
    : machineController(machineController),
      lastFeedrate(1000.0f)
{
  Debug::info("GCodeParser", "Initialized with machine controller");
}

GCodeParseResult GCodeParser::parse(const String &command)
{
  Debug::verbose("GCodeParser", "Parsing command: " + command);

  // Ignore empty lines and comments
  String trimmedCmd = command;
  trimmedCmd.trim();

  if (trimmedCmd.length() == 0 || trimmedCmd.startsWith(";"))
  {
    Debug::verbose("GCodeParser", "Ignoring empty line or comment");
    return GCodeParseResult::SUCCESS;
  }

  // Remove inline comments
  int commentIdx = trimmedCmd.indexOf(';');
  if (commentIdx >= 0)
  {
    trimmedCmd = trimmedCmd.substring(0, commentIdx);
    trimmedCmd.trim();
    Debug::verbose("GCodeParser", "Removed inline comment. Trimmed command: " + trimmedCmd);
  }

  // Validate the command first
  String errorMessage;
  if (!validate(trimmedCmd, errorMessage))
  {
    Debug::error("GCodeParser", "Command validation failed: " + errorMessage);
    return GCodeParseResult::PARSE_ERROR;
  }

  // Check if motion planner is full
  if (machineController->getMotionPlanner()->isFull())
  {
    Debug::warning("GCodeParser", "Motion planner queue is full");
    return GCodeParseResult::QUEUE_FULL;
  }

  // Extract G/M code
  char codeType;
  int code = extractCode(trimmedCmd, codeType);

  Debug::verbose("GCodeParser", "Extracted code: " + String(codeType) + String(code));

  // Parse parameters
  std::map<char, float> params;
  try
  {
    params = parseParameters(trimmedCmd);
  }
  catch (const std::exception &e)
  {
    Debug::error("GCodeParser", "Parameter parsing error: " + String(e.what()));
    return GCodeParseResult::PARSE_ERROR;
  }

  // Log parsed parameters
  Debug::verbose("GCodeParser", "Parsed parameters count: " + String(params.size()));
  for (const auto &param : params)
  {
    Debug::verbose("GCodeParser", "Parameter " + String(param.first) + ": " + String(param.second));
    lastParams[param.first] = param.second;
  }

  // Execute the command
  bool result = false;
  try
  {
    if (codeType == 'G')
    {
      Debug::verbose("GCodeParser", "Executing G-code: G" + String(code));
      result = executeGCode(code, params);
    }
    else if (codeType == 'M')
    {
      Debug::verbose("GCodeParser", "Executing M-code: M" + String(code));
      result = executeMCode(code, params);
    }
    else
    {
      Debug::error("GCodeParser", "Unsupported code type: " + String(codeType));
      return GCodeParseResult::PARSE_ERROR;
    }
  }
  catch (const std::exception &e)
  {
    Debug::error("GCodeParser", "Exception during command execution: " + String(e.what()));
    return GCodeParseResult::PARSE_ERROR;
  }
  catch (...)
  {
    Debug::error("GCodeParser", "Unknown exception during command execution");
    return GCodeParseResult::PARSE_ERROR;
  }

  if (!result)
  {
    Debug::warning("GCodeParser", "Failed to execute command: " + command);
    return GCodeParseResult::PARSE_ERROR;
  }
  else
  {
    Debug::verbose("GCodeParser", "Command executed successfully");
  }

  if (result)
    Debug::info("GCodeParser", "Executing: " + command);

  return GCodeParseResult::SUCCESS;
}

std::vector<String> GCodeParser::getSupportedCodes() const
{
  std::vector<String> codes;

  // G-codes
  codes.push_back("G0 - Rapid Move");
  codes.push_back("G1 - Linear Move");
  codes.push_back("G4 - Dwell");
  codes.push_back("G28 - Home");
  codes.push_back("G90 - Absolute Positioning");
  codes.push_back("G91 - Relative Positioning");
  codes.push_back("G92 - Set Position");

  // M-codes
  codes.push_back("M0 - Program Pause");
  codes.push_back("M1 - Program Pause");
  codes.push_back("M2 - Program End");
  codes.push_back("M30 - Program End");
  codes.push_back("M112 - Emergency Stop");

  return codes;
}

std::map<char, float> GCodeParser::parseParameters(const String &command)
{
  std::map<char, float> params;

  Debug::verbose("GCodeParser", "Parsing parameters from: " + command);

  for (int i = 0; i < command.length(); i++)
  {
    char c = command.charAt(i);

    // Skip non-parameter letters or already processed code letter
    if (!isalpha(c) || c == 'G' || c == 'g' || c == 'M' || c == 'm')
    {
      continue;
    }

    // Find the parameter value
    int valueStart = i + 1;
    int valueEnd = valueStart;

    // Find the end of the number
    while (valueEnd < command.length() &&
           (isdigit(command.charAt(valueEnd)) ||
            command.charAt(valueEnd) == '.' ||
            command.charAt(valueEnd) == '-'))
    {
      valueEnd++;
    }

    // Extract and convert the value
    if (valueEnd > valueStart)
    {
      String valueStr = command.substring(valueStart, valueEnd);
      float value = valueStr.toFloat();

      // Store the parameter (uppercase for consistency)
      params[toupper(c)] = value;

      Debug::verbose("GCodeParser", "Parsed parameter: " + String(toupper(c)) + " = " + String(value));

      // Skip to the end of this parameter
      i = valueEnd - 1;
    }
  }

  return params;
}

int GCodeParser::extractCode(const String &command, char &codeType)
{
  // Find G or M code
  for (int i = 0; i < command.length(); i++)
  {
    char c = toupper(command.charAt(i));

    if (c == 'G' || c == 'M')
    {
      codeType = c;

      // Find the code number
      int valueStart = i + 1;
      int valueEnd = valueStart;

      // Find the end of the number
      while (valueEnd < command.length() &&
             (isdigit(command.charAt(valueEnd)) ||
              command.charAt(valueEnd) == '.'))
      {
        valueEnd++;
      }

      // Extract and convert the value
      if (valueEnd > valueStart)
      {
        String valueStr = command.substring(valueStart, valueEnd);
        int extractedCode = valueStr.toInt();

        Debug::verbose("GCodeParser", "Extracted " + String(codeType) + "-code: " + String(extractedCode));
        return extractedCode;
      }
    }
  }

  Debug::warning("GCodeParser", "No valid code found in command");
  return -1; // No valid code found
}

bool GCodeParser::executeGCode(int code, const std::map<char, float> &params)
{
  switch (code)
  {
  case 0: // G0: Rapid move
  case 1:
  { // G1: Linear move
    // Get coordinates
    float x = NAN, y = NAN, z = NAN;
    float f = lastFeedrate;

    // Extract parameters with defaults from last command
    auto it = params.find('X');
    if (it != params.end())
      x = it->second;

    it = params.find('Y');
    if (it != params.end())
      y = it->second;

    it = params.find('Z');
    if (it != params.end())
      z = it->second;

    it = params.find('F');
    if (it != params.end())
    {
      f = it->second;
      lastFeedrate = f;
    }

    // Handle relative mode
    if (!machineController->isAbsoluteMode())
    {
      // In relative mode, get current positions and add the specified offsets
      std::vector<float> currentPos = machineController->getCurrentWorkPosition();

      // Only modify coordinates that were specified in the command
      if (!isnan(x))
        x += currentPos[0];
      if (!isnan(y))
        y += currentPos[1];
      if (!isnan(z) && currentPos.size() > 2)
        z += currentPos[2];
    }

    // Execute move if any coordinates specified
    if (!isnan(x) || !isnan(y) || !isnan(z))
    {
      MovementType moveType = (code == 0) ? RAPID_MOVE : LINEAR_MOVE;
      return machineController->moveTo(x, y, z, f, moveType);
    }

    return true;
  }

  case 4:
  { // G4: Dwell
    // Get dwell time
    float time = 0;

    auto it = params.find('P');
    if (it != params.end())
    {
      time = it->second / 1000.0f; // P is in milliseconds
    }

    it = params.find('S');
    if (it != params.end())
    {
      time = it->second; // S is in seconds
    }

    if (time > 0)
    {
      delay(time * 1000); // Convert to milliseconds
    }

    return true;
  }

  case 21: // G21: Set units to mm
    return true;

  case 28:
  { // G28: Home
    bool homeX = false, homeY = false, homeZ = false;
    bool homeSpecific = false;

    // Check for specific axes to home
    auto it = params.find('X');
    if (it != params.end())
    {
      homeX = true;
      homeSpecific = true;
    }

    it = params.find('Y');
    if (it != params.end())
    {
      homeY = true;
      homeSpecific = true;
    }

    it = params.find('Z');
    if (it != params.end())
    {
      homeZ = true;
      homeSpecific = true;
    }

    // If no specific axes mentioned, home all
    if (!homeSpecific)
    {
      return machineController->homeAll();
    }
    else
    {
      bool success = true;
      if (homeZ)
        success &= machineController->homeAxis("Z");
      if (homeX)
        success &= machineController->homeAxis("X");
      if (homeY)
        success &= machineController->homeAxis("Y");
      return success;
    }
  }

  case 90: // G90: Set to absolute positioning
    machineController->setAbsoluteMode(true);
    return true;

  case 91: // G91: Set to relative positioning
    machineController->setAbsoluteMode(false);
    return true;

  case 92:
  { // G92: Set work position
    std::vector<float> currentWorldPos = machineController->getCurrentWorldPosition();
    std::vector<float> newWorkOffset = currentWorldPos;

    auto it = params.find('X');
    if (it != params.end())
      newWorkOffset[0] = currentWorldPos[0] - it->second;

    it = params.find('Y');
    if (it != params.end())
      newWorkOffset[1] = currentWorldPos[1] - it->second;

    it = params.find('Z');
    if (it != params.end())
      newWorkOffset[2] = currentWorldPos[2] - it->second;

    machineController->setWorkOffset(newWorkOffset);
    return true;
  }

  default:
    Debug::error("GCodeParser","Unsupported G-code: G" + String(code));
    return false;
  }
}

bool GCodeParser::executeMCode(int code, const std::map<char, float> &params)
{
  switch (code)
  {
  case 0: // M0: Program pause
  case 1: // M1: Program pause
    return true;

  case 2:  // M2: Program end
  case 30: // M30: Program end
    return true;

  case 112: // M112: Emergency stop
    machineController->emergencyStop();
    return true;

  default:
    Debug::error("GCodeParser", "Unsupported M-code: M" + String(code));
    return false;
  }
}

bool GCodeParser::validate(const String &command, String &errorMessage)
{
  errorMessage = "";

  // Ignore empty lines and comments
  String trimmedCmd = command;
  trimmedCmd.trim();

  if (trimmedCmd.length() == 0 || trimmedCmd.startsWith(";"))
  {
    return true; // Empty lines and comments are valid
  }

  // Remove inline comments
  int commentIdx = trimmedCmd.indexOf(';');
  if (commentIdx >= 0)
  {
    trimmedCmd = trimmedCmd.substring(0, commentIdx);
    trimmedCmd.trim();
  }

  // Extract G/M code
  char codeType;
  int code = extractCode(trimmedCmd, codeType);

  if (code < 0)
  {
    errorMessage = "Invalid G-code format: " + command;
    return false;
  }

  // Parse parameters
  std::map<char, float> params;
  try
  {
    params = parseParameters(trimmedCmd);
  }
  catch (const std::exception &e)
  {
    errorMessage = "Parameter parsing error: " + String(e.what());
    return false;
  }

  // Validate based on code type
  if (codeType == 'G')
  {
    if (!isGCodeSupported(code))
    {
      errorMessage = "Unsupported G-code: G" + String(code);
      return false;
    }

    // Validate specific G-codes
    switch (code)
    {
    case 0: // G0: Rapid move
    case 1: // G1: Linear move
      // At least one axis should be specified
      if (params.find('X') == params.end() &&
          params.find('Y') == params.end() &&
          params.find('Z') == params.end())
      {
        errorMessage = "G" + String(code) + " requires at least one axis (X, Y, Z)";
        return false;
      }
      break;

    case 28: // G28: Home
      // No validation needed for G28
      break;

    case 92: // G92: Set position
      // At least one axis should be specified
      if (params.find('X') == params.end() &&
          params.find('Y') == params.end() &&
          params.find('Z') == params.end())
      {
        errorMessage = "G92 requires at least one axis (X, Y, Z)";
        return false;
      }
      break;
    }
  }
  else if (codeType == 'M')
  {
    if (!isMCodeSupported(code))
    {
      errorMessage = "Unsupported M-code: M" + String(code);
      return false;
    }
  }
  else
  {
    errorMessage = "Unsupported code type: " + String(codeType);
    return false;
  }

  return true;
}

bool GCodeParser::isGCodeSupported(int code) const
{
  // List of supported G-codes
  static const int supportedCodes[] = {0, 1, 4, 20, 21, 28, 90, 91, 92};
  static const int numCodes = sizeof(supportedCodes) / sizeof(supportedCodes[0]);

  for (int i = 0; i < numCodes; i++)
  {
    if (code == supportedCodes[i])
    {
      return true;
    }
  }

  return false;
}

bool GCodeParser::isMCodeSupported(int code) const
{
  // List of supported M-codes
  static const int supportedCodes[] = {0, 1, 2, 3, 4, 5, 30, 112};
  static const int numCodes = sizeof(supportedCodes) / sizeof(supportedCodes[0]);

  for (int i = 0; i < numCodes; i++)
  {
    if (code == supportedCodes[i])
    {
      return true;
    }
  }

  return false;
}