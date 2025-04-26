/**
 * @file GCodeParser.cpp
 * @brief Implementation of the GCodeParser class for the pipeline architecture
 */

 #include "GCodeParser.h"
 #include "Debug.h"
 #include <map>
 
 GCodeParser::GCodeParser(MotionControl* motionControl)
     : motionControl(motionControl),
       lastFeedrate(1000.0f),
       absoluteMode(true)
 {
   Debug::info("GCodeParser", "Initialized with motion control");
   
   // Initialize lastTargetPosition
   lastTargetPosition.resize(3, 0.0f);
 }
 
 GCodeParseResult GCodeParser::parse(const String &command)
 {
   //Debug::verbose("GCodeParser", "Parsing command: " + command);
 
   // Ignore empty lines and comments
   String trimmedCmd = command;
   trimmedCmd.trim();
 
   if (trimmedCmd.length() == 0 || trimmedCmd.startsWith(";"))
   {
     //Debug::verbose("GCodeParser", "Ignoring empty line or comment");
     return GCodeParseResult::SUCCESS;
   }
 
   // Remove inline comments
   int commentIdx = trimmedCmd.indexOf(';');
   if (commentIdx >= 0)
   {
     trimmedCmd = trimmedCmd.substring(0, commentIdx);
     trimmedCmd.trim();
     //Debug::verbose("GCodeParser", "Removed inline comment. Trimmed command: " + trimmedCmd);
   }
 
   // Validate the command first
   String errorMessage;
   if (!validate(trimmedCmd, errorMessage))
   {
     Debug::error("GCodeParser", "Command validation failed: " + errorMessage);
     return GCodeParseResult::PARSE_ERROR;
   }
 
   // Check if motion control is available and has space
   if (!motionControl || !motionControl->hasSpace())
   {
     //Debug::warning("GCodeParser", "Motion control not available or queue is full");
     return GCodeParseResult::QUEUE_FULL;
   }
 
   // Extract G/M code
   char codeType;
   int code = extractCode(trimmedCmd, codeType);
 
   //Debug::verbose("GCodeParser", "Extracted code: " + String(codeType) + String(code));
 
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
 
   // Store parameters for future reference
   for (const auto &param : params)
   {
     lastParams[param.first] = param.second;
   }
 
   // Execute the command
   bool result = false;
   try
   {
     if (codeType == 'G')
     {
       //Debug::verbose("GCodeParser", "Executing G-code: G" + String(code));
       result = executeGCode(code, params);
     }
     else if (codeType == 'M')
     {
       //Debug::verbose("GCodeParser", "Executing M-code: M" + String(code));
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
     //Debug::warning("GCodeParser", "Failed to execute command: " + command);
     return GCodeParseResult::PARSE_ERROR;
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
   codes.push_back("G2 - Clockwise Arc");
   codes.push_back("G3 - Counter-Clockwise Arc");
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
         return valueStr.toInt();
       }
     }
   }
 
   return -1; // No valid code found
 }
 
 bool GCodeParser::executeGCode(int code, const std::map<char, float> &params)
 {
   // Make sure we have motion control
   if (!motionControl) {
     Debug::error("GCodeParser", "No motion control available");
     return false;
   }
   
   // Get machine controller for status operations
   MachineController* machineController = motionControl->getMachineController();
   if (!machineController) {
     Debug::error("GCodeParser", "No machine controller available");
     return false;
   }
 
   switch (code)
   {
   case 0: // G0: Rapid move
   case 1: // G1: Linear move
   {
     // Get current position for unspecified axes
     std::vector<float> currentPos = machineController->getCurrentWorkPosition();
     
     // Ensure lastTargetPosition is initialized with the right size
     if (lastTargetPosition.empty() || lastTargetPosition.size() < currentPos.size()) {
       lastTargetPosition = currentPos;
     }
     
     // Use the last target position as the base for the new move
     float x = lastTargetPosition.size() > 0 ? lastTargetPosition[0] : currentPos[0];
     float y = lastTargetPosition.size() > 1 ? lastTargetPosition[1] : currentPos[1];
     float z = lastTargetPosition.size() > 2 ? lastTargetPosition[2] : currentPos[2];
     float f = lastFeedrate;
 
     // Extract parameters with explicit updates from this command
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
     if (!absoluteMode)
     {
       // In relative mode, add offsets to current position (not last target)
       if (params.find('X') != params.end())
         x = currentPos[0] + params.at('X');
       if (params.find('Y') != params.end())
         y = currentPos[1] + params.at('Y');
       if (params.find('Z') != params.end() && currentPos.size() > 2)
         z = currentPos[2] + params.at('Z');
     }
 
     // Update last target position
     if (lastTargetPosition.size() < 3) {
       lastTargetPosition.resize(3);
     }
     lastTargetPosition[0] = x;
     lastTargetPosition[1] = y;
     lastTargetPosition[2] = z;
 
     // Create target position vector
     std::vector<float> targetPos = {x, y, z};
 
     // Execute move if any coordinates specified in the command
     if (params.find('X') != params.end() || params.find('Y') != params.end() || params.find('Z') != params.end())
     {
       MovementType moveType = (code == 0) ? RAPID_MOVE : LINEAR_MOVE;
       return motionControl->executeLinearMove(targetPos, f, moveType);
     }
 
     return true;
   }
 
   case 2: // G2: Clockwise arc
   case 3: // G3: Counter-clockwise arc
   {
     // Get current position for unspecified axes
     std::vector<float> currentPos = machineController->getCurrentWorkPosition();
     
     // Ensure lastTargetPosition is initialized with the right size
     if (lastTargetPosition.empty() || lastTargetPosition.size() < currentPos.size()) {
       lastTargetPosition = currentPos;
     }
     
     // Use the last target position as the base for endpoint
     float x = lastTargetPosition.size() > 0 ? lastTargetPosition[0] : currentPos[0];
     float y = lastTargetPosition.size() > 1 ? lastTargetPosition[1] : currentPos[1];
     float z = lastTargetPosition.size() > 2 ? lastTargetPosition[2] : currentPos[2];
     float f = lastFeedrate;
     
     // Get center offsets (I, J, K)
     float i = 0.0f, j = 0.0f, k = 0.0f;
     
     // Extract endpoint parameters
     auto it = params.find('X');
     if (it != params.end())
       x = it->second;
 
     it = params.find('Y');
     if (it != params.end())
       y = it->second;
 
     it = params.find('Z');
     if (it != params.end())
       z = it->second;
       
     // Extract center offset parameters
     it = params.find('I');
     if (it != params.end())
       i = it->second;
       
     it = params.find('J');
     if (it != params.end())
       j = it->second;
       
     it = params.find('K');
     if (it != params.end())
       k = it->second;
 
     it = params.find('F');
     if (it != params.end())
     {
       f = it->second;
       lastFeedrate = f;
     }
 
     // Handle relative mode
     if (!absoluteMode)
     {
       // In relative mode, add offsets to current position (not last target)
       if (params.find('X') != params.end())
         x = currentPos[0] + params.at('X');
       if (params.find('Y') != params.end())
         y = currentPos[1] + params.at('Y');
       if (params.find('Z') != params.end() && currentPos.size() > 2)
         z = currentPos[2] + params.at('Z');
     }
 
     // Update last target position
     if (lastTargetPosition.size() < 3) {
       lastTargetPosition.resize(3);
     }
     lastTargetPosition[0] = x;
     lastTargetPosition[1] = y;
     lastTargetPosition[2] = z;
 
     // Create target position and center offset vectors
     std::vector<float> targetPos = {x, y, z};
     std::vector<float> centerOffset = {i, j, k};
 
     // Execute arc move
     bool isClockwise = (code == 2);
     return motionControl->executeArcMove(targetPos, centerOffset, f, isClockwise);
   }
 
   case 4: // G4: Dwell
   {
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
       return motionControl->executeDwell(time);
     }
 
     return true;
   }
 
   case 21: // G21: Set units to mm
     return true;
 
   case 28: // G28: Home
   {
     std::vector<String> axesToHome;
     
     // Check for specific axes to home
     auto it = params.find('X');
     if (it != params.end()) {
       axesToHome.push_back("X");
     }
 
     it = params.find('Y');
     if (it != params.end()) {
       axesToHome.push_back("Y");
     }
 
     it = params.find('Z');
     if (it != params.end()) {
       axesToHome.push_back("Z");
     }
 
     // Execute homing
     bool success = motionControl->executeHoming(axesToHome);
     
     // Update last target position to match homed position
     if (success) {
       lastTargetPosition = machineController->getCurrentWorkPosition();
     }
     
     return success;
   }
 
   case 90: // G90: Set to absolute positioning
     absoluteMode = true;
     if (machineController) {
       machineController->setAbsoluteMode(true);
     }
     return true;
 
   case 91: // G91: Set to relative positioning
     absoluteMode = false;
     if (machineController) {
       machineController->setAbsoluteMode(false);
     }
     return true;
 
   case 92: // G92: Set work position
   {
     // This will be implemented in the machine controller
     if (!machineController) {
       return false;
     }
     
     std::vector<float> currentWorldPos = machineController->getCurrentWorldPosition();
     std::vector<float> newWorkOffset = machineController->getWorkOffset();
     
     auto it = params.find('X');
     if (it != params.end()) {
       // Calculate the new offset: world_position - desired_work_position
       newWorkOffset[0] = currentWorldPos[0] - it->second;
     }
     
     it = params.find('Y');
     if (it != params.end()) {
       newWorkOffset[1] = currentWorldPos[1] - it->second;
     }
     
     it = params.find('Z');
     if (it != params.end()) {
       newWorkOffset[2] = currentWorldPos[2] - it->second;
     }
     
     machineController->setWorkOffset(newWorkOffset);
     
     // Update last target position to match new work position
     std::vector<float> newWorkPos = machineController->getCurrentWorkPosition();
     lastTargetPosition = newWorkPos;
     
     return true;
   }
 
   default:
     Debug::error("GCodeParser","Unsupported G-code: G" + String(code));
     return false;
   }
 }
 
 bool GCodeParser::executeMCode(int code, const std::map<char, float> &params)
 {
   // Get machine controller for system operations
   MachineController* machineController = motionControl ? motionControl->getMachineController() : nullptr;
   
   switch (code)
   {
   case 0: // M0: Program pause
   case 1: // M1: Program pause
     // Wait for all motion to complete
     if (motionControl) {
       motionControl->executeDwell(0); // Wait with no added dwell
     }
     return true;
 
   case 2:  // M2: Program end
   case 30: // M30: Program end
     // Wait for all motion to complete 
     if (motionControl) {
       motionControl->executeDwell(0);
     }
     return true;
 
   case 112: // M112: Emergency stop
     if (machineController) {
       machineController->emergencyStop();
     }
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
 
     case 2: // G2: Clockwise arc
     case 3: // G3: Counter-clockwise arc
       // Need endpoint and center offset
       if ((params.find('X') == params.end() && params.find('Y') == params.end()) ||
           (params.find('I') == params.end() && params.find('J') == params.end()))
       {
         errorMessage = "G" + String(code) + " requires endpoint (X,Y) and center offset (I,J)";
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
   static const int supportedCodes[] = {0, 1, 2, 3, 4, 20, 21, 28, 90, 91, 92};
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
   static const int supportedCodes[] = {0, 1, 2, 30, 112};
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