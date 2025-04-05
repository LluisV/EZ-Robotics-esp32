/**
 * @file GCodeParser.cpp
 * @brief Implementation of the GCodeParser class
 */

 #include "GCodeParser.h"
 #include <map>
 
 GCodeParser::GCodeParser(MachineController* machineController)
   : machineController(machineController),
     lastFeedrate(1000.0f),
     absoluteMode(true),
     imperialUnits(false)
 {
 }
 
 bool GCodeParser::parse(const String& command) {
   // Ignore empty lines and comments
   String trimmedCmd = command;
   trimmedCmd.trim();
   
   if (trimmedCmd.length() == 0 || trimmedCmd.startsWith(";")) {
     return true;
   }
   
   // Remove inline comments
   int commentIdx = trimmedCmd.indexOf(';');
   if (commentIdx >= 0) {
     trimmedCmd = trimmedCmd.substring(0, commentIdx);
     trimmedCmd.trim();
   }
   
   // Extract G/M code
   char codeType;
   int code = extractCode(trimmedCmd, codeType);
   
   if (code < 0) {
     Serial.println("Invalid G-code format: " + command);
     return false;
   }
   
   // Parse parameters
   std::map<char, float> params = parseParameters(trimmedCmd);
   
   // Maintain last parameters for modal commands
   for (const auto& param : params) {
     lastParams[param.first] = param.second;
   }
   
   // Execute the command
   bool result = false;
   if (codeType == 'G') {
     result = executeGCode(code, params);
   } else if (codeType == 'M') {
     result = executeMCode(code, params);
   } else {
     Serial.println("Unsupported code type: " + String(codeType));
     return false;
   }
   
   if (!result) {
     Serial.println("Failed to execute command: " + command);
   }
   
   return result;
 }
 
 std::vector<String> GCodeParser::getSupportedCodes() const {
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
 
 std::map<char, float> GCodeParser::parseParameters(const String& command) {
   std::map<char, float> params;
   
   for (int i = 0; i < command.length(); i++) {
     char c = command.charAt(i);
     
     // Skip non-parameter letters or already processed code letter
     if (!isalpha(c) || c == 'G' || c == 'g' || c == 'M' || c == 'm') {
       continue;
     }
     
     // Find the parameter value
     int valueStart = i + 1;
     int valueEnd = valueStart;
     
     // Find the end of the number
     while (valueEnd < command.length() && 
            (isdigit(command.charAt(valueEnd)) || 
             command.charAt(valueEnd) == '.' || 
             command.charAt(valueEnd) == '-')) {
       valueEnd++;
     }
     
     // Extract and convert the value
     if (valueEnd > valueStart) {
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
 
 int GCodeParser::extractCode(const String& command, char& codeType) {
   // Find G or M code
   for (int i = 0; i < command.length(); i++) {
     char c = toupper(command.charAt(i));
     
     if (c == 'G' || c == 'M') {
       codeType = c;
       
       // Find the code number
       int valueStart = i + 1;
       int valueEnd = valueStart;
       
       // Find the end of the number
       while (valueEnd < command.length() && 
              (isdigit(command.charAt(valueEnd)) || 
               command.charAt(valueEnd) == '.')) {
         valueEnd++;
       }
       
       // Extract and convert the value
       if (valueEnd > valueStart) {
         String valueStr = command.substring(valueStart, valueEnd);
         return valueStr.toInt();
       }
     }
   }
   
   return -1; // No valid code found
 }
 
 bool GCodeParser::executeGCode(int code, const std::map<char, float>& params) {
   switch (code) {
     case 0: // G0: Rapid move
     case 1: { // G1: Linear move
       // Get coordinates
       float x = NAN, y = NAN, z = NAN;
       float f = lastFeedrate;
       
       // Extract parameters with defaults from last command
       auto it = params.find('X');
       if (it != params.end()) x = it->second;
       
       it = params.find('Y');
       if (it != params.end()) y = it->second;
       
       it = params.find('Z');
       if (it != params.end()) z = it->second;
       
       it = params.find('F');
       if (it != params.end()) {
         f = it->second;
         lastFeedrate = f;
       }
       
       // Convert to mm if in imperial mode
       if (imperialUnits) {
         if (!isnan(x)) x *= 25.4f;
         if (!isnan(y)) y *= 25.4f;
         if (!isnan(z)) z *= 25.4f;
         f *= 25.4f;
       }
       
       // Execute move if any coordinates specified
       if (!isnan(x) || !isnan(y) || !isnan(z)) {
         MovementType moveType = (code == 0) ? RAPID_MOVE : LINEAR_MOVE;
         return machineController->moveTo(x, y, z, f, moveType);
       }
       
       return true;
     }
     
     case 4: { // G4: Dwell
       // Get dwell time
       float time = 0;
       
       auto it = params.find('P');
       if (it != params.end()) {
         time = it->second / 1000.0f; // P is in milliseconds
       }
       
       it = params.find('S');
       if (it != params.end()) {
         time = it->second; // S is in seconds
       }
       
       if (time > 0) {
         delay(time * 1000); // Convert to milliseconds
       }
       
       return true;
     }
     
     case 20: // G20: Set units to inches
       imperialUnits = true;
       return true;
       
     case 21: // G21: Set units to mm
       imperialUnits = false;
       return true;
       
     case 28: { // G28: Home
       bool homeX = false, homeY = false, homeZ = false;
       bool homeSpecific = false;
       
       // Check for specific axes to home
       auto it = params.find('X');
       if (it != params.end()) {
         homeX = true;
         homeSpecific = true;
       }
       
       it = params.find('Y');
       if (it != params.end()) {
         homeY = true;
         homeSpecific = true;
       }
       
       it = params.find('Z');
       if (it != params.end()) {
         homeZ = true;
         homeSpecific = true;
       }
       
       // If no specific axes mentioned, home all
       if (!homeSpecific) {
         return machineController->homeAll();
       } else {
         bool success = true;
         if (homeZ) success &= machineController->homeAxis("Z");
         if (homeX) success &= machineController->homeAxis("X");
         if (homeY) success &= machineController->homeAxis("Y");
         return success;
       }
     }
     
     case 90: // G90: Set to absolute positioning
       absoluteMode = true;
       return true;
       
     case 91: // G91: Set to relative positioning
       absoluteMode = false;
       return true;
       
     case 92: { // G92: Set position
       std::vector<float> currentPos = machineController->getCurrentPosition();
       std::vector<float> offsets = currentPos;
       
       auto it = params.find('X');
       if (it != params.end()) offsets[0] = it->second;
       
       it = params.find('Y');
       if (it != params.end()) offsets[1] = it->second;
       
       it = params.find('Z');
       if (it != params.end()) offsets[2] = it->second;
       
       machineController->setWorkOffset(offsets);
       return true;
     }
     
     default:
       Serial.println("Unsupported G-code: G" + String(code));
       return false;
   }
 }
 
 bool GCodeParser::executeMCode(int code, const std::map<char, float>& params) {
   switch (code) {
     case 0:  // M0: Program pause
     case 1:  // M1: Program pause
       // Just pause, wait for user input
       Serial.println("Program paused. Send any command to continue.");
       while (!Serial.available()) {
         delay(100);
       }
       // Clear input buffer
       while (Serial.available()) Serial.read();
       return true;
       
     case 2:  // M2: Program end
     case 30: // M30: Program end
       Serial.println("Program completed.");
       return true;
       
     case 112: // M112: Emergency stop
       machineController->emergencyStop();
       return true;
       
     default:
       Serial.println("Unsupported M-code: M" + String(code));
       return false;
   }
 }