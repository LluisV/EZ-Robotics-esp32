/**
 * @file GCodeParser.h
 * @brief G-code parser for CNC controller with pipeline architecture
 */

 #ifndef GCODE_PARSER_H
 #define GCODE_PARSER_H
 
 #include <Arduino.h>
 #include <vector>
 #include <map>
 #include "MotionControl.h"
 #include "CommonTypes.h"
 
 // Forward declarations
 class MotionControl;
 
 /**
  * @brief G-code parser class
  * 
  * Parses GCode commands and forwards them to the Motion Control system
  * for execution. Handles syntax validation and parameter extraction.
  */
 class GCodeParser
 {
 public:
   /**
    * @brief Construct a new GCodeParser
    * @param motionControl Reference to the motion control system
    */
   GCodeParser(MotionControl* motionControl);
 
   /**
    * @brief Parse and execute a G-code command
    * @param command G-code command string
    * @return GCodeParseResult indicating parsing and queueing status
    */
   GCodeParseResult parse(const String &command);
 
   /**
    * @brief Get a list of supported G-codes
    * @return List of supported G-codes
    */
   std::vector<String> getSupportedCodes() const;
 
 private:
   MotionControl* motionControl;          ///< Reference to motion control system
   float lastFeedrate;                    ///< Last used feedrate
   std::vector<float> lastTargetPosition; ///< Last commanded target position
   bool absoluteMode;                     ///< Current coordinate mode (true for absolute, false for relative)
 
   // Last command parameters
   std::map<char, float> lastParams;
 
   /**
    * @brief Parse G-code parameters from a command string
    * @param command G-code command string
    * @return Map of parameters (char to float)
    */
   std::map<char, float> parseParameters(const String &command);
 
   /**
    * @brief Extract the G or M code from a command string
    * @param command G-code command string
    * @param codeType Output - will be 'G' or 'M'
    * @return Code number
    */
   int extractCode(const String &command, char &codeType);
 
   /**
    * @brief Execute a G-code
    * @param code G-code number
    * @param params Parameters map
    * @return True if successful, false otherwise
    */
   bool executeGCode(int code, const std::map<char, float> &params);
 
   /**
    * @brief Execute an M-code
    * @param code M-code number
    * @param params Parameters map
    * @return True if successful, false otherwise
    */
   bool executeMCode(int code, const std::map<char, float> &params);
 
   /**
    * @brief Validate a G-code line without executing it
    * @param command G-code command string
    * @param errorMessage Output parameter for error message if invalid
    * @return True if valid, false otherwise
    */
   bool validate(const String &command, String &errorMessage);
 
   /**
    * @brief Check if a G-code is supported
    * @param code G-code number
    * @return True if supported, false otherwise
    */
   bool isGCodeSupported(int code) const;
 
   /**
    * @brief Check if an M-code is supported
    * @param code M-code number
    * @return True if supported, false otherwise
    */
   bool isMCodeSupported(int code) const;
 };
 
 #endif // GCODE_PARSER_H