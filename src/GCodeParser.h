/**
 * @file GCodeParser.h
 * @brief G-code parser for CNC controller
 */

 #ifndef GCODE_PARSER_H
 #define GCODE_PARSER_H
 
 #include <Arduino.h>
 #include <vector>
 #include <map>
 #include "MachineController.h"
 
 /**
  * @brief G-code parser class
  */
 class GCodeParser {
 public:
   /**
    * @brief Construct a new GCodeParser
    * @param machineController MachineController reference
    */
   GCodeParser(MachineController* machineController);
   
   /**
    * @brief Parse and execute a G-code command
    * @param command G-code command string
    * @return True if successful, false otherwise
    */
   bool parse(const String& command);
   
   /**
    * @brief Get a list of supported G-codes
    * @return List of supported G-codes
    */
   std::vector<String> getSupportedCodes() const;
 
 private:
   MachineController* machineController;  ///< MachineController reference
   float lastFeedrate;                    ///< Last used feedrate
   bool absoluteMode;                     ///< True if in absolute mode, false if in relative mode
   bool imperialUnits;                    ///< True if using imperial units, false if metric
   
   // Last command parameters
   std::map<char, float> lastParams;
   
   /**
    * @brief Parse G-code parameters from a command string
    * @param command G-code command string
    * @return Map of parameters (char to float)
    */
   std::map<char, float> parseParameters(const String& command);
   
   /**
    * @brief Extract the G or M code from a command string
    * @param command G-code command string
    * @param codeType Output - will be 'G' or 'M'
    * @return Code number
    */
   int extractCode(const String& command, char& codeType);
   
   /**
    * @brief Execute a G-code
    * @param code G-code number
    * @param params Parameters map
    * @return True if successful, false otherwise
    */
   bool executeGCode(int code, const std::map<char, float>& params);
   
   /**
    * @brief Execute an M-code
    * @param code M-code number
    * @param params Parameters map
    * @return True if successful, false otherwise
    */
   bool executeMCode(int code, const std::map<char, float>& params);
 };
 
 #endif // GCODE_PARSER_H