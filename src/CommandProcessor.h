/**
 * @file CommandProcessor.h
 * @brief Command processor for handling special commands and information requests
 */

 #ifndef COMMAND_PROCESSOR_H
 #define COMMAND_PROCESSOR_H
 
 #include <Arduino.h>
 #include "MachineController.h"
 
 /**
  * @brief Command processor class
  */
 class CommandProcessor {
 public:
   /**
    * @brief Construct a new CommandProcessor
    * @param machineController MachineController reference
    * @param configManager ConfigManager reference
    */
   CommandProcessor(MachineController* machineController, ConfigManager* configManager);
   
   /**
    * @brief Process an information request command
    * @param command Information command
    * @return Response string
    */
   String processInfoCommand(const String& command);
   
   /**
    * @brief Process an immediate command
    * @param command Immediate command
    * @return True if command was processed successfully
    */
   bool processImmediateCommand(const String& command);
   
   /**
    * @brief Process a setting command
    * @param command Setting command
    * @return True if command was processed successfully
    */
   bool processSettingCommand(const String& command);
 
 private:
   MachineController* machineController;  ///< MachineController reference
   ConfigManager* configManager;
 };
 
 #endif // COMMAND_PROCESSOR_H