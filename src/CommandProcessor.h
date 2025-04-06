/**
 * @file CommandProcessor.h
 * @brief Command processor for handling special commands and information requests
 */

 #ifndef COMMAND_PROCESSOR_H
 #define COMMAND_PROCESSOR_H
 
 #include <Arduino.h>
 #include "MachineController.h"
 
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
 
   /**
    * @brief Get pointer to the MachineController
    * @return MachineController pointer
    */
   MachineController* getMachineController() {
     return machineController;
   }
 
 private:
   MachineController* machineController;  ///< MachineController reference
   ConfigManager* configManager;
 };
 
 #endif // COMMAND_PROCESSOR_H