/**
 * @file MotorManager.cpp
 * @brief Implementation of the MotorManager class using FluidNC-style motion system
 */

 #include "MotorManager.h"
 #include "Debug.h"
 #include "Stepping.h"
 #include "MotionSystem.h"
 
 MotorManager::MotorManager() : configManager(nullptr)
 {
 }
 
 MotorManager::~MotorManager()
 {
   // Clean up motors
   for (Motor *motor : motors)
   {
     delete motor;
   }
   motors.clear();
 }
 
 bool MotorManager::initialize(ConfigManager *configManager)
 {
   Debug::info("MotorManager", "Starting motor initialization");
 
   this->configManager = configManager;
   if (!configManager)
   {
     Debug::error("MotorManager", "Invalid configuration manager");
     return false;
   }
 
   // Clean up existing motors
   Debug::verbose("MotorManager", "Cleaning up existing motors");
   for (Motor *motor : motors)
   {
     Debug::verbose("MotorManager", "Deleting motor: " + (motor ? motor->getName() : "Unknown"));
     delete motor;
   }
   motors.clear();
 
   // Create motors based on configuration
   int numMotors = configManager->getNumMotors();
   Debug::verbose("MotorManager", "Total motors in configuration: " + String(numMotors));
 
   int successfulInitCount = 0;
   int failedInitCount = 0;
 
   for (int i = 0; i < numMotors; i++)
   {
     const MotorConfig *motorConfig = configManager->getMotorConfig(i);
     if (motorConfig)
     {
       Debug::verbose("MotorManager", "Attempting to initialize motor: " + motorConfig->name);
 
       Motor *motor = new Motor(motorConfig);
       if (motor->initialize())
       {
         motors.push_back(motor);
         successfulInitCount++;
         Debug::info("MotorManager", "Successfully initialized motor: " + motorConfig->name);
       }
       else
       {
         Debug::error("MotorManager", "Failed to initialize motor: " + motorConfig->name);
         delete motor;
         failedInitCount++;
       }
     }
     else
     {
       Debug::warning("MotorManager", "Null motor configuration at index " + String(i));
       failedInitCount++;
     }
   }
 
   Debug::info("MotorManager", "Motor initialization summary: " +
                                   String(successfulInitCount) + " successful, " +
                                   String(failedInitCount) + " failed");
 
   return motors.size() > 0;
 }
 
 Motor *MotorManager::getMotor(int index)
 {
   if (index >= 0 && index < motors.size())
   {
     return motors[index];
   }
 
   Debug::warning("MotorManager", "Invalid motor index requested: " + String(index) +
                                      ". Total motors: " + String(motors.size()));
   return nullptr;
 }
 
 Motor *MotorManager::getMotorByName(const String &name)
 {
   for (Motor *motor : motors)
   {
     if (motor && motor->getName() == name)
     {
       return motor;
     }
   }
 
   Debug::warning("MotorManager", "Motor not found with name: " + name);
   return nullptr;
 }
 
 int MotorManager::getNumMotors() const
 {
   return motors.size();
 }
 
 bool MotorManager::update()
 {
   // Update each motor's state
   for (Motor *motor : motors)
   {
     if (motor)
     {
       motor->update();
     }
   }
   return true;
 }
 
 bool MotorManager::isAnyMotorMoving() const
 {
   // This now depends on the motion system rather than individual motors
   // Check with the motion system if any movement is in progress
   for (Motor *motor : motors)
   {
     if (motor && motor->isMoving())
     {
       return true;
     }
   }
   return false;
 }
 
 bool MotorManager::moveTo(const std::vector<float> &positions, float feedrate)
 {
   if (positions.size() != motors.size())
   {
     Debug::error("MotorManager", "Position array size mismatch in feedrate move");
     return false;
   }
 
   // In the new architecture with FluidNC-style motion planning,
   // we don't set each motor's speed individually for coordinated motion.
   // Instead, we create a single move and let the planner handle it.
   
   // Prepare plan_line_data for the planner
   plan_line_data_t pl_data;
   memset(&pl_data, 0, sizeof(pl_data));
   pl_data.feed_rate = feedrate;  // Use provided feedrate
   
   // Convert target positions to array format for planner
   float target[MAX_N_AXIS] = {0};
   
   // Determine if any motor needs to move
   bool needsMove = false;
   
   for (size_t i = 0; i < motors.size(); i++)
   {
     if (i < MAX_N_AXIS) {
       float currentPos = motors[i]->getPositionInUnits();
       target[i] = positions[i];
       
       // Check if this motor needs to move
       if (fabs(positions[i] - currentPos) >= 0.0001f) {
         needsMove = true;
       }
     }
   }
   
   // If no movement needed, return success
   if (!needsMove) {
     return true;
   }
   
   // Add the move to the planner
   bool success = Planner::buffer_line(target, &pl_data);
   if (success) {
     // Wake up stepper system if needed
     Stepper::wake_up();
     
     Debug::verbose("MotorManager", "Added coordinated move at " + 
                    String(feedrate) + " mm/min");
   } else {
     Debug::error("MotorManager", "Failed to add move to planner");
   }
   
   return success;
 }
 
 void MotorManager::stopAll(bool immediate)
 {
   Debug::info("MotorManager", "Stopping all motors" +
                                   String(immediate ? " (IMMEDIATE)" : " (DECELERATE)"));
 
   // In FluidNC architecture, we need to:
   // 1. Clear the planner queue
   // 2. Stop current stepping operations
   Planner::reset();
   Stepper::stop_stepping();
   
   // Also notify all motors to update their status
   for (Motor *motor : motors)
   {
     if (motor) {
       motor->stop(immediate);
     }
   }
 }
 
 bool MotorManager::homeAll()
 {
   Debug::info("MotorManager", "Starting home all sequence");
   bool success = true;
   unsigned long startTime = millis();
 
   // Home sequence prioritized order
   std::vector<String> homeOrder = {"Z", "X", "Y"};
 
   for (const String &axisName : homeOrder)
   {
     Motor *motor = getMotorByName(axisName);
     if (motor)
     {
       Debug::verbose("MotorManager", "Homing " + axisName + " axis");
       bool axisHomeSuccess = motor->startHoming();
 
       if (!axisHomeSuccess)
       {
         Debug::error("MotorManager", "Failed to start homing for " + axisName + " axis");
         success = false;
         continue;
       }
 
       // Wait for homing to complete
       unsigned long axisStartTime = millis();
       while (motor->getStatus() == HOMING)
       {
         update();
         delay(10);
 
         // Process motion system updates during homing
         MotionSystem::update();
 
         // Timeout check (30 seconds)
         if (millis() - axisStartTime > 30000)
         {
           Debug::error("MotorManager", axisName + " axis homing timed out");
           success = false;
           break;
         }
       }
 
       Debug::info("MotorManager", axisName + " axis homing " +
                                       String(motor->getStatus() != HOMING ? "COMPLETED" : "FAILED"));
     }
     else
     {
       Debug::warning("MotorManager", axisName + " axis motor not found");
     }
   }
 
   // Home any remaining motors not in the main axes
   Debug::verbose("MotorManager", "Checking for additional motors to home");
   for (Motor *motor : motors)
   {
     if (motor && motor->getName() != "X" && motor->getName() != "Y" && motor->getName() != "Z")
     {
       Debug::info("MotorManager", "Homing additional motor: " + motor->getName());
 
       bool additionalAxisSuccess = motor->startHoming();
 
       if (!additionalAxisSuccess)
       {
         Debug::error("MotorManager", "Failed to start homing for additional motor: " + motor->getName());
         success = false;
         continue;
       }
 
       // Wait for homing to complete
       unsigned long additionalAxisStartTime = millis();
       while (motor->getStatus() == HOMING)
       {
         update();
         delay(10);
         
         // Process motion system updates during homing
         MotionSystem::update();
 
         // Timeout check (30 seconds)
         if (millis() - additionalAxisStartTime > 30000)
         {
           Debug::error("MotorManager", motor->getName() + " additional axis homing timed out");
           success = false;
           break;
         }
       }
 
       Debug::info("MotorManager", motor->getName() + " additional axis homing " +
                                       String(motor->getStatus() != HOMING ? "COMPLETED" : "FAILED"));
     }
   }
 
   // Overall homing process summary
   unsigned long totalTime = millis() - startTime;
   Debug::info("MotorManager", "Home all sequence " +
                                   String(success ? "COMPLETED SUCCESSFULLY" : "FAILED") +
                                   " in " + String(totalTime) + " ms");
 
   return success;
 }
 
 bool MotorManager::homeMotor(int motorIndex)
 {
   Motor *motor = getMotor(motorIndex);
   if (motor)
   {
     Debug::info("MotorManager", "Starting homing for motor at index " + String(motorIndex));
     return motor->startHoming();
   }
 
   Debug::error("MotorManager", "Cannot home motor at index " + String(motorIndex) +
                                    ". Motor not found.");
   return false;
 }
 
 bool MotorManager::homeMotorByName(const String &motorName)
 {
   Motor *motor = getMotorByName(motorName);
   if (motor)
   {
     Debug::info("MotorManager", "Starting homing for motor: " + motorName);
     return motor->startHoming();
   }
 
   Debug::error("MotorManager", "Cannot home motor: " + motorName +
                                    ". Motor not found.");
   return false;
 }