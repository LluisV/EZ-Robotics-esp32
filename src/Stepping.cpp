/**
 * @file Stepping.cpp
 * @brief Implementation of the step generation system
 */

 #include "Stepping.h"
 #include "Motor.h"
 #include "Debug.h"
 #include "Stepper.h"
 
 // Static member initialization
 hw_timer_t* Stepping::stepTimer = nullptr;
 portMUX_TYPE Stepping::timerMux = portMUX_INITIALIZER_UNLOCKED;
 volatile bool Stepping::timerRunning = false;
 
 uint32_t Stepping::pulseUsecs = 4;
 uint32_t Stepping::directionDelayUsecs = 0;
 
 std::vector<Motor*> Stepping::motors[MAX_N_AXIS][2];
 uint8_t Stepping::lastDirMask = 255; // Invalid value to force first-time update
 int32_t Stepping::axisSteps[MAX_N_AXIS] = {0};
 
 // Forward declaration
 void IRAM_ATTR Stepping::onStepperTimer() {
     portENTER_CRITICAL_ISR(&timerMux);
     bool continueRunning = pulseFunc();
     portEXIT_CRITICAL_ISR(&timerMux);
     
     if (!continueRunning) {
         stopTimer();
     }
 }
 
 void Stepping::init() {
     // Initialize the hardware timer
     stepTimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 MHz counting frequency)
     timerAttachInterrupt(stepTimer, &onStepperTimer, true);
     timerAlarmWrite(stepTimer, 1000, true); // Default period 1000 ticks (1ms)
     
     Debug::info("Stepping", "Initialized with pulse width " + String(pulseUsecs) + "us, dir delay " + 
                             String(directionDelayUsecs) + "us");
     
     // Reset all counters and state
     reset();
 }
 
 void Stepping::reset() {
     stopTimer();
     lastDirMask = 255; // Force direction update
     
     // Reset axis step counters
     for (int i = 0; i < MAX_N_AXIS; i++) {
         axisSteps[i] = 0;
     }
 }
 
 void Stepping::registerMotor(Motor* motor, size_t axis, size_t motorNum) {
     if (axis < MAX_N_AXIS && motorNum < 2) {
         motors[axis][motorNum].push_back(motor);
         Debug::verbose("Stepping", "Registered motor on axis " + String(axis) + 
                                    ", motor " + String(motorNum));
     }
 }
 
 void IRAM_ATTR Stepping::step(uint8_t stepMask, uint8_t dirMask) {
     // Set direction pins if they've changed
     if (dirMask != lastDirMask) {
        for (int axis = 0; axis < MAX_N_AXIS; axis++) {
            // FIXED: Skip if this axis has no motors
            if (motors[axis][0].empty() && motors[axis][1].empty()) {
                continue;
            }
            
            bool dir = bitRead(dirMask, axis);
            bool oldDir = bitRead(lastDirMask, axis);
            
            if (dir != oldDir || lastDirMask == 255) {
                for (int motorNum = 0; motorNum < 2; motorNum++) {
                    for (Motor* motor : motors[axis][motorNum]) {
                        if (motor && motor->getConfig()) {
                            // Set direction pin
                            digitalWrite(motor->getConfig()->dirPin, dir ^ motor->getConfig()->invertDirection);
                        }
                    }
                }
            }
        }
        
        // Wait for direction setup time if needed
        if (directionDelayUsecs > 0) {
            delayMicroseconds(directionDelayUsecs);
        }
        
        lastDirMask = dirMask;
    }
    
    // Now set step pins high
    for (int axis = 0; axis < MAX_N_AXIS; axis++) {
        // FIXED: Skip if this axis has no motors
        if (motors[axis][0].empty() && motors[axis][1].empty()) {
            continue;
        }
        
        if (bitRead(stepMask, axis)) {
            // Update step counter
            int increment = bitRead(dirMask, axis) ? -1 : 1;
            axisSteps[axis] += increment;
            
            // Step all motors on this axis
            for (int motorNum = 0; motorNum < 2; motorNum++) {
                for (Motor* motor : motors[axis][motorNum]) {
                    if (motor && motor->getConfig()) {
                        // Set step pin high (or low if inverted)
                        digitalWrite(motor->getConfig()->stepPin, !motor->getConfig()->invertStep);
                    }
                }
            }
        }
    }
    
    // Wait for pulse duration
    if (pulseUsecs > 0) {
        delayMicroseconds(pulseUsecs);
    }
 }
 
 void IRAM_ATTR Stepping::unstep() {
     // Turn off all step pins
     for (int axis = 0; axis < MAX_N_AXIS; axis++) {
         for (int motorNum = 0; motorNum < 2; motorNum++) {
             for (Motor* motor : motors[axis][motorNum]) {
                 digitalWrite(motor->getConfig()->stepPin, motor->getConfig()->invertStep);
             }
         }
     }
 }
 
 void Stepping::setTimerPeriod(uint32_t timerTicks) {
     if (stepTimer) {
         timerAlarmWrite(stepTimer, timerTicks, true);
     }
 }
 
 void Stepping::startTimer() {
     if (stepTimer && !timerRunning) {
         timerRunning = true;
         timerAlarmEnable(stepTimer);
     }
 }
 
 void Stepping::stopTimer() {
     if (stepTimer && timerRunning) {
         timerAlarmDisable(stepTimer);
         timerRunning = false;
     }
 }
 
 uint32_t Stepping::maxPulsesPerSec() {
     // Calculate maximum step frequency based on pulse length and direction delay
     uint32_t cycleTime = pulseUsecs + 1; // Minimum 1us between pulses
     return 1000000 / cycleTime;
 }
 
 void Stepping::setPulseLength(uint32_t pulseUsecs) {
     Stepping::pulseUsecs = pulseUsecs;
 }
 
 void Stepping::setDirectionDelay(uint32_t delayUsecs) {
     Stepping::directionDelayUsecs = delayUsecs;
 }
 
 // This is the step pulse generation callback function
 bool Stepping::pulseFunc() {
    // Call the Stepper module's pulse function which handles
    // all the segment buffer and step generation logic
    return Stepper::pulse_func();
}