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
 
 // Debug counters
 volatile uint32_t Stepping::totalStepCount = 0;
 volatile uint32_t Stepping::stepCounts[MAX_N_AXIS] = {0};
 
 // Flag for delayed timer start to prevent watchdog issues
 static bool delayedTimerStart = false;
 
 // Forward declaration
 void IRAM_ATTR Stepping::onStepperTimer() {
     // Keep ISR as short as possible to prevent watchdog timeouts
     bool continueRunning = false;
     
     // Use critical section only for the data we need to protect
     portENTER_CRITICAL_ISR(&timerMux);
     continueRunning = Stepper::pulse_func();
     portEXIT_CRITICAL_ISR(&timerMux);
     
     if (!continueRunning) {
         // Don't stop the timer directly from ISR - set a flag
         timerRunning = false;
     }
 }
 
 void Stepping::init() {
     // Initialize the hardware timer with a safer configuration
     if (stepTimer != nullptr) {
         timerEnd(stepTimer);
         stepTimer = nullptr;
     }
     
     // Use timer 1 instead of 0 (timer 0 might be used by the system)
     stepTimer = timerBegin(1, 80, true); // Timer 1, prescaler 80 (1 MHz counting frequency)
     
     // Attach interrupt with edge-triggered mode (false) instead of level-triggered (true)
     // This can help prevent watchdog timeouts
     timerAttachInterrupt(stepTimer, &onStepperTimer, false);
     
     // Default period 2000 ticks (2ms) - slower than before but safer
     timerAlarmWrite(stepTimer, 2000, true);
     
     // Don't auto-start the timer
     timerAlarmDisable(stepTimer);
     
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
         stepCounts[i] = 0;
     }
     
     totalStepCount = 0;
 }
 
 void Stepping::registerMotor(Motor* motor, size_t axis, size_t motorNum) {
     if (axis < MAX_N_AXIS && motorNum < 2) {
         motors[axis][motorNum].push_back(motor);
         Debug::info("Stepping", "Registered motor " + motor->getName() + " on axis " + String(axis) + 
                                ", motor " + String(motorNum));
     } else {
         Debug::error("Stepping", "Failed to register motor - Invalid axis " + String(axis) + 
                                " or motor " + String(motorNum));
     }
 }
 
 void IRAM_ATTR Stepping::step(uint8_t stepMask, uint8_t dirMask) {
     // Set direction pins if they've changed
     if (dirMask != lastDirMask) {
         for (int axis = 0; axis < MAX_N_AXIS; axis++) {
             // Skip if this axis has no motors
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
         
         // Wait for direction setup time if needed, but keep it short
         // for ISR safety
         if (directionDelayUsecs > 0 && directionDelayUsecs < 50) {
             delayMicroseconds(directionDelayUsecs);
         }
         
         lastDirMask = dirMask;
     }
     
     // Now set step pins high
     for (int axis = 0; axis < MAX_N_AXIS; axis++) {
         // Skip if this axis has no motors
         if (motors[axis][0].empty() && motors[axis][1].empty()) {
             continue;
         }
         
         if (bitRead(stepMask, axis)) {
             // Update step counter
             int increment = bitRead(dirMask, axis) ? -1 : 1;
             axisSteps[axis] += increment;
             stepCounts[axis]++; // Count total steps regardless of direction
             totalStepCount++;   // Count all steps
             
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
     
     // Wait for pulse duration, but keep it short for ISR safety
     if (pulseUsecs > 0 && pulseUsecs < 30) {
         delayMicroseconds(pulseUsecs);
     }
     
     // Reset step pins immediately to keep ISR short
     unstep();
 }
 
 void IRAM_ATTR Stepping::unstep() {
     // Turn off all step pins
     for (int axis = 0; axis < MAX_N_AXIS; axis++) {
         for (int motorNum = 0; motorNum < 2; motorNum++) {
             for (Motor* motor : motors[axis][motorNum]) {
                 if (motor && motor->getConfig()) {
                     digitalWrite(motor->getConfig()->stepPin, motor->getConfig()->invertStep);
                 }
             }
         }
     }
 }
 
 void Stepping::setTimerPeriod(uint32_t timerTicks) {
     if (stepTimer) {
         // Add some safety bounds
         uint32_t safe_ticks = timerTicks;
         
         // Enforce minimum period of 500µs - this is critical for stability
         if (safe_ticks < 500) safe_ticks = 500;       
         
         // Cap maximum period at 50ms
         if (safe_ticks > 50000) safe_ticks = 50000;   
         
         timerAlarmWrite(stepTimer, safe_ticks, true);
     }
 }
 
 void Stepping::startTimer() {
    if (stepTimer && !timerRunning) {
        Debug::info("Stepping", "Starting stepper timer");
        
        // Try immediate start with protections
        timerRunning = true;
        delayedTimerStart = false;
        
        try {
            timerAlarmEnable(stepTimer);
            Debug::info("Stepping", "Stepper timer started immediately");
        } catch(...) {
            // If this fails, fall back to delayed start
            timerRunning = false;
            delayedTimerStart = true;
            Debug::info("Stepping", "Scheduled timer start (delayed)");
        }
    }
}

void Stepping::forceTimerStart() {
    if (stepTimer) {
        // Direct timer enable without flags
        timerRunning = true;
        delayedTimerStart = false;
        timerAlarmEnable(stepTimer);
        Debug::info("Stepping", "Timer start forced");
    }
}
 
 void Stepping::processDelayedStart() {
     // This should be called from the main loop
     if (delayedTimerStart && !timerRunning && stepTimer != nullptr) {
         // We're outside the interrupt context, safe to start timer
         timerRunning = true;
         delayedTimerStart = false;
         
         // Make sure alarm is configured properly 
         timerAlarmEnable(stepTimer);
         
         Debug::info("Stepping", "Stepper timer started (delayed)");
     }
 }
 
 void Stepping::stopTimer() {
     if (stepTimer && timerRunning) {
         Debug::info("Stepping", "Stopping stepper timer");
         timerAlarmDisable(stepTimer);
         timerRunning = false;
         delayedTimerStart = false;
     }
 }
 
 uint32_t Stepping::maxPulsesPerSec() {
     // Calculate maximum step frequency based on pulse length and direction delay
     uint32_t cycleTime = pulseUsecs + 1; // Minimum 1us between pulses
     return 1000000 / cycleTime;
 }
 
 void Stepping::setPulseLength(uint32_t pulseUsecs) {
     // Limit to safe values
     if (pulseUsecs > 20) pulseUsecs = 20;
     
     Stepping::pulseUsecs = pulseUsecs;
     Debug::info("Stepping", "Pulse length set to " + String(pulseUsecs) + "us");
 }
 
 void Stepping::setDirectionDelay(uint32_t delayUsecs) {
     // Limit to safe values
     if (delayUsecs > 30) delayUsecs = 30;
     
     Stepping::directionDelayUsecs = delayUsecs;
     Debug::info("Stepping", "Direction delay set to " + String(delayUsecs) + "us");
 }
 
 // Get step counts for diagnostics
 uint32_t Stepping::getTotalStepCount() {
     return totalStepCount;
 }
 
 uint32_t Stepping::getAxisStepCount(int axis) {
     if (axis >= 0 && axis < MAX_N_AXIS) {
         return stepCounts[axis];
     }
     return 0;
 }
 
 void Stepping::printStepCounts() {
     String counts = "Step counts - Total: " + String(totalStepCount);
     for (int i = 0; i < MAX_N_AXIS; i++) {
         counts += ", Axis " + String(i) + ": " + String(stepCounts[i]);
     }
     Debug::info("Stepping", counts);
 }
 
 bool Stepping::isTimerRunning() {
     return timerRunning;
 }
 
 bool Stepping::isTimerStartPending() {
     return delayedTimerStart;
 }
 
 // This is the step pulse generation callback function
 bool Stepping::pulseFunc() {
     static uint32_t lastStepCount = 0;
     static unsigned long lastLogTime = 0;
     
     // Call the Stepper module's pulse function which handles
     // all the segment buffer and step generation logic
     bool result = Stepper::pulse_func();
     
     // Periodically log stepping activity (outside the ISR)
     // Using a static counter that can be checked elsewhere
     if (totalStepCount != lastStepCount) {
         lastStepCount = totalStepCount;
         lastLogTime = millis();
     }
     
     return result;
 }