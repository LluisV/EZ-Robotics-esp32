#ifndef FREERTOS_CONFIG_OVERRIDE_H
#define FREERTOS_CONFIG_OVERRIDE_H

// Include the default ESP32 FreeRTOS configuration
#include <freertos/FreeRTOSConfig.h>

// Override configuration settings
#undef configGENERATE_RUN_TIME_STATS
#define configGENERATE_RUN_TIME_STATS           1

#undef configUSE_TRACE_FACILITY
#define configUSE_TRACE_FACILITY                1

#undef configUSE_STATS_FORMATTING_FUNCTIONS
#define configUSE_STATS_FORMATTING_FUNCTIONS    1

// Define functions required for runtime stats
#if configGENERATE_RUN_TIME_STATS
    extern "C" {
        void vConfigureTimerForRunTimeStats(void);
        unsigned long ulGetRunTimeCounterValue(void);
    }
    #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vConfigureTimerForRunTimeStats()
    #define portGET_RUN_TIME_COUNTER_VALUE() ulGetRunTimeCounterValue()
#endif

#endif // FREERTOS_CONFIG_OVERRIDE_H