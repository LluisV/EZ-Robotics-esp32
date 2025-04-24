/**
 * @file CommonTypes.h
 * @brief Common type definitions for the CNC controller
 */

 #ifndef COMMON_TYPES_H
 #define COMMON_TYPES_H

 #include <Arduino.h>
 #include <vector>
 
 // Number of axes
 #define MAX_N_AXIS 6 // X, Y, Z, A, B, C
 #define X_AXIS 0
 #define Y_AXIS 1
 #define Z_AXIS 2
 #define A_AXIS 3
 #define B_AXIS 4
 #define C_AXIS 5
 
 // Useful bit manipulation macros
 #define bit(b) (1UL << (b))
 #define bitRead(value, bit) (((value) >> (bit)) & 0x01)
 #define bitSet(value, bit) ((value) |= (1UL << (bit)))
 #define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
 #define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
 #define bitnum_to_mask(axis) bit(axis)
 
 // Movement types
 enum MovementType {
     RAPID_MOTION = 0,
     LINEAR_MOTION,
     ARC_MOTION,
     CIRCULAR_MOTION
 };

  // Define planner data condition flags for motion types
  struct PlMotion {
    uint8_t rapidMotion : 1;     // True for rapid moves, false for feed moves
    uint8_t systemMotion : 1;    // Special single motion (homing, parking, etc.)
    uint8_t noFeedOverride : 1;  // Ignore feed override
    uint8_t inverseTime : 1;     // Interpret feed rate as inverse time
};

// Plan structure for linear motion
struct plan_line_data_t {
    float        feed_rate;       // Desired feed rate in mm/min
    PlMotion     motion;          // Motion condition flags
    int32_t      line_number;     // G-code line number
    bool         is_jog;          // True if this is a jog motion
    bool         limits_checked;  // True if soft limits already checked
};


  // This struct stores a linear movement of a g-code block motion
  struct plan_block_t {
    // Fields used by the bresenham algorithm for tracing line 
    uint32_t steps[MAX_N_AXIS];     // Step count along each axis
    uint32_t step_event_count;      // Total number of steps in this block
    uint8_t  direction_bits;        // Direction bits for each axis

    // Block condition data
    PlMotion     motion;       // Block motion conditions
    int32_t      line_number;  // Block line number for reporting

    // Fields used by the motion planner to manage acceleration
    float entry_speed_sqr;      // Entry speed at start of block (mm/min)^2
    float max_entry_speed_sqr;  // Maximum allowable entry speed (mm/min)^2
    float acceleration;         // Acceleration in (mm/min^2)
    float millimeters;          // Total travel distance for this block (mm)

    // Stored rate limiting data
    float max_junction_speed_sqr;  // Junction entry speed limit (mm/min)^2
    float rapid_rate;              // Maximum rate for this block (mm/min)
    float programmed_rate;         // Programmed rate (mm/min)

    bool is_jog;                 // Indicates if this block is a jog motion
};

  /**
 * @brief Enum for G-code parsing and queueing results
 */
enum class GCodeParseResult
{
    PARSE_ERROR = -1,     ///< Parsing failed, command is invalid
    QUEUE_FULL = 0,       ///< Motion planner queue is full, retry later
    SUCCESS = 1           ///< Successfully parsed and queued
};
 
 // Motor and Axis masks for homing and limit processing
 typedef uint32_t MotorMask;
 typedef uint8_t AxisMask;
 
 // Segment structure for the segment buffer
 struct Segment {
     uint16_t n_step;             // Number of step events to be executed for this segment
     uint16_t isrPeriod;          // Time to next ISR tick in units of timer ticks
     uint8_t st_block_index;      // Stepper block data index
     uint8_t amass_level;         // AMASS level for the ISR to execute this segment
     std::vector<float> jointPositions;
     std::vector<float> desiredVelocities;
     std::vector<float> adjustedVelocities;
     float distance;
 };
 
 // Scheduled move structure for the move queue
 struct ScheduledMove {
    std::vector<float> targetPosition;   // Target position of the move
    std::vector<float> startPosition;    // Starting position of the move
    std::vector<float> moveVector;       // Movement vector (difference between target and start)
     float totalDistance;                 // Total distance of the move
     float segmentationProgress;          // Progress of segmentation (0.0-1.0)
     int totalSegments;                   // Total number of segments for this move
     MovementType type;                   // Type of movement
     float feedrate;                      // Feedrate in mm/min
 };
 
 // Override states
 struct Override {
     uint8_t feedOverride;        // Feed override value in percent
     uint8_t rapidOverride;       // Rapid override value in percent
     uint8_t spindleOverride;     // Spindle override value in percent
 };
 
 #endif // COMMON_TYPES_H