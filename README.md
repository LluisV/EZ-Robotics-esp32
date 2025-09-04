# ESP32 CNC Controller

A high-performance, ESP32-based CNC controller with FluidNC-style motion planning, dual-core processing, and real-time G-code execution.

## Features

### Core Capabilities
- **Multi-axis Support**: Configurable for up to 6 axes (X, Y, Z, A, B, C)
- **FluidNC-style Motion Planning**: Advanced trajectory planning with look-ahead and junction deviation
- **Dual-Core Architecture**: 
  - Core 0: Communication, file management, and telemetry
  - Core 1: Real-time motion control and step generation
- **Real-time G-code Processing**: Queue-based command processing with priority levels
- **20MHz Step Generation**: High-frequency stepper control for smooth motion

### Motion Control
- **Advanced Planning**: Acceleration/deceleration profiles with S-curve smoothing
- **AMASS**: Adaptive Multi-Axis Step Smoothing for improved motion quality
- **Junction Deviation**: Smooth cornering without full stops
- **Feed Rate Override**: Dynamic speed adjustment during operation
- **Soft Limits**: Software-enforced travel boundaries

### Communication & Interface
- **Serial Communication**: 115200 baud serial interface for commands and responses
- **Telemetry System**: Real-time position and velocity reporting
- **File Transfer Protocol**: Upload/download G-code files via serial
- **Job Management**: Run, pause, resume, and stop G-code programs

### File System
- **SPIFFS Storage**: On-board file storage for G-code programs
- **File Operations**: List, upload, download, delete G-code files
- **G-code Validation**: Pre-flight checking of G-code files before execution
- **Job Recovery**: Resume capability after power loss

## Hardware Requirements

### Microcontroller
- **ESP32 Development Board** (ESP32-WROOM-32 or equivalent)
- Minimum 4MB flash memory
- Dual-core processor @ 240MHz

### Pin Connections

Default pin assignments (configurable via config.json):

| Motor | Step Pin | Dir Pin | Endstop Pin |
|-------|----------|---------|-------------|
| X     | GPIO 13  | GPIO 12 | GPIO 14     |
| Y     | GPIO 33  | GPIO 32 | GPIO 15     |
| Z     | GPIO 26  | GPIO 25 | GPIO 4      |

### Wiring Requirements
- **Stepper Drivers**: Compatible with step/direction interface (A4988, DRV8825, TMC2209, etc.)
- **Endstops**: Normally open or normally closed switches (configurable)
- **Power Supply**: Appropriate voltage for your stepper motors

## Installation

### PlatformIO Setup

1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
3. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/esp32-cnc-controller.git
   cd esp32-cnc-controller
   ```
4. Open the project in VS Code
5. Build and upload:
   ```bash
   pio run -t upload
   ```

### Configuration

The system uses a JSON configuration file stored in SPIFFS (`/config.json`):

```json
{
  "machine": {
    "machineName": "3-Axis CNC",
    "defaultFeedrate": 1000.0,
    "maxFeedrate": 2000.0,
    "maxAcceleration": 20,
    "junctionDeviation": 0.05,
    "arcTolerance": 0.002,
    "telemetry": {
      "enabled": true,
      "updatePositionFrequency": 10
    }
  },
  "motors": [
    {
      "name": "X",
      "stepPin": 13,
      "dirPin": 12,
      "endstopPin": 14,
      "endstopInverted": false,
      "stepsPerRev": 1600,
      "leadScrewPitch": 5.0,
      "maxSpeed": 2000,
      "homeSpeed": 1000,
      "minPosition": 0.0,
      "maxPosition": 240.0
    }
    // ... additional motor configurations
  ]
}
```

## Usage

### Basic Commands

#### Movement Commands
```gcode
G0 X10 Y20        ; Rapid move to X=10, Y=20
G1 X30 F1000      ; Linear move to X=30 at 1000mm/min
G28               ; Home all axes
G28 X Y           ; Home specific axes
G90               ; Absolute positioning mode
G91               ; Relative positioning mode
G92 X0 Y0         ; Set current position as work zero
```

#### System Commands
```
!M112             ; Emergency stop
?POS              ; Query current position
?STATUS           ; Query system status
?ENDSTOPS         ; Query endstop states
?CONFIG           ; Display configuration
?HELP             ; Show command help
```

#### File Commands
```
@LIST             ; List G-code files
@SEND filename    ; Download file from controller
@RECEIVE filename size ; Upload file to controller
@DELETE filename  ; Delete file
@RUN filename     ; Execute G-code file
@PAUSE            ; Pause current job
@RESUME           ; Resume paused job
@STOP             ; Stop current job
```

### G-code Support

#### Supported G-codes
- **G0**: Rapid positioning
- **G1**: Linear interpolation
- **G4**: Dwell
- **G21**: Set units to millimeters
- **G28**: Home axes
- **G90**: Absolute positioning
- **G91**: Incremental positioning
- **G92**: Set position

#### Supported M-codes
- **M0/M1**: Program stop
- **M2/M30**: Program end
- **M112**: Emergency stop

### Serial Communication Protocol

Connect via serial terminal at 115200 baud:

```bash
# Linux/Mac
screen /dev/ttyUSB0 115200

# Windows (using PuTTY or similar)
COM3, 115200, 8N1
```

### Telemetry Format

Real-time position updates (when enabled):
```json
[TELEMETRY]{
  "work":{"X":10.500,"Y":25.300,"Z":5.000},
  "world":{"X":10.500,"Y":25.300,"Z":5.000},
  "velocity":150.000,
  "velocityVector":{"X":106.066,"Y":106.066,"Z":0.000}
}
```

## Architecture

### Module Overview

```
┌─────────────────────────────────────────────┐
│            Application Layer                 │
├─────────────────────────────────────────────┤
│  CommunicationManager │ JobManager           │
│  CommandProcessor     │ GCodeValidator       │
├─────────────────────────────────────────────┤
│           Motion Control Layer               │
├─────────────────────────────────────────────┤
│  MachineController   │ GCodeParser           │
│  Scheduler           │ CommandQueue          │
├─────────────────────────────────────────────┤
│          Motion Planning Layer               │
├─────────────────────────────────────────────┤
│  Planner            │ Stepper                │
│  MotionSystem       │ Stepping               │
├─────────────────────────────────────────────┤
│           Hardware Layer                     │
├─────────────────────────────────────────────┤
│  MotorManager       │ Motor                  │
│  ConfigManager      │ FileManager            │
└─────────────────────────────────────────────┘
```

### Core Components

#### Motion Planning
- **Planner**: Velocity planning and trajectory generation
- **Stepper**: Segment buffer management and timing
- **Stepping**: Hardware-level step pulse generation
- **Scheduler**: High-level motion coordination

#### Control System
- **MachineController**: Coordinate system management and motion commands
- **MotorManager**: Individual motor control and homing
- **GCodeParser**: G-code interpretation and execution

#### Communication
- **CommunicationManager**: Serial I/O and protocol handling
- **CommandQueue**: Priority-based command buffering
- **CommandProcessor**: Special command handling

#### File Management
- **FileManager**: SPIFFS file operations
- **JobManager**: G-code program execution
- **GCodeValidator**: Syntax checking and validation

### Task Distribution

| Task | Core | Priority | Function |
|------|------|----------|----------|
| Motion Task | 1 | 10 | Real-time motion control |
| Communication Task | 0 | 1 | Serial I/O and file operations |
| Diagnostic Task | 0 | 1 | System monitoring and logging |

## Debugging

### Enable Debug Output

Debug levels can be set in `main.cpp`:
```cpp
#define DEBUG_ENABLED true
#define DEBUG_LEVEL DEBUG_VERBOSE  // ERROR, WARNING, INFO, or VERBOSE
```

### Debug Commands
```
?DEBUG ON         ; Enable debug output
?DEBUG OFF        ; Disable debug output
?DEBUG LEVEL 3    ; Set debug level (0-3)
?DEBUG DIAG       ; Print system diagnostics
```

### Common Issues

#### Motors Not Moving
1. Check step/direction pin connections
2. Verify motor driver enable pins
3. Check power supply to drivers
4. Verify steps/mm configuration

#### Endstops Not Working
1. Check endstop wiring (NO vs NC)
2. Verify `endstopInverted` setting
3. Use `?ENDSTOPS` to monitor state

#### File Transfer Issues
1. Ensure correct file size in `@RECEIVE` command
2. Check available SPIFFS space
3. Verify serial flow control settings

## Safety Features

- **Emergency Stop**: Hardware and software E-stop capability
- **Soft Limits**: Configurable travel boundaries
- **Endstop Protection**: Automatic stop on endstop trigger
- **Watchdog Timer**: System reset on lock-up
- **Command Validation**: G-code syntax checking

## License

This project is licensed under the MIT License - see LICENSE file for details.
