# Motor Control System - FPGA Implementation

## Overview
Sistem kontrol motor berbasis FPGA dengan dukungan kontrol PWM manual dan kontrol kecepatan PID (RPM). Sistem ini menggunakan komunikasi UART untuk konfigurasi dan monitoring real-time, dengan antarmuka GUI Python untuk kemudahan penggunaan.

## Features
- **Dual Motor Control**: Kontrol independen untuk 2 motor
- **Two Control Modes**: 
  - PWM Manual Mode: Kontrol langsung duty cycle PWM
  - PID Speed Mode: Kontrol kecepatan (RPM) dengan feedback encoder
- **Real-time Monitoring**: Monitoring posisi encoder, RPM, dan error PID
- **UART Communication**: Protokol komunikasi robust dengan retry mechanism
- **Python GUI**: Antarmuka grafis untuk konfigurasi dan monitoring
- **Encoder Feedback**: Simple encoder dengan perhitungan RPM berbasis PPR

## System Architecture

### Hardware Components
- **FPGA**: Nexys A7-50T (Artix-7)
- **Motors**: 2 DC motors dengan encoder
- **Communication**: UART (115200 baud)
- **PWM Output**: 2 channel PWM dengan kontrol arah

### Software Components
1. **FPGA Verilog Modules**:
   - `top_motor_control.v`: Top-level module
   - `pid_controller.v`: PID controller implementation
   - `simple_encoder.v`: Simplified encoder decoder
   - `pwm_gen.v`: PWM generator
   - `uart_rx.v` / `uart_tx.v`: UART communication
   - `encoder_data_tx.v`: Encoder data transmission

2. **Python GUI**: `motor_gui.py`
   - Real-time monitoring dan kontrol
   - Parameter tuning untuk PID
   - Mode selection (PWM/PID)

## Control Modes

### 1. PWM Manual Mode
- **Input**: PWM duty cycle (-CCR to +CCR)
- **Output**: Direct PWM signal ke motor
- **Direction**: Otomatis berdasarkan tanda PWM value
- **Use Case**: Open-loop control, testing motor

### 2. PID Speed Mode  
- **Input**: Target RPM (setpoint)
- **Feedback**: Actual RPM dari encoder
- **Controller**: PID dengan parameter tunable (Kp, Ki, Kd)
- **Output**: PWM value hasil perhitungan PID
- **Features**:
  - Anti-windup protection
  - Integral reset ketika setpoint = 0
  - Support negative setpoint (reverse direction)

## UART Protocol

### Command Format
All commands use big-endian byte ordering.

#### Motor PWM Commands (3 bytes)
```
Motor 0: [0xAA][PWM_High][PWM_Low]
Motor 1: [0xBB][PWM_High][PWM_Low]
Response: [0xAA] or [0xBB]
```

#### Configuration Commands (3 bytes)
```
PSC: [0xCC][PSC_High][PSC_Low]          // PWM prescaler
CCR: [0xDD][CCR_High][CCR_Low]          // PWM compare value (max duty)
PPR: [0x11][PPR_High][PPR_Low]          // Pulses per revolution
Response: [0xCC], [0xDD], or [0x11]
```

#### PID Mode Commands (3 bytes)
```
Mode: [0x22][Motor_ID][Mode]            // Mode: 0=PWM, 1=PID
Response: [0x22]
```

#### PID Parameter Commands (4 bytes)
```
Setpoint: [0x33][Motor_ID][High][Low]   // Target RPM (signed)
Kp:       [0x44][Motor_ID][High][Low]   // Proportional gain
Ki:       [0x55][Motor_ID][High][Low]   // Integral gain  
Kd:       [0x66][Motor_ID][High][Low]   // Derivative gain
Response: [0x33], [0x44], [0x55], or [0x66]
```

#### Query Commands
```
Query: [0xEE]                           // Query configuration
Response: [0xEE]
```

### Encoder Data Transmission (Auto)
Encoder data ditransmisikan otomatis setiap ~333ms (3Hz) dengan format:
```
[0xE0][Enc0_Pos_High][Enc0_Pos_Low][Enc1_Pos_High][Enc1_Pos_Low]
[0xE1][RPM0_High][RPM0_Low][RPM1_High][RPM1_Low]
[0xE2][Direction_Flags]
```

## PID Controller Implementation

### Algorithm
```
Error = Setpoint - Feedback_RPM
P_term = Kp × Error
I_term += Ki × Error × dt
D_term = Kd × (Error - Previous_Error) / dt
Output = P_term + I_term + D_term
```

### Key Features
- **Sampling Rate**: 100Hz (10ms period)
- **Scaling**: All gains scaled by 256 (Kp=256 means gain=1.0)
- **Anti-windup**: Integral clamping to prevent overflow
- **Setpoint Zero Reset**: When setpoint=0, integral and output reset to 0
- **Output Clamping**: Output limited to ±CCR value

### Parameter Guidelines
- **Kp (Proportional)**: Start with 256 (gain=1.0), adjust for responsiveness
- **Ki (Integral)**: Start with 64 (gain=0.25), adjust for steady-state error
- **Kd (Derivative)**: Start with 16 (gain=0.0625), adjust for overshoot

## Encoder Implementation

### Type: Simplified Encoder (Not True Quadrature)
- **Resolution**: 1x (only rising edge of channel A)
- **Direction**: Based on channel B state during channel A rising edge
- **Position**: 16-bit signed counter
- **RPM Calculation**: Based on pulse count and PPR

### RPM Calculation Formula
```
RPM = (Pulse_Count × 600) / PPR
```
Where:
- Pulse_Count: Number of pulses in 100ms window
- 600 = 60 seconds/minute × 10 (conversion factor for 100ms window)
- PPR: Pulses Per Revolution (configurable)

### Limitations
- **Lower Resolution**: 4x less resolution than true quadrature
- **Single Edge Detection**: Only detects rising edges of channel A
- **Direction Sensing**: Basic direction detection using channel B

## GUI Application Features

### Connection Management
- COM port selection
- Connection status indicator
- Automatic retry on connection failure

### Motor Control
- **PWM Mode**: Direct PWM value input (-999 to +999)
- **PID Mode**: Setpoint input (-3000 to +3000 RPM)
- Mode selection per motor (independent)

### Parameter Configuration
- PSC, CCR, PPR settings
- PID parameters (Kp, Ki, Kd) per motor
- Input validation and range checking

### Real-time Monitoring
- Current RPM and position display
- PID error visualization
- Color-coded status indicators
- Communication log with timestamps

### Layout
```
┌─ Connection ─────┬─ Configuration ──┐
│ COM Port         │ PSC, CCR, PPR    │
│ Connect/Disc.    │ Query Settings   │
├─ Motor Control ──┼─ PID Control ────┤
│ PWM Motor 0/1    │ Mode Selection   │
│ Send PWM/Stop    │ Setpoint, Kp,Ki,Kd │
├─ Real-time Monitoring ─────────────┤
│ RPM, Position, Setpoint, Error     │
├─ Communication Log ─────────────────┤
│ Command history and responses      │
└─────────────────────────────────────┘
```

## File Structure
```
e:\motor-pid-fpga\
├── motor_gui.py                    # Python GUI application
├── Nexys-A7-50T-Master.xdc        # FPGA constraints file
├── README.md                       # This documentation
└── motor_control/                  # Vivado project
    └── motor_control.srcs/sources_1/new/
        ├── top_motor_control.v     # Top-level module
        ├── pid_controller.v        # PID controller
        ├── simple_encoder.v        # Encoder decoder
        ├── pwm_gen.v              # PWM generator
        ├── uart_rx.v              # UART receiver
        ├── uart_tx.v              # UART transmitter
        └── encoder_data_tx.v      # Encoder data transmission
```

## Getting Started

### Hardware Setup
1. Connect motors to PWM outputs (PWM_OUT[1:0])
2. Connect motor direction pins to MOTOR_IN[3:0]
3. Connect encoder channels A&B to ENC_A[1:0] and ENC_B[1:0]
4. Connect UART to USB-Serial converter (115200 baud)

### Software Setup
1. **FPGA Programming**:
   - Open `motor_control.xpr` in Vivado
   - Generate bitstream
   - Program FPGA

2. **Python GUI**:
   ```bash
   cd e:\motor-pid-fpga
   python motor_gui.py
   ```

### Basic Operation
1. **Connection**: Select COM port and connect
2. **Configuration**: Set PSC, CCR, PPR values
3. **Motor Test**: 
   - Start with PWM mode
   - Test motor response with small PWM values
4. **PID Tuning**:
   - Switch to PID mode
   - Start with default parameters
   - Adjust Kp for responsiveness
   - Add Ki for steady-state accuracy
   - Add Kd if oscillation occurs

## Performance Characteristics

### UART Communication
- **Baud Rate**: 115200 bps
- **Command Response Time**: <150μs
- **Encoder Data Rate**: 3Hz (every ~333ms)
- **Retry Mechanism**: 3 attempts for failed commands

### PID Control
- **Update Rate**: 100Hz (10ms period)
- **Settling Time**: Depends on tuning (typically 100-500ms)
- **Steady-State Error**: <1 RPM with proper Ki tuning
- **Max RPM**: Limited by encoder PPR and motor characteristics

### System Latency
- **Command to Response**: <1ms
- **Setpoint to Motor Response**: ~10ms (PID update rate)
- **Encoder to GUI Update**: ~333ms (encoder transmission rate)

## Troubleshooting

### Common Issues

#### "Wrong ACK" Errors
- **Cause**: UART timing conflicts
- **Solution**: Already implemented retry mechanism and command prioritization

#### PID Oscillation
- **Cause**: Kd too high or Kp too aggressive
- **Solution**: Reduce Kd first, then reduce Kp

#### Motor Not Responding to Setpoint=0
- **Cause**: Integral windup
- **Solution**: Implemented automatic integral reset when setpoint=0

#### Encoder Direction Wrong
- **Cause**: Encoder wiring or direction logic
- **Solution**: Check wiring, direction logic already corrected in code

#### Negative RPM Issues
- **Cause**: Signed/unsigned data handling
- **Solution**: All setpoint handling uses signed 16-bit values

### Debug Features
- **LED Indicators**: Show motor directions on board LEDs
- **Communication Log**: All UART traffic logged in GUI
- **Real-time Monitoring**: Live display of all system parameters

## Future Improvements

### Potential Enhancements
1. **True Quadrature Encoder**: 4x resolution improvement
2. **Velocity Feedforward**: Better tracking of changing setpoints
3. **Current Sensing**: Motor current monitoring and protection
4. **Position Control**: Closed-loop position control mode
5. **Advanced GUI**: Plotting, data logging, parameter auto-tuning

### Known Limitations
1. **Encoder Resolution**: Only 1x quadrature resolution
2. **Fixed Sample Rate**: PID runs at fixed 100Hz
3. **Single Motor Driver**: Only supports simple PWM+direction motors
4. **No Safety Features**: No overcurrent or overspeed protection

## Technical Specifications

### FPGA Resources (Artix-7)
- **Clock**: 100MHz system clock
- **LUTs**: ~500 (estimated)
- **Block RAM**: Minimal usage
- **DSP Slices**: Used for multiplication in PID

### Timing Constraints
- **System Clock**: 100MHz (10ns period)
- **UART Clock**: Derived from system clock
- **Setup/Hold**: Met for all critical paths

### Parameter Ranges
- **PWM Value**: -999 to +999 (signed 16-bit)
- **RPM Setpoint**: -3000 to +3000 RPM
- **PID Gains**: 0 to 65535 (scaled by 256)
- **PSC**: 0 to 65535
- **CCR**: 0 to 65535  
- **PPR**: 1 to 65535

---
**Last Updated**: December 2024  
**Version**: 1.0  
**Author**: Motor Control System Team
