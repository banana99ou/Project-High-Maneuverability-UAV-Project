# High Maneuverability UAV Project

ESP32-based quadcopter flight controller using an MPU-6050 IMU with DMP (Digital Motion Processor) for attitude estimation, PID control for stabilisation, and iBus receiver input from an RC transmitter.

## Hardware

| Component | Role |
|---|---|
| ESP32 | Flight controller MCU |
| MPU-6050 | 6-axis IMU with DMP for yaw/pitch/roll |
| iBus RC receiver (6-ch) | Pilot input (roll, pitch, yaw, throttle) via UART2 |
| Brushless ESCs (x4) | Motor speed control (PWM via ESP32Servo) |
| Brushless motors (x4) | Quadcopter propulsion |

## How It Works

1. **IMU**: MPU-6050 DMP outputs quaternion → yaw/pitch/roll at high rate via I2C interrupt.
2. **RC Input**: iBus protocol parsed on UART2 — 6 channels decoded per frame.
3. **PID**: Per-axis PID loops compute motor corrections from the error between stick input and measured attitude.
4. **Mixer**: Roll/pitch/yaw corrections are mixed into 4 motor outputs with throttle as baseline.
5. **Failsafe**: Transmitter kill switch immediately zeros all motors.

## Directory Structure

| Directory | Description |
|---|---|
| `main/` | Integrated flight controller firmware — IMU + iBus + PID + motor mixer |
| `Motor_Test/` | ESC/motor test sketch (PWM range discovery, calibration sequence) |
| `Reciver_test/` | Standalone iBus receiver test |
| `Ref/` | Wiring diagrams, pinout screenshots, ESP32 pin reference |

## Branches

| Branch | Description |
|---|---|
| `master` | Main flight controller code (ESP32) |
| `Arduino-Nano` | Earlier port targeting Arduino Nano |

## Related

The `mpuproject` repo (MPU-6050 sensor fusion experiments) was the precursor R&D that fed into this flight controller. It will be consolidated here as a branch in the future.
