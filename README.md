# High-Speed Line Follower Robot (ESP32 + Arduino)

A high-performance line-following robot powered by an ESP32 microcontroller and built with the Arduino framework. This project implements an object-oriented architecture in C++ with PID control to achieve fast, stable line tracking. It is designed for hobbyists, researchers, and developers interested in robotics, control systems, and embedded development with ESP32.

## Key Features
- **High-Speed Line Tracking**: Optimized PID loop and sensor processing for rapid response on straightaways and smooth curves.
- **Modular OOP Design**: Encapsulated classes for sensor array, PID controller, and motor driver, making code reusable and extensible.
- **Configurable Sensor Array**: Support for a multi-sensor infrared (IR) array (e.g., 6–16 IR sensors), with easy calibration routines.
- **Flexible Motor Control**: Interface with common motor driver modules (e.g., L298N, TB6612FNG, or similar), configurable PWM parameters.
- **Tuning & Calibration Utilities**: Built-in routines to calibrate sensor thresholds and PID gains, with serial feedback.
- **Optional Telemetry (Wi-Fi)**: (Optional) ESP32-based Wi-Fi telemetry: monitor sensor readings, PID output, and robot status in real-time via a simple web interface or serial-over-Wi-Fi.
- **Expandable Architecture**: Ready for extensions such as obstacle detection, speed monitoring, or integration with other sensors/modules.

## Repository Structure


## Hardware Requirements
- **Microcontroller**: ESP32 development board (e.g., ESP32 DevKitC).
- **Sensors**: Infrared line-tracking sensors (e.g., QRE1113GR modules) in an array (6 to 16 sensors).  
- **Motor Driver**: Compatible dual H-bridge (e.g., L298N, TB6612FNG, or similar), able to drive the chosen DC motors.
- **Motors & Chassis**: High-RPM DC motors with wheels, chassis supporting stable operation at speed.
- **Power Supply**: Sufficient battery or regulated supply for ESP32, motors, and sensors (e.g., LiPo battery pack + voltage regulators).
- **Miscellaneous**: Wires, connectors, mounting hardware, optional Wi-Fi antenna optimization.

> **Note**: Adjust voltage levels and wiring per your components. Ensure motors do not introduce excessive noise on the ESP32’s power lines—use decoupling capacitors or separate supplies if needed.

## Wiring & Pin Assignment
1. **IR Sensors**  
   - Connect each sensor’s VCC and GND to the 3.3V and GND rails.  
   - Sensor outputs to designated ESP32 GPIO pins (e.g., GPIO 32–39 for analog-capable inputs, or digital pins if digital sensors).  
   - Label pins in `SensorArray` constructor or config header.

2. **Motor Driver**  
   - Connect driver inputs (IN1/IN2, ENA, etc.) to ESP32 PWM-capable GPIOs.  
   - Ensure correct logic levels: ESP32 outputs 3.3V.  
   - Motor power supply separate from ESP32 3.3V; common ground required.

3. **Power**  
   - ESP32 powered via regulated 5V/3.3V supply.  
   - Motors powered by battery; include decoupling capacitors near driver.  
   - Ensure grounds are tied together (ESP32 GND to motor battery GND).

4. **Optional Telemetry**  
   - Wi-Fi: Use ESP32’s built-in Wi-Fi. No extra wiring unless external antenna is used.  
   - Serial debugging: Connect ESP32 UART to USB for logs.

*(Include a schematic or Fritzing diagram in `hardware/` for clarity.)*

## Software Setup
1. **Arduino IDE**  
   - Install ESP32 board support in Arduino IDE via Boards Manager.
2. **Dependencies / Libraries**  
   - QTRSensors Arduino library by Pololu.  
   - Optionally: WiFi.h, WebServer.h (for telemetry), or AsyncWebServer for advanced UI.  
   - If using specific sensor libraries, list them here (e.g., for digital IR sensors). Otherwise, code reads analog/digital pins directly.

3. **Clone Repository**  
   ```bash
   git clone https://github.com/Looping-Labs/line-follower
   cd line-follower
