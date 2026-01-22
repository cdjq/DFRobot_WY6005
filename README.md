# DFRobot_WY6005

## Overview
The DFRobot_WY6005 is a high-precision 3D ToF (Time of Flight) sensor module that provides accurate distance measurements in a compact form factor. It features serial communication interface, multiple operating modes, and high-resolution distance data, making it suitable for applications in robotics, automation, 3D scanning, and proximity sensing.

The sensor includes built-in signal processing capabilities and supports various output modes to meet different application requirements. Each WY6005 sensor is factory calibrated and can be easily integrated into your project via the serial interface.

## Product Link
[WY6005 ToF Sensor](https://www.dfrobot.com/)

## Table of Contents
* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
This library provides a way to drive the WY6005 ToF sensor under the Arduino IDE and Raspberry Pi. It allows you to configure the sensor, read distance data, and perform various operations to suit your application needs.

## Installation
### Arduino Library Installation
1. Download the library file from the GitHub repository.
2. Paste it into the Arduino libraries directory.
3. Open the examples folder and run the demo sketch to test the sensor.

### Raspberry Pi Python Library Installation
1. Navigate to the `python/raspberrypi` directory.
2. Install the required dependencies (if any).
3. Run the example script to test the sensor.

## Methods
### Arduino C++ Library
```C++
/**
 * @fn DFRobot_WY6005
 * @brief Constructor, passing in serial port and configuration
 * @param serial Hardware serial port pointer
 * @param config Serial port configuration (e.g., SERIAL_8N1)
 * @param rxPin RX pin number
 * @param txPin TX pin number
 */
DFRobot_WY6005(HardwareSerial &serial, uint32_t config, int8_t rxPin, int8_t txPin);

/**
 * @fn begin
 * @brief Initialize the sensor
 * @param baudRate Serial communication baud rate
 */
void begin(uint32_t baudRate);

/**
 * @fn triggerGetRaw
 * @brief Trigger one frame and read raw x/y/z values (no filtering)
 * @param xBuf Buffer for x values (may be NULL if not needed)
 * @param yBuf Buffer for y values (may be NULL if not needed)
 * @param zBuf Buffer for z values (may be NULL if not needed)
 * @param iBuf Buffer for i values (may be NULL if not needed)
 * @param maxPoints Maximum points to parse (caller buffer length)
 * @param timeoutMs Timeout in milliseconds to wait for a complete frame
 * @return Number of points parsed, or -1 on error/timeout
 */
int triggerGetRaw(int16_t* xBuf, int16_t* yBuf, int16_t* zBuf, int16_t* iBuf, int maxPoints, uint32_t timeoutMs);

/**
 * @fn triggerOneFrame
 * @brief Trigger one frame data output
 * @return Whether the operation was successful
 * @retval true: Success
 * @retval false: Failure
 */
bool triggerOneFrame(void);

/**
 * @fn saveConfig
 * @brief Save configuration to sensor
 * @return Whether the operation was successful
 * @retval true: Success
 * @retval false: Failure
 */
bool saveConfig(void);

/**
 * @fn configSinglePointMode
 * @brief Configure single point mode
 * @param line Line number
 * @param point Point number
 * @return Whether the configuration was successful
 * @retval true: Success
 * @retval false: Failure
 */
bool configSinglePointMode(uint8_t line, uint8_t point);

/**
 * @fn configSingleLineMode
 * @brief Configure single line mode
 * @param line Line number
 * @param startPoint Start point number
 * @param endPoint End point number
 * @return Whether the configuration was successful
 * @retval true: Success
 * @retval false: Failure
 */
bool configSingleLineMode(uint8_t line, uint8_t startPoint, uint8_t endPoint);

/**
 * @fn configFullOutputMode
 * @brief Configure full output mode
 * @return Whether the configuration was successful
 * @retval true: Success
 * @retval false: Failure
 */
bool configFullOutputMode(void);

/**
 * @fn configSingleFrameMode
 * @brief Configure single frame mode
 * @return Whether the configuration was successful
 * @retval true: Success
 * @retval false: Failure
 */
bool configSingleFrameMode(void);

/**
 * @fn configContinuousMode
 * @brief Configure continuous mode
 * @return Whether the configuration was successful
 * @retval true: Success
 * @retval false: Failure
 */
bool configContinuousMode(void);
```

### Raspberry Pi Python Library
```python
from DFRobot_WY6005 import DFRobot_WY6005
import serial
import sys
import time

# Select the correct serial port:
# - For USB-to-serial: '/dev/ttyUSB0'
# - For Raspberry Pi GPIO (TX/RX): '/dev/serial0' (recommended)
try:
    wy6005 = DFRobot_WY6005(port="/dev/serial0", baudrate=921600)
except serial.SerialException as e:
    print(f"Error: Could not open serial port: {e}")
    sys.exit(1)

# Demo mode selection
# 0: Full Output (8x64 points)
# 1: Single Line (Line 4, points 1-64)
# 2: Single Point (Line 4, Point 32)
demo_mode = 0

def setup():
    print("WY6005 init...")
    while not wy6005.config_single_frame_mode():
        print("failed, Connection error or device busy!")
        time.sleep(1)
    print("successed")
    if demo_mode == 0:
        wy6005.config_full_output_mode()
    elif demo_mode == 1:
        wy6005.config_single_line_mode(4, 1, 64)
    elif demo_mode == 2:
        wy6005.config_single_point_mode(4, 32)

def loop():
    list_x, list_y, list_z, list_i = wy6005.trigger_get_raw(timeout_ms=1000)
    if len(list_x) > 0:
        print(f"Received {len(list_x)} points")
        for idx, x_val in enumerate(list_x):
            print(f"Point[{idx}]: X:{x_val} Y:{list_y[idx]} Z:{list_z[idx]} I:{list_i[idx]}")
    else:
        print("No data received or timeout")
    time.sleep(2)

if __name__ == "__main__":
    setup()
    try:
        while True:
            loop()
    except KeyboardInterrupt:
        wy6005.close()
        print("Program stopped")
```

## Compatibility
| Platform | Work Well | Work Wrong | Untested | Remarks |
|----------|-----------|------------|----------|---------|
| Arduino UNO |  |√ | |  |
| Arduino MEGA2560 | √ | | | Use HardwareSerial |
| ESP32 | √ | | | Use Serial1 |
| ESP8266 |  | | |  |
| Raspberry Pi 3/4 | √ | | | Use UART |
| STM32 | | |  | |
| Micro:bit | | |  | |

## History
- Date: 2026-1-21
- Version: V1.0.0
- Initial release of the DFRobot_WY6005 library

## Credits
Written by fary (feng.yang@dfrobot.com), 2025.04.10
[DFRobot Website](https://www.dfrobot.com/)
