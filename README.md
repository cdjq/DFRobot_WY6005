# DFRobot_64x8DTOF
- [中文版](./README_CN.md)

## Overview
The DFRobot_64x8DTOF is a high-precision DToF (Time of Flight) sensor module that provides accurate distance measurements in a compact form factor. It features serial communication interface, multiple operating modes, and high-resolution distance data, making it suitable for applications in robotics, automation,proximity sensing.

The sensor includes built-in signal processing capabilities and supports various output modes to meet different application requirements. Each 64x8DTOF sensor is factory calibrated and can be easily integrated into your project via the serial interface.

## Product Link
[64x8DTOF DToF Sensor](https://www.dfrobot.com/)

```text
SKU: 64x8DTOF
```

## Table of Contents
* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
This library provides a way to drive the 64x8DTOF ToF sensor under the Arduino IDE. It allows you to configure the sensor, read distance data, and perform various operations to suit your application needs.

## Installation
### Arduino Library Installation
1. Download the library file from the GitHub repository.
2. Paste it into the Arduino libraries directory.
3. Open the examples folder and run the demo sketch to test the sensor.

## Methods
### Arduino C++ Library
```C++
/**
 * @fn DFRobot_64x8DTOF
 * @brief Constructor, passing in serial port and configuration
 * @param serial Hardware serial port reference
 * @param config Serial port configuration (e.g., SERIAL_8N1)
 * @param rxPin RX pin number (optional for platforms that support remapping)
 * @param txPin TX pin number (optional for platforms that support remapping)
 */
DFRobot_64x8DTOF(HardwareSerial &serial, uint32_t config, int8_t rxPin, int8_t txPin);

/**
 * @fn begin
 * @brief Initialize the sensor serial port
 * @param baudRate Serial communication baud rate
 */
void begin(uint32_t baudRate);

/**
 * @fn getData
 * @brief Trigger one frame and read raw x/y/z values (no filtering)
 * @param timeoutMs Timeout in milliseconds to wait for a complete frame
 * @return Number of points parsed, or -1 on error/timeout
 */
int getData(uint32_t timeoutMs = 500);

/**
 * @fn configMeasureMode
 * @brief Configure measurement output mode — Full output (all points).
 * @return bool True if configuration succeeded and stream control restored,
 *              false on communication error or device rejection.
 */
bool configMeasureMode(void);

/**
 * @fn configMeasureMode
 * @brief Configure measurement output mode — Single line.
 * @param lineNum Line index to output (1..8).
 * @return bool True if configuration succeeded and stream control restored,
 *              false on communication error or invalid arguments.
 */
bool configMeasureMode(uint8_t lineNum);

/**
 * @fn configMeasureMode
 * @brief Configure measurement output mode — Single point.
 * @param lineNum Line index containing the point (1..8).
 * @param pointNum Point index within the line (0..64).
 * @return bool True if configuration succeeded and stream control restored,
 *              false on communication error or invalid arguments.
 */
bool configMeasureMode(uint8_t lineNum, uint8_t pointNum);

/**
 * @fn configFrameMode
 * @brief Configure whether sensor runs in single-frame or continuous frame mode
 * @param mode Frame mode (eFrameSingle or eFrameContinuous)
 * @return bool type, indicates the configuration status
 * @retval true Configuration successful
 * @retval false Configuration failed
 */
bool configFrameMode(eFrameMode_t mode);
```

<!-- Raspberry Pi Python usage removed as requested -->

## Compatibility
| Platform | Work Well | Work Wrong | Untested | Remarks |
|----------|-----------|------------|----------|---------|
| Arduino UNO |  |√| | |
| Arduino MEGA2560 | √ | | | |
| ESP32 | √ | | | Use Serial1 |

## History
- Date: 2026-1-26
- Version: V1.0.0

## Credits
Written by PLELES (PLELES@dfrobot.com), 2026.01.26
[DFRobot Website](https://www.dfrobot.com/)
