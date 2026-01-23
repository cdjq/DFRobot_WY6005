# DFRobot_WY6005
- [中文版](./README_CN.md)

## Overview
The DFRobot_WY6005 is a high-precision 3D ToF (Time of Flight) sensor module that provides accurate distance measurements in a compact form factor. It features serial communication interface, multiple operating modes, and high-resolution distance data, making it suitable for applications in robotics, automation, 3D scanning, and proximity sensing.

The sensor includes built-in signal processing capabilities and supports various output modes to meet different application requirements. Each WY6005 sensor is factory calibrated and can be easily integrated into your project via the serial interface.

## Product Link
[WY6005 ToF Sensor](https://www.dfrobot.com/)

```text
SKU: WY6005
```

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

## Methods
### Arduino C++ Library
```C++
/**
 * @fn DFRobot_WY6005
 * @brief Constructor, passing in serial port and configuration
 * @param serial Hardware serial port reference
 * @param config Serial port configuration (e.g., SERIAL_8N1)
 * @param rxPin RX pin number (optional for platforms that support remapping)
 * @param txPin TX pin number (optional for platforms that support remapping)
 */
DFRobot_WY6005(HardwareSerial &serial, uint32_t config, int8_t rxPin, int8_t txPin);

/**
 * @fn begin
 * @brief Initialize the sensor serial port
 * @param baudRate Serial communication baud rate
 */
void begin(uint32_t baudRate);

/**
 * @fn clearBuffer
 * @brief Clear the serial receive buffer
 */
void clearBuffer(void);

/**
 * @fn getPointData
 * @brief Trigger one frame and read raw x/y/z values (no filtering)
 * @param xBuf Buffer for x values
 * @param yBuf Buffer for y values
 * @param zBuf Buffer for z values
 * @param iBuf Buffer for intensity values
 * @param timeoutMs Timeout in milliseconds to wait for a complete frame
 * @return Number of points parsed, or -1 on error/timeout
 */
int getPointData(int16_t* xBuf, int16_t* yBuf, int16_t* zBuf, int16_t* iBuf, uint32_t timeoutMs);

/**
 * @fn triggerOneFrame
 * @brief Trigger one frame data output (sends AT+SPAD_TRIG_ONE_FRAME=1)
 * @return Whether the operation was successful
 */
bool triggerOneFrame(void);

/**
 * @fn saveConfig
 * @brief Save pending configuration into sensor
 * @return Whether the operation was successful
 */
bool saveConfig(void);

/**
 * @fn configSinglePointMode
 * @brief Configure single point output mode
 * @param line Line number (1..8)
 * @param point Point index (0..64)
 * @return bool type, indicates the configuration status
 * @retval true Configuration successful
 * @retval false Configuration failed
 */
bool configSinglePointMode(uint8_t line, uint8_t point);

/**
 * @fn configSingleLineMode
 * @brief Configure single line output mode (outputs a full row)
 * @param line Line number (1..8)
 * @param startPoint Start point index (0..64)
 * @param endPoint End point index (0..64)
 * @return bool type, indicates the configuration status
 * @retval true Configuration successful
 * @retval false Configuration failed
 */
bool configSingleLineMode(uint8_t line, uint8_t startPoint, uint8_t endPoint);

/**
 * @fn configFrameMode
 * @brief Configure whether sensor runs in single-frame or continuous frame mode
 * @param mode Frame mode (eFrameSingle or eFrameContinuous)
 * @return bool type, indicates the configuration status
 * @retval true Configuration successful
 * @retval false Configuration failed
 */
bool configFrameMode(eFrameMode_t mode);

/**
 * @fn configMeasureMode
 * @brief Configure measurement output mode (single-point / single-line / full)
 * @param mode Measurement mode (eMeasureMode_t)
 * @param arg1 For single-point: line (1..8). For single-line: line (1..8). For full: ignored.
 * @param arg2 For single-point: point (0..64). Otherwise ignored.
 * @return bool type, indicates the configuration status
 * @retval true Configuration successful
 * @retval false Configuration failed
 */
bool configMeasureMode(eMeasureMode_t mode, uint8_t arg1 = 0, uint8_t arg2 = 0);

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
- Date: 2026-1-21
- Version: V1.0.0

## Credits
Written by PLELES (PLELES@dfrobot.com), 2025.04.10
[DFRobot Website](https://www.dfrobot.com/)
