/*!
 * @file DFRobot_WY6005.h
 * @brief DFRobot_WY6005 class infrastructure
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (https://github.com/PLELES)
 * @version V1.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_WY6005
 */
#ifndef _DFROBOT_WY6005_H_
#define _DFROBOT_WY6005_H_

#include <Arduino.h>
#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...)                \
  {                             \
    Serial.print("[");          \
    Serial.print(__FUNCTION__); \
    Serial.print(":");          \
    Serial.print(__LINE__);     \
    Serial.print(" ] ");        \
    char _buf[128];             \
    sprintf(_buf, __VA_ARGS__); \
    Serial.println(_buf);       \
  }
#else
#define DBG(...)
#endif

#define WY6005_RESPONSE_TIMEOUT    500 /*!< Response timeout in milliseconds */
#define WY6005_MAX_RETRY_COUNT     5   /*!< Maximum number of retries for commands */
#define WY6005_RESPONSE_OK_SEQ_LEN 4   /*!< Length of OK response sequence */
#define WY6005_SYNC_BYTE_0         0x0A
#define WY6005_SYNC_BYTE_1         0x4F
#define WY6005_SYNC_BYTE_2         0x4B
#define WY6005_SYNC_BYTE_3         0x0A
#define WY6005_FRAME_HEADER_SIZE   4
#define WY6005_POINT_DATA_SIZE     8
#define WY6005_MAX_POINTS          (64 * 8)

class DFRobot_WY6005 {
private:
  HardwareSerial* _serial; /*!< Hardware serial port */
  uint32_t        _config; /*!< Serial port configuration */
  int8_t          _rxPin;  /*!< RX pin number */
  int8_t          _txPin;  /*!< TX pin number */

  /**
   * @fn sendCommand
   * @brief Send AT command and wait for response
   * @param command AT command to send
   * @return Whether the command was sent successfully
   * @retval true: Success
   * @retval false: Failure
   */
  bool sendCommand(const String& command);

  /**
   * @fn setStreamControl
   * @brief Set stream control
   * @param enable Enable stream control
   * @return Whether the setting was successful
   * @retval true: Success
   * @retval false: Failure
   */
  bool setStreamControl(bool enable);

  /**
   * @fn setFrameMode
   * @brief Set frame mode
   * @param continuousMode Continuous mode flag
   * @return Whether the setting was successful
   * @retval true: Success
   * @retval false: Failure
   */
  bool setFrameMode(bool continuousMode);

  /**
   * @fn setOutputLineData
   * @brief Set output line data
   * @param line Line number
   * @param startPoint Start point number
   * @param endPoint End point number
   * @return Whether the setting was successful
   * @retval true: Success
   * @retval false: Failure
   */
  bool setOutputLineData(uint8_t line, uint8_t startPoint, uint8_t endPoint);

  int _startPoint;  /*!< Start point */
  int _endPoint;    /*!< End point */
  int _totalPoints; /*!< Total points */

public:
  /**
   * @enum FrameMode_t
   * @brief Frame mode enumeration: single frame / continuous frame
   */
  typedef enum {
    eFrameSingle     = 0, /*!< Single frame mode */
    eFrameContinuous = 1  /*!< Continuous frame mode */
  } eFrameMode_t;

  /**
   * @enum eMeasureMode_t
   * @brief Measurement output mode: single point / single line / full output
   */
   typedef enum {
    eMeasureModeSinglePoint = 0, /*!< Single point output (provide line and point) */
    eMeasureModeSingleLine  = 1, /*!< Single line output (provide line only) */
    eMeasureModeFull        = 2  /*!< Full output (all points) */
  } eMeasureMode_t;

  /**
   * @struct sPoint_t
   * @brief WY6005 sensor point data structure
   */
  typedef struct {
    int16_t xBuf[WY6005_MAX_POINTS]; /*!< X-axis coordinate buffer */
    int16_t yBuf[WY6005_MAX_POINTS]; /*!< Y-axis coordinate buffer */
    int16_t zBuf[WY6005_MAX_POINTS]; /*!< Z-axis coordinate buffer (Distance) */
    int16_t iBuf[WY6005_MAX_POINTS]; /*!< Intensity buffer */
  } sPoint_t;

  /**
   * @fn DFRobot_WY6005
   * @brief Constructor, passing in serial port and configuration
   * @param serial Hardware serial port pointer
   * @param config Serial port configuration (e.g., SERIAL_8N1)
   * @param rxPin RX pin number
   * @param txPin TX pin number
   */
  DFRobot_WY6005(HardwareSerial& serial, uint32_t config, int8_t rxPin, int8_t txPin);

  /**
   * @fn begin
   * @brief Initialize the sensor
   * @param baudRate Serial communication baud rate
   */
  void begin(uint32_t baudRate);

  /**
   * @fn clearBuffer
   * @brief Clear serial receive buffer
   */
  void clearBuffer(void);

  /**
   * @fn parsePointData
   * @brief Parse point data
   * @param pointData Point data to parse
   * @param x X-axis coordinate
   * @param y Y-axis coordinate
   * @param z Z-axis coordinate (Distance)
   * @param i Intensity
   */
  void parsePointData(const uint8_t* pointData, int16_t* x, int16_t* y, int16_t* z, int16_t* i);

  /**
   * @fn getPointData
   * @brief Trigger one frame and read raw x/y/z values (no filtering)
   * @param xBuf Buffer for x values 
   * @param yBuf Buffer for y values 
   * @param zBuf Buffer for z values 
   * @param iBuf Buffer for i values 
   * @param maxPoints Maximum points to parse (caller buffer length)
   * @param timeoutMs Timeout in milliseconds to wait for a complete frame
   * @return Number of points parsed, or -1 on error/timeout
   */
  int getPointData(int16_t* xBuf, int16_t* yBuf, int16_t* zBuf, int16_t* iBuf,uint32_t timeoutMs);

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
   * @fn configMeasureMode
   * @brief Configure measurement output mode
   * @param mode Measurement mode (eMeasureMode_t)
   * @param arg1 For single-point: line (1..8). For single-line: line (1..8).
   * @param arg2 For single-point: point index (0..64). Ignored for other modes.
   * @return bool type, indicates the configuration status
   * @retval true Configuration successful
   * @retval false Configuration failed
   */
  bool configMeasureMode(eMeasureMode_t mode, uint8_t arg1 = 0, uint8_t arg2 = 0);

  /**
   * @fn configFrameMode
   * @brief Configure frame mode (single frame or continuous)
  * @param mode Frame mode (eFrameSingle or eFrameContinuous)
  * @return bool type, indicates the configuration status
  * @retval true Configuration successful
  * @retval false Configuration failed
  */
  bool configFrameMode(eFrameMode_t mode);

  sPoint_t point; /*!< Point data */
};

#endif
