/*!
 * @file DFRobot_WY6005.cpp
 * @brief DFRobot_WY6005 class implementation
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (https://github.com/PLELES)
 * @version V1.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_WY6005
 */
#include "DFRobot_WY6005.h"

DFRobot_WY6005::DFRobot_WY6005(HardwareSerial& serial, uint32_t config, int8_t rxPin, int8_t txPin)
{
  _serial = &serial;
  _config = config;
  _rxPin  = rxPin;
  _txPin  = txPin;
  DBG("WY6005 initialized with serial port (by reference), config=0x%lx, rxPin=%d, txPin=%d", config, rxPin, txPin);
  _startPoint  = 0;
  _endPoint    = 0;
  _totalPoints = 0;
}

void DFRobot_WY6005::begin(uint32_t baudRate)
{
  //115200 baud rate is not supported now only support 921600
  if (baudRate == 115200) {
    DBG("115200 baud rate is not supported");
  }

#if defined(ESP32)
  _serial->begin(baudRate, _config, _rxPin, _txPin);
#elif defined(ARDUINO_ARCH_ESP8266)
  // ESP8266 HardwareSerial doesn't support rx/tx pin remap in begin
  DBG("ESP8266 only supports SoftwareSerial");
  _serial->begin(baudRate, _config);
#elif defined(ARDUINO_ARCH_AVR)
  // AVR (UNO, etc.) only supports begin(baud) or begin(baud, config)
#if defined(SERIAL_8N1)
  _serial->begin(baudRate, _config);
#else
  _serial->begin(baudRate);
#endif
#else
// Fallback: try two-arg form first, otherwise single arg.
#if defined(HAVE_HWSERIAL1) || defined(SERIAL_8N1)
  _serial->begin(baudRate, _config);
#else
  _serial->begin(baudRate);
#endif
#endif
  delay(400);
  DBG("Serial port started with baud rate: %lu", baudRate);
}

void DFRobot_WY6005::clearBuffer(void)
{
  while(_serial->available()) _serial->read();
  delay(300);
}

bool DFRobot_WY6005::sendCommand(const String& command)
{
  DBG("Sending command: %s", command.c_str());
  _serial->print(command);
  _serial->print("\n");
  return true;
}

bool DFRobot_WY6005::setStreamControl(bool enable)
{
  String command = "AT+STREAM_CONTROL=" + String(enable ? "1" : "0");
  return sendCommand(command);
}

bool DFRobot_WY6005::setFrameMode(bool continuousMode)
{
  String command = "AT+SPAD_FRAME_MODE=" + String(continuousMode ? "1" : "0");
  return sendCommand(command);
}

bool DFRobot_WY6005::setOutputLineData(uint8_t line, uint8_t startPoint, uint8_t endPoint)
{
  String command = "AT+SPAD_OUTPUT_LINE_DATA=" + String(line) + "," + String(startPoint) + "," + String(endPoint);
  return sendCommand(command);
}

bool DFRobot_WY6005::triggerOneFrame(void)
{
  String command = "AT+SPAD_TRIG_ONE_FRAME=1";
  return sendCommand(command);
}

bool DFRobot_WY6005::saveConfig(void)
{
  String command = "AT+SAVE_CONFIG";
  return sendCommand(command);
}


bool DFRobot_WY6005::configMeasureMode(eMeasureMode_t mode, uint8_t arg1, uint8_t arg2)
{
  if (!setStreamControl(false)) return false;
  delay(700);

  if (mode == eMeasureModeSinglePoint) {
    // arg1: Line, arg2: Point
    DBG("Config: Single Point, Line: %d, Point: %d", arg1, arg2);
    if (!setOutputLineData(arg1, arg2, arg2)) return false;
    _totalPoints = 1;
  } 
  else if (mode == eMeasureModeSingleLine) {
    // arg1: Line. Start: 1, End: 64
    DBG("Config: Single Line, Line: %d", arg1);
    if (!setOutputLineData(arg1, 1, 64)) return false;
    _totalPoints = 64;
  }
  else if (mode == eMeasureModeFull) {
    if(!setOutputLineData(0, 0, 0)) return false;
    _totalPoints = WY6005_MAX_POINTS;
    DBG("Config: Full Mode");
  }

  delay(700);
  if (!saveConfig()) {
    DBG("Warning: saveConfig failed");
  }
  delay(700);
  return setStreamControl(true);
}


bool DFRobot_WY6005::configFrameMode(eFrameMode_t mode)
{
  DBG("Configuring frame mode: %d", mode);

  if (!setStreamControl(false))
    return false;
   delay(700);
  bool frameMode = ((mode == eFrameSingle) ? true : false);

  if (!setFrameMode(frameMode))
    return false;
     delay(700);
  if (!saveConfig()) {
    DBG("Warning: saveConfig failed after configFrameMode");
  }
  delay(700);
  return setStreamControl(true);
}

void DFRobot_WY6005::parsePointData(const uint8_t* pointData, int16_t* x, int16_t* y, int16_t* z, int16_t* i)
{
  *x = (int16_t)((pointData[1] << 8) | pointData[0]);
  *y = (int16_t)((pointData[3] << 8) | pointData[2]);
  *z = (int16_t)((pointData[5] << 8) | pointData[4]);
  *i = (int16_t)((pointData[7] << 8) | pointData[6]);
}

int DFRobot_WY6005::getPointData(int16_t* xBuf, int16_t* yBuf, int16_t* zBuf, int16_t* iBuf, uint32_t timeoutMs)
{
  // Decide how many points to read: prefer configured _totalPoints if set, otherwise use maxPoints
  int       points          = _totalPoints;
  const int headerSize      = WY6005_FRAME_HEADER_SIZE;
  const int pointDataSize   = WY6005_POINT_DATA_SIZE;
  int       expectedDataLen = points * pointDataSize;
  int       totalFrameSize  = headerSize + expectedDataLen;

  uint8_t frameBuffer[600];

  if (totalFrameSize > sizeof(frameBuffer))
    return -2;

  // Clear buffer before sending trigger command to avoid reading previous responses
  while(_serial->available()) _serial->read();

  _serial->print("AT+SPAD_TRIG_ONE_FRAME=1");
  _serial->print("\n");

  uint32_t start      = millis();
  int      frameIndex = 0;
  bool     inSyncMode = false;

  while ((millis() - start) < timeoutMs) {
    if (_serial->available()) {
      uint8_t c = _serial->read();

      if (inSyncMode) {
        frameBuffer[frameIndex++] = c;

        // Check if we have received the full frame
        if (frameIndex == totalFrameSize) {
          // Parse all points
          for (int i = 0; i < points; i++) {
            int      pIdx  = headerSize + i * pointDataSize;
            uint8_t* pData = &frameBuffer[pIdx];
            parsePointData(pData, &xBuf[i], &yBuf[i], &zBuf[i], &iBuf[i]);
          }
          return points;
        }
      } else {
        // State machine for header validation (0x0A, 0x4F, 0x4B, 0x0A)
        switch (frameIndex) {
          case 0:
            if (c == WY6005_SYNC_BYTE_0)
              frameBuffer[frameIndex++] = c;
            break;
          case 1:
            if (c == WY6005_SYNC_BYTE_1) {
              frameBuffer[frameIndex++] = c;
            } else {
              frameIndex = 0;
              if (c == WY6005_SYNC_BYTE_0)
                frameBuffer[frameIndex++] = c;
            }
            break;
          case 2:
            if (c == WY6005_SYNC_BYTE_2) {
              frameBuffer[frameIndex++] = c;
            } else {
              frameIndex = 0;
              if (c == WY6005_SYNC_BYTE_0)
                frameBuffer[frameIndex++] = c;
            }
            break;
          case 3:
            if (c == WY6005_SYNC_BYTE_3) {
              frameBuffer[frameIndex++] = c;
              inSyncMode                = true;
            } else {
              frameIndex = 0;
              if (c == WY6005_SYNC_BYTE_0)
                frameBuffer[frameIndex++] = c;
            }
            break;
        }
      }
    }
  }
  return -1;    // Timeout
}
