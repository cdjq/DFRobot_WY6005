/*!
 * @file DFRobot_64x8DTOF.cpp
 * @brief DFRobot_64x8DTOF class implementation
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (https://github.com/PLELES)
 * @version V1.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_64x8DTOF
 */
#include "DFRobot_64x8DTOF.h"

DFRobot_64x8DTOF::DFRobot_64x8DTOF(HardwareSerial& serial, uint32_t config, int8_t rxPin, int8_t txPin)
{
  _serial = &serial;
  _config = config;
  _rxPin  = rxPin;
  _txPin  = txPin;
  DBG("64x8DTOF initialized with serial port (by reference), config=0x%lx, rxPin=%d, txPin=%d", config, rxPin, txPin);
  _startPoint  = 0;
  _endPoint    = 0;
  _totalPoints = 0;
}

void DFRobot_64x8DTOF::begin(uint32_t baudRate)
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

void DFRobot_64x8DTOF::clearBuffer(void)
{
  while(_serial->available()) _serial->read();
}

bool DFRobot_64x8DTOF::sendCommand(const String& command)
{
  DBG("Sending command: %s", command.c_str());
  _serial->print(command);
  _serial->print("\n");
  return true;
}

bool DFRobot_64x8DTOF::setStreamControl(bool enable)
{
  String command = "AT+STREAM_CONTROL=" + String(enable ? "1" : "0");
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::setFrameMode(bool continuousMode)
{
  String command = "AT+SPAD_FRAME_MODE=" + String(continuousMode ? "1" : "0");
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::setOutputLineData(uint8_t line, uint8_t startPoint, uint8_t endPoint)
{
  String command = "AT+SPAD_OUTPUT_LINE_DATA=" + String(line) + "," + String(startPoint) + "," + String(endPoint);
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::triggerOneFrame(void)
{
  String command = "AT+SPAD_TRIG_ONE_FRAME=1";
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::saveConfig(void)
{
  String command = "AT+SAVE_CONFIG";
  return sendCommand(command);
}

bool DFRobot_64x8DTOF::configMeasureMode(uint8_t lineNum)
{
  if (!setStreamControl(false)) return false;
  delay(700);

  if(!setOutputLineData(lineNum, 1, 64)) return false;
  _totalPoints = 64;
  DBG("Config: Single Line, Line: %d", lineNum);
  delay(700);

  if (!saveConfig()) {
    DBG("Warning: saveConfig failed");
  }

  delay(700);
  return setStreamControl(true);
}


bool DFRobot_64x8DTOF::configMeasureMode(uint8_t lineNum, uint8_t pointNum)
{
  if (!setStreamControl(false)) return false;
  delay(700);

  if (!setOutputLineData(lineNum, pointNum, pointNum)) return false;
  _totalPoints = 1;
  DBG("Config: Single Point, Line: %d, Point: %d", lineNum, pointNum);
  delay(700);

  if (!saveConfig()) {
    DBG("Warning: saveConfig failed");
  }
  return setStreamControl(true);

}

bool DFRobot_64x8DTOF::configMeasureMode(void)
{
  if (!setStreamControl(false)) return false;
  delay(700);

  if(!setOutputLineData(0, 0, 0)) return false;
  _totalPoints = DTOF64X8_MAX_POINTS;
  DBG("Config: Full Mode");
  delay(700);

  if (!saveConfig()) {
    DBG("Warning: saveConfig failed");
  }
  delay(700);
  return setStreamControl(true);

}

bool DFRobot_64x8DTOF::configFrameMode(eFrameMode_t mode)
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

void DFRobot_64x8DTOF::parsePointData(const uint8_t* pointData, int16_t* x, int16_t* y, int16_t* z, int16_t* i)
{
  *x = (int16_t)((pointData[1] << 8) | pointData[0]);
  *y = (int16_t)((pointData[3] << 8) | pointData[2]);
  *z = (int16_t)((pointData[5] << 8) | pointData[4]);
  *i = (int16_t)((pointData[7] << 8) | pointData[6]);
}

int DFRobot_64x8DTOF::getData(uint32_t timeoutMs)
{
  // Decide how many points to read: prefer configured _totalPoints if set, otherwise use maxPoints
  int       points          = _totalPoints;
  const int headerSize      = DTOF64X8_FRAME_HEADER_SIZE;
  const int pointDataSize   = DTOF64X8_POINT_DATA_SIZE;
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
            parsePointData(pData, &point.xBuf[i], &point.yBuf[i], &point.zBuf[i], &point.iBuf[i]);
          }
          return points;
        }
      } else {
        // State machine for header validation (0x0A, 0x4F, 0x4B, 0x0A)
        switch (frameIndex) {
          case 0:
            if (c == DTOF64X8_SYNC_BYTE_0)
              frameBuffer[frameIndex++] = c;
            break;
          case 1:
            if (c == DTOF64X8_SYNC_BYTE_1) {
              frameBuffer[frameIndex++] = c;
            } else {
              frameIndex = 0;
              if (c == DTOF64X8_SYNC_BYTE_0)
                frameBuffer[frameIndex++] = c;
            }
            break;
          case 2:
            if (c == DTOF64X8_SYNC_BYTE_2) {
              frameBuffer[frameIndex++] = c;
            } else {
              frameIndex = 0;
              if (c == DTOF64X8_SYNC_BYTE_0)
                frameBuffer[frameIndex++] = c;
            }
            break;
          case 3:
            if (c == DTOF64X8_SYNC_BYTE_3) {
              frameBuffer[frameIndex++] = c;
              inSyncMode                = true;
            } else {
              frameIndex = 0;
              if (c == DTOF64X8_SYNC_BYTE_0)
                frameBuffer[frameIndex++] = c;
            }
            break;
        }
      }
    }
  }
  return -1;    // Timeout
}
