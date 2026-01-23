/*!
 * @file getSingleLine.ino
 * @brief Example of getting single line data
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (https://github.com/PLELES)
 * @version V1.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_WY6005
 */
#include "DFRobot_WY6005.h"
DFRobot_WY6005 wy6005(Serial1, SERIAL_8N1, 25, 26);
#define LINE_NUM    4

void setup()
{
  Serial.begin(115200);
  wy6005.begin(921600);
  // Configure single frame mode using new API
  if (!wy6005.configFrameMode(DFRobot_WY6005::eFrameSingle)) {
    Serial.println("configFrameMode failed");
  }
  Serial.println("Configuring Single Line Mode...");
  if (!wy6005.configMeasureMode(DFRobot_WY6005::eMeasureModeSingleLine, LINE_NUM)) {
    Serial.println("Configuration failed!");
  } else {
    Serial.println("Configuration successful.");
    Serial.print("Mode: Single Line (Line: ");
    Serial.print(LINE_NUM);
    Serial.println(")");
  }
  wy6005.clearBuffer();
}

void loop()
{
  int parsed = wy6005.getPointData(wy6005.point.xBuf, wy6005.point.yBuf, wy6005.point.zBuf, wy6005.point.iBuf, 300);
  Serial.println(parsed);
  if (parsed > 0) {
    Serial.print("Parsed ");
    Serial.print(parsed);
    Serial.println(" points:");
    for (int i = 0; i < parsed; i++) {
      char buf[80];
      sprintf(buf, "[%02d] x:%04dmm y:%04dmm z:%04dmm i:%04d", i, wy6005.point.xBuf[i], wy6005.point.yBuf[i], wy6005.point.zBuf[i], wy6005.point.iBuf[i]);
      Serial.println(buf);
    }
  } else if (parsed == 0) {
    Serial.println("No points parsed");
  } else {
    Serial.println("Error or timeout while reading frame");
  }
}
