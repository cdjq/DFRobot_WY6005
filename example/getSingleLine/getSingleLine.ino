/*!
 * @file getSingleLine.ino
 * @brief Example of getting single line data
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (https://github.com/PLELES)
 * @version V1.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_64x8DTOF
 */
#include "DFRobot_64x8DTOF.h"
DFRobot_64x8DTOF dtof64x8(Serial1, SERIAL_8N1, 25, 26);
#define LINE_NUM    4

void setup()
{
  Serial.begin(115200);
  dtof64x8.begin(921600);
  // Configure single frame mode using new API
  if (!dtof64x8.configFrameMode(DFRobot_64x8DTOF::eFrameSingle)) {
    Serial.println("configFrameMode failed");
  }
  Serial.println("Configuring Single Line Mode...");
  if (!dtof64x8.configMeasureMode(LINE_NUM)) {
    Serial.println("Configuration failed!");
  } else {
    Serial.println("Configuration successful.");
    Serial.print("Mode: Single Line (Line: ");
    Serial.print(LINE_NUM);
    Serial.println(")");
  }
  delay(300);
}

void loop()
{
  int parsed = dtof64x8.getData(300);
  
  if (parsed > 0) {
    Serial.print("Parsed ");
    Serial.print(parsed);
    Serial.println(" points:");
    for (int i = 0; i < parsed; i++) {
      char buf[80];
      sprintf(buf, "[%02d] x:%04dmm y:%04dmm z:%04dmm i:%04d", i, dtof64x8.point.xBuf[i], dtof64x8.point.yBuf[i], dtof64x8.point.zBuf[i], dtof64x8.point.iBuf[i]);
      Serial.println(buf);
    }
  } else if (parsed == 0) {
    Serial.println(parsed);
    Serial.println("No points parsed");
  } else {
    Serial.println(parsed);
    Serial.println("Error or timeout while reading frame");
  }
  delay(500);
}
