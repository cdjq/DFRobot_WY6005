/*!
 * @file getMultiPoint.ino
 * @brief Example of getting multiple points (multi-point mode)
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (https://github.com/PLELES)
 * @version V1.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_64x8DTOF
 */

#include "DFRobot_64x8DTOF.h"

// Configure demo: line and point range
#define DTOF_LINE 4
#define DTOF_START_POINT 1
#define DTOF_END_POINT 10

// Use Serial1 for device communication on boards that support it
DFRobot_64x8DTOF dtof64x8(Serial1, SERIAL_8N1, 25, 26);

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); }
  while(!dtof64x8.begin());

  Serial.println("DFRobot 64x8DTOF Multi-Point Demo");
  // Also demonstrate configuring single-frame mode (retry until success)
  Serial.println("Configuring frame mode: Single Frame (demo)...");
  while (!dtof64x8.configFrameMode(DFRobot_64x8DTOF::eFrameSingle)) {
    Serial.println("configFrameMode failed, retrying...");
    delay(200);
  }
  Serial.println("Frame mode configured: Single Frame");

  // Configure multi-point mode using macros (retry until success)
  Serial.println("Configuring Multi-Point Mode...");
  while (!dtof64x8.configMeasureMode(DTOF_LINE, DTOF_START_POINT, DTOF_END_POINT)) {
    Serial.println("configMeasureMode failed, retrying...");
    delay(200);
  }
  Serial.print("Configured multi-point mode: line=");
  Serial.print(DTOF_LINE);
  Serial.print(" start=");
  Serial.print(DTOF_START_POINT);
  Serial.print(" end=");
  Serial.println(DTOF_END_POINT);
  delay(300);
}

void loop() {
  int cnt = dtof64x8.getData(300);
  if (cnt > 0) {
    Serial.print("Got points: ");
    Serial.println(cnt);
    for (int i = 0; i < cnt; i++) {
      char numbuf[16];
      Serial.print(DTOF_START_POINT + i);
      Serial.print(": X="); sprintf(numbuf, "%04d", dtof64x8.point.xBuf[i]); Serial.print(numbuf); Serial.print(" mm");
      Serial.print(" Y="); sprintf(numbuf, "%04d", dtof64x8.point.yBuf[i]); Serial.print(numbuf); Serial.print(" mm");
      Serial.print(" Z="); sprintf(numbuf, "%04d", dtof64x8.point.zBuf[i]); Serial.print(numbuf); Serial.print(" mm");
      Serial.print(" I="); Serial.println(dtof64x8.point.iBuf[i]);
    }
  } else {
    Serial.println("getData timeout or error");
  }
  delay(500);
}
