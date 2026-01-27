/*!
 * @file getSinglePoint.ino
 * @brief Example of getting single point data
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (https://github.com/PLELES)
 * @version V1.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_64x8DTOF
 */
#include "DFRobot_64x8DTOF.h"

#define LINE_NUM  4
#define POINT_NUM 2
// Instantiate the sensor object
// Use Serial1 for communication, change pins 25/26 to your actual RX/TX pins
DFRobot_64x8DTOF dtof64x8(Serial1, SERIAL_8N1, 25, 26);

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  while (!dtof64x8.begin());
  Serial.println("WY6005 Single Point Demo");

  // Retry configuring frame mode until success
  Serial.println("Configuring frame mode: Single Frame...");
  while (!dtof64x8.configFrameMode(DFRobot_64x8DTOF::eFrameSingle)) {
    Serial.println("Error: configFrameMode failed, retrying...");
    delay(200);
  }
  Serial.println("Configuration Successful!");
  // 2. Configure Single Point Mode (retry until success)
  // Example: Line 4, Point 32 (Center of the sensor roughly)
  // Line range: 1-8
  // Point range: 1-64
  Serial.println("Configuring Single Point Mode (Line 4, Point 32)...");
  while (!dtof64x8.configMeasureMode(LINE_NUM, POINT_NUM)) {
    Serial.println("Configuration Failed, retrying...");
    delay(200);
  }
  Serial.println("Configuration Successful!");
  delay(300);
}

void loop()
{
  int parsed = dtof64x8.getData(300);

  if (parsed > 0) {
    char numbuf[16];
    Serial.print("Point Data -> ");
    Serial.print("X: ");
    sprintf(numbuf, "%04d", dtof64x8.point.xBuf[0]);
    Serial.print(numbuf);
    Serial.print(" mm, ");
    Serial.print("Y: ");
    sprintf(numbuf, "%04d", dtof64x8.point.yBuf[0]);
    Serial.print(numbuf);
    Serial.print(" mm, ");
    Serial.print("Z: ");
    sprintf(numbuf, "%04d", dtof64x8.point.zBuf[0]);
    Serial.print(numbuf);
    Serial.print(" mm, ");
    Serial.print("I: ");
    Serial.println(dtof64x8.point.iBuf[0]);
  } else if (parsed == -1) {
    Serial.println("Read timeout");
  } else if (parsed == -2) {
    Serial.println("Error Out of databuf");
  }

  delay(500);
}
