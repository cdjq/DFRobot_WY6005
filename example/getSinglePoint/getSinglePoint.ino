/*!
 * @file getSinglePoint.ino
 * @brief Example of getting single point data
 * @copyright  Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [PLELES] (https://github.com/PLELES)
 * @version V1.0
 * @date 2026-1-21
 * @url https://github.com/DFRobot/DFRobot_WY6005
 */
#include "DFRobot_WY6005.h"

// Instantiate the sensor object
// Use Serial1 for communication, change pins 25/26 to your actual RX/TX pins
DFRobot_WY6005 wy6005(Serial1, SERIAL_8N1, 25, 26);

void setup()
{
  Serial.begin(115200);
  wy6005.begin(921600);
  Serial.println("WY6005 Single Point Demo");
  
    if (!wy6005.configMeasureFrameMode(DFRobot_WY6005::eFrameModeSingle)) {
    Serial.println("Error: configMeasureFrameMode failed");
  }

  // 2. Configure Single Point Mode
  // Example: Line 4, Point 32 (Center of the sensor roughly)
  // Line range: 1-8
  // Point range: 0-63
  Serial.println("Configuring Single Point Mode (Line 4, Point 32)...");
  if (wy6005.configSinglePointMode(4, 32)) {
    Serial.println("Configuration Successful!");
  } else {
    Serial.println("Configuration Failed!");
  }
  wy6005.clearBuffer();
}

void loop()
{
  int pointsRead = wy6005.triggerGetRaw(wy6005.point.xBuf, wy6005.point.yBuf, 
                                        wy6005.point.zBuf, wy6005.point.iBuf, 
                                        300);

  if (pointsRead > 0) {
    Serial.print("Point Data -> ");
    Serial.print("X: "); Serial.print(wy6005.point.xBuf[0]); Serial.print(" mm, ");
    Serial.print("Y: "); Serial.print(wy6005.point.yBuf[0]); Serial.print(" mm, ");
    Serial.print("Z: "); Serial.print(wy6005.point.zBuf[0]); Serial.print(" mm, ");
    Serial.print("I: "); Serial.println(wy6005.point.iBuf[0]); 
  } else {
    Serial.println("Read timeout or error");
  }

  delay(500);
}
