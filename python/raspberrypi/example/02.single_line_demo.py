# -*- coding: utf-8 -*-
'''
@file single_line_demo.py
@brief Single line mode demo - Read one line (64 points) from DFRobot_64x8DTOF.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT license (MIT)
@author [PLELES] (https://github.com/PLELES)
@version  V1.0
@date  2026-1-26
'''

import sys
import time
import serial

sys.path.append("../")
from DFRobot_64x8DTOF import DFRobot_64x8DTOF

# Initialize sensor with UART port
# If you are using a USB-to-serial converter, use '/dev/ttyUSB0'
# If you are using Raspberry Pi GPIO (TX/RX), use '/dev/serial0' (Recommended for GPIO)
# Note: Ensure Serial Hardware is enabled in 'sudo raspi-config' -> Interface Options
try:
  dtof64x8 = DFRobot_64x8DTOF(port="/dev/serial0", baudrate=921600)
except serial.SerialException as e:
  print(f"Error: Could not open serial port: {e}")
  print("Please allow permission to read/write the serial port or change the port name.")
  sys.exit(1)


def setup():
  print("DFRobot_64x8DTOF init...")

  if not dtof64x8.begin():
    print("Beginning sensor failed!")
    sys.exit(1)

  # Configure to single frame mode (retry until success)
  print("Configuring frame mode: Single Frame...")
  while not dtof64x8.config_frame_mode(dtof64x8.FRAME_MODE_SINGLE):
    print("Config Single Frame Mode failed, retrying...")
    time.sleep(0.2)
  print("Config Single Frame Mode: Success")

  # Configure to single line mode (Line 4, all 64 points) (retry until success)
  print("Configuring Single Line Mode (Line 4)...")
  while not dtof64x8.config_measure_mode(4):
    print("Config Single Line Mode failed, retrying...")
    time.sleep(0.2)
  print("Config Single Line Mode (Line 4): Success")

  time.sleep(2)

def loop():
  # Trigger acquisition of one frame
  # returns lists: x, y, z, intensity
  list_x, list_y, list_z, list_i = dtof64x8.get_data(timeout_ms=500)

  if len(list_x) > 0:
    print(f"Received {len(list_x)} points from line 4")
    # Print data for every point
    for idx, x_val in enumerate(list_x):
      print(f"Point[{idx:02d}]: X:{x_val:04d} mm Y:{list_y[idx]:04d} mm Z:{list_z[idx]:04d} mm I:{list_i[idx]}")
  else:
    print("No data received or timeout")

  time.sleep(2)


if __name__ == "__main__":
  setup()
  try:
    while True:
      loop()
  except KeyboardInterrupt:
    dtof64x8.close()
    print("Program stopped")
