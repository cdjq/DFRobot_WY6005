# -*- coding: utf-8 -*-
'''
@file full_output_demo.py
@brief Full output mode demo - Read all 8x64 points from DFRobot_64x8DTOF.
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

  # Configure to single frame mode
  ret = dtof64x8.config_frame_mode(dtof64x8.FRAME_MODE_SINGLE)
  print(f"Config Single Frame Mode: {'Success' if ret else 'Failed'}")

  # Configure to full output mode (get all 64*8 points)
  ret = dtof64x8.config_measure_mode()
  print(f"Config Full Output Mode: {'Success' if ret else 'Failed'}")


def loop():
  # Trigger acquisition of one frame
  # returns lists: x, y, z, intensity
  list_x, list_y, list_z, list_i = dtof64x8.get_data(timeout_ms=500)

  if len(list_x) > 0:
    print(f"Received {len(list_x)} points")
    # Print data for every point
    for idx, x_val in enumerate(list_x):
      print(f"Point[{idx}]: X:{x_val} Y:{list_y[idx]} Z:{list_z[idx]} I:{list_i[idx]}")
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