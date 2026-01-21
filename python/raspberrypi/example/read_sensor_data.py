# -*- coding: utf-8 -*-
'''
@file read_sensor_data.py
@brief Read sensor data from DFRobot_WY6005.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT license (MIT)
@author [PLELES] (https://github.com/PLELES)
@version  V1.0
@date  2026-1-21
@https://github.com/DFRobot/DFRobot_WY6005
'''

import sys
import time

sys.path.append("../")
from DFRobot_WY6005 import DFRobot_WY6005

# Initialize sensor with UART port
# Please change '/dev/ttyUSB0' to your actual serial port
wy6005 = DFRobot_WY6005(port="/dev/ttyUSB0", baudrate=921600)

# Demo Mode Selection
# 0: Full Output Mode (All 8x64 points)
# 1: Single Line Mode (Line 4, points 0-63)
# 2: Single Point Mode (Line 4, Point 32)
DEMO_MODE = 0

def setup():
  print("WY6005 init...")

  # Retry configuration until success
  while not wy6005.config_single_frame_mode():
    print("failed, Connection error or device busy!")
    time.sleep(1)
  
  print("successed")

  ret = False
  if DEMO_MODE == 0:
    # Configure to full output mode (get all 64*8 points)
    ret = wy6005.config_full_output_mode()
    print(f"Config Full Output Mode: {'Success' if ret else 'Failed'}")
  elif DEMO_MODE == 1:
    # Configure to single line mode (Line 4, all 64 points)
    # config_single_line_mode(line, start_point, end_point)
    ret = wy6005.config_single_line_mode(4, 1, 64)
    print(f"Config Single Line Mode (Line 4): {'Success' if ret else 'Failed'}")
  elif DEMO_MODE == 2:
    # Configure to single point mode (Line 4, Point 32)
    # config_single_point_mode(line, point)
    ret = wy6005.config_single_point_mode(4, 32)
    print(f"Config Single Point Mode (Line 4, Point 32): {'Success' if ret else 'Failed'}")


def loop():
  # Trigger acquisition of one frame
  # returns lists: x, y, z, intensity
  list_x, list_y, list_z, list_i = wy6005.trigger_get_raw(timeout_ms=1000)
  
  if len(list_x) > 0:
    print(f"Received {len(list_x)} points")
    # Print the middle point data as example
    idx = len(list_x) // 2
    print(f"Sample Point[{idx}]: X:{list_x[idx]} Y:{list_y[idx]} Z:{list_z[idx]} I:{list_i[idx]}")
  else:
    print("No data received or timeout")
  
  time.sleep(1)

if __name__ == "__main__":
  setup()
  try:
    while True:
      loop()
  except KeyboardInterrupt:
    wy6005.close()
    print("Program stopped")
