# -*- coding: utf-8 -*-
"""
@file DFRobot_WY6005.py
@brief Python driver for the DFRobot WY6005 64x8 dToF sensor (UART).
@copyright  Copyright (c) 2025 DFRobot Co.Ltd
@license    The MIT license (MIT)
@author     [fary](feng.yang@dfrobot.com)
@version    V1.0
@date       2025-12-15
@url        https://github.com/DFRobot/DFRobot_WY6005
"""

import struct
import time
from typing import List, Optional, Tuple
import serial


class DFRobot_WY6005:
  WY6005_SYNC_BYTES = bytes([0x0A, 0x4F, 0x4B, 0x0A])
  WY6005_FRAME_HEADER_SIZE = 4
  WY6005_POINT_DATA_SIZE = 8
  WY6005_MAX_POINTS = 64 * 8  # 64 points per line, 8 lines

  def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.5):
    """
    @brief Initialize sensor over UART.
    @param port UART device path, e.g. '/dev/ttyUSB0'.
    @param baudrate Baudrate, default 921600.
    @param timeout Read timeout in seconds.
    """
    self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    self.total_points = self.WY6005_MAX_POINTS

  def close(self):
    if self.ser and self.ser.is_open:
      self.ser.close()

  def _send_command(self, command: str) -> bool:
    cmd = (command + "\n").encode("ascii")
    self.ser.write(cmd)
    return True

  def set_stream_control(self, enable: bool) -> bool:
    return self._send_command(f"AT+STREAM_CONTROL={'1' if enable else '0'}")

  def set_frame_mode(self, continuous: bool) -> bool:
    return self._send_command(f"AT+SPAD_FRAME_MODE={'1' if continuous else '0'}")

  def set_output_line_data(self, line: int, start_point: int, point_count: int) -> bool:
    return self._send_command(f"AT+SPAD_OUTPUT_LINE_DATA={line},{start_point},{point_count}")

  def trigger_one_frame(self) -> bool:
    return self._send_command("AT+SPAD_TRIG_ONE_FRAME=1")

  def save_config(self) -> bool:
    return self._send_command("AT+SAVE_CONFIG")

  def config_single_point_mode(self, line: int, point: int) -> bool:
    if line < 1 or line > 8:
      return False
    if point < 0 or point > 64:
      return False
    if not self.set_stream_control(False):
      return False
    time.sleep(0.7)
    if not self.set_output_line_data(line, point, point):
      return False
    time.sleep(0.7)
    self.save_config()
    time.sleep(0.7)
    self.total_points = 1
    return self.set_stream_control(True)

  def config_single_line_mode(self, line: int, start_point: int, end_point: int) -> bool:
    if line < 1 or line > 8:
      return False
    if start_point < 0 or start_point > 64 or end_point < 0 or end_point > 64:
      return False
    if start_point > end_point:
      return False
    if not self.set_stream_control(False):
      return False
    time.sleep(0.7)
    if not self.set_output_line_data(line, start_point, end_point):
      return False
    time.sleep(0.7)
    self.save_config()
    time.sleep(0.7)
    self.total_points = end_point - start_point + 1
    return self.set_stream_control(True)

  def config_full_output_mode(self) -> bool:
    # Per device manual: line=0, start=32, count=32 outputs all points.
    if not self.set_stream_control(False):
      return False
    time.sleep(0.7)
    if not self.set_output_line_data(0, 32, 32):
      return False
    time.sleep(0.7)
    self.save_config()
    time.sleep(0.7)
    self.total_points = self.WY6005_MAX_POINTS
    return self.set_stream_control(True)

  def config_single_frame_mode(self) -> bool:
    if not self.set_stream_control(False):
      return False
    if not self.set_frame_mode(True):
      return False
    self.save_config()
    return self.set_stream_control(True)

  def config_continuous_mode(self) -> bool:
    if not self.set_stream_control(False):
      return False
    if not self.set_frame_mode(False):
      return False
    self.save_config()
    return self.set_stream_control(True)

  def _find_sync(self, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    sync_len = len(self.WY6005_SYNC_BYTES)
    idx = 0
    while time.time() < deadline:
      b = self.ser.read(1)
      if not b:
        continue
      if b[0] == self.WY6005_SYNC_BYTES[idx]:
        idx += 1
        if idx == sync_len:
          return True
      else:
        idx = 1 if b[0] == self.WY6005_SYNC_BYTES[0] else 0
    return False

  def trigger_get_raw(self, max_points: Optional[int] = None, timeout_ms: int = 500) -> Tuple[List[int], List[int], List[int], List[int]]:
    """
    @brief Trigger one frame and read raw x/y/z/i arrays.
    @param max_points Optional cap. If not set, uses configured total_points.
    @param timeout_ms Timeout to wait for a frame.
    @return tuple of lists (x, y, z, intensity); empty lists if timeout/error.
    """
    points = self.total_points if self.total_points else (max_points or self.WY6005_MAX_POINTS)
    point_bytes = points * self.WY6005_POINT_DATA_SIZE
    total_size = self.WY6005_FRAME_HEADER_SIZE + point_bytes

    # Flush buffer to avoid stale data
    self.ser.reset_input_buffer()

    # Trigger frame
    self.trigger_one_frame()

    if not self._find_sync(timeout_ms / 1000.0):
      return [], [], [], []

    payload = self.ser.read(total_size)
    if len(payload) != total_size:
      return [], [], [], []

    x_list: List[int] = []
    y_list: List[int] = []
    z_list: List[int] = []
    i_list: List[int] = []

    for i in range(points):
      offset = self.WY6005_FRAME_HEADER_SIZE + i * self.WY6005_POINT_DATA_SIZE
      if offset + self.WY6005_POINT_DATA_SIZE > len(payload):
        break
      x, y, z, inten = struct.unpack_from('<hhhh', payload, offset)
      x_list.append(x)
      y_list.append(y)
      z_list.append(z)
      i_list.append(inten)

    return x_list, y_list, z_list, i_list


if __name__ == "__main__":
  # Simple sanity check (requires actual hardware connected)
  sensor = DFRobot_WY6005(port="/dev/ttyUSB0", baudrate=921600)
  try:
    sensor.config_single_frame_mode()
    sensor.config_full_output_mode()
    xs, ys, zs, is_ = sensor.trigger_get_raw(timeout_ms=300)
    print(f"Received points: {len(xs)}")
  finally:
    sensor.close()
