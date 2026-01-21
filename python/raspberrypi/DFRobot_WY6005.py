# -*- coding: utf-8 -*-
'''
@file DFRobot_WY6005.py
@brief Define the basic structure and methods of the DFRobot_WY6005 class.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT license (MIT)
@author [PLELES] (https://github.com/PLELES)
@version  V1.0
@date  2026-1-21
@https://github.com/DFRobot/DFRobot_WY6005
'''

import struct
import time
import serial


class DFRobot_WY6005:
  WY6005_SYNC_BYTES = bytes([0x0A, 0x4F, 0x4B, 0x0A])
  WY6005_FRAME_HEADER_SIZE = 4
  WY6005_POINT_DATA_SIZE = 8
  WY6005_MAX_POINTS = 64 * 8  # 64 points per line, 8 lines

  def __init__(self, port, baudrate=921600, timeout=0.5):
    '''
    @fn __init__
    @brief Initialize sensor over UART.
    @param port: UART device path, e.g. '/dev/ttyUSB0'.
    @param baudrate: Baudrate, default 921600.
    @param timeout: Read timeout in seconds.
    @return None
    '''
    self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    self.total_points = self.WY6005_MAX_POINTS

  def close(self):
    '''
    @fn close
    @brief Close the serial port.
    @return None
    '''
    if self.ser and self.ser.is_open:
      self.ser.close()

  def _send_command(self, command):
    '''
    @fn _send_command
    @brief Send a command to the sensor.
    @param command: Command string to send.
    @return bool: True if command sent successfully.
    '''
    cmd = (command + "\n").encode("ascii")
    self.ser.write(cmd)
    return True

  def set_stream_control(self, enable):
    '''
    @fn set_stream_control
    @brief Set stream control.
    @param enable: Enable stream control.
    @return bool: True if command sent successfully.
    '''
    return self._send_command(f"AT+STREAM_CONTROL={'1' if enable else '0'}")

  def set_frame_mode(self, continuous):
    '''
    @fn set_frame_mode
    @brief Set frame mode.
    @param continuous: Enable continuous mode.
    @return bool: True if command sent successfully.
    '''
    return self._send_command(f"AT+SPAD_FRAME_MODE={'1' if continuous else '0'}")

  def set_output_line_data(self, line, start_point, point_count):
    '''
    @fn set_output_line_data
    @brief Set output line data.
    @param line: Line number.
    @param start_point: Start point number.
    @param point_count: Point count.
    @return bool: True if command sent successfully.
    '''
    return self._send_command(f"AT+SPAD_OUTPUT_LINE_DATA={line},{start_point},{point_count}")

  def trigger_one_frame(self):
    '''
    @fn trigger_one_frame
    @brief Trigger one frame data output.
    @return bool: True if command sent successfully.
    '''
    return self._send_command("AT+SPAD_TRIG_ONE_FRAME=1")

  def save_config(self):
    '''
    @fn save_config
    @brief Save configuration to sensor.
    @return bool: True if command sent successfully.
    '''
    return self._send_command("AT+SAVE_CONFIG")

  def config_single_point_mode(self, line, point):
    '''
    @fn config_single_point_mode
    @brief Configure single point mode.
    @param line: Line number.
    @param point: Point number.
    @return bool: True if configuration successful.
    '''
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

  def config_single_line_mode(self, line, start_point, end_point):
    '''
    @fn config_single_line_mode
    @brief Configure single line mode.
    @param line: Line number.
    @param start_point: Start point number.
    @param end_point: End point number.
    @return bool: True if configuration successful.
    '''
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

  def config_full_output_mode(self):
    '''
    @fn config_full_output_mode
    @brief Configure full output mode.
    @return bool: True if configuration successful.
    '''
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

  def config_single_frame_mode(self):
    '''
    @fn config_single_frame_mode
    @brief Configure single frame mode.
    @return bool: True if configuration successful.
    '''
    if not self.set_stream_control(False):
      return False
    if not self.set_frame_mode(True):
      return False
    self.save_config()
    return self.set_stream_control(True)

  def config_continuous_mode(self):
    '''
    @fn config_continuous_mode
    @brief Configure continuous mode.
    @return bool: True if configuration successful.
    '''
    if not self.set_stream_control(False):
      return False
    if not self.set_frame_mode(False):
      return False
    self.save_config()
    return self.set_stream_control(True)

  def _find_sync(self, timeout_s):
    '''
    @fn _find_sync
    @brief Find the synchronization bytes in the UART stream.
    @param timeout_s: Timeout in seconds.
    @return bool: True if sync bytes found, False otherwise.
    '''
    deadline = time.time() + timeout_s
    sync_len = len(self.WY6005_SYNC_BYTES)
    idx = 0
    while time.time() < deadline:
      recv_byte = self.ser.read(1)
      if not recv_byte:
        continue
      if recv_byte[0] == self.WY6005_SYNC_BYTES[idx]:
        idx += 1
        if idx == sync_len:
          return True
      else:
        idx = 1 if recv_byte[0] == self.WY6005_SYNC_BYTES[0] else 0
    return False

  def trigger_get_raw(self, max_points=None, timeout_ms=500):
    '''
    @fn trigger_get_raw
    @brief Trigger one frame and read raw x/y/z/i arrays.
    @param max_points: Optional cap. If not set, uses configured total_points.
    @param timeout_ms: Timeout to wait for a frame.
    @return tuple of lists (x, y, z, intensity); empty lists if timeout/error.
    '''
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

    x_list = []
    y_list = []
    z_list = []
    i_list = []

    for i in range(points):
      offset = self.WY6005_FRAME_HEADER_SIZE + i * self.WY6005_POINT_DATA_SIZE
      if offset + self.WY6005_POINT_DATA_SIZE > len(payload):
        break
      val_x, val_y, val_z, val_i = struct.unpack_from('<hhhh', payload, offset)
      x_list.append(val_x)
      y_list.append(val_y)
      z_list.append(val_z)
      i_list.append(val_i)

    return x_list, y_list, z_list, i_list
