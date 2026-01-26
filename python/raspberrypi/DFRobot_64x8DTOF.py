# -*- coding: utf-8 -*-
'''
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT license (MIT)
@author [PLELES] (https://github.com/PLELES)
@version  V1.0
@date  2026-1-21
@file DFRobot_64x8DTOF.py
@brief Define the basic structure and methods of the DFRobot_64x8DTOF class.
'''

import struct
import time
import serial

# pylint: disable=invalid-name

class DFRobot_64x8DTOF:
  DTOF64X8_SYNC_BYTES = bytes([0x0A, 0x4F, 0x4B, 0x0A])
  DTOF64X8_FRAME_HEADER_SIZE = 4
  DTOF64X8_POINT_DATA_SIZE = 8
  DTOF64X8_MAX_POINTS = 64 * 8  
  
  FRAME_MODE_SINGLE     = 0x00
  FRAME_MODE_CONTINUOUS = 0x01

  MEASURE_MODE_SINGLE_POINT = 0x00
  MEASURE_MODE_SINGLE_LINE = 0x01
  MEASURE_MODE_FULL = 0x02
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
    self.total_points = self.DTOF64X8_MAX_POINTS

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

  
  def _set_stream_control(self, enable):
    '''
    @fn _set_stream_control
    @brief Set stream control (private helper).
    @param enable: Enable stream control (bool).
    @return bool: True if command sent successfully.
    '''
    return self._send_command(f"AT+STREAM_CONTROL={'1' if enable else '0'}")

  
  def _set_frame_mode(self, continuous):
    '''
    @fn _set_frame_mode
    @brief Set frame mode (private helper).
    @param continuous: True for single-frame/continuous flag meaning per-device protocol.
    @return bool: True if command sent successfully.
    '''
    return self._send_command(f"AT+SPAD_FRAME_MODE={'1' if continuous else '0'}")

  
  def _set_output_line_data(self, line, start_point, end_point):
    '''
    @fn _set_output_line_data
    @brief Set which line/points are output by the sensor (private helper).
    @param line: Line number (0..8). Use 0 for global/full configuration.
    @param start_point: Start point index (0..63).
    @param end_point: End point index (0..63).
    @return bool: True if command sent successfully.
    '''
    if line < 0 or line > 8:
      return False
    if start_point < 0 or start_point > 63:
      return False
    if end_point < 0 or end_point > 63:
      return False
    if end_point < start_point:
      return False
    return self._send_command(f"AT+SPAD_OUTPUT_LINE_DATA={line},{start_point},{end_point}")

  
  def _trigger_one_frame(self):
    '''
    @fn _trigger_one_frame
    @brief Trigger a single frame output from the sensor (private helper).
    @return bool: True if command sent successfully.
    '''
    return self._send_command("AT+SPAD_TRIG_ONE_FRAME=1")

  
  def _save_config(self):
    '''
    @fn _save_config
    @brief Persist current configuration to sensor (private helper).
    @return bool: True if command sent successfully.
    '''
    return self._send_command("AT+SAVE_CONFIG")

  def _config_single_point_mode(self, line, point):
    '''
    @fn config_single_point_mode
    @brief Configure single point mode.
    @param line: Line number.
    @param point: Point number.
    @return bool: True if configuration successful.
    '''
    if not self._set_stream_control(False):
      return False
    time.sleep(0.7)
    if not self._set_output_line_data(line, point, point):
      return False
    time.sleep(0.7)
    self._save_config()
    time.sleep(0.7)
    self.total_points = 1
    return self._set_stream_control(True)

  def _config_single_line_mode(self, line, start_point, end_point):
    '''
    @fn config_single_line_mode
    @brief Configure single line mode.
    @param line: Line number.
    @param start_point: Start point number.
    @param end_point: End point number.
    @return bool: True if configuration successful.
    '''
    if not self._set_stream_control(False):
      return False
    time.sleep(0.7)
    if not self._set_output_line_data(line, start_point, end_point):
      return False
    time.sleep(0.7)
    self._save_config()
    time.sleep(0.7)
    self.total_points = end_point - start_point + 1
    return self._set_stream_control(True)

  def _config_full_output_mode(self):
    '''
    @fn config_full_output_mode
    @brief Configure full output mode.
    @return bool: True if configuration successful.
    '''
    # Per device manual: line=0, start=32, count=32 outputs all points.
    if not self._set_stream_control(False):
      return False
    time.sleep(0.7)
    # device expects start and count; for full output the manual uses start=32,count=32 -> end=63
    if not self._set_output_line_data(0, 32, 63):
      return False
    time.sleep(0.7)
    self._save_config()
    time.sleep(0.7)
    self.total_points = self.DTOF64X8_MAX_POINTS
    return self._set_stream_control(True)
  def _find_sync(self, timeout_s):
    '''
    @fn _find_sync
    @brief Find the synchronization bytes in the UART stream.
    @param timeout_s: Timeout in seconds.
    @return bool: True if sync bytes found, False otherwise.
    '''
    deadline = time.time() + timeout_s
    sync_len = len(self.DTOF64X8_SYNC_BYTES)
    idx = 0
    while time.time() < deadline:
      recv_byte = self.ser.read(1)
      if not recv_byte:
        continue
      if recv_byte[0] == self.DTOF64X8_SYNC_BYTES[idx]:
        idx += 1
        if idx == sync_len:
          return True
      else:
        idx = 1 if recv_byte[0] == self.DTOF64X8_SYNC_BYTES[0] else 0
    return False



  def config_frame_mode(self, mode):
    '''
    @fn config_frame_mode
    @brief Unified frame mode API.
    @param mode: Must be one of the class constants: FRAME_MODE_SINGLE or FRAME_MODE_CONTINUOUS.
    @return bool: True if configuration successful.
    '''
    if not isinstance(mode, int):
      return False
    if mode not in (self.FRAME_MODE_SINGLE, self.FRAME_MODE_CONTINUOUS):
      return False
    frame_mode = mode

    # Perform sequence: stop stream -> set frame mode -> save -> start stream
    if not self._set_stream_control(False):
      return False
    time.sleep(0.7)
    if not self._set_frame_mode(True if frame_mode == self.FRAME_MODE_SINGLE else False):
      return False
    time.sleep(0.7)
    self._save_config()
    time.sleep(0.7)
    return self._set_stream_control(True)

  def config_measure_mode(self, *args):
    '''
    @fn config_measure_mode
    @brief Unified measurement mode API (variable arguments).
    @details
      - no args -> full output mode
      - (line,) -> single line mode (line: 1..8)
      - (line, point) -> single point mode (line:1..8, point:0..63)
    @return bool: True if configuration successful.
    '''
    if len(args) == 0:
      return self._config_full_output_mode()
    if len(args) == 1:
      line = int(args[0])
      return self._config_single_line_mode(line, 1, 64)
    if len(args) == 2:
      line = int(args[0])
      point = int(args[1])
      return self._config_single_point_mode(line, point)
    raise TypeError('config_measure_mode accepts 0,1 or 2 arguments')



  def get_data(self, timeout_ms=500):
    '''
    @fn get_data
    @brief Trigger one frame and read raw x/y/z/i arrays.
    @param timeout_ms: Timeout to wait for a frame (ms).
    @return tuple of lists (x, y, z, intensity); empty lists if timeout/error.
    '''
    points = self.total_points
    point_bytes = points * self.DTOF64X8_POINT_DATA_SIZE
    # total_size should only be the data size (header handled by _find_sync)
    total_size = point_bytes

    # Flush buffer to avoid stale data
    self.ser.reset_input_buffer()

    # Try multiple attempts to get sync
    max_retries = 3
    for attempt in range(max_retries):
      self._trigger_one_frame()
      if self._find_sync(timeout_ms / 1000.0):
        break
      if attempt == max_retries - 1:
        return [], [], [], []

    payload = self.ser.read(total_size)
    if len(payload) != total_size:
      return [], [], [], []

    x_list = []
    y_list = []
    z_list = []
    i_list = []

    for i in range(points):
      offset = i * self.DTOF64X8_POINT_DATA_SIZE
      if offset + self.DTOF64X8_POINT_DATA_SIZE > len(payload):
        break
      val_x, val_y, val_z, val_i = struct.unpack_from('<hhhh', payload, offset)
      x_list.append(val_x)
      y_list.append(val_y)
      z_list.append(val_z)
      i_list.append(val_i)

    return x_list, y_list, z_list, i_list
