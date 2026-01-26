# DFRobot_64x8DTOF

* [中文版](./README_CN.md)

The DFRobot 64x8DTOF is a high-precision 3D ToF (Time of Flight) sensor module providing dense distance point-cloud output over a serial interface. This document describes using the 64x8DTOF Python driver on Raspberry Pi.

## Product Link
[64x8DTOF ToF Sensor](https://www.dfrobot.com/)

```text
SKU: 64x8DTOF
```

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Usage](#usage)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

This Python package provides a simple interface to configure the 64x8DTOF sensor, select output mode, trigger frames and read raw X/Y/Z/I point data from the device via serial.

## Installation

To use this library, first download the library to your Raspberry Pi, then open the example folder. To run an example demox.py, enter `python demox.py` in the command line. For example, to run the 01. full_output_demo.py example, you need to enter:

```bash
python 01. full_output_demo.py
```

## Methods

```python
def config_frame_mode(self, mode):
    """Set the sensor frame mode.
    @details Configure whether the sensor operates in single-frame or continuous mode.
    @param mode: Use the class constants FRAME_MODE_SINGLE or FRAME_MODE_CONTINUOUS.
    @return bool: True on success, False on failure.
    @retval True Configuration successful
    @retval False Configuration failed
    """

def config_measure_mode(self, *args):
    """Configure measurement output mode.
    @details Set which points/lines the sensor outputs based on variable arguments.
    @param *args: Variable arguments for mode selection:
    @n - no args -> full output (all points)
    @n - (line,) -> single line mode (line: 1..8)
    @n - (line, point) -> single point mode (line:1..8, point:0..63)
    @return bool: True on success, False on failure.
    @retval True Configuration successful
    @retval False Configuration failed
    """

def get_data(self, timeout_ms=500):
    """Trigger one frame and read raw X/Y/Z/intensity arrays.
    @details This method will:
    @n 1. Flush the serial input buffer.
    @n 2. Trigger a single frame using the device command.
    @n 3. Wait for the sync sequence and then read N * DTOF64X8_POINT_DATA_SIZE bytes, where N is the configured total_points.
    @n 4. Parse each point using little-endian signed 16-bit integers (struct.unpack_from('<hhhh')) yielding (x, y, z, intensity) per point.
    @param timeout_ms: Timeout in milliseconds to wait for frame sync and payload.
    @return tuple: (x_list, y_list, z_list, intensity_list); returns four empty lists on timeout or error.
    @retval (x_list, y_list, z_list, intensity_list): On success, four lists of equal length N containing parsed values.
    @retval ([], [], [], []): On timeout waiting for sync or if the payload length does not match expected size.
    """
```

## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |           |            |    √     |         |
| Python3 |     √     |            |          |         |

## History

- Date: 2026-1-21
- Version: V1.0.0

## Credits

Written by PLELES (PLELES@dfrobot.com)
[DFRobot Website](https://www.dfrobot.com/)