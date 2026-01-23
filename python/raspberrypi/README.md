# DFRobot_WY6005

* [中文版](./README_CN.md)

The DFRobot WY6005 is a high-precision 3D ToF (Time of Flight) sensor module providing dense distance point-cloud output over a serial interface. This document describes using the WY6005 Python driver on Raspberry Pi.

## Product Link
[WY6005 ToF Sensor](https://www.dfrobot.com/)

```text
SKU: WY6005
```

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Usage](#usage)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

This Python package provides a simple interface to configure the WY6005 sensor, select output mode, trigger frames and read raw X/Y/Z/I point data from the device via serial.

## Installation

1. Ensure `pyserial` is installed:

```bash
pip install pyserial
```

2. Connect the WY6005 to the Raspberry Pi UART (`/dev/serial0` recommended) or use a USB-serial adapter.

## Usage

Example usage (adjust `port` and `baudrate` as needed):

```python
from DFRobot_WY6005 import DFRobot_WY6005
import time

wy = DFRobot_WY6005(port="/dev/serial0", baudrate=921600)

# Configure single-frame mode and set demo mode
wy.config_single_frame_mode()
wy.config_full_output_mode()  # or config_single_line_mode(...), config_single_point_mode(...)

while True:
    list_x, list_y, list_z, list_i = wy.trigger_get_raw(timeout_ms=1000)
    if len(list_x) > 0:
        print(f"Received {len(list_x)} points")
        for idx in range(len(list_x)):
            print(f"Pt[{idx}] X:{list_x[idx]} Y:{list_y[idx]} Z:{list_z[idx]} I:{list_i[idx]}")
    else:
        print("No data or timeout")
    time.sleep(1)

wy.close()
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