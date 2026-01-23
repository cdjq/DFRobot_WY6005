# DFRobot_WY6005

* [English Version](./README.md)

WY6005 是一款高精度 3D ToF（飞行时间）传感器模块，通过串口输出点云/距离数据。此文档说明如何在树莓派上使用 WY6005 的 Python 驱动。

## 产品链接
[WY6005 ToF Sensor](https://www.dfrobot.com/)

```text
SKU: WY6005
```

## 目录

* [概述](#概述)
* [安装](#安装)
* [使用示例](#使用示例)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)

## 概述

该 Python 库用于通过串口与 WY6005 通信，包含配置模式、触发采样以及接收原始 X/Y/Z/I 点云数据的接口，便于快速在项目中集成距离或点云数据采集。

## 安装

1. 安装依赖：

```bash
pip install pyserial
```

2. 将设备连接到树莓派串口（推荐使用 `/dev/serial0`）或使用 USB 转串口。

## 使用示例

```python
from DFRobot_WY6005 import DFRobot_WY6005
import time

wy = DFRobot_WY6005(port="/dev/serial0", baudrate=921600)

wy.config_single_frame_mode()
wy.config_full_output_mode()

while True:
  list_x, list_y, list_z, list_i = wy.trigger_get_raw(timeout_ms=1000)
  if len(list_x) > 0:
    print(f"接收到 {len(list_x)} 个点")
    for i in range(len(list_x)):
      print(f"点[{i}] X:{list_x[i]} Y:{list_y[i]} Z:{list_z[i]} I:{list_i[i]}")
  else:
    print("未接收到数据或超时")
  time.sleep(1)

wy.close()
```

## 兼容性

* RaspberryPi 版本

| 板子型号     | 运行良好 | 运行异常 | 未测试 | 备注 |
| ------------ | :------: | :------: | :----: | ---- |
| RaspberryPi2 |          |          |   √    |      |
| RaspberryPi3 |          |          |   √    |      |
| RaspberryPi4 |    √     |          |        |      |

* Python 版本

| Python  | 运行良好 | 运行异常 | 未测试 | 备注 |
| ------- | :------: | :------: | :----: | ---- |
| Python2 |          |          |   √    |      |
| Python3 |    √     |          |        |      |

## 历史

- Date: 2026-1-21
- Version: V1.0.0

## 创作者

作者: PLELES (PLELES@dfrobot.com)
[DFRobot 网站](https://www.dfrobot.com/)