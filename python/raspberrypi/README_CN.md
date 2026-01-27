# DFRobot_64x8DTOF

* [English Version](./README.md)

64x8DTOF 是一款高精度 3D ToF（飞行时间）传感器模块，通过串口输出点云/距离数据。此文档说明如何在树莓派上使用 64x8DTOF 的 Python 驱动。

## 产品链接
[64x8DTOF ToF Sensor](https://www.dfrobot.com/)

```text
SKU: SEN0682
```

## 目录

* [概述](#概述)
* [安装](#安装)
* [使用示例](#使用示例)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)

## 概述

该 Python 库用于通过串口与 64x8DTOF 通信，包含配置模式、触发采样以及接收原始 X/Y/Z/I 点云数据的接口，便于快速在项目中集成距离或点云数据采集。

## 安装

要使用这个库，首先将库下载到Raspberry Pi，然后打开例程文件夹。要执行一个例程demox.py，请在命令行中输入python demox.py。例如，要执行01. full_output_demo.py例程，你需要输入:

```bash
python 01.full_output_demo.py
```

## 方法

```python
def config_frame_mode(self, mode):
    """设置传感器帧模式。
    @details 配置传感器是单帧模式还是连续模式。
    @param mode: 使用类常量 FRAME_MODE_SINGLE 或 FRAME_MODE_CONTINUOUS。
    @return bool: 成功返回 True，失败返回 False。
    @retval True 配置成功
    @retval False 配置失败
    """

def config_measure_mode(self, *args):
    """配置测量输出模式。
    @details 根据可变参数设置传感器输出的点/线。
    @param *args: 模式选择的变量参数：
    @n - 无参数 -> 全点输出（所有点）
    @n - (line,) -> 单线模式（line: 1..8）
    @n - (line, point) -> 单点模式（line:1..8, point:1-64）
    @n - (line, start_point, end_point) -> 多点模式（line:1..8, start:1..64, end:1..64），用于配置同一行上的一段连续点输出
    @return bool: 成功返回 True，失败返回 False。
    @retval True 配置成功
    @retval False 配置失败
    """

def begin(self):
    """初始化串口并开启传感器数据流。
    @details
    - 检查已配置的波特率是否为 921600（库默认）；不是则返回 False。
    - 等待约 0.4 秒以让串口稳定。
    - 调用内部方法 `_set_stream_control(True)` 尝试开启传感器数据流。
    - 返回 True 表示成功开启并准备就绪，返回 False 表示初始化或开启流失败。
    @return bool: 初始化并开启数据流成功返回 True，否则返回 False。
    """

def get_data(self, timeout_ms=500):
    """触发一帧并读取原始 X/Y/Z/强度 数组。
    @details 本方法执行流程：
    @n 1. 清空串口输入缓冲区以避免过期数据。
    @n 2. 发送触发单帧命令到设备。
    @n 3. 等待同步字节序列，随后读取 N * DTOF64X8_POINT_DATA_SIZE 字节，其中 N 为当前配置的 total_points。
    @n 4. 使用小端有符号 16 位整型解析每个点（struct.unpack_from('<hhhh')），得到每点 (x, y, z, intensity)。
    @param timeout_ms: 等待帧同步与有效负载的超时时间（毫秒）。
    @return tuple: (x_list, y_list, z_list, intensity_list)；超时或错误时返回四个空列表。
    @retval (x_list, y_list, z_list, intensity_list)：成功时返回四个长度相同的列表，长度为 N。
    @retval ([], [], [], [])：等待同步超时或读取到的负载长度与预期不符时返回空列表组。
    """
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
