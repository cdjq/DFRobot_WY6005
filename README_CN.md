# DFRobot_WY6005
- [English Version](./README.md)

## 概述
DFRobot_WY6005 是一款高精度 DToF（飞行时间）传感器模块，提供紧凑尺寸下的精确距离测量。模块支持串口通信、多种工作模式和高分辨率的距离点云输出，适用于机器人、自动化、接近检测等场景。

## 产品链接
[WY6005 DToF Sensor](https://www.dfrobot.com/)

```text
SKU: WY6005
```

## 目录

  * [概要](#概要)
  * [安装](#安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [致谢](#致谢)

## 概要
本库提供在 Arduino 与树莓派（Raspberry Pi）上驱动 WY6005 传感器的实现，包含传感器配置、触发采样、读取原始点云数据等功能，便于在不同应用中集成实时距离或点云采集。

## 安装

### Arduino 库安装
1. 从仓库下载库文件。
2. 将库复制到 Arduino 的 `libraries` 目录中。
3. 打开 `examples` 文件夹中的示例草图并上传测试。


## 方法
### Arduino C++ 库
```C++
/**
 * @fn DFRobot_WY6005
 * @brief 构造函数，传入串口和配置
 * @param serial 硬件串口引用
 * @param config 串口配置（例如 SERIAL_8N1）
 * @param rxPin RX 引脚号（支持重映射的平台可选）
 * @param txPin TX 引脚号（支持重映射的平台可选）
 */
DFRobot_WY6005(HardwareSerial &serial, uint32_t config, int8_t rxPin, int8_t txPin);

/**
 * @fn begin
 * @brief 初始化传感器串口
 * @param baudRate 串口波特率
 */
void begin(uint32_t baudRate);

/**
 * @fn clearBuffer
 * @brief 清空串口接收缓冲区
 */
void clearBuffer(void);

/**
 * @fn triggerGetRaw
 * @brief 触发一次并读取原始 x/y/z 值（不滤波）
 * @param xBuf x 值缓冲
 * @param yBuf y 值缓冲
 * @param zBuf z 值缓冲
 * @param iBuf 强度值缓冲
 * @param timeoutMs 超时（毫秒）
 * @return 解析到的点数，超时或错误返回 -1
 */
int triggerGetRaw(int16_t* xBuf, int16_t* yBuf, int16_t* zBuf, int16_t* iBuf, uint32_t timeoutMs);

/**
 * @fn triggerOneFrame
 * @brief 触发一次数据输出（发送 AT+SPAD_TRIG_ONE_FRAME=1）
 * @return 操作是否成功
 */
bool triggerOneFrame(void);

/**
 * @fn saveConfig
 * @brief 将当前配置保存到传感器
 * @return 操作是否成功
 */
bool saveConfig(void);

/**
 * @fn configSinglePointMode
 * @brief 配置单点输出模式
 * @param line 行号（1..8）
 * @param point 点索引（0..64）
 * @return bool 类型，表示配置状态
 * @retval true 配置成功
 * @retval false 配置失败
 */
bool configSinglePointMode(uint8_t line, uint8_t point);

/**
 * @fn configSingleLineMode
 * @brief 配置单行输出模式（输出整行或指定起止点）
 * @param line 行号（1..8）
 * @param startPoint 起始点索引（0..64）
 * @param endPoint 结束点索引（0..64）
 * @return bool 类型，表示配置状态
 * @retval true 配置成功
 * @retval false 配置失败
 */
bool configSingleLineMode(uint8_t line, uint8_t startPoint, uint8_t endPoint);

/**
 * @fn configMeasureFrameMode
 * @brief 配置传感器为单帧或连续帧模式
 * @param mode 模式（eFrameModeSingle 或 eFrameModeContinuous）
 * @return bool 类型，表示配置状态
 * @retval true 配置成功
 * @retval false 配置失败
 */
bool configMeasureFrameMode(eFrameMode_t mode);
```

## 兼容性
| 平台 | 运行良好 | 运行异常 | 未测试 | 备注 |
|------|------|------|--------|------|
| Arduino UNO |  | √ |  |  |
| Arduino MEGA2560 | √ |  |  |  |
| ESP32 | √ |  |  | 使用 Serial1 |

## 历史
- Date: 2026-1-21
- Version: V1.0.0

## 创作者
作者: PLELES (PLELES@dfrobot.com)，2025.04.10
[DFRobot Website](https://www.dfrobot.com/)





