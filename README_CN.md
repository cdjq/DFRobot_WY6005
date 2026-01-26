# DFRobot_64x8DTOF
- [English Version](./README.md)

## 概述
DFRobot_64x8DTOF 是一款高精度 DToF（飞行时间）传感器模块，提供紧凑尺寸下的精确距离测量。模块支持串口通信、多种工作模式和高分辨率的距离点云输出，适用于机器人、自动化、接近检测等场景。

## 产品链接
[64x8DTOF DToF Sensor](https://www.dfrobot.com/)

```text
SKU: 64x8DTOF
```

## 目录

  * [概要](#概要)
  * [安装](#安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [致谢](#致谢)

## 概要
本库提供在 Arduino 上驱动 64x8DTOF 传感器的实现，包含传感器配置、触发采样、读取原始点云数据等功能，便于在不同应用中集成实时距离或点云采集。

## 安装

### Arduino 库安装
1. 从仓库下载库文件。
2. 将库复制到 Arduino 的 `libraries` 目录中。
3. 打开 `examples` 文件夹中的示例草图并上传测试。


## 方法
### Arduino C++ 库
```C++
/**
 * @fn DFRobot_64x8DTOF
 * @brief 构造函数，传入串口和配置
 * @param serial 硬件串口引用
 * @param config 串口配置（例如 SERIAL_8N1）
 * @param rxPin RX 引脚号（支持重映射的平台可选）
 * @param txPin TX 引脚号（支持重映射的平台可选）
 */
DFRobot_64x8DTOF(HardwareSerial &serial, uint32_t config, int8_t rxPin, int8_t txPin);

/**
 * @fn begin
 * @brief 初始化传感器串口
 * @param baudRate 串口波特率
 */
void begin(uint32_t baudRate);

/**
 * @fn getData
 * @brief 触发一次并读取原始 x/y/z 值（不滤波）
 * @param timeoutMs 超时（毫秒）
 * @return 解析到的点数，超时或错误返回 -1
 */
int getData(uint32_t timeoutMs = 500);

/**
 * @fn configMeasureMode
 * @brief 配置测量输出模式 — 全输出（所有点）
 * @return bool 如果配置成功且流控制恢复则为 true，通信错误或设备拒绝时为 false
 */
bool configMeasureMode(void);

/**
 * @fn configMeasureMode
 * @brief 配置测量输出模式 — 单行
 * @param lineNum 要输出的行索引（1..8）
 * @return bool 如果配置成功且流控制恢复则为 true，通信错误或参数无效时为 false
 */
bool configMeasureMode(uint8_t lineNum);

/**
 * @fn configMeasureMode
 * @brief 配置测量输出模式 — 单点
 * @param lineNum 包含该点的行索引（1..8）
 * @param pointNum 行内的点索引（0..64）
 * @return bool 如果配置成功且流控制恢复则为 true，通信错误或参数无效时为 false
 */
bool configMeasureMode(uint8_t lineNum, uint8_t pointNum);

/**
 * @fn configFrameMode
 * @brief 配置传感器为单帧或连续帧模式
 * @param mode 模式（eFrameSingle 或 eFrameContinuous）
 * @return bool 类型，表示配置状态
 * @retval true 配置成功
 * @retval false 配置失败
 */
bool configFrameMode(eFrameMode_t mode);
```

## 兼容性
| 平台 | 运行良好 | 运行异常 | 未测试 | 备注 |
|------|------|------|--------|------|
| Arduino UNO |  | √ |  |  |
| Arduino MEGA2560 | √ |  |  |  |
| ESP32 | √ |  |  | 使用 Serial1 |

## 历史
- Date: 2026-1-26
- Version: V1.0.0

## 创作者
作者: PLELES (PLELES@dfrobot.com)，2026.01.26
[DFRobot Website](https://www.dfrobot.com/)





