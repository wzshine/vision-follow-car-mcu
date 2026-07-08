# vision-follow-car-mcu (DlaBoard_B)

本仓库是 [vision-follow-car](https://github.com/DaliangAuto/vision-follow-car) 项目的 **MCU / 底层控制模块**。

**作者**: DaliangAuto
**开发板**: DlaBoard_B
**主控**: STM32F407VET6 (168 MHz, Cortex-M4F)

仓库: [github.com/DaliangAuto/vision-follow-car-mcu](https://github.com/DaliangAuto/vision-follow-car-mcu)

---

固件运行于 **DlaBoard_B** 控制板（STM32F407VET6），实现面向自动驾驶数据采集与上位机（Jetson）跟随 / 自动驾驶的车辆底层控制器。通过 UART 接收 Jetson 指令，驱动转向 / 油门 / 刹车，并在数据采集时回传车辆行驶数据。支持六种工作模式：**手动、遥控、训练（数据采集）、跟随（Jetson 闭环）、采集（SLAM）、自动驾驶**。

## DlaBoard_B 控制板购买

- 中国境内（淘宝）：<https://item.taobao.com/item.htm?id=1063856776213>
- 国际地区：<https://shop.daliangauto.com/>

## 技术要点

- **开发板**: DlaBoard_B
- **MCU**: STM32F407VET6, 168 MHz, Cortex-M4F
- **通信**: USART1 ↔ Jetson（115200，22 字节帧，CRC16-MODBUS）
- **模式**: 手动 / 遥控 / 训练（50Hz 上报）/ 跟随（0x10 指令）/ 采集（SLAM）/ 自动驾驶（0x11 指令）
- **板载**: 6 轴 IMU、CAN（FOC 驱动 / 转向伺服 / 刹车）、EEPROM

## 文档

- 上位机 ↔ MCU 协议（模式、跟随 `0x10` / 自动驾驶 `0x11` / 状态 `0x03`）：[AUTONOMOUS_MODE_SPEC.md](AUTONOMOUS_MODE_SPEC.md)
- 板卡硬件规格（电气、引脚、连接器）：[DlaBoard_B 规格书](https://github.com/DaliangAuto/vision-follow-car/blob/main/docs/board_spec_CN.md)

## 构建

STM32CubeIDE / CubeMX 工程。用 STM32CubeIDE 打开本目录编译，或基于 `.ioc` 使用 ARM GCC 工具链。通过 SWD（H1 排针）或 USB1 一键 ISP 烧录。

## 使用说明

使用本仓库代码、协议或文档时，**请注明出处**：
[DaliangAuto/vision-follow-car-mcu](https://github.com/DaliangAuto/vision-follow-car-mcu)

---

[English](README.md)
