# vision-follow-car-mcu (DlaBoard_B)

This repository is the **MCU / low-level control module** of the [vision-follow-car](https://github.com/DaliangAuto/vision-follow-car) project.

**Author**: DaliangAuto
**Board**: DlaBoard_B
**MCU**: STM32F407VET6 (168 MHz, Cortex-M4F)

Repository: [github.com/DaliangAuto/vision-follow-car-mcu](https://github.com/DaliangAuto/vision-follow-car-mcu)

---

The firmware runs on the **DlaBoard_B** control board (STM32F407VET6) and implements the vehicle low-level controller for autonomous-driving data collection and host (Jetson) follow / autonomous control. It receives commands from Jetson over UART, drives steering / throttle / brake, and reports vehicle data during data collection. Six working modes are supported: **manual, remote (RC), training (data logging), follow (Jetson closed-loop), collect (SLAM logging) and autonomous**.

## Where to buy the DlaBoard_B board

- China (Taobao): <https://item.taobao.com/item.htm?id=1063856776213>
- International: <https://shop.daliangauto.com/>

## Stack

- **Board**: DlaBoard_B
- **MCU**: STM32F407VET6, 168 MHz, Cortex-M4F
- **Host link**: USART1 ↔ Jetson, 115200, 22-byte frames, CRC16-MODBUS
- **Modes**: Manual / Remote / Training (50 Hz uplink) / Follow (TYPE=0x10) / Collect / Autonomous (TYPE=0x11)
- **On-board**: 6-axis IMU, CAN (FOC drive / steering servo / brake), EEPROM

## Documentation

- Host ↔ MCU protocol (modes, follow `0x10` / autonomous `0x11` / status `0x03`): [AUTONOMOUS_MODE_SPEC.md](AUTONOMOUS_MODE_SPEC.md)
- Board hardware specification (electrical, pinout, connectors): [DlaBoard_B specification](https://github.com/DaliangAuto/vision-follow-car/blob/main/docs/board_spec.md)

## Build

STM32CubeIDE / CubeMX project. Open this folder in STM32CubeIDE and build, or use the `.ioc` with the ARM GCC toolchain. Flash over SWD (header H1) or the USB1 one-click ISP.

## Usage

If you use this project (code, protocol, or documentation), please **cite the source**:
[DaliangAuto/vision-follow-car-mcu](https://github.com/DaliangAuto/vision-follow-car-mcu)

---

[中文](README_CN.md)
