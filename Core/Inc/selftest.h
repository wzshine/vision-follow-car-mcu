#ifndef SELFTEST_H
#define SELFTEST_H

/*
 * DlaBoard_B 接口自测固件入口（仅在 -DSELFTEST_BUILD 时由 main 调用）。
 * 经 USART1（USB1，115200 8N1）提供文本 CLI；命令见 selftest.c 顶部说明。
 * 阶段 1：板载/无需跳线的测试（IMU/EEPROM/ADC/CAN 环回/LED/蜂鸣器）。
 */
void SelfTest_Run(void);   /* 不返回：初始化测试外设后进入 CLI 主循环 */

#endif /* SELFTEST_H */
