      WheelSpeed_Task();
      IMU_Task();   // LSM6DSVETR 轮询(内部限频~100Hz)，更新 g_imu

      // 连接状态：心跳 + 超时 判断
      {
          static uint8_t was_disconnected = 0;
          static uint8_t was_connected = 0;
          static uint32_t last_heartbeat_ms = 0;
          static uint32_t last_refresh_ms = 0;
          uint32_t now = HAL_GetTick();
          if (now - Screen_GetLastRxMs() > SCREEN_TIMEOUT_MS) {
              was_connected = 0;
              if (!was_disconnected) {
                  Screen_SetConnectionStatus(0);
                  was_disconnected = 1;
              }
          } else {
              was_disconnected = 0;
              if (!was_connected) {
                  Screen_SetConnectionStatus(1);
                  was_connected = 1;
              }
              // 心跳：MCU主动发0x83读变量，屏回复即刷新last_rx，不依赖触摸
              if (now - last_heartbeat_ms >= SCREEN_HEARTBEAT_INTERVAL_MS) {
                  last_heartbeat_ms = now;
                  Screen_ReadVar(SCREEN_ADDR_CONNECTION, 1);
              }
              // 每秒刷新档位/速度/模式/连接状态到屏
              if (now - last_refresh_ms >= 1000) {
                  last_refresh_ms = now;
                  Screen_SendData(SCREEN_ADDR_GEAR, current_gear);
                  Screen_SendData(SCREEN_ADDR_SPEED, current_speed_level);
                  Screen_SendData(SCREEN_ADDR_MODE,
                      g_steering_mode == MODE_MANUAL   ? SCREEN_KEY_MODE_MANUAL :
                      g_steering_mode == MODE_FOLLOW   ? SCREEN_KEY_MODE_FOLLOW :
                      g_steering_mode == MODE_TRAINING ? SCREEN_KEY_MODE_TRAINING :
                      g_steering_mode == MODE_COLLECT  ? SCREEN_KEY_MODE_COLLECT :
                      g_steering_mode == MODE_AUTO     ? SCREEN_KEY_MODE_AUTO :
                                                         SCREEN_KEY_MODE_REMOTE);
                  Screen_SetConnectionStatus(1);
                  Screen_SetEnableIndicator(g_dbg_ven);  // 使能指示 0x0608("R"/空格)，重连后刷新
              }
          }
      }

      // 轮速显示 0x02E5：传整型 km/h。若屏会除以10显示，则发 speed_01(如250=25km/h)
      {
          static uint32_t last_speed_display_ms = 0;
          uint32_t t = HAL_GetTick();
          if (t - last_speed_display_ms >= 200) {
              last_speed_display_ms = t;
              uint16_t speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, t, WheelSpeed_GetSpeed_01kmh());  // 0.1 km/h
              uint16_t speed_kmh = (speed_01 + 5) / 10;        // 四舍五入整 km/h
              if (speed_kmh > 999) speed_kmh = 999;
              Screen_SendData(SCREEN_ADDR_SPEED_DISPLAY, speed_kmh);  // 屏 int 0小数
          }
      }

      // 电量显示 0x05EB：0=100% 1=75% 2=50% 3=25% 4=空
      {
          static uint32_t last_battery_ms = 0;
          uint32_t t = HAL_GetTick();
          if (t - last_battery_ms >= 1000) {
              last_battery_ms = t;
              uint8_t bat_level = Battery_GetLevel();
              Screen_SendData(SCREEN_ADDR_BATTERY, (uint16_t)bat_level);
          }
      }

      // VBUS 母线电压/电流显示：数据来自 FOC 0x212 遥测。屏地址待定(screen.h 中为 0)，
      // 填入实际地址前按 !=0 守卫不发屏。电压 0.1V/字，电流 0.1A/字(含符号)。
      {
          static uint32_t last_vbus_ms = 0;
          uint32_t t = HAL_GetTick();
          if (t - last_vbus_ms >= 200) {
              last_vbus_ms = t;
              if (SCREEN_ADDR_VBUS_VOLT != 0x0000) {
                  uint16_t volt_01 = (uint16_t)(FocCAN_GetVbus_mV(&g_foc_can, t) / 100U); // mV→0.1V
                  Screen_SendData(SCREEN_ADDR_VBUS_VOLT, volt_01);
              }
              if (SCREEN_ADDR_VBUS_CURR != 0x0000) {
                  int16_t curr_01 = FocCAN_GetIbus_cA(&g_foc_can, t) / 10;             // cA→0.1A
                  Screen_SendData(SCREEN_ADDR_VBUS_CURR, (uint16_t)curr_01);
              }
          }
      }

      uint16_t key_value = 0, key_addr = 0;
      Screen_GetKeyAndAddr(&key_value, &key_addr);
      if (key_value != 0) {
          // 按来源地址区分：档位 0x05E1 / 模式 0x05A1 / 速度 0x05C1，避免 P档(0x0008) 与 遥控(0x0008) 冲突
          if (key_addr == SCREEN_ADDR_MODE) {
              if (key_value == SCREEN_KEY_MODE_MANUAL) {
                  g_steering_mode = MODE_MANUAL;
                  servo_enabled_sent = 0;
                  need_return_home = 1;
                  {
                      TrainFrameData_t tf = {
                          .seq = TrainUART_GetNextSeq(),
                          .ts_ms = HAL_GetTick(),
                          .speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, HAL_GetTick(), WheelSpeed_GetSpeed_01kmh()),
                          .steer_01 = 0, .thr_ref = 0, .brake = 1,
                          .gear = (current_gear == SCREEN_KEY_GEAR_D) ? 3 : (current_gear == SCREEN_KEY_GEAR_R) ? 1 : (current_gear == SCREEN_KEY_GEAR_N) ? 2 : 0
                      };
                      TrainUART_SendManualNotify(&tf);
                  }
              } else if (key_value == SCREEN_KEY_MODE_REMOTE) {
                  if (g_steering_mode != MODE_REMOTE && g_steering_mode != MODE_TRAINING && g_steering_mode != MODE_FOLLOW) {
                      servo_enabled_sent = 0;
                  }
                  g_steering_mode = MODE_REMOTE;
                  auto_state = AUTO_STATE_IDLE;
                  last_target_position = 0x7FFFFFFF;
                  {
                      TrainFrameData_t tf = {
                          .seq = TrainUART_GetNextSeq(),
                          .ts_ms = HAL_GetTick(),
                          .speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, HAL_GetTick(), WheelSpeed_GetSpeed_01kmh()),
                          .steer_01 = 0, .thr_ref = 0, .brake = 1,
                          .gear = (current_gear == SCREEN_KEY_GEAR_D) ? 3 : (current_gear == SCREEN_KEY_GEAR_R) ? 1 : (current_gear == SCREEN_KEY_GEAR_N) ? 2 : 0
                      };
                      TrainUART_SendRemoteNotify(&tf);
                  }
              } else if (key_value == SCREEN_KEY_MODE_TRAINING) {
                  if (g_steering_mode != MODE_REMOTE && g_steering_mode != MODE_TRAINING && g_steering_mode != MODE_FOLLOW) {
                      servo_enabled_sent = 0;
                  }
                  g_steering_mode = MODE_TRAINING;
                  auto_state = AUTO_STATE_IDLE;
                  last_target_position = 0x7FFFFFFF;
                  /* 训练流为 TYPE_CTRL(0x01) 周期帧;不再在此发 0x03——0x03 现专属
                   * 自动驾驶就绪/状态(见 AUTONOMOUS_MODE_SPEC §4)。 */
              } else if (key_value == SCREEN_KEY_MODE_COLLECT) {
                  /* 采集(SLAM)：可遥控驾驶(伺服跟随 RC 转向，同遥控)，20ms 上报
                   * TYPE_COLLECT 帧 + IMU 行。从 伺服未使能的模式(手动/训练)进入时复位
                   * servo_enabled_sent 以触发伺服使能序列。单向数据流，无需 Jetson 握手帧。 */
                  if (g_steering_mode != MODE_REMOTE && g_steering_mode != MODE_FOLLOW &&
                      g_steering_mode != MODE_COLLECT) {
                      servo_enabled_sent = 0;
                  }
                  g_steering_mode = MODE_COLLECT;
                  auto_state = AUTO_STATE_IDLE;
                  last_target_position = 0x7FFFFFFF;
              } else if (key_value == SCREEN_KEY_MODE_FOLLOW) {
                  servo_enabled_sent = 0;  // 进入跟随时重新初始化伺服
                  g_steering_mode = MODE_FOLLOW;
                  auto_state = AUTO_STATE_IDLE;
                  last_target_position = 0x7FFFFFFF;
                  // 跟随模式入口：清空控制量，等待第一条有效指令
                  s_follow_cmd.brake = 1;
                  s_follow_cmd.thr_ref = 0;
                  s_follow_cmd.steer_01 = 0;
                  last_follow_cmd_ms = HAL_GetTick();
              } else if (key_value == SCREEN_KEY_MODE_AUTO) {
                  servo_enabled_sent = 0;  // 进入自动驾驶时重新初始化伺服
                  g_steering_mode = MODE_AUTO;
                  auto_state = AUTO_STATE_IDLE;
                  last_target_position = 0x7FFFFFFF;
                  // 自动模式入口：安全态，等握手(armed)与第一条有效 0x11
                  s_auto_cmd.brake = 1;
                  s_auto_cmd.target_speed_mm_s = 0;
                  s_auto_cmd.steer_01 = 0;
                  s_auto_cmd.flags = 0;
                  last_auto_cmd_ms = HAL_GetTick();
                  auto_ready_sent = 0;       // 进入立即发 1 帧就绪
                  last_auto_status_ms = 0;
                  s_auto_speed_i = 0;
                  s_auto_throttle_permille = 0;
                  s_auto_dir = 0;
              }
          } else if (key_addr == SCREEN_ADDR_GEAR) {
              if (key_value == SCREEN_KEY_GEAR_P || key_value == SCREEN_KEY_GEAR_R ||
                  key_value == SCREEN_KEY_GEAR_N || key_value == SCREEN_KEY_GEAR_D) {
                  current_gear = key_value;
              }
          } else if (key_addr == SCREEN_ADDR_SPEED) {
              if (key_value == SCREEN_KEY_SPEED_EXTRA_LOW || key_value == SCREEN_KEY_SPEED_LOW ||
                  key_value == SCREEN_KEY_SPEED_MEDIUM || key_value == SCREEN_KEY_SPEED_HIGH) {
                  current_speed_level = key_value;
              }
          }
      }

      // 跟随模式：轮询上位机指令、超时保护、控制量限幅
      if (g_steering_mode == MODE_FOLLOW) {
          if (TrainUART_PollFollowCmd(&s_follow_cmd)) {
              last_follow_cmd_ms = HAL_GetTick();
              follow_ready_sent = 1;  // 收到有效 0x10，停止重发 ready
          }
          if (HAL_GetTick() - last_follow_cmd_ms > FOLLOW_CMD_TIMEOUT_MS) {
              s_follow_cmd.brake = 1;
              s_follow_cmd.thr_ref = 0;
              s_follow_cmd.steer_01 = 0;
          }
          if (s_follow_cmd.steer_01 > MAX_STEER_01) s_follow_cmd.steer_01 = MAX_STEER_01;
          if (s_follow_cmd.steer_01 < -MAX_STEER_01) s_follow_cmd.steer_01 = -MAX_STEER_01;
          if (s_follow_cmd.thr_ref > 1000) s_follow_cmd.thr_ref = 1000;
      }

      // 自动驾驶模式：轮询 0x11、看门狗失效保护、转角/硬限速 clamp(见 AUTONOMOUS_MODE_SPEC §3/§6)
      if (g_steering_mode == MODE_AUTO) {
          if (TrainUART_PollAutoCmd(&s_auto_cmd)) {
              last_auto_cmd_ms = HAL_GetTick();
          }
          // §6.1 指令看门狗：超过 AUTO_CMD_TIMEOUT_MS 未收到有效 0x11 → 安全态
          if (HAL_GetTick() - last_auto_cmd_ms > AUTO_CMD_TIMEOUT_MS) {
              s_auto_cmd.brake = 1;
              s_auto_cmd.target_speed_mm_s = 0;
              s_auto_cmd.steer_01 = 0;
              s_auto_cmd.flags = 0;
          }
          // 转角限幅
          if (s_auto_cmd.steer_01 > MAX_STEER_01) s_auto_cmd.steer_01 = MAX_STEER_01;
          if (s_auto_cmd.steer_01 < -MAX_STEER_01) s_auto_cmd.steer_01 = -MAX_STEER_01;
          // §6.2 硬限速：独立于 Jetson 再 clamp(双保险)
          if (s_auto_cmd.target_speed_mm_s > AUTO_MAX_SPEED_MM_S) s_auto_cmd.target_speed_mm_s = AUTO_MAX_SPEED_MM_S;
          if (s_auto_cmd.target_speed_mm_s < -AUTO_MAX_SPEED_MM_S) s_auto_cmd.target_speed_mm_s = -AUTO_MAX_SPEED_MM_S;
      }

      // 档位 → Stop(PB14)；Revert(PB13) 由电机块单独控制，档位块不设置
      // JQ50V300: 停止接GND有效；反转接GND=反转。D/R档时 Revert 由摇杆/踏板决定
      {
          if (current_gear == SCREEN_KEY_GEAR_P) {
              Board_Relay_MotorStop( GPIO_PIN_SET);
              Board_Relay_MotorRevert( GPIO_PIN_SET);
          } else if (current_gear == SCREEN_KEY_GEAR_N) {
              Board_Relay_MotorStop( GPIO_PIN_RESET);        // 停止
              Board_Relay_MotorRevert( GPIO_PIN_SET);
          } else {
              Board_Relay_MotorStop( GPIO_PIN_SET);
              // Revert 不在此设置，由下方电机块根据摇杆/踏板设置
          }
      }

      /*
       * 刹车板 DlaCart_v2.1 (CAN 0x220) 已纳入正式自动刹车：见下方电机块附近的
       * BrakeCAN_Send(foc_brake)（与 FOC 刹车同源：P档/刹车开关/跟随brake/撤防）。
       * UART "BRAKE 1/0/OFF" 仍作台架测试旁路，测试保持期间正式刹车自动让位。
       */

      /* PRND：四路档位开关输入（board_inputs.h：P/R/N/D 引脚）；模式仅来自串口屏 0x05A1，无模式 GPIO */
      #define GEAR_DEBOUNCE_MS  50
      #define GEAR_BOOT_IGNORE_MS  500
      {
          static uint8_t prev_P = 1, prev_D = 1, prev_R = 1, prev_N = 1;
          static uint32_t press_start_P = 0, press_start_D = 0, press_start_R = 0, press_start_N = 0;
          static uint8_t triggered_P = 0, triggered_D = 0, triggered_R = 0, triggered_N = 0;
          uint32_t now = HAL_GetTick();
          uint8_t raw_P = HAL_GPIO_ReadPin(GEAR_SW_GPIO_Port, GEAR_P_Pin);
          uint8_t raw_R = HAL_GPIO_ReadPin(GEAR_SW_GPIO_Port, GEAR_R_Pin);
          uint8_t raw_N = HAL_GPIO_ReadPin(GEAR_SW_GPIO_Port, GEAR_N_Pin);
          uint8_t raw_D = HAL_GPIO_ReadPin(GEAR_SW_GPIO_Port, GEAR_D_Pin);

          if (now > GEAR_BOOT_IGNORE_MS) {
              if (raw_P == GPIO_PIN_RESET) {
                  if (prev_P == GPIO_PIN_SET) press_start_P = now;
                  if (!triggered_P && (now - press_start_P) >= GEAR_DEBOUNCE_MS) {
                      current_gear = SCREEN_KEY_GEAR_R;
                      Screen_SetGear(SCREEN_KEY_GEAR_R);
                      triggered_P = 1;
                  }
              } else {
                  triggered_P = 0;
              }
              prev_P = raw_P;

              if (raw_D == GPIO_PIN_RESET) {
                  if (prev_D == GPIO_PIN_SET) press_start_D = now;
                  if (!triggered_D && (now - press_start_D) >= GEAR_DEBOUNCE_MS) {
                      current_gear = SCREEN_KEY_GEAR_P;
                      Screen_SetGear(SCREEN_KEY_GEAR_P);
                      triggered_D = 1;
                  }
              } else {
                  triggered_D = 0;
              }
              prev_D = raw_D;

              if (raw_R == GPIO_PIN_RESET) {
                  if (prev_R == GPIO_PIN_SET) press_start_R = now;
                  if (!triggered_R && (now - press_start_R) >= GEAR_DEBOUNCE_MS) {
                      current_gear = SCREEN_KEY_GEAR_D;
                      Screen_SetGear(SCREEN_KEY_GEAR_D);
                      triggered_R = 1;
                  }
              } else {
                  triggered_R = 0;
              }
              prev_R = raw_R;

              if (raw_N == GPIO_PIN_RESET) {
                  if (prev_N == GPIO_PIN_SET) press_start_N = now;
                  if (!triggered_N && (now - press_start_N) >= GEAR_DEBOUNCE_MS) {
                      current_gear = SCREEN_KEY_GEAR_P;
                      Screen_SetGear(SCREEN_KEY_GEAR_P);
                      triggered_N = 1;
                  }
              } else {
                  triggered_N = 0;
              }
              prev_N = raw_N;
          }
      }

      // CAN 伺服：遥控/训练/采集/跟随模式下发心跳（训练/采集不使能电机，仅读位置反馈）
      if ((g_steering_mode == MODE_REMOTE || g_steering_mode == MODE_TRAINING ||
           g_steering_mode == MODE_COLLECT || g_steering_mode == MODE_FOLLOW ||
           g_steering_mode == MODE_AUTO) && HAL_GetTick() - last_link_ms >= 50) {
          ServoCAN_SendLink(&g_servo);
          last_link_ms = HAL_GetTick();
      }

      // 处理CAN接收消息：排空 FIFO0，逐帧分发（伺服 0x64+id / FOC2 状态 0x211 / 母线遥测 0x212）
      // 单帧 if 会在多源同时到达时丢帧(车速/状态滞后)，故用 while 排空。
      {
          CAN_RxHeaderTypeDef rxHeader;
          uint8_t rxData[8];
          while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
              if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
                  break;
              }
              ServoCAN_OnRxFrame(&g_servo, rxHeader.StdId, rxData);
              FocCAN_OnRxFrame(&g_foc_can, rxHeader.StdId, rxData);
          }
      }

      /* FOC2 行走请求：控制板接入时，手动踏板/遥控 RC/跟随指令统一转换为 CAN 0x210。 */
      int16_t motor_pwm_us;
      #define THROTTLE_ADC_DEAD  850
      #define THROTTLE_ADC_FULL  3500
      #define STICK_ACTIVE_THRESH  80   // 摇杆偏离中位超过此值认为有输入
      int16_t pedal_pwm = 1500;
      if (current_gear == SCREEN_KEY_GEAR_D || current_gear == SCREEN_KEY_GEAR_R) {
          uint32_t adc = HAL_ADC_GetValue(&hadc1);
          if (adc > 4095) adc = 4095;
          if (adc >= THROTTLE_ADC_DEAD) {
              uint32_t range = THROTTLE_ADC_FULL - THROTTLE_ADC_DEAD;
              if (adc > THROTTLE_ADC_FULL) adc = THROTTLE_ADC_FULL;
              uint32_t offset = ((adc - THROTTLE_ADC_DEAD) * 500) / range;
              pedal_pwm = (current_gear == SCREEN_KEY_GEAR_D) ? (1500 + (int16_t)offset) : (1500 - (int16_t)offset);
          }
      }
      if (g_steering_mode == MODE_MANUAL) {
          motor_pwm_us = pedal_pwm;
      } else if (g_steering_mode == MODE_FOLLOW) {
          motor_pwm_us = 1500;  // 默认中位，由下方跟随逻辑覆盖
      } else if (g_steering_mode == MODE_AUTO) {
          motor_pwm_us = 1500;  // 自动模式不走踏板/摇杆，由下方速度闭环覆盖 foc_throttle
      } else if (g_steering_mode == MODE_REMOTE) {
          motor_pwm_us = RC_GetPulseWidth_CH2_us();
      } else {
          int16_t stick_pwm = RC_GetPulseWidth_CH2_us();
          if (current_gear != SCREEN_KEY_GEAR_D && current_gear != SCREEN_KEY_GEAR_R) {
              motor_pwm_us = stick_pwm;  // P/N 档仅摇杆
          } else {
              int16_t s_off = stick_pwm - 1500;
              if (s_off > STICK_ACTIVE_THRESH || s_off < -STICK_ACTIVE_THRESH) {
                  motor_pwm_us = stick_pwm;  // 摇杆有输入：优先摇杆，向前=前进/向后=后退
              } else {
                  motor_pwm_us = pedal_pwm;  // 摇杆中立：用踏板，方向由档位决定
              }
          }
      }

      if (g_steering_mode == MODE_FOLLOW) {
          if (s_follow_cmd.brake) {
              motor_pwm_us = 1500;
          } else {
              uint16_t thr = s_follow_cmd.thr_ref;
              if (thr > 1000) thr = 1000;
              motor_pwm_us = 1500 + (int16_t)(thr * 500 / 1000);
          }
      }

      // 是否允许行走：P 档禁用，N 档保持 FOC ready/零目标，D/R 档按控制量输出。
      bool allow_motor = (current_gear == SCREEN_KEY_GEAR_D || current_gear == SCREEN_KEY_GEAR_R);
      uint8_t vehicle_enabled = Board_VehicleEnableRead();
      allow_motor = (allow_motor && (vehicle_enabled != 0U)) ? true : false;
      /* 统一刹车意图(foc_brake)：跟随模式取上位机 brake；否则 刹车开关(PE2/PE3 低=按下)
       * 或 P 档 或 撤防 任一成立即刹车。FOC 电机(0x210)与物理刹车板(0x220)同源于此。 */
      uint8_t brake_sw = ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET ||
                           HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)) ? 1U : 0U;
      /* RC3 远程刹车：单通道两位开关(~1000/~2000us)直接表示刹车/释放，开关位置本身即保持
       * 状态，无需软件自锁。RC3 处于刹车位时在所有模式下(含跟随)都强制刹车。 */
      g_rc_brake = RC_GetSwitch_RC3();
      /* 自动驾驶 arm 判定(§5)：屏选 AUTO + FOC 状态新鲜 + 整车使能 + D 档 + Jetson 心跳
       * (0x11 新鲜) + Jetson 请求使能(FLAGS.AUTO_ENABLE) + 无急停/接管。任一不满足→不 arm
       * →foc_brake 强制刹车保持安全态。急停/接管(遥控刹车/刹车开关)优先级最高。 */
      uint8_t auto_foc_valid = FocCAN_StatusFresh(&g_foc_can, HAL_GetTick());
      uint8_t auto_cmd_fresh = ((HAL_GetTick() - last_auto_cmd_ms) <= AUTO_CMD_TIMEOUT_MS) ? 1U : 0U;
      uint8_t auto_override = (g_rc_brake || brake_sw) ? 1U : 0U;
      uint8_t auto_armed = 0U;
      if (g_steering_mode == MODE_AUTO) {
          auto_armed = (auto_foc_valid && (vehicle_enabled != 0U) &&
                        (current_gear == SCREEN_KEY_GEAR_D) && auto_cmd_fresh &&
                        ((s_auto_cmd.flags & TRAIN_AUTO_FLAG_ENABLE) != 0U) &&
                        (auto_override == 0U)) ? 1U : 0U;
      }
      uint8_t foc_brake = (g_steering_mode == MODE_FOLLOW) ? s_follow_cmd.brake
                          : (g_steering_mode == MODE_AUTO) ? ((s_auto_cmd.brake || (auto_armed == 0U)) ? 1U : 0U)
                          : ((brake_sw || (vehicle_enabled == 0U) || (current_gear == SCREEN_KEY_GEAR_P)) ? 1U : 0U);
      if (g_rc_brake) foc_brake = 1U;   /* 远程遥控刹车(RC3)在任意模式下强制刹车 */
      /* 联调诊断捕获 */
      if (foc_brake != g_dbg_fbrake) { g_dbg_fbrake_chg++; }
      g_dbg_gear = current_gear; g_dbg_bsw = brake_sw;
      g_dbg_ven = vehicle_enabled; g_dbg_fbrake = foc_brake;

      /* 整车使能文本指示：使能→屏 0x0608 显示 "R"，未使能→空格。状态变化即写。 */
      {
          static uint8_t s_last_en_ind = 0xFFu;
          if (vehicle_enabled != s_last_en_ind) {
              s_last_en_ind = vehicle_enabled;
              Screen_SetEnableIndicator(vehicle_enabled);
          }
      }
      uint8_t foc_allow_drive = (vehicle_enabled != 0U) ? 1U : 0U;
      uint8_t foc_gear_proto = 0U; /* 0=N 1=F 2=R */
      uint16_t foc_throttle_permille = 0U;
      int16_t foc_steer_permille = 0;

      if (!allow_motor || (foc_brake != 0U)) {
          motor_pwm_us = 1500;
      } else {
          int16_t offset = motor_pwm_us - 1500;
          int16_t foc_offset = (int16_t)((float)offset * Screen_GetSpeedScale(current_speed_level));

          if (foc_offset > 500) foc_offset = 500;
          if (foc_offset < -500) foc_offset = -500;

          #define FOC_DRIVE_DEAD_US  50
          if (offset > FOC_DRIVE_DEAD_US) {
              foc_gear_proto = 1U;
              foc_throttle_permille = (uint16_t)((int32_t)foc_offset * 1000L / 500L);
          } else if (offset < -FOC_DRIVE_DEAD_US) {
              foc_gear_proto = 2U;
              foc_throttle_permille = (uint16_t)((int32_t)(-foc_offset) * 1000L / 500L);
          } else {
              foc_gear_proto = 0U;
              foc_throttle_permille = 0U;
          }
      }

      foc_steer_permille = 0;

      /* 自动驾驶速度闭环(§1)：把 TARGET_SPEED(mm/s) 转成 throttle_permille。速度环在本板,
       * CAN 0x210 仍开环油门(FOC_double 不改)。20ms 定步长跑整数 PI;方向取 target 符号
       * (支持后退:D 档下负速→FOC gear=R)。未 arm/刹车/零速→输出 0 并清积分。 */
      if (g_steering_mode == MODE_AUTO) {
          static uint32_t last_auto_pi_ms = 0;
          uint32_t pi_now = HAL_GetTick();
          if (pi_now - last_auto_pi_ms >= FOC_CAN_TX_PERIOD_MS) {
              last_auto_pi_ms = pi_now;
              if (!allow_motor || (foc_brake != 0U) || (s_auto_cmd.target_speed_mm_s == 0)) {
                  s_auto_throttle_permille = 0U;
                  s_auto_dir = 0U;
                  s_auto_speed_i = 0;
              } else {
                  int16_t tgt = s_auto_cmd.target_speed_mm_s;
                  s_auto_dir = (tgt > 0) ? 1U : 2U;
                  int32_t tgt_abs = (tgt > 0) ? tgt : -tgt;
                  uint16_t meas_01kmh = FocCAN_GetSpeed_01kmh(&g_foc_can, pi_now, WheelSpeed_GetSpeed_01kmh());
                  int32_t meas_mm_s = (int32_t)meas_01kmh * 1000L / 36L;  /* 0.1km/h→mm/s */
                  int32_t err = tgt_abs - meas_mm_s;
                  s_auto_speed_i += err;
                  if (s_auto_speed_i > AUTO_SPD_I_MAX) s_auto_speed_i = AUTO_SPD_I_MAX;
                  if (s_auto_speed_i < -AUTO_SPD_I_MAX) s_auto_speed_i = -AUTO_SPD_I_MAX;
                  int32_t out = (err * AUTO_SPD_KP_NUM) / AUTO_SPD_KP_DEN
                              + (s_auto_speed_i * AUTO_SPD_KI_NUM) / AUTO_SPD_KI_DEN;
                  if (out < 0) out = 0;
                  if (out > (int32_t)AUTO_MAX_THROTTLE_PERMILLE) out = (int32_t)AUTO_MAX_THROTTLE_PERMILLE;
                  s_auto_throttle_permille = (uint16_t)out;
              }
          }
          /* 每次迭代用最新 PI 输出(20ms 刷新)覆盖 FOC 行走请求 */
          foc_throttle_permille = s_auto_throttle_permille;
          foc_gear_proto = s_auto_dir;
      }

      // 行走电机正式改由 FOC2 CAN 执行，本板旧 PWM/Revert 路线保持停机。
      Board_Relay_MotorRevert(GPIO_PIN_SET);
      Motor_Control_Stop();

      /* FOC2 板：周期下发正式驾驶请求（CAN ID 0x210） */
      {
          static uint32_t last_foc_tx_ms = 0;
          uint32_t foc_now = HAL_GetTick();
          /* 修复(2026-06-02)：AUTO/FOLLOW 模式 Jetson 下发二进制帧(0x11/0x10)，
           * 而本行消费者(CLI / 采集MAP-SAVE)会从同一环形缓冲偷走帧字节 →0x11/0x10
           * 解析失败→auto_cmd_fresh 闪断→arm 掉→刹车点动。这两种模式下让二进制
           * poll(PollAutoCmd/PollFollowCmd)独占串口；CLI/MAP-SAVE 本就不在这两模式用。 */
          if (g_steering_mode != MODE_AUTO && g_steering_mode != MODE_FOLLOW) {
              FocCAN_TestPollUart(&g_foc_can);
          }
          if (foc_now - last_foc_tx_ms >= FOC_CAN_TX_PERIOD_MS) {
              last_foc_tx_ms = foc_now;
              if (FocCAN_TestActive() != 0U) {
                  uint16_t test_thr_permille;
                  int16_t test_steer_permille;
                  uint8_t test_allow;
                  uint8_t test_brake;
                  uint8_t test_gear;
                  FocCAN_TestGet(&test_thr_permille, &test_steer_permille,
                                 &test_allow, &test_brake, &test_gear);
                  (void)FocCAN_SendDriveRequest(&g_foc_can, test_thr_permille,
                                                test_steer_permille,
                                                (vehicle_enabled != 0U) ? test_allow : 0U,
                                                (vehicle_enabled != 0U) ? test_brake : 1U,
                                                test_gear,
                                                FocCAN_TestTimeout10ms());
              } else {
                  (void)FocCAN_SendDriveRequest(&g_foc_can, foc_throttle_permille,
                                                foc_steer_permille,
                                                foc_allow_drive, foc_brake,
                                                foc_gear_proto,
                                                FOC_CAN_DEFAULT_TIMEOUT_10MS);
              }
          }
      }

      /* 刹车已迁移到 FOC 板(板载 H 桥驱动刹车电缸)，刹车意图 foc_brake 已随
       * 0x210 flags bit1 发给 FOC 板执行(见上 FocCAN_SendDriveRequest)。
       * 独立刹车板已移除，不再发 0x220。原代码保留备查:
       * if (FocCAN_BrakeTestActive() == 0U) { BrakeCAN_Send(foc_brake); } */

      /* 转向控制 */
      {
          static uint8_t manual_disable_sent = 0;
          static uint8_t training_disable_sent = 0;
          switch (g_steering_mode) {
          case MODE_MANUAL:
              training_disable_sent = 0;
              follow_ready_sent = 0;
              last_ready_send_ms = 0;
              if (!manual_disable_sent) {
                  ServoCAN_Enable(&g_servo, 0);
                  manual_disable_sent = 1;
              }
              break;

          case MODE_TRAINING: {
              follow_ready_sent = 0;
              last_ready_send_ms = 0;
              // 训练模式：不使能伺服电机，仅通过 CAN 接收位置反馈用于读取转向角(人工手动转向)
              manual_disable_sent = 0;
              if (!training_disable_sent) {
                  ServoCAN_Enable(&g_servo, 0);
                  training_disable_sent = 1;
              }
              // 首次收到伺服心跳时，以当前位置为“中位”（本训练会话的零点）
              if (!home_position_valid && g_servo.heartbeat_seen) {
                  home_position = g_servo.abs_pos;
                  home_position_valid = 1;
              }
              break;
          }

          case MODE_COLLECT:   /* 采集(SLAM)：可遥控驾驶——伺服使能并跟随 RC 转向(同遥控)，
                                * 油门支持 RC 摇杆/踏板；同时 20ms 上报 TYPE_COLLECT 帧 + IMU 行。 */
          case MODE_REMOTE: {
              training_disable_sent = 0;
              follow_ready_sent = 0;
              last_ready_send_ms = 0;
              static uint32_t last_remote_enable_ms = 0;
              static uint8_t returning_to_home = 0;
              static uint8_t servo_init_phase = 0;
              manual_disable_sent = 0;
              uint32_t now = HAL_GetTick();
              if (!servo_enabled_sent) {
                  if (servo_init_phase == 3) servo_init_phase = 0;
                  if (servo_init_phase == 0) {
                      ServoCAN_Enable(&g_servo, 1);
                      last_remote_enable_ms = now;
                      servo_init_phase = 1;
                  } else if (servo_init_phase == 1 && (now - last_remote_enable_ms >= 20)) {
                      ServoCAN_SetMaxSpeedForPositioning(&g_servo, AUTO_ROTATION_SPEED);
                      last_remote_enable_ms = now;
                      servo_init_phase = 2;
                  } else if (servo_init_phase == 2 && (now - last_remote_enable_ms >= 50)) {
                      if (home_position_valid) {
                          returning_to_home = 1;
                      } else {
                          home_position = g_servo.abs_pos;
                          home_position_valid = 1;
                      }
                      last_target_position = 0x7FFFFFFF;
                      servo_enabled_sent = 1;
                      servo_init_phase = 3;
                  }
              } else if (now - last_remote_enable_ms >= 200) {
                  ServoCAN_Enable(&g_servo, 1);
                  last_remote_enable_ms = now;
              }

              int32_t target_position;
              if (returning_to_home) {
                  target_position = home_position;
                  int32_t err = g_servo.abs_pos - home_position;
                  if (err < 0) err = -err;
                  if (err <= POSITION_TOLERANCE)
                      returning_to_home = 0;
              } else {
                  int16_t pwm_us = RC_GetPulseWidth_us();
                  #define STEER_DEADZONE_US  25
                  int16_t centered = pwm_us;
                  if (pwm_us > 1500 - STEER_DEADZONE_US && pwm_us < 1500 + STEER_DEADZONE_US)
                      centered = 1500;
                  int32_t offset = -((int32_t)(centered - 1500) * STEERING_POSITION_RANGE) / 500;
                  target_position = home_position + offset;
                  if (target_position < home_position - STEERING_POSITION_RANGE)
                      target_position = home_position - STEERING_POSITION_RANGE;
                  if (target_position > home_position + STEERING_POSITION_RANGE)
                      target_position = home_position + STEERING_POSITION_RANGE;
              }

              int32_t pos_diff = target_position - last_target_position;
              if (pos_diff < 0) pos_diff = -pos_diff;
              int32_t err_to_target = g_servo.abs_pos - target_position;
              if (err_to_target < 0) err_to_target = -err_to_target;
              int16_t pwm_us = RC_GetPulseWidth_us();
              int8_t stick_centered = (pwm_us >= 1475 && pwm_us <= 1525);
              if (stick_centered && err_to_target <= POSITION_TOLERANCE)
                  ;
              else if (pos_diff > 500 || last_target_position == 0x7FFFFFFF) {
                  ServoCAN_GoToAbsolutePosition(&g_servo, target_position);
                  last_target_position = target_position;
              }
              break;
          }

          case MODE_FOLLOW: {
              // 跟随模式：伺服使能，目标由上位机指令控制
              training_disable_sent = 0;
              static uint32_t last_follow_enable_ms = 0;
              static uint8_t follow_init_phase = 0;
              manual_disable_sent = 0;
              uint32_t now = HAL_GetTick();
              if (!servo_enabled_sent) {
                  if (follow_init_phase == 3) follow_init_phase = 0;
                  if (follow_init_phase == 0) {
                      ServoCAN_Enable(&g_servo, 1);
                      last_follow_enable_ms = now;
                      follow_init_phase = 1;
                  } else if (follow_init_phase == 1 && (now - last_follow_enable_ms >= 20)) {
                      ServoCAN_SetMaxSpeedForPositioning(&g_servo, AUTO_ROTATION_SPEED);
                      last_follow_enable_ms = now;
                      follow_init_phase = 2;
                  } else if (follow_init_phase == 2 && (now - last_follow_enable_ms >= 50)) {
                      if (!home_position_valid && g_servo.heartbeat_seen) {
                          home_position = g_servo.abs_pos;
                          home_position_valid = 1;
                      }
                      last_target_position = 0x7FFFFFFF;
                      servo_enabled_sent = 1;
                      follow_init_phase = 3;
                  }
              } else if (now - last_follow_enable_ms >= 200) {
                  ServoCAN_Enable(&g_servo, 1);
                  last_follow_enable_ms = now;
              }
              // 未收到有效 0x10 前：进入时立刻发 1 帧，之后每 100ms 重发 TYPE=0x02
              if (!follow_ready_sent && (last_ready_send_ms == 0 || (now - last_ready_send_ms >= FOLLOW_READY_RESEND_MS))) {
                  TrainFrameData_t tf = {
                      .seq = TrainUART_GetNextSeq(),
                      .ts_ms = now,
                      .speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, now, WheelSpeed_GetSpeed_01kmh()),
                      .steer_01 = 0,
                      .thr_ref = 0,
                      .brake = 1,
                      .gear = (current_gear == SCREEN_KEY_GEAR_D) ? 3 : (current_gear == SCREEN_KEY_GEAR_R) ? 1 : 0
                  };
                  TrainUART_SendFollowReady(&tf);
                  last_ready_send_ms = now;
              }
              // 应用上位机转向指令（s_follow_cmd 已在电机块中轮询更新）
              if (home_position_valid) {
                  int32_t half = (int32_t)STEERING_POSITION_RANGE / 2;
                  if (half <= 0) half = 1;
                  int32_t target_position = home_position - (int32_t)s_follow_cmd.steer_01 * half / (10 * STEERING_PHYSICAL_DEG);
                  if (target_position < home_position - STEERING_POSITION_RANGE)
                      target_position = home_position - STEERING_POSITION_RANGE;
                  if (target_position > home_position + STEERING_POSITION_RANGE)
                      target_position = home_position + STEERING_POSITION_RANGE;
                  int32_t pos_diff = target_position - last_target_position;
                  if (pos_diff < 0) pos_diff = -pos_diff;
                  if (pos_diff > 300 || last_target_position == 0x7FFFFFFF) {
                      ServoCAN_GoToAbsolutePosition(&g_servo, target_position);
                      last_target_position = target_position;
                  }
              }
              break;
          }

          case MODE_AUTO: {
              // 自动驾驶：伺服使能、目标由 0x11 指令(s_auto_cmd.steer_01)控制(伺服序列同跟随)。
              // 速度闭环 + 看门狗 + arm 判定在电机块/轮询块中处理;此处只管转向伺服。
              training_disable_sent = 0;
              follow_ready_sent = 0;
              last_ready_send_ms = 0;
              static uint32_t last_auto_enable_ms = 0;
              static uint8_t auto_init_phase = 0;
              manual_disable_sent = 0;
              uint32_t now = HAL_GetTick();
              if (!servo_enabled_sent) {
                  if (auto_init_phase == 3) auto_init_phase = 0;
                  if (auto_init_phase == 0) {
                      ServoCAN_Enable(&g_servo, 1);
                      last_auto_enable_ms = now;
                      auto_init_phase = 1;
                  } else if (auto_init_phase == 1 && (now - last_auto_enable_ms >= 20)) {
                      ServoCAN_SetMaxSpeedForPositioning(&g_servo, AUTO_ROTATION_SPEED);
                      last_auto_enable_ms = now;
                      auto_init_phase = 2;
                  } else if (auto_init_phase == 2 && (now - last_auto_enable_ms >= 50)) {
                      if (!home_position_valid && g_servo.heartbeat_seen) {
                          home_position = g_servo.abs_pos;
                          home_position_valid = 1;
                      }
                      last_target_position = 0x7FFFFFFF;
                      servo_enabled_sent = 1;
                      auto_init_phase = 3;
                  }
              } else if (now - last_auto_enable_ms >= 200) {
                  ServoCAN_Enable(&g_servo, 1);
                  last_auto_enable_ms = now;
              }
              // 应用 Jetson 转向指令(s_auto_cmd.steer_01，0.1° 左负右正;沿用跟随的位置换算)
              if (home_position_valid) {
                  int32_t half = (int32_t)STEERING_POSITION_RANGE / 2;
                  if (half <= 0) half = 1;
                  int32_t target_position = home_position - (int32_t)s_auto_cmd.steer_01 * half / (10 * STEERING_PHYSICAL_DEG);
                  if (target_position < home_position - STEERING_POSITION_RANGE)
                      target_position = home_position - STEERING_POSITION_RANGE;
                  if (target_position > home_position + STEERING_POSITION_RANGE)
                      target_position = home_position + STEERING_POSITION_RANGE;
                  int32_t pos_diff = target_position - last_target_position;
                  if (pos_diff < 0) pos_diff = -pos_diff;
                  if (pos_diff > 300 || last_target_position == 0x7FFFFFFF) {
                      ServoCAN_GoToAbsolutePosition(&g_servo, target_position);
                      last_target_position = target_position;
                  }
              }
              break;
          }
      }  // switch (g_steering_mode)

      }

      /* 自动驾驶状态上报 0x03(§4)：进入立即发 1 帧就绪，之后 ~10Hz 持续发状态。
       * armed 等由本板安全条件决定;Jetson 收到 armed=1 才正式发非安全 0x11(握手 §4)。 */
      if (g_steering_mode == MODE_AUTO) {
          uint32_t now_st = HAL_GetTick();
          if (!auto_ready_sent || (now_st - last_auto_status_ms >= AUTO_STATUS_PERIOD_MS)) {
              last_auto_status_ms = now_st;
              auto_ready_sent = 1;
              uint8_t status = TRAIN_AUTO_ST_IN_AUTO;
              if (auto_armed)      status |= TRAIN_AUTO_ST_ARMED;
              if (auto_foc_valid)  status |= TRAIN_AUTO_ST_FOC_VALID;
              if (vehicle_enabled) status |= TRAIN_AUTO_ST_VEH_EN;
              if (auto_override)   status |= TRAIN_AUTO_ST_OVERRIDE;
              // 实测车速 mm/s(0.1km/h→mm/s)，方向按当前 FOC 驱动方向
              uint32_t meas_01kmh = FocCAN_GetSpeed_01kmh(&g_foc_can, now_st, WheelSpeed_GetSpeed_01kmh());
              int32_t spd_mm_s = (int32_t)meas_01kmh * 1000L / 36L;
              if (foc_gear_proto == 2U) spd_mm_s = -spd_mm_s;
              // 实测转向反馈 0.1°(左负右正)，由伺服位置反算(同训练帧约定)
              int16_t steer_fb = 0;
              if (home_position_valid) {
                  int32_t half = (int32_t)STEERING_POSITION_RANGE / 2;
                  if (half <= 0) half = 1;
                  int32_t angle_deg = (int32_t)(home_position - g_servo.abs_pos) * STEERING_PHYSICAL_DEG / half;
                  if (angle_deg > STEERING_PHYSICAL_DEG) angle_deg = STEERING_PHYSICAL_DEG;
                  if (angle_deg < -STEERING_PHYSICAL_DEG) angle_deg = -STEERING_PHYSICAL_DEG;
                  steer_fb = (int16_t)(angle_deg * 10);
              }
              uint8_t gear_proto = (current_gear == SCREEN_KEY_GEAR_P) ? 0 :
                                   (current_gear == SCREEN_KEY_GEAR_R) ? 1 :
                                   (current_gear == SCREEN_KEY_GEAR_N) ? 2 : 3;
              AutoStatus_t st = {
                  .seq = TrainUART_GetNextSeq(),
                  .ts_ms = now_st,
                  .spd_meas_mm_s = (int16_t)spd_mm_s,
                  .steer_fb_01 = steer_fb,
                  .status = status,
                  .gear = gear_proto
              };
              TrainUART_SendAutoStatus(&st);
          }
      }

      /* 训练 / 采集模式：20ms 周期上报驾驶数据 (USART1)。
       * 训练→TYPE_CTRL(0x01) 供自动驾驶训练；采集→TYPE_COLLECT(0x07) 供 SLAM。
       * 帧体相同(基础行驶数据)，仅 TYPE 不同，两路彻底分开。
       * 注：IMU(SLAM 用)字段为后续帧扩展，当前帧 [18:19] 仅 2 字节预留容不下，
       *     待 IMU 驱动 + Jetson 帧格式约定后加长，见 docs/00_集成Review与计划_CN.md。 */
      if (g_steering_mode == MODE_TRAINING || g_steering_mode == MODE_COLLECT) {
          static uint32_t last_train_send_ms = 0;
          uint32_t now = HAL_GetTick();
          if (now - last_train_send_ms >= 20) {
              last_train_send_ms = now;
              #define TRAIN_THR_ADC_DEAD  1100
              #define TRAIN_THR_ADC_FULL  3100
              #define TRAIN_THR_DEAD_PCT  30

              uint32_t adc = HAL_ADC_GetValue(&hadc1);
              if (adc > 4095) adc = 4095;
              uint16_t thr_ref = 0;
              if (adc >= TRAIN_THR_ADC_DEAD) {
                  uint32_t range = TRAIN_THR_ADC_FULL - TRAIN_THR_ADC_DEAD;
                  if (adc > TRAIN_THR_ADC_FULL) adc = TRAIN_THR_ADC_FULL;
                  thr_ref = (uint16_t)((adc - TRAIN_THR_ADC_DEAD) * 1000 / range);
                  if (thr_ref > 1000) thr_ref = 1000;
                  if (thr_ref < TRAIN_THR_DEAD_PCT) thr_ref = 0;
              }

              int16_t steer_01 = 0;
              if (home_position_valid) {
                  int32_t half = (int32_t)STEERING_POSITION_RANGE / 2;
                  if (half <= 0) half = 1;
                  // 左负右正；物理转角约±37°，按实测标定
                  int32_t angle_deg = (int32_t)(home_position - g_servo.abs_pos) * STEERING_PHYSICAL_DEG / half;
                  if (angle_deg > STEERING_PHYSICAL_DEG) angle_deg = STEERING_PHYSICAL_DEG;
                  if (angle_deg < -STEERING_PHYSICAL_DEG) angle_deg = -STEERING_PHYSICAL_DEG;
                  steer_01 = (int16_t)(angle_deg * 10);  // 0.1° 单位，来源：伺服电机反馈
              }

              uint8_t gear_proto = 0;
              if (current_gear == SCREEN_KEY_GEAR_P) gear_proto = 0;
              else if (current_gear == SCREEN_KEY_GEAR_R) gear_proto = 1;
              else if (current_gear == SCREEN_KEY_GEAR_N) gear_proto = 2;
              else if (current_gear == SCREEN_KEY_GEAR_D) gear_proto = 3;

              // 刹车：PB3 行程开关按下=1（接地读低），P档/远程刹车自锁(RC3/RC4)也视为刹车
              uint8_t brake_sw = ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET || HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)) ? 1 : 0;
              uint8_t brake_val = (brake_sw || (current_gear == SCREEN_KEY_GEAR_P) || g_rc_brake) ? 1 : 0;

              TrainFrameData_t tf = {
                  .seq     = TrainUART_GetNextSeq(),
                  .ts_ms   = now,
                  .speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, now, WheelSpeed_GetSpeed_01kmh()),
                  .steer_01 = steer_01,
                  .thr_ref = thr_ref,
                  .brake   = brake_val,
                  .gear    = gear_proto
              };
              if (g_steering_mode == MODE_COLLECT) {
                  TrainUART_SendCollectFrame(&tf);   /* SLAM 采集流 */
              } else {
                  TrainUART_SendFrame(&tf);          /* 自动驾驶训练流 */
              }
          }
      }

      /* 采集模式：50Hz 发 IMU CSV 行给 Jetson(/imu)。格式与 vehicle_driver/imu_parser 一致：
       * IMU,ax,ay,az,gx,gy,gz,roll,pitch,yaw。ax..az/gx..gz 为 LSM6DSV 原始 int16
       * (±4g, ±2000dps；Jetson 端 LSB_PER_G/GYRO_SCALE 据此换算)；roll/pitch/yaw 发 0
       * (LSM6DSV 无姿态融合，Jetson 也未用这三项)。与二进制采集帧在同一串口复用，
       * 文本/二进制由 Jetson 按内容 demux。 */
      if (g_steering_mode == MODE_COLLECT || g_steering_mode == MODE_AUTO) {
          /* 方案A(2026-06-02)：自动驾驶模式同样发 50Hz IMU 行，使 Jetson 的
           * odom->base_link 在自动驾驶途中持续更新(AMCL 定位前提)。轮速另由 0x03
           * 状态帧 SPD_MEAS 提供给 Jetson 端里程计。 */
          static uint32_t last_imu_tx_ms = 0;
          uint32_t now_imu = HAL_GetTick();
          if (now_imu - last_imu_tx_ms >= 20) {
              last_imu_tx_ms = now_imu;
              if (g_imu.ok) {
                  char imu_line[80];
                  (void)snprintf(imu_line, sizeof(imu_line),
                                 "IMU,%d,%d,%d,%d,%d,%d,0,0,0\r\n",
                                 (int)g_imu.ax, (int)g_imu.ay, (int)g_imu.az,
                                 (int)g_imu.gx, (int)g_imu.gy, (int)g_imu.gz);
                  TrainUART_SendText(imu_line);
              }
          }
      }

      /* 采集确认灯 0x0618 + 保存状态灯 0x0619：状态由唯一的行消费者
       * FocCAN_TestPollUart 解析 Jetson 回传的 "MAP,1/0"、"SAVE,1/0" 并写入
       * g_host_*（避免本处再 PollLine 与 CLI 抢行导致灯闪）。Jetson 每 ~0.5s 各发一次，
       * >1.5s 未刷新视为断流→灭灯；切出采集模式也灭灯。 */
      {
          static uint8_t  collect_ack_shown = 0;
          static uint8_t  save_ok_shown = 0;
          static uint32_t last_refresh_ms = 0;
          uint32_t now_ack = HAL_GetTick();

          if (g_steering_mode == MODE_COLLECT) {
              uint8_t ca = (g_host_collect_ack && (now_ack - g_host_map_rx_ms <= 1500)) ? 1 : 0;
              uint8_t so = (g_host_save_ok && (now_ack - g_host_save_rx_ms <= 1500)) ? 1 : 0;
              if (ca != collect_ack_shown || so != save_ok_shown ||
                  (now_ack - last_refresh_ms >= 1000)) {
                  collect_ack_shown = ca;
                  save_ok_shown = so;
                  last_refresh_ms = now_ack;
                  Screen_SetCollectAck(ca);
                  Screen_SetSaveStatus(so);
              }
          } else if (collect_ack_shown || save_ok_shown) {
              collect_ack_shown = 0;                     /* 离开采集模式：两灯灭一次 */
              save_ok_shown = 0;
              Screen_SetCollectAck(0);
              Screen_SetSaveStatus(0);
          }
      }

      // 非阻塞：无固定延时，主循环全速运行
