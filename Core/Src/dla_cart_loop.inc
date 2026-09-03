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
              uint16_t speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, t);  // 0.1 km/h
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
                          .speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, HAL_GetTick()),
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
                          .speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, HAL_GetTick()),
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
              /*
               * ⚠ 实体排挡杆压过触摸屏 —— 但**只在真见过那个开关之后**。
               *
               * 三态的理由在 board_inputs.h：U7 没接的车四路全高，
               * 那也会稳定地解出 P，可那不是开关在说话，是没有开关。
               * ⇒ 判据是 g_gear_src，不是"这一轮解出了什么"：
               *   一根断掉的线不该把控制权悄悄交回屏幕。
               */
              if (g_gear_src == GEAR_SRC_SWITCH) {
                  /* 有实体开关，屏不再能设档。静默忽略即可 —— 屏上那个键
                   * 本来就该在装了排挡杆的车上消失。 */
              } else if (key_value == SCREEN_KEY_GEAR_P || key_value == SCREEN_KEY_GEAR_R ||
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
          /*
           * 超时 -> 刹车 + 零油门，但**转向保持不动**，不回中。
           *
           * ! 从前这里把 steer_01 清零，而清零在下游被换算成
           *   target = home_position，也就是**一次主动的回正动作**。
           *   2026-08-28 与 FOC_steer session 对齐：那侧超时的处置是
           *   Steer_Stop() 保持位置，注释写明"意外的自动回正比原地不动
           *   更危险"。两边策略相反必须有一个让步，让步的是这一侧。
           *
           * ! 那这不就是"守着一个已经死掉的源的陈旧值"吗 —— 和档位带
           *   来自一块拔掉的屏是同一个形状？**不是。** 差别在方向：
           *   守着陈旧的档位带会**允许车比该有的更快**；而守着陈旧的
           *   转向角只是**让轮子留在原地**，替代方案(回正)反倒是一次
           *   没人要求的运动。**安全方向不同，处置就不同。**
           *   ⇒ 但它仍然要**看得见**：DRIVE_BLOCK_HOST_STALE 会置位,
           *     WHY? 和 0x03[16:17] 都报得出来。
           */
          if ((int32_t)(HAL_GetTick() - last_follow_cmd_ms) > g_p_follow_cmd_timeout) {
              s_follow_cmd.brake = 1;
              s_follow_cmd.thr_ref = 0;
              /* steer_01 保持上一次的值 —— 见上 */
          }
          if (s_follow_cmd.steer_01 > g_p_steer_max) s_follow_cmd.steer_01 = (int16_t)g_p_steer_max;
          if (s_follow_cmd.steer_01 < -g_p_steer_max) s_follow_cmd.steer_01 = (int16_t)(-g_p_steer_max);
          if (s_follow_cmd.thr_ref > 1000) s_follow_cmd.thr_ref = 1000;
      }

      // 自动驾驶模式：轮询 0x11、看门狗失效保护、转角/硬限速 clamp(见 AUTONOMOUS_MODE_SPEC §3/§6)
      if (g_steering_mode == MODE_AUTO) {
          if (TrainUART_PollAutoCmd(&s_auto_cmd)) {
              last_auto_cmd_ms = HAL_GetTick();
          }
          // §6.1 指令看门狗：超过 AUTO_CMD_TIMEOUT_MS 未收到有效 0x11 → 安全态
          if ((int32_t)(HAL_GetTick() - last_auto_cmd_ms) > g_p_auto_cmd_timeout) {
              s_auto_cmd.brake = 1;
              s_auto_cmd.target_speed_mm_s = 0;
              s_auto_cmd.flags = 0;
              /* steer_01 保持上一次的值 —— 理由见上面 FOLLOW 那段 */
          }
          // 转角限幅
          if (s_auto_cmd.steer_01 > g_p_steer_max) s_auto_cmd.steer_01 = (int16_t)g_p_steer_max;
          if (s_auto_cmd.steer_01 < -g_p_steer_max) s_auto_cmd.steer_01 = (int16_t)(-g_p_steer_max);
          // §6.2 硬限速：独立于 Jetson 再 clamp(双保险)
          if (s_auto_cmd.target_speed_mm_s > g_p_auto_max_speed) s_auto_cmd.target_speed_mm_s = (int32_t)g_p_auto_max_speed;
          if (s_auto_cmd.target_speed_mm_s < -g_p_auto_max_speed) s_auto_cmd.target_speed_mm_s = (int32_t)(-g_p_auto_max_speed);
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

      /*
       * PRND：U7 上的四位置选择开关。**接线标准和全部判据在 board_inputs.h。**
       *
       * 这里从"边沿闩锁"改成了"电平解码"（2026-08-30）。改的不是风格：
       * 闩锁模型下线掉了就没有边沿，档位**停在原地** —— 一台挂着 D 档、
       * 没有档位输入、而且没有任何东西发现的车。电平模型下四路全高
       * 解不出档位，落 P。
       *
       * 顺带修掉了那段没有依据的映射：PD0→R、PD1→D、PD3→P、PD4→P ——
       * 没有针脚能产生 N，两个产生 P，唯一产生 D 的是排第二位的那个。
       */
      {
          static uint16_t gear_cand = 0U;        /* 上一次解出来的候选档位 */
          static uint32_t gear_cand_since = 0U;  /* 它稳定了多久 */
          uint32_t now = HAL_GetTick();

          /* 低有效：按下(选中)为 0。四路各占一位，只为了下面数个数。 */
          uint8_t low_P = (HAL_GPIO_ReadPin(GEAR_SW_GPIO_Port, GEAR_P_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
          uint8_t low_R = (HAL_GPIO_ReadPin(GEAR_SW_GPIO_Port, GEAR_R_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
          uint8_t low_N = (HAL_GPIO_ReadPin(GEAR_SW_GPIO_Port, GEAR_N_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
          uint8_t low_D = (HAL_GPIO_ReadPin(GEAR_SW_GPIO_Port, GEAR_D_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
          uint8_t n_low = (uint8_t)(low_P + low_R + low_N + low_D);
          uint16_t decoded;

          if (n_low == 1U) {
              decoded = low_P ? SCREEN_KEY_GEAR_P
                      : low_R ? SCREEN_KEY_GEAR_R
                      : low_N ? SCREEN_KEY_GEAR_N
                              : SCREEN_KEY_GEAR_D;
          } else {
              /*
               * 零路或多路 —— 两种都是"解不出来"，都落 P。
               *
               * ⚠ 多路时**不按优先级取一个**：从 P 扫到 D 会路过 R 和 N，
               *   取优先级等于把一次故障读成一次正常换挡。
               * ⚠ 两个计数器分开，因为要查的线不一样：零路是断线/没插，
               *   多路是短路/先通后断。合成一个"档位故障"计数就分不出来了。
               */
              decoded = SCREEN_KEY_GEAR_P;
              if (now > GEAR_BOOT_IGNORE_MS) {
                  if (n_low == 0U) { if (g_gear_src == GEAR_SRC_SWITCH) g_gear_none_count++; }
                  else             { g_gear_ambig_count++; }
              }
          }
          g_dbg_gear_raw = (uint8_t)((low_P ? 1U : 0U) | (low_R ? 2U : 0U) |
                                     (low_N ? 4U : 0U) | (low_D ? 8U : 0U));

          /*
           * 防抖加在**解出来的档位**上，不是加在每一路电平上。
           * ⇒ 扫杆途中那些瞬时的"零路/多路"被滤掉，不会打出一个刹车脉冲。
           */
          if (decoded != gear_cand) {
              gear_cand = decoded;
              gear_cand_since = now;
          }

          if ((now > GEAR_BOOT_IGNORE_MS) &&
              ((int32_t)(now - gear_cand_since) >= g_p_gear_debounce)) {
              /*
               * ⚠ 只有**恰好一路为低**才算"见过这个开关"。
               * 一台没接 U7 的车四路全高，那也会稳定地解出 P —— 但那不是
               * 开关在说话，是没有开关。把它算成"见过"会把屏的设档权
               * 无声地夺走。
               */
              if (n_low == 1U) {
                  g_gear_src = GEAR_SRC_SWITCH;
              }
              if ((g_gear_src == GEAR_SRC_SWITCH) && (current_gear != decoded)) {
                  current_gear = decoded;
                  Screen_SetGear(decoded);
              }
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
              /* 每帧先清零：HAL 只写 DLC 个字节，不清就会读到上一帧的残留。 */
              memset(rxData, 0, sizeof(rxData));
              if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
                  break;
              }
              /*
               * 本板消费的每一帧(0x211/0x212/0x213/0x214 来自驱动板，0x64 来自
               * 转向)都是 DLC=8，解析器也都按固定偏移取字节。短帧必须丢掉，
               * 不能当成 8 字节来读 —— 从前不查 DLC 且 rxData 在 while 外面
               * 声明、从不清零，于是一帧 DLC=5 的 0x212 会让 [5:6](车速)
               * 读到上一帧的残留字节，而那个车速正是喂给 AUTO 速度环的。
               * 驱动板那侧一直是 len != 8 就丢，只有这一侧漏了。
               */
              if (rxHeader.DLC != 8U) {
                  g_dbg_can_short++;
                  continue;
              }
              ServoCAN_OnRxFrame(&g_servo, rxHeader.StdId, rxData);
              FocCAN_OnRxFrame(&g_foc_can, rxHeader.StdId, rxData);
          }

          /*
           * 总线出错这件事，从前这块板一次都没有问过。
           *
           * AutoBusOff 是开着的，所以外设自己会从 bus-off 里恢复 —— 但
           * **没有任何人知道它去过**。一条反复进出 bus-off 的总线，对应用
           * 和对 Jetson 都是完全不可见的；接收 FIFO 也是 ReceiveFifoLocked
           * = DISABLE，溢出直接覆盖，不锁不计。
           * 于是"链路好像有点不稳"永远只能靠猜。
           *
           * 这里只数，不做任何处置：恢复交给外设，判断交给上面那几个
           * 新鲜度/回环判据。一个自己参与决策的诊断量，坏了会把决策一起带偏。
           */
          if (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_FOV0) != RESET) {
              __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
              g_dbg_can_fifo_ovf++;
          }
          {
              static uint8_t was_busoff = 0;
              uint8_t now_busoff = (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_BOF) != RESET) ? 1U : 0U;
              if (now_busoff && !was_busoff) {
                  g_dbg_can_busoff++;    /* 数的是**进入**次数，不是当前状态 */
              }
              was_busoff = now_busoff;
              g_dbg_can_busoff_now = now_busoff;
          }
      }

      /* FOC2 行走请求：控制板接入时，手动踏板/遥控 RC/跟随指令统一转换为 CAN 0x210。 */
      int16_t motor_pwm_us;
      /*
       * 踏板只解算一次，得到一个**与档位无关**的 g_pedal_permille(0..1000)：
       * 那是"司机踩了多少"。往下分两路用 ——
       *   驱动路径按档位给它方向(D 加、R 减)，N/P 档不出力；
       *   训练/采集帧原样上报，因为录的就是司机的输入。
       *
       * ! 从前这两路各自从 ADC 重算一遍，而且用的是**两套不同的标定**
       *   (850/3500 与 1100/3100)。数字不一样只是表象，病根是同一个物理量
       *   有两个家。现在只有一处解算，它们不可能再分岔。
       */
      {
          uint32_t adc = HAL_ADC_GetValue(&hadc1);
          /* 标定要读同一个源 —— 另开一路采样，两个源迟早会分岔。 */
          g_dbg_adc_raw = (uint16_t)((adc > 4095U) ? 4095U : adc);
          uint16_t pm = 0U;
          if (adc > 4095U) adc = 4095U;
          /* ⚠ 读运行时值，不读宏 —— 这三个是可写参数（perm=rw），
           * 用户用 `param set thr_adc_dead <n>` 标定踏板。
           * ⚠⚠ range 要防 0：满点 <= 零点时除零。参数层的范围校验拦不住
           *   这种**两个参数之间**的关系，只能在用的地方挡。
           *   挡法是"这一脚当没踩"，而不是夹一个假的比例。 */
          if ((adc >= (uint32_t)g_p_thr_adc_dead) &&
              (g_p_thr_adc_full > g_p_thr_adc_dead)) {
              uint32_t range = (uint32_t)(g_p_thr_adc_full - g_p_thr_adc_dead);
              if (adc > (uint32_t)g_p_thr_adc_full) adc = (uint32_t)g_p_thr_adc_full;
              pm = (uint16_t)(((adc - (uint32_t)g_p_thr_adc_dead) * 1000U) / range);
              if (pm > 1000U) pm = 1000U;
              if (pm < (uint16_t)g_p_thr_dead) pm = 0U;   /* 踏板回位间隙 */
          }
          g_pedal_permille = pm;
      }
      int16_t pedal_pwm = 1500;
      if (current_gear == SCREEN_KEY_GEAR_D || current_gear == SCREEN_KEY_GEAR_R) {
          int16_t offset = (int16_t)(g_pedal_permille / 2U);   /* 0..1000‰ -> 0..500us */
          pedal_pwm = (current_gear == SCREEN_KEY_GEAR_D) ? (int16_t)(1500 + offset)
                                                         : (int16_t)(1500 - offset);
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
      /* 统一刹车意图(foc_brake)的输入之一：刹车踏板开关 PE2/PE3(低=按下)。
       * 裁决在下面 mode_brake / force_brake 那一段。
       * 刹车由驱动板板载 H 桥执行，随 0x210 flags bit1 下发；独立刹车板
       * (0x220)已移除，见本文件靠后 BrakeCAN_Send 处的说明。 */
      uint8_t brake_sw = ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET ||
                           HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET)) ? 1U : 0U;
      /* RC3 远程刹车：单通道两位开关(~1000/~2000us)直接表示刹车/释放，开关位置本身即保持
       * 状态，无需软件自锁。RC3 处于刹车位时在所有模式下(含跟随)都强制刹车。 */
      g_rc_brake = RC_GetSwitch_RC3();
      /* 自动驾驶 arm 判定(§5)：屏选 AUTO + FOC 状态新鲜 + 整车使能 + D 档 + Jetson 心跳
       * (0x11 新鲜) + Jetson 请求使能(FLAGS.AUTO_ENABLE) + 无急停/接管。任一不满足→不 arm
       * →foc_brake 强制刹车保持安全态。急停/接管(遥控刹车/刹车开关)优先级最高。 */
      /*
       * "驱动板还能不能把车交给它" —— 三条都要：帧在来、它自己在跑
       * (序列号在动)、而且它还认我们这一路(0x211[7] bit7)。
       *
       * 从前这里只问第一条。驱动板特意把后两条送上来 —— 简报 §2.1 的原话是
       * "先确认你真的在用它，而不是只解析了不看" —— 而这一侧确实只解析不看：
       * 三个字段解出来只进了 ST? 的打印。一块应用挂死、发送邮箱里还压着帧
       * 自动重传的驱动板，从"最近有没有帧到"看过去完全健康，控制板会继续
       * armed、继续下发。 */
      uint8_t auto_foc_valid = FocCAN_PeerAlive(&g_foc_can, HAL_GetTick());
      /* 车速反馈是否算数。AUTO 是速度闭环，没有反馈就不能 arm —— 见
       * FocCAN_SpeedFeedbackValid 的注释：状态帧(0x211)和遥测帧(0x212)
       * 是两帧，前者活着而后者死了的时候，速度环会把"没数据"当成"0 km/h"
       * 一路积分到油门上限。 */
      uint8_t auto_speed_valid = FocCAN_SpeedFeedbackValid(&g_foc_can, HAL_GetTick());
      uint8_t auto_cmd_fresh = ((int32_t)(HAL_GetTick() - last_auto_cmd_ms) <= g_p_auto_cmd_timeout) ? 1U : 0U;
      uint8_t auto_override = (g_rc_brake || brake_sw) ? 1U : 0U;
      uint8_t auto_armed = 0U;
      if (g_steering_mode == MODE_AUTO) {
          auto_armed = (auto_foc_valid && auto_speed_valid && (vehicle_enabled != 0U) &&
                        (current_gear == SCREEN_KEY_GEAR_D) && auto_cmd_fresh &&
                        ((s_auto_cmd.flags & TRAIN_AUTO_FLAG_ENABLE) != 0U) &&
                        (auto_override == 0U)) ? 1U : 0U;
      }
      /*
       * 刹车意图分两层，因为它们回答的是两个不同的问题：
       *
       *   mode_brake  —— "这个模式此刻想不想刹车"。只有跟随/自动有话说。
       *   force_brake —— "有没有人正在要求刹车，不管哪个模式"。
       *
       * 从前这是一个三目表达式，跟随分支只取上位机那个字节，于是
       * PD10(整车使能)、刹车踏板开关(PE2/PE3)、P 档在 MODE_FOLLOW 下
       * 全都不参与刹车判断 —— 跟随模式里踩物理刹车踏板不刹车，
       * 而 AUTONOMOUS_MODE_SPEC.md §6.3 写的是"任意模式强制"。
       * 只有 RC3 因为写在表达式外面才幸免。
       *
       * 现在把"任意模式强制"这件事真的写成一个跟模式无关的项。
       * 不确定的那一档取安全方向：任一路要求刹车即刹车。
       */
      uint8_t mode_brake = (g_steering_mode == MODE_FOLLOW) ? (s_follow_cmd.brake ? 1U : 0U)
                           : (g_steering_mode == MODE_AUTO) ? ((s_auto_cmd.brake || (auto_armed == 0U)) ? 1U : 0U)
                           : 0U;
      /* 遥控是 REMOTE/COLLECT 里的油门与转向来源。那两个模式下链路失效
       * (曾经活过、现在断了)要刹车 —— 摇杆通道已经交出中位，车会滑行，
       * 但滑行不是停住。其余模式遥控不是控制源，断了不该影响司机开车。
       * RC3 急停通道的失效在任何模式下都算数，它走 g_rc_brake。 */
      uint8_t rc_ctrl_lost = ((g_steering_mode == MODE_REMOTE || g_steering_mode == MODE_COLLECT) &&
                              ((RC_LinkState_CH2() == RC_LINK_LOST) ||
                               (RC_LinkState_CH1() == RC_LINK_LOST))) ? 1U : 0U;
      uint8_t force_brake = (brake_sw ||
                             (vehicle_enabled == 0U) ||
                             (current_gear == SCREEN_KEY_GEAR_P) ||
                             rc_ctrl_lost ||
                             g_rc_brake) ? 1U : 0U;
      uint8_t foc_brake = (mode_brake || force_brake) ? 1U : 0U;

      /*
       * 记下**为什么**不允许驱动，就在裁决它的这一处。
       *
       * 驱动板那侧的原话：从它看过去，"使能开关没合"和"控制板挂了"是同一个
       * 现象 —— 都是 allow_drive=0，而这两种要查的线完全不一样。互锁不该绕,
       * 该做的是让它能说出理由。同一条道理在板内也成立：这些条件今天散在
       * 七处各算各的，从外面看长得一模一样。
       *
       * 位在这里只是**记录**，不参与任何判断 —— 判断还是上面那几行。
       * 一个自己参与决策的诊断量，坏了的时候会连决策一起带偏。
       */
      {
          uint16_t blk = 0U;
          if (vehicle_enabled == 0U)                       blk |= DRIVE_BLOCK_VEH_ENABLE;
          if (!(current_gear == SCREEN_KEY_GEAR_D ||
                current_gear == SCREEN_KEY_GEAR_R))        blk |= DRIVE_BLOCK_GEAR;
          if (brake_sw)                                    blk |= DRIVE_BLOCK_BRAKE_SW;
          if (g_rc_brake)                                  blk |= DRIVE_BLOCK_RC_BRAKE;
          if (rc_ctrl_lost)                                blk |= DRIVE_BLOCK_RC_LOST;
          if (FocCAN_EstopActive() != 0U)                  blk |= DRIVE_BLOCK_ESTOP;
          if (FocCAN_TestActive() != 0U)                   blk |= DRIVE_BLOCK_CLI_TEST;
          /* ⚠ 这一条在**任何模式**下都成立，所以不在下面的 AUTO 分支里：
           * 一块处在本地模式的驱动板，不会因为司机在踩踏板就听总线的。
           * ⚠ 只对 1 反应 —— 0 同时符合"没锁"和"固件比 0.4.11 老"
           * (0.4.8~0.4.10 锁着也发 0)。见 foc_can.h b6 那一段。 */
          if (FocCAN_PeerLocalLock(&g_foc_can) != 0U)      blk |= DRIVE_BLOCK_FOC_LOCKED;
          if (g_steering_mode == MODE_AUTO) {
              if (current_gear != SCREEN_KEY_GEAR_D)       blk |= DRIVE_BLOCK_GEAR;
              /* 分三条报，因为它们要查的线不一样：线断了 / 板卡住了 /
               * 板还在跑但已经不认我们。 */
              if (!FocCAN_StatusFresh(&g_foc_can, HAL_GetTick()))
                                                           blk |= DRIVE_BLOCK_FOC_STALE;
              if (FocCAN_PeerFrozen(&g_foc_can, HAL_GetTick()))
                                                           blk |= DRIVE_BLOCK_FOC_FROZEN;
              if (!FocCAN_PeerBelievesUs(&g_foc_can))       blk |= DRIVE_BLOCK_FOC_DISBELIEF;
              if (!auto_speed_valid)                       blk |= DRIVE_BLOCK_SPEED_INVALID;
              if (!auto_cmd_fresh)                         blk |= DRIVE_BLOCK_HOST_STALE;
              if ((s_auto_cmd.flags & TRAIN_AUTO_FLAG_ENABLE) == 0U)
                                                           blk |= DRIVE_BLOCK_HOST_DISABLE;
          } else if (g_steering_mode == MODE_FOLLOW) {
              if ((int32_t)(HAL_GetTick() - last_follow_cmd_ms) > g_p_follow_cmd_timeout)
                                                           blk |= DRIVE_BLOCK_HOST_STALE;
          }
          g_drive_block = blk;
      }
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
      /* 转向的动作闸也从这里喂 —— 整车使能断开时，驱动被禁止而转向
       * 从前照样使能、照样下发目标。闸放在 servo_can 里，见那边 may_actuate
       * 的说明：在九个调用点各补一个 if，是把"规则写在调用点里"这个病抄九遍。 */
      ServoCAN_SetActuationAllowed(&g_servo, vehicle_enabled);
      uint8_t foc_allow_drive = (vehicle_enabled != 0U) ? 1U : 0U;
      uint8_t foc_gear_proto = 0U; /* 0=N 1=F 2=R */
      uint16_t foc_throttle_permille = 0U;
      int16_t foc_steer_permille = 0;

      if (!allow_motor || (foc_brake != 0U)) {
          motor_pwm_us = 1500;
      } else {
          int16_t offset = motor_pwm_us - 1500;
          /* 档位**不再**乘进油门。它随 0x210 flags bit2-7 发给驱动板，由驱动板
           * 套用到转速上限上。
           * ! 原来这里是 offset * Screen_GetSpeedScale(档位) —— 把档位当扭矩档，
           *   低速档削掉 65% 扭矩 ⇒ 带人起不动车。驱动板上这个缺陷已经修过
           *   (那边叫缺陷 J)，本板原封不动地留着同一个做法，直到今天。
           *   **档位缩放的是能跑多快，不是能出多大力。** */
          int16_t foc_offset = offset;

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
                  uint16_t meas_01kmh = FocCAN_GetSpeed_01kmh(&g_foc_can, pi_now);
                  int32_t meas_mm_s = (int32_t)meas_01kmh * 1000L / 36L;  /* 0.1km/h→mm/s */
                  int32_t err = tgt_abs - meas_mm_s;
                  s_auto_speed_i += err;
                  if (s_auto_speed_i > AUTO_SPD_I_MAX) s_auto_speed_i = AUTO_SPD_I_MAX;
                  if (s_auto_speed_i < -AUTO_SPD_I_MAX) s_auto_speed_i = -AUTO_SPD_I_MAX;
                  int32_t out = (err * AUTO_SPD_KP_NUM) / AUTO_SPD_KP_DEN
                              + (s_auto_speed_i * AUTO_SPD_KI_NUM) / AUTO_SPD_KI_DEN;
                  if (out < 0) out = 0;
                  if (out > g_p_auto_max_throttle) out = g_p_auto_max_throttle;
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

      /*
       * 屏上选的速度档，发给驱动板去限**转速上限**(不是限扭矩)。
       * 四个档 0.35/0.60/0.80/1.00 -> 35/60/80/100 百分比。
       *
       * ⚠ 屏断了就停止主张这个字段(发 0 = 不声明)，驱动板会沿用上一次的。
       *
       * ⚠ ==0 是"从来没说过话"，和"说过然后不说了"要分开 ——
       *   和遥控那三态是同一条规则。台架实测发现的：屏从来没接过的时候
       *   screen_last_rx_ms 是 0，而 now-0 在开机头 8 秒里小于超时，
       *   于是**一块从来不存在的屏，在每次上电后的头 8 秒都在主张一个
       *   速度上限**。只判超时不判"有没有过"，就会漏掉这一段。
       *
       *   驱动板把档位带放上总线的理由是"别再守着一块可能已经拔掉几周的
       *   面板上的档位"。而 current_speed_level 是一个**锁存的屏幕按键值**,
       *   本来没有任何新鲜度检查 —— 控制板在上游一跳原样复现了同一个条件,
       *   在每一帧上永远主张一个档位带。于是"0 = 不声明"这条路径在真实
       *   系统里从来走不到：它在驱动板那侧有测试覆盖，在线上是死的。
       *
       *   屏的 8 秒超时本来就在上面判着(第 11 行)，那里只点了个指示灯。
       */
      const uint8_t band_percent =
          ((Screen_GetLastRxMs() == 0U) ||
           ((HAL_GetTick() - Screen_GetLastRxMs()) > SCREEN_TIMEOUT_MS))
          ? 0U
          : (uint8_t)(Screen_GetSpeedScale(current_speed_level) * 100.0f + 0.5f);
      g_dbg_band = band_percent;

      /* FOC2 板：周期下发正式驾驶请求（CAN ID 0x210） */
      {
          static uint32_t last_foc_tx_ms = 0;
          uint32_t foc_now = HAL_GetTick();
          /*
           * ⭐⭐ 2026-08-30：这里从前有一个模式判断，现在没有了。
           *
           * 原来那条（2026-06-02 修的一个真实缺陷）：AUTO/FOLLOW 下 Jetson 在发
           * 二进制帧，而行消费者会从**同一个环形缓冲**偷走帧字节 ⇒ 0x11/0x10
           * 解析失败 ⇒ auto_cmd_fresh 闪断 ⇒ arm 掉 ⇒ 刹车点动。
           * 代价是**恰恰在最需要问板子话的两个模式下，CLI 是关的**。
           *
           * 分线之后（CLI 在 USART1/USB1，Jetson 在 UART5/USB2）这一路上
           * 根本没有二进制 ⇒ 偷不走任何东西 ⇒ 那个判断连同它的代价一起没了。
           * **自动驾驶跑着的时候也能问板子话。**
           */
          {
              FocCAN_TestPollUart(&g_foc_can);
              /* Jetson 回的 MAP,/SAVE, 在另一根线上，另一个消费者。 */
              FocCAN_PollJetsonLines();
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
                                                FocCAN_TestTimeout10ms(),
                                                band_percent);
              } else {
                  (void)FocCAN_SendDriveRequest(&g_foc_can, foc_throttle_permille,
                                                foc_steer_permille,
                                                foc_allow_drive, foc_brake,
                                                foc_gear_proto,
                                                FOC_CAN_DEFAULT_TIMEOUT_10MS,
                                                band_percent);
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
                      /* ! 只有真的发出去了才推进。闸(整车使能)挡下时
                       *   ServoCAN_Enable 返回 HAL_ERROR —— 从前这里不看返回值,
                       *   于是使能没发出去，序列却照样走完并置 servo_enabled_sent,
                       *   **把一个没做成的步骤记成做过了**。 */
                      if (ServoCAN_Enable(&g_servo, 1) == HAL_OK) {
                          last_remote_enable_ms = now;
                          servo_init_phase = 1;
                      }
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
                      /* 同 REMOTE 分支：只有真的发出去了才推进 —— 闸挡下时
                       * 不能把一个没做成的步骤记成做过了。 */
                      if (ServoCAN_Enable(&g_servo, 1) == HAL_OK) {
                          last_follow_enable_ms = now;
                          follow_init_phase = 1;
                      }
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
                      .speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, now),
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
                      /* 同 REMOTE 分支：只有真的发出去了才推进 —— 闸挡下时
                       * 不能把一个没做成的步骤记成做过了。 */
                      if (ServoCAN_Enable(&g_servo, 1) == HAL_OK) {
                          last_auto_enable_ms = now;
                          auto_init_phase = 1;
                      }
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
              uint32_t meas_01kmh = FocCAN_GetSpeed_01kmh(&g_foc_can, now_st);
              /*
               * 符号用**量出来的**方向，量不出来的时候才退回命令方向。
               *
               * 规范 §4 写的是"实测车速 mm/s，+前进 −后退"，而这里原来取的是
               * foc_gear_proto —— 本板此刻正在下发的方向。命令倒车而车还在
               * 往前滑的那一段，报出去的是负速度：**幅值是量的，符号是编的**,
               * 而 Jetson 把它直接折进里程计(§8)，分不出来。
               *
               * 停住的时候(dir==0)符号没有意义，退回命令方向不会造成误解。
               */
              int8_t meas_dir = FocCAN_GetSpeedDirection(&g_foc_can, now_st);
              uint8_t spd_reverse = (meas_dir != 0)
                                    ? ((meas_dir < 0) ? 1U : 0U)
                                    : ((foc_gear_proto == 2U) ? 1U : 0U);
              int16_t spd_mm_s = speed_01kmh_to_mm_s(meas_01kmh, spd_reverse);
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
                  .gear = gear_proto,
                  /* AUTO 下 CLI 是关的，WHY? 问不出来 —— 这两个字段是
                   * "为什么不让走"和"这块板是哪一版"唯一到得了 Jetson 的路。 */
                  .block = g_drive_block,
                  .fw_ver = (uint8_t)((FW_VER_MAJOR << 4) | (FW_VER_MINOR & 0x0FU)),
                  .fw_patch = (uint8_t)FW_VER_PATCH
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
              /*
               * 油门直接用统一解算出来的 g_pedal_permille(见本文件上方踏板块)。
               *
               * ! 这里原来自己从 ADC 重算，用的还是**另一套标定**
               *   (1100/3100，而驱动路径是 850/3500) —— 同一路 ADC、
               *   同一个踏板、同一个文件里的两条曲线，几百行之隔，
               *   两边谁也没提过对方。ADC=3300 时车走 92% 而这里报 100%,
               *   ADC=1000 时车已经在出力而这里报 0。
               *   **训练模式录的就是"司机踩了多少"，而录的和车做的不是一回事。**
               */
              uint16_t thr_ref = g_pedal_permille;

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
                  .speed_01 = FocCAN_GetSpeed_01kmh(&g_foc_can, now),
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
                  /* ⚠ 去 Jetson 那根线（UART5），不是 CLI 那根 ——
                   * 50Hz 的 CSV 灌进上位机的控制台会把每一次读都变成
                   * 在一片 IMU 行里找回复。 */
                  TrainUART_SendJetsonText(imu_line);
              }
          }
      }

      /* 采集确认灯 0x0618 + 保存状态灯 0x0619：状态由 UART5 那一路的行消费者
       * FocCAN_PollJetsonLines 解析 Jetson 回传的 "MAP,1/0"、"SAVE,1/0" 并写入
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

      /*
       * 喂狗放在整趟的最后一行，不放在开头，也不放在任何中断里 ——
       * 喂狗这个动作要证明的是"控制循环刚刚完整跑完了一趟"。
       * 放在开头，等于只证明"有人调用了这个函数"；放在中断里更糟，
       * 主循环死透了 SysTick 照样在跑，狗会被一直喂到天荒地老。
       */
      /* param list 一次发一行：一行约 190 字节，115200 阻塞发送约 16ms，
       * 五十行一口气发完就是 800ms 不回主循环 —— 而 0x210 周期 20ms、
       * 驱动板超时 300ms。**列一次参数会让车失去指令。** */
      CartParam_ListStep();
      /* `tlm` 同理，六行。两个泵写成两个独立的状态机而不是一个：
       * 两条命令可以先后进来，共用一个下标会让后来的那条把前一条截断，
       * 而截断后的那半张表**本身是格式良好的**。 */
      CartTlm_Step();
      /* fault / fault codes 同理，见 DBIS §4.5。 */
      CartFault_Step();
      CartCalib_Step();
      CartSelftest_Step();
      CartFault_CodesStep();

      Board_WatchdogKick();
