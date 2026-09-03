# 自动驾驶模式 接口规范 — Jetson ↔ 控制板 两侧对齐契约
日期：2026-06-02　｜　给 **Windows 新 session（实现控制板固件自动驾驶模式）** + 本工作区 Jetson 侧

> **这份文档是两侧实现的唯一契约。** 固件侧（Windows，CubeIDE 编译 + ISP 烧录）和 Jetson 侧（`~/ros2_ws` 的 `cmd_sender.py`/`serial_bridge_node.py`/`protocol.py`）都照这份写，避免再出现"发指令侧和控制侧对不上"（如此前 STEER_SIGN 方向）的问题。
> 帧/字段/约定 与现有协议见 `~/Autodrive/MCU_DOCUMENTATION.md §5`。

---

## 0. 背景与决定（重要）
- 现状：Jetson 侧之前用 **跟随模式 0x10** 发指令，但**控制侧自动驾驶并未匹配实现**。
- 决定（用户 2026-06-02）：**不复用「跟随模式」**，改用协议里预留、目前未启用的 **「自动驾驶模式」**（`TYPE=0x11` 指令帧 / `TYPE=0x03` 就绪帧）。
- 控制侧固件代码位置：`/media/tdl/新加卷/selfdriving/STM32/code/FOC_double`（本机挂载盘；编译/烧录在 Windows）。

## 1. 为什么不复用跟随模式（请固件作者确认动机）
1. 跟随模式用**开环踏板** `THR_REF(0~1000)`；自动驾驶需要**闭环速度控制**——Nav2 下发的是速度目标(m/s)，车应不论坡度/负载都达到该速度。
2. 自动驾驶需要**独立状态机 + 指令看门狗失效保护 + 急停/接管优先级**，安全语义与跟随不同。
3. 独立 `TYPE`，与跟随互不干扰，便于单独测试与上线。

## 2. 物理层 / 帧框架（沿用现有，零改动）
- 22 字节定长帧，小端；`SOF=0xAA 0x55`，`VER=0x01`，`SEQ` u16，`CRC16-MODBUS`(覆盖偏移 2~19)。
- USART1，115200 8N1。
- ✅ **已定稿(2026-06-02 固件侧)**：`0x11` 由 **DlaBoard_A(F407)** 的 USART1 直接解析——它本来就是 Jetson 串口的落点、跟随 `0x10` 的解析板、且独占转向伺服(CAN)与 FOC 行走请求(CAN 0x210)。**自动驾驶模式逻辑全部在 DlaBoard_A**（与跟随同构）；FOC_double(G473) 不改，仅经既有 CAN 0x210 执行行走/0x211 回速度。速度闭环放在 DlaBoard_A（见 §3 单位说明、§6）。

## 3. Host→MCU：自动驾驶指令帧 `TYPE=0x11`
（沿用 22 字节帧布局；下列为该 TYPE 下各偏移的语义）

| 偏移 | 长度 | 字段 | 类型 | 单位/范围 | 说明 |
|------|------|------|------|-----------|------|
| 0-1 | 2 | SOF | - | 0xAA 0x55 | |
| 2 | 1 | VER | u8 | 0x01 | |
| 3 | 1 | TYPE | u8 | **0x11** | 自动驾驶指令 |
| 4-5 | 2 | SEQ | u16 | | 递增包序号 |
| 6-9 | 4 | TS_MS | u32 | ms | Jetson 时间戳（可选用于诊断）|
| 12-13 | 2 | STEER | **int16** | 0.1°，**左负右正** | 目标前轮转角；范围 ±MAX_STEER(实测，暂 370=37°)|
| 14-15 | 2 | TARGET_SPEED | **int16** | **mm/s**，+前进 −后退 | 目标车速；0=停。MCU 侧再按整车限速硬 clamp |
| 16 | 1 | BRAKE | u8 | 0/1 | 1=主动刹车（优先于 TARGET_SPEED）|
| 17 | 1 | GEAR | u8 | 0=P 1=R 2=N 3=D | 可选；自动模式方向以 TARGET_SPEED 符号为准，本字段回显/校验 |
| 18 | 1 | FLAGS | u8 | bit0=AUTO_ENABLE | Jetson 请求使能；MCU 仍以自身安全条件为准 |
| 19 | 1 | RES | u8 | 0 | 保留 |
| 20-21 | 2 | CRC16 | u16 | | CRC16-MODBUS(2~19) |

- **发送频率**：固定 20~50 Hz。**每帧即心跳**（见 §6 看门狗）。
- ✅ **单位已定稿**：`TARGET_SPEED` = **mm/s**（int16，±32.7 m/s）。速度闭环在 **DlaBoard_A** 内实现（mm/s 目标 ↔ 实测车速 PI → throttle_permille 经 CAN 0x210 给 FOC_double，CAN 协议不变、FOC 仍开环油门）。不改用 0.1km/h。
- ✅ **FLAGS.bit0=AUTO_ENABLE 已纳入握手**：Jetson 即便在 armed=0 时也应持续发 0x11（`TARGET_SPEED=0/BRAKE=1` 等安全值 + 置 `AUTO_ENABLE`）以建立心跳；MCU 据此 + 自身安全条件判 arm（见 §4/§5）。`GEAR` 仅回显/校验，方向以 `TARGET_SPEED` 符号为准。

## 4. MCU→Host：自动驾驶就绪/状态帧 `TYPE=0x03`（✅ 已定稿布局）
- 进入自动驾驶模式时立即发 1 帧就绪（类比跟随 0x02）；之后 ~10 Hz（100ms）持续发状态。
- 22 字节帧，小端，CRC16-MODBUS(2~19)。各偏移语义：

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0-1 | 2 | SOF | - | 0xAA 0x55 |
| 2 | 1 | VER | u8 | 0x01 |
| 3 | 1 | TYPE | u8 | **0x03** 自动驾驶就绪/状态 |
| 4-5 | 2 | SEQ | u16 | |
| 6-9 | 4 | TS_MS | u32 | MCU 时间戳 |
| 10-11 | 2 | SPD_MEAS | **int16** | 实测车速 mm/s，+前进 −后退。⚠ 2026-08-27 起符号取自**驱动板报的带符号轮速**(0x211)，而不是本板正在下发的方向；量不出来时(不新鲜或在停车噪声里)才退回命令方向 |
| 12-13 | 2 | STEER_FB | **int16** | 实测转向 0.1°，左负右正(伺服位置反算) |
| 14 | 1 | STATUS | u8 | 位域，见下 |
| 15 | 1 | GEAR | u8 | 0=P 1=R 2=N 3=D |
| 16-17 | 2 | **BLOCK** | u16 LE | **为什么不允许驱动**的位图，0=没有任何一条在挡着。位定义见固件 `Core/Inc/drive_block.h` |
| 18 | 1 | **FW_VER** | u8 | 高 4 位 major，低 4 位 minor |
| 19 | 1 | **FW_PATCH** | u8 | patch |
| 20-21 | 2 | CRC16 | u16 | CRC16-MODBUS(2~19) |

- **STATUS 位**（与固件 `train_uart.h` 一致）：`bit0=ARMED`(可接受 0x11) `bit1=FOC_VALID` `bit2=VEHICLE_ENABLE` `bit3=ESTOP_OR_OVERRIDE`(遥控/手动接管或刹车开关) `bit4=IN_AUTO`。
- **`[16:19]` 于 2026-08-27 由"保留 0"改为承载 BLOCK 与固件版本。**
  `VER` 不动。⚠ **理由不是"加字段不动版本"** —— 那是 `=tlm` 文本记录的规矩，
  套不到帧协议上（2026-08-28 在 CAN `0x211` 那边被驱动板 session 更正过，
  一并订正在这里：CAN 的字节位置是两块板共有的，判据是**两边的头文件有没有
  一起改**）。

  这里能安全填的真正理由不一样、而且更强：**本规范原本就把 `[16:19]` 声明为
  "RES … 0"**，也就是**规范自己许诺过这几个字节可以被忽略**，
  一个守规范的读取方读到非 0 也会继续忽略。判据是"规范有没有说过它可以被忽略",
  不是"这算加还是算改"。

  ⚠ 为什么非加不可：**AUTO 模式下 MCU 的文本 CLI 是关闭的**（二进制帧要
  独占那个接收环形缓冲，见 §9 那个已修的争用缺陷），所以诊断命令 `WHY?`
  恰恰**在最需要它的时候问不出来**。0x03 是自动驾驶期间唯一的上行通道，
  而它这四个字节一直空着。

  `BLOCK` 只是**记录**，不参与任何判断——判断仍在 `ARMED` 那一组条件里。
  `ARMED=0` 时 `BLOCK` 告诉你是哪一条不满足；`ARMED=1` 时它应当是 0。

  ⚠ 固件侧未在台架验证：AUTO 模式只能由串口屏 0x05A1 进入，而验证用的
  板子上没有接屏。字段取自 `g_drive_block`，那个值本身已由 `WHY?` 在台架
  上验证（使能断开+P 档报 `0x0003 ven,gear`）。**上位机接入时请先核对
  一帧真实的 0x03。**

- **握手**：MCU `ARMED=1` 的条件见 §5；Jetson 收到 `ARMED=1` 后才把 `TARGET_SPEED` 升到非零；未 armed 时 MCU 一律保持刹车（固件已强制 `BRAKE=1`）。

## 5. 状态机 / 进入·退出（✅ 固件已实现）
- 固件已新增模式 `MODE_AUTO`（`steering.h` 枚举，与 `MODE_FOLLOW` 并列）；屏键 `SCREEN_KEY_MODE_AUTO=0x0001`@0x05A1 切入。
- **arm 条件（已实现）**：屏选 AUTO + `FOC 状态新鲜(foc_valid)` + 整车使能 + **档位 D** + Jetson 心跳(0x11 在 300ms 内新鲜) + Jetson 请求使能(`FLAGS.AUTO_ENABLE`) + 无急停/接管。任一不满足 → 不 arm，`foc_brake` 强制刹车保持安全态。
- **退出/降级（已实现）**：物理档位离开 D / 屏切走 / 遥控刹车(RC3) / 刹车开关(PE2·PE3) → 退出 arm → 刹车；转向由看门狗(指令清零)回中。遥控/手动接管(RC3/刹车开关)优先级**最高**（任意模式强制 `foc_brake`）。
- 方向：D 档下 `TARGET_SPEED` 符号决定 FOC 方向(F/R)，支持后退；后退仍需 D 档 + arm（与跟随/手动一致，R 由 FOC 侧执行）。

## 6. 安全要求（自动驾驶核心）— ✅ 固件已实现
1. **指令看门狗**：`AUTO_CMD_TIMEOUT_MS=300ms` 未收到有效 0x11 → `TARGET_SPEED=0` + `BRAKE=1` + `STEER=0`(看门狗清零指令，伺服回中) + 清积分。
2. **硬限速**：`AUTO_MAX_SPEED_MM_S=1500`(1.5 m/s) 对 TARGET_SPEED 独立 clamp；油门另有 `AUTO_MAX_THROTTLE_PERMILLE=600` 上限（均为保守起点，台架标定后调）。
3. **急停最高优先**：刹车开关/遥控刹车(RC3)在任意模式强制 `foc_brake`，自动模式下同时撤 arm。
4. **上电默认安全**：开机 `MODE_MANUAL`；`s_auto_cmd` 初值 `BRAKE=1`、零速。
5. **速度闭环平滑**：整数 PI(`Kp=3/10, Ki=1/50, ∫限幅 30000`)按 20ms 定步长跑，积分限幅抗饱和。⚠️ **斜坡/阶跃限制与增益需台架标定**（§9），增益在 `dla_cart_app.c` 顶部 `AUTO_SPD_*` 宏。

## 7. Jetson 侧 — ✅ 已完成(2026-06-02)，已 colcon build + 导入/单测通过
> 固件侧(DlaBoard_A)已实现并通过编译(见 §10)。Jetson 侧改动已在 `~/ros2_ws/src/vehicle_driver` 完成，照 §3/§4 定稿布局对齐：
- ✅ `protocol.py`：`build_auto_frame()`(0x11)+`parse_auto_status()`(0x03)+常量(`TRAIN_TYPE_CMD_AUTO/AUTO_STATUS`、`AUTO_FLAG_ENABLE`、`AUTO_ST_*`、`GEAR_D`)。单测：0x11 字段/CRC、0x03 往返、坏CRC拒收 全过。
- ✅ `cmd_sender.py`：`twist_to_auto()` 出 `(steer_01, target_speed_mm_s, brake)`；`STEER_SIGN=-1.0`(对齐 MCU 左负右正；自测 w>0→steer_01<0 ✓)；不再发 0x10。
- ✅ `serial_bridge_node.py`：参数 `auto_enable`(仅 nav launch 开)；33Hz 定速发 0x11 心跳；按 `0x03.STATUS.ARMED` 门控速度(未 armed/丢帧→speed=0+brake=1)；解析 0x03→发 `/vehicle/auto_armed`(Bool)。
- ✅ `vehicle_nav.launch.py`：serial_bridge 传 `auto_enable=True`。
- ✅ `WHEELBASE_M=0.67`、`minimum_turning_radius=0.9`。`MAX_STEER_DEG`/`STEER_SIGN` 物理朝向待台架 §9 确认。

（原始需求，留档）：
- `protocol.py`：新增 `build_auto_frame(steer_01, target_speed_mm_s, brake, gear, flags)` → `TYPE=0x11`（布局见 §3：STEER@12 int16 0.1°、TARGET_SPEED@14 int16 mm/s、BRAKE@16、GEAR@17、FLAGS@18 bit0=AUTO_ENABLE）；新增 `0x03` 解析（布局见 §4，STATUS 位）。
- `cmd_sender.py`：`/cmd_vel`(Twist) → 自行车模型 → `STEER(0.1°, 左负右正)` + `TARGET_SPEED(mm/s)`；**不再发 0x10**；持续置 `AUTO_ENABLE`(armed=0 时发安全值 speed=0/brake=1 维持心跳)。
- `serial_bridge_node.py`：解析 0x03，按 `STATUS.ARMED` 决定是否升速；20~50Hz 发 0x11 当心跳；停发由 MCU 300ms 看门狗兜底。
- 已定参数：`WHEELBASE_M=0.67`、`minimum_turning_radius=0.9`、`MAX_STEER_DEG`(待实测)、`STEER_SIGN`(台架定，见 §9)。

## 8. 拍板清单 — ✅ 已定稿(2026-06-02 固件侧)
- [x] 0x11 解析板：**DlaBoard_A**（自动模式逻辑全在本板，FOC_double 仅经 CAN 0x210 执行，不改）。
- [x] TARGET_SPEED 单位：**mm/s**；速度闭环在 DlaBoard_A(PI→throttle_permille)。
- [x] 允许后退：**是**，D 档下 TARGET_SPEED 负值→FOC 方向 R。
- [x] 进入 AUTO：**强制 D 档 + 整车使能 + foc_valid + Jetson 心跳 + AUTO_ENABLE**，否则不 arm。
- [x] 看门狗超时：**300 ms**。
- [x] ✅ STEER 物理符号实测确认(台架2026-06-02)：发 +20°→前轮**右**偏,符合 MCU 左负右正 → Jetson `STEER_SIGN=-1.0` 正确(ROS左转w>0→负STEER→左)。舵机闭环跟随准(cmd 20°→fb 20°)。
- [ ] 速度环增益 / 限速值台架标定(`AUTO_SPD_*`、`AUTO_MAX_SPEED_MM_S`、`AUTO_MAX_THROTTLE_PERMILLE`)。台架现象:发 200mm/s 实测在 166~222 间波动(±~15%),需调 PI 增益使其平稳。
- [x] ✅ **MODE_AUTO 下的 odom 来源 — 方案A 已实现(2026-06-02)**。`odom→base_link`(AMCL 定位前提)分两路喂：**陀螺Z/角速度** 来自固件——`dla_cart_loop.inc` 把 50Hz `IMU,` 行的发送条件从"仅 COLLECT"扩为 **`MODE_COLLECT || MODE_AUTO`**(`IMU_Task()` 在主循环无条件 ~100Hz 采样，故 AUTO 下 g_imu 新鲜)；**线速度** 来自 0x03 的 `SPD_MEAS`——Jetson `serial_bridge._handle_auto_status` 把 `speed_mm_s` 折成里程计线速度(mm/s→0.1km/h)。两侧均已编译/烧录/导入通过。台架(§9)再核对 odom 直行x↑/左转yaw↑。

## 9. 联调验收（两侧都实现后）
1. **台架**（前轮+驱动轮架空，断电可达）：
   - Jetson 发 0x11 扫 STEER → 看舵机**方向/角度**对不对（确认 STEER 左负右正物理成立 → 定 Jetson `STEER_SIGN`）。
   - 发 TARGET_SPEED → 看 FOC 转速是否闭环跟随、方向对。
   - **断帧测试**：Jetson 停发 → 确认 300ms 内自动刹停 + 回中。
   - 急停/遥控接管 → 确认立即退出 AUTO。
2. **地面**：极低速、短距离、人握遥控随时接管，跑 Nav2 出的 /cmd_vel 闭环。

### ✅ 台架联调结果(2026-06-02，驱动轮悬空，工具 `~/bench_auto_test.py`/`~/wdiag2.py`)
- 0x03 状态握手 ✅(~10Hz);arm 条件 = `foc_valid(=FocCAN状态新鲜) + 整车使能 + D档 + 0x11心跳<300ms + AUTO_ENABLE + 无急停`,**只需"使能+D档"**(foc_valid 开机即满足,无需"标定")。
- 转向 ✅:cmd 20°→舵机反馈 20°;+→右,确认 `STEER_SIGN=-1.0`。
- 速度 ✅:cmd 200mm/s→实测 ~200(±15% 波动,PI 待调,§8)。
- 看门狗 ✅:停发 0x11 → ~300ms de-arm + 刹车。
- **🐞 发现并修复关键 bug**:原 Windows 版自动模式下 `FocCAN_TestPollUart`(USART1 行消费者)与 `TrainUART_PollAutoCmd`(0x11 二进制消费者)**并发抢同一环形缓冲**,偷走 0x11 字节→解析失败→`auto_cmd_fresh` 闪断→arm 掉→刹车点动(ARMED 60帧里闪 12~19 次)。**修复**(`dla_cart_loop.inc`):`AUTO`/`FOLLOW` 模式下不调 `FocCAN_TestPollUart`,让二进制 poll 独占串口(这两模式 Jetson 只发二进制;CLI/MAP-SAVE 不在这两模式用)。修后 ARMED 60/60 稳、速度无断点。已编译(笔记本 apt GCC)+ISP 烧录验证。

## 10. 固件侧实现记录（DlaBoard_A，2026-06-02）
已实现并通过 CubeIDE GCC 编译（`Debug/DlaBoard_A.elf`，无 error/warning）。改动文件：
- `Core/Inc/steering.h`：枚举加 `MODE_AUTO`。
- `Core/Inc/train_uart.h` / `Core/Src/train_uart.c`：
  - 新增 `AutoCmd_t` + `TrainUART_PollAutoCmd()` 解析 `0x11`；新增 `AutoStatus_t` + `TrainUART_SendAutoStatus()` 发 `0x03`。
  - RX 状态机重构为通用 `poll_frame()`，`PollFollowCmd`/`PollAutoCmd` 共用（按 TYPE 分发）。
  - 移除旧 `TrainUART_SendAutoReady()`（原训练入口误用 0x03，已删；0x03 现专属自动状态）。
  - 新增 `TRAIN_AUTO_FLAG_ENABLE` / `TRAIN_AUTO_ST_*` 位定义。
- `Core/Src/dla_cart_app.c`：`MODE_AUTO` 参数宏(`AUTO_CMD_TIMEOUT_MS/AUTO_MAX_SPEED_MM_S/AUTO_MAX_THROTTLE_PERMILLE/AUTO_SPD_*`)、静态状态、速度环积分项。
- `Core/Src/dla_cart_loop.inc`：屏键进入 `MODE_AUTO`、0x11 轮询+看门狗+clamp、arm 判定、速度闭环 PI(20ms)→FOC 0x210、`case MODE_AUTO` 转向伺服、0x03 状态上报(10Hz)、伺服心跳/模式刷新加 AUTO。

**烧录**（Windows，STLINK，F407 SN 见 `--list`）：
`STM32_Programmer_CLI -c port=SWD sn=<F407> -w Debug/DlaBoard_A.elf -v -rst`

**✅ Linux(Ubuntu 笔记本)上也能编译+烧录(2026-06-02 已验证，方案A 即如此烧入)：**
- 工具链：`sudo apt install gcc-arm-none-eabi`(10.3；ST 原版 13.3，编 F4 HAL 无碍)。
- 适配 CubeIDE 生成的 makefile(已改，CubeIDE 在 Windows 重新生成会覆盖回去，再编需重做)：① `Debug/makefile` 里 linker script 的 Windows 绝对路径 `E:\...STM32F407VETX_FLASH.ld` → 相对 `../STM32F407VETX_FLASH.ld`；② 各 `Debug/**/subdir.mk` 去掉 ST 专有 flag `-fcyclomatic-complexity`(主线 GCC 不认)。
- 编译：`cd Debug && make clean && make -j4 all`(clean 必须，避开 Windows 残留 .d 路径)。产物 `DlaBoard_A.hex`(~55KB)。
- 烧录(ISP/CH340，自动下载电路 DTR=复位/RTS=BootLoader)：控制板 CH340 接笔记本(`/dev/ttyUSB0`)，`cd Debug && python3 ISP.py /dev/ttyUSB0 DlaBoard_A.hex`。需 pyserial、用户在 dialout 组。校验通过后自动跳转运行。

---
**对齐口径**：先按本规范各自实现；§8 任一项一旦定了，**就回来改这份文档**，两侧再同步。Jetson 侧入口 `~/ros2_ws/HANDOFF.md`（本工作区无法访问 Jetson，§7 改动需在 Jetson 机执行）。
