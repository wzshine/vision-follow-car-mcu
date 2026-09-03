#ifndef CART_PARAMS_H
#define CART_PARAMS_H

/*
 * 整车控制板的可调常量 —— **每个事实一个家**。
 *
 * 从前它们散在三处：dla_cart_app.c 的文件级 #define、dla_cart_loop.inc
 * 里**函数体内部**的 #define、以及若干直接写在表达式里的字面量。
 * 后果之一已经踩到了：同一个油门踏板、同一路 ADC，在同一个文件里有**两套
 * 标定** —— 驱动路径 850/3500，报给 Jetson 的训练帧 1100/3100。
 * 车按一条曲线走，录给模仿学习的是另一条曲线，两条谁也没提过对方。
 *
 * 把它们收到一处，还有第二个理由：参数表要能自描述，而
 * **表里的 max/default 必须由真正执行那件事的常量推导，不能手写**。
 * 表写一个范围、代码执行另一个范围，上位机就会按表画滑块、按代码夹值，
 * 而框里的数不是机器里的数。要做到这一点，执行的那一方和描述的那一方
 * 必须 include 同一个头文件。
 */

/* ---- 转向几何(第三方伺服路线，FOC_steer 接通后这一组会重写) ---- */
#define POSITION_PER_REVOLUTION  10000
#define AUTO_ROTATION_SPEED      280      /* 厂商单位，含义未文档化 */
#define POSITION_TOLERANCE       100
#define STEERING_GEAR_RATIO      15
#define STEERING_RANGE_DEG       180
#define STEERING_PHYSICAL_DEG    37       /* 物理转角上限，0.1° 单位是 x10 */
#define STEERING_POSITION_RANGE  ((STEERING_RANGE_DEG * STEERING_GEAR_RATIO * POSITION_PER_REVOLUTION / 360) * 40 / 10)
#define MAX_STEER_01             370      /* = STEERING_PHYSICAL_DEG * 10 */

/* ---- 串口屏 ---- */
#define SCREEN_HEARTBEAT_INTERVAL_MS  2000

/* ---- 上位机(Jetson)链路 ---- */
#define FOLLOW_CMD_TIMEOUT_MS      200U
#define FOLLOW_READY_RESEND_MS     100U
#define AUTO_CMD_TIMEOUT_MS        300U   /* §6.1 指令看门狗：超时→刹车+停+回中 */
#define AUTO_STATUS_PERIOD_MS      100U   /* §4 状态帧 ~10Hz */

/* ---- 整车使能与档位输入 ---- */
#define VEHICLE_ENABLE_DEBOUNCE_MS 50U
#define GEAR_DEBOUNCE_MS           50U
#define GEAR_BOOT_IGNORE_MS        500U

/* ---- 自动驾驶(见 AUTONOMOUS_MODE_SPEC §6) ----
 * 速度环增益/限速是保守起点，仍待台架标定(§9)。 */
#define AUTO_MAX_SPEED_MM_S        1500   /* §6.2 硬限速 1.5 m/s，与上位机独立双保险 */
#define AUTO_MAX_THROTTLE_PERMILLE 600U   /* 自动模式油门上限，0..1000 */
#define AUTO_SPD_KP_NUM            3L
#define AUTO_SPD_KP_DEN            10L
#define AUTO_SPD_KI_NUM            1L
#define AUTO_SPD_KI_DEN            50L
#define AUTO_SPD_I_MAX             30000L /* 积分限幅(抗饱和) */

/*
 * ---- 油门踏板标定：一路 ADC，**一套**常量 ----
 *
 * ! 从前这里有两套。驱动路径(车真正听的那条)是 850/3500；
 *   训练/采集帧报给 Jetson 的是 1100/3100，写在几百行之外的另一个
 *   函数体内 #define 里，两边谁也没提过对方。
 *
 *   数字上的差不是小事：ADC=3300 时驱动路径给 92%，训练帧报 100%；
 *   ADC=1000 时驱动路径已经在出力，训练帧还报 0。
 *   **模仿学习录的是"司机踩了多少"，而它录的和车做的不是一回事。**
 *
 * ! 保留的是 850/3500，因为那是**真正被执行**的那一套 —— 报出去的值
 *   要和车做的事对得上，那就得以做事的那一方为准。
 *   ⚠ 这两套哪一套更贴合踏板的物理行程，源码里判断不出来，需要台架实测。
 *   现在起它们至少是同一套。
 */
#define THROTTLE_ADC_DEAD    850U
#define THROTTLE_ADC_FULL    3500U
#define THROTTLE_DEAD_PERMILLE 30U   /* 小于此值的油门当作 0(踏板回位间隙) */
#define STICK_ACTIVE_THRESH  80      /* 摇杆偏离中位超过此值算有输入 */

#endif
