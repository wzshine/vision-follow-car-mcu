#include "imu_lsm6dsv.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/* ---- LSM6DSV 寄存器 ---- */
#define LSM_WHO_AM_I     0x0FU
#define LSM_CTRL1        0x10U   /* [3:0]ODR_XL [6:4]OP_MODE_XL */
#define LSM_CTRL2        0x11U   /* [3:0]ODR_G  [6:4]OP_MODE_G  */
#define LSM_CTRL3        0x12U   /* bit0 SW_RESET, bit2 IF_INC, bit6 BDU, bit7 BOOT */
#define LSM_CTRL6        0x15U   /* [3:0]FS_G */
#define LSM_CTRL8        0x17U   /* [1:0]FS_XL */
#define LSM_OUT_TEMP_L   0x20U   /* TEMP(2) GYRO(6) ACCEL(6) 连续自增 */

#define LSM_WHOAMI_VAL   0x70U

#define LSM_READ_BIT     0x80U

/* 配置值：120Hz 高性能；BDU+IF_INC；量程 加速度±4g、角速度±2000dps（原始 LSB 上报，
 * 单位换算与 Jetson 帧格式一并最终确定）。 */
#define LSM_CTRL1_VAL    0x06U   /* OP_MODE_XL=000(HP) | ODR_XL=0110(120Hz) */
#define LSM_CTRL2_VAL    0x06U   /* OP_MODE_G =000(HP) | ODR_G =0110(120Hz) */
#define LSM_CTRL3_VAL    0x44U   /* BDU(0x40) | IF_INC(0x04) */
#define LSM_CTRL6_VAL    0x04U   /* FS_G  = ±2000 dps */
#define LSM_CTRL8_VAL    0x01U   /* FS_XL = ±4 g */

#define IMU_POLL_PERIOD_MS  10U
#define IMU_FRESH_MS        200U

Imu_t g_imu;

static inline void cs_low(void)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); }

static void imu_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2];
  tx[0] = reg & 0x7FU;          /* bit7=0 写 */
  tx[1] = val;
  cs_low();
  (void)HAL_SPI_Transmit(&hspi1, tx, 2U, 10U);
  cs_high();
}

static void imu_read_regs(uint8_t reg, uint8_t *buf, uint16_t n)
{
  uint8_t addr = (uint8_t)(reg | LSM_READ_BIT);   /* bit7=1 读, IF_INC 自增 */
  cs_low();
  (void)HAL_SPI_Transmit(&hspi1, &addr, 1U, 10U);
  (void)HAL_SPI_Receive(&hspi1, buf, n, 20U);
  cs_high();
}

static uint8_t imu_read_reg(uint8_t reg)
{
  uint8_t v = 0U;
  imu_read_regs(reg, &v, 1U);
  return v;
}

static int16_t s16le(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint8_t IMU_Init(void)
{
  g_imu.ok = 0U;
  g_imu.read_count = 0U;
  g_imu.last_ms = 0U;

  /* CubeMX 默认 /2(≈42MHz) 超 LSM6DSV 10MHz 上限 → 降到 /16(≈5.25MHz) 重初始化。 */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    return 0U;
  }

  cs_high();
  HAL_Delay(10);

  /* 软复位后等待，再校验 WHO_AM_I */
  imu_write_reg(LSM_CTRL3, 0x01U);   /* SW_RESET */
  HAL_Delay(20);

  g_imu.whoami = imu_read_reg(LSM_WHO_AM_I);
  if (g_imu.whoami != LSM_WHOAMI_VAL) {
    return 0U;
  }

  imu_write_reg(LSM_CTRL3, LSM_CTRL3_VAL);   /* BDU + IF_INC */
  imu_write_reg(LSM_CTRL6, LSM_CTRL6_VAL);   /* 陀螺量程 */
  imu_write_reg(LSM_CTRL8, LSM_CTRL8_VAL);   /* 加速度量程 */
  imu_write_reg(LSM_CTRL1, LSM_CTRL1_VAL);   /* 加速度 120Hz HP */
  imu_write_reg(LSM_CTRL2, LSM_CTRL2_VAL);   /* 陀螺 120Hz HP */
  HAL_Delay(5);

  g_imu.ok = 1U;
  return 1U;
}

void IMU_Task(void)
{
  static uint32_t last_ms = 0U;
  uint32_t now = HAL_GetTick();
  uint8_t b[14];

  if (g_imu.ok == 0U) {
    return;
  }
  if ((now - last_ms) < IMU_POLL_PERIOD_MS) {
    return;
  }
  last_ms = now;

  /* TEMP(0x20..21) GYRO(0x22..27) ACCEL(0x28..2D) 一次自增读取 */
  imu_read_regs(LSM_OUT_TEMP_L, b, 14U);

  g_imu.temp = s16le(&b[0]);
  g_imu.gx   = s16le(&b[2]);
  g_imu.gy   = s16le(&b[4]);
  g_imu.gz   = s16le(&b[6]);
  g_imu.ax   = s16le(&b[8]);
  g_imu.ay   = s16le(&b[10]);
  g_imu.az   = s16le(&b[12]);
  g_imu.last_ms = now;
  g_imu.read_count++;
}

uint8_t IMU_Fresh(uint32_t now_ms)
{
  if ((g_imu.ok == 0U) || (g_imu.last_ms == 0U)) {
    return 0U;
  }
  return ((now_ms - g_imu.last_ms) <= IMU_FRESH_MS) ? 1U : 0U;
}
