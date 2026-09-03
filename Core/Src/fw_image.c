#include "fw_image.h"
#include "fw_version.h"
#include <stdio.h>
#include <string.h>

/*
 * 头本身。放进 `.fw_header` 段，由链接脚本钉到 FLASH 起点 +0x200。
 *
 * ⚠ `used` + `section` 两个属性都要：`-ffunction-sections -fdata-sections`
 *   加上 `--gc-sections` 会把一个"没人引用"的常量整个丢掉，而这个头**本来
 *   就没有人引用** —— 它是给外面的工具读的。丢掉之后镜像里没有头，
 *   而编译和链接都不会说一个字。
 */
__attribute__((used, section(".fw_header")))
const fw_image_t g_fw_image_header = {
  .magic0      = FW_IMAGE_MAGIC0,
  .magic1      = FW_IMAGE_MAGIC1,
  /* ⚠ 编译期填不出来：长度要等链接完，CRC 要等长度。
   * 盖章前保持 0xFFFFFFFF —— 那正是"没盖章"的表示，不是 0。
   * 0 是一个合法的长度也是一个合法的 CRC。 */
  .image_bytes = FW_IMAGE_UNSTAMPED,
  .image_crc32 = FW_IMAGE_UNSTAMPED,
  .ver_major   = (uint8_t)FW_VER_MAJOR,
  .ver_minor   = (uint8_t)FW_VER_MINOR,
  .ver_patch   = (uint8_t)FW_VER_PATCH,
  .ver_flags   = (uint8_t)((FW_IS_DEBUG != 0U) ? FW_IMAGE_FLAG_DEBUG : 0U),
  .scm         = FW_SCM,
};

extern uint32_t _estack;     /* 只为拿到 flash 基址做个符号锚点，未使用 */

/* flash 基址：向量表就在这里，镜像从这里开始算长度。 */
#define FLASH_BASE_ADDR   0x08000000UL
#define FLASH_SIZE_BYTES  (512UL * 1024UL)

const fw_image_t *FwImage_Header(void)
{
  return (const fw_image_t *)(FLASH_BASE_ADDR + FW_IMAGE_HEADER_OFFSET);
}

/* 和上位机 image.ts / isp.ts 同一个 CRC-32（反射，0xEDB88320）。 */
static uint32_t crc32_run(const uint8_t *p, uint32_t n, uint32_t skip_at, uint32_t skip_len)
{
  uint32_t crc = 0xFFFFFFFFU;
  uint32_t i, b;
  for (i = 0U; i < n; i++) {
    /* ⭐ 本字段读作 0 —— 校验和不能覆盖它自己。 */
    uint8_t v = ((i >= skip_at) && (i < (skip_at + skip_len))) ? 0U : p[i];
    crc ^= (uint32_t)v;
    for (b = 0U; b < 8U; b++) {
      crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

fw_image_state_t FwImage_Check(void)
{
  const fw_image_t *h = FwImage_Header();
  uint32_t n;

  if ((h->magic0 != FW_IMAGE_MAGIC0) || (h->magic1 != FW_IMAGE_MAGIC1)) {
    return FW_IMAGE_BAD_MAGIC;
  }
  if ((h->image_bytes == FW_IMAGE_UNSTAMPED) || (h->image_crc32 == FW_IMAGE_UNSTAMPED)) {
    return FW_IMAGE_UNSTAMPED_;
  }
  n = h->image_bytes;
  if ((n == 0U) || (n > FLASH_SIZE_BYTES)) {
    return FW_IMAGE_BAD_LENGTH;
  }
  if (crc32_run((const uint8_t *)FLASH_BASE_ADDR, n,
                FW_IMAGE_HEADER_OFFSET + 12U, 4U) != h->image_crc32) {
    return FW_IMAGE_BAD_CRC;
  }
  return FW_IMAGE_OK;
}

void FwImage_Describe(char *out, unsigned size)
{
  const fw_image_t *h = FwImage_Header();
  char scm[FW_IMAGE_SCM_LEN + 1U];
  const char *verdict;

  if ((out == NULL) || (size == 0U)) { return; }

  memcpy(scm, h->scm, FW_IMAGE_SCM_LEN);
  scm[FW_IMAGE_SCM_LEN] = '\0';

  switch (FwImage_Check()) {
  case FW_IMAGE_OK:        verdict = "ok";        break;
  /* ⚠ `unstamped` 是上位机单独分了一支的词（stamped=false, imageOk=false）。
   * 写 `bad` 是在说一件没发生过的坏事。 */
  case FW_IMAGE_UNSTAMPED_: verdict = "unstamped"; break;
  case FW_IMAGE_BAD_CRC:   verdict = "bad";       break;
  default:                 verdict = "unstamped"; break;
  }

  (void)snprintf(out, size, "%u.%u.%u %s%s, crc %s",
                 (unsigned)h->ver_major, (unsigned)h->ver_minor, (unsigned)h->ver_patch,
                 scm,
                 ((h->ver_flags & FW_IMAGE_FLAG_DEBUG) != 0U) ? " DEBUG" : "",
                 verdict);
}
