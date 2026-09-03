#ifndef FW_IMAGE_H
#define FW_IMAGE_H

#include <stdint.h>

/*
 * ==================== 镜像自述头（DBIS §11） ====================
 *
 * 一段固件说出**它自己是什么**：多长、校验和多少、哪个版本、从哪个 commit 来、
 * 是不是 Debug 构建。
 *
 * ⚠⚠ **布局不是我定的，是上位机已经在解析的那一份。**
 * 判据是 `DlaStudio/packages/board-tools/src/image.ts` 里那几个常量：
 *
 *     HEADER_OFFSET = 0x200
 *     OFF_BYTES     = 8      image_bytes  (u32)
 *     OFF_CRC       = 12     image_crc32  (u32)
 *     OFF_VER       = 16     major/minor/patch (3 × u8)
 *     OFF_FLAGS     = 19     ver_flags    (bit0 = debug)
 *     OFF_SCM       = 20     scm 文本，12 字节
 *     MAGIC         = 4F 43 49 4D 21 47 30 31
 *
 * 差一个字节，`hasMagic()` 就认不出这是一个头，于是：
 *   - `imageCrc32()` 退化成整文件 CRC32（和我们盖的章对不上）；
 *   - `fw_flash_plan` 报"未盖章"；
 *   - `ver` 的 `image :` 行没有可信的字节数。
 *
 * ⚠ **magic 和驱动板用的是同一串**（`OCIM!G01`），尽管那个 `G` 看着像 G4 系列。
 *   它在上位机那边的含义是"这是一个 Daliang 固件镜像头"，不是芯片型号。
 *   另起一串 = 上位机认不出来，而认不出来和没有头是同一个结果。
 *
 * ---- 头为什么钉在 0x200，而不是"向量表后面" ----
 *
 * 加载方必须在**同意运行这段代码之前**读到它，所以不能问镜像自己头在哪。
 * 而"向量表后面"是一个会随中断向量数量变化的偏移 —— 那一天它会静默地移位。
 * ⇒ 由链接脚本钉死（`STM32F407VETX_FLASH.ld` 的 `.fw_header` 段）。
 * ⚠ F407 的向量表是 0x198 字节，0x200 正好在它之后，且八字节对齐。
 *
 * ---- 谁来填 ----
 *
 * 编译期只能填版本、flags、scm —— **长度和 CRC 要等链接完才知道**，
 * 由 `tools/stamp_image.py` 在构建之后写进 .bin/.hex（"盖章"）。
 * ⚠ 没盖章的镜像 `image_bytes` / `image_crc32` 是 0xFFFFFFFF，
 *   固件如实报 `crc unstamped` —— **"没有这个信息"和"这个信息是好的"
 *   必须是两个不同的答案**。
 */

#define FW_IMAGE_HEADER_OFFSET  0x200U
#define FW_IMAGE_MAGIC0         0x4D49434FUL   /* 'O','C','I','M' 小端 */
#define FW_IMAGE_MAGIC1         0x31304721UL   /* '!','G','0','1' 小端 */
#define FW_IMAGE_SCM_LEN        12U
#define FW_IMAGE_UNSTAMPED      0xFFFFFFFFUL

/** Debug 构建。⚠ 这一位是**编译期常量**，所以它在磁盘上的字节里就有 ——
 *  和 image_bytes / image_crc32 不同，后两者是事后盖上去的。
 *  ⇒ 上位机据此可以在**烧进去之前**就知道这个文件是不是 Debug 构建。 */
#define FW_IMAGE_FLAG_DEBUG     0x01U

typedef struct {
  uint32_t magic0;
  uint32_t magic1;
  uint32_t image_bytes;   /* 从向量表开头算起。盖章前 0xFFFFFFFF */
  uint32_t image_crc32;   /* 覆盖 image_bytes 个字节，**本字段读作 0** */
  uint8_t  ver_major;
  uint8_t  ver_minor;
  uint8_t  ver_patch;
  uint8_t  ver_flags;
  char     scm[FW_IMAGE_SCM_LEN];
} fw_image_t;

typedef enum {
  FW_IMAGE_OK = 0,      /* 盖过章，而且 CRC 和 flash 里的内容一致 */
  /* ⚠ 尾下划线不是笔误：上面那个 FW_IMAGE_UNSTAMPED 是**宏**（0xFFFFFFFF），
   * 同名的枚举成员会被它展开成一个数字常量，编译器报
   * "expected identifier before numeric constant"。驱动板那份也这么写。 */
  FW_IMAGE_UNSTAMPED_,  /* 编出来了但没盖章：开发构建 */
  FW_IMAGE_BAD_MAGIC,   /* 头不在它该在的地方 */
  FW_IMAGE_BAD_CRC,     /* 盖过章，但 flash 已经和它对不上了 */
  FW_IMAGE_BAD_LENGTH   /* 头声称的长度放不进 flash */
} fw_image_state_t;

/** 本镜像自带的那个头。永远不为 NULL。 */
const fw_image_t *FwImage_Header(void);

/*
 * 现场把整段镜像重算一遍 CRC 并和头里的比。
 *
 * ⚠ **每次调用真的重算**，不是报一个开机时存下来的结果。
 *   这条命令要回答的是"flash 现在还是不是当初烧进去的那些字节"，
 *   而一个在损坏发生之前算出来的缓存答案，正是那个会答错的答案。
 *   512 KB 全算约十几毫秒，它由人敲出来，不在任何循环里。
 */
fw_image_state_t FwImage_Check(void);

/** 一行给人看的：`0.1.0 4dacba0+ DEBUG, crc ok`。写进 `ver` 的 firmware 行。 */
void FwImage_Describe(char *out, unsigned size);

#endif
