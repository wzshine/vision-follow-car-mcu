#ifndef DLA_CART_APP_H
#define DLA_CART_APP_H

#include <stdint.h>

/* 独立看门狗。Init 要在 main() 里尽早调用 —— 它同时负责取走并清掉复位原因,
 * 那是唯一能知道"上一次是被狗咬醒的"的地方。实现与超时取值见 dla_cart_app.c。 */
void Board_WatchdogInit(void);
void Board_WatchdogKick(void);
extern volatile uint32_t g_reset_cause;   /* 上电时的 RCC->CSR >> 24 */

void DlaCart_AppInit(void);
void DlaCart_AppLoopIteration(void);

#endif
