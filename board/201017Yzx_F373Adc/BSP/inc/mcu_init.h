#ifndef __MCU_INIT_H
#define __MCU_INIT_H
#ifdef __cplusplus
 extern "C" {
#endif

//#include "main.h"
#include "stm32f3xx_hal.h"
#include "bsp_usart.h"
#include "bsp_gpio.h"
#include "tos_k.h"

void board_init(void);
void SystemClock_Config(void);
void FeedIWDG(void);
void ToggleLed(void);
void SetBeeper(int high);
void Start4Phase(void);
#ifdef __cplusplus
}
#endif
#endif /*__ __MCU_INIT_H */
