#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7rsxx_hal.h"

#include "stm32h7rsxx_ll_ucpd.h"
#include "stm32h7rsxx_ll_bus.h"
#include "stm32h7rsxx_ll_cortex.h"
#include "stm32h7rsxx_ll_rcc.h"
#include "stm32h7rsxx_ll_system.h"
#include "stm32h7rsxx_ll_utils.h"
#include "stm32h7rsxx_ll_pwr.h"
#include "stm32h7rsxx_ll_gpio.h"
#include "stm32h7rsxx_ll_dma.h"

#include "stm32h7rsxx_ll_exti.h"

#include <stdbool.h>

void Error_Handler(void);

/* Defined in main.c — toggles the touchforce.v1 touch event
 * streamer. Called from protocol_task.c on a SetTouchStreaming
 * request. Default state at boot is enabled. */
void Touch_SetStreaming(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
