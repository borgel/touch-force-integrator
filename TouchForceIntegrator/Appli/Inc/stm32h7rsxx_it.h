#ifndef __STM32H7RSxx_IT_H
#define __STM32H7RSxx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void GPDMA1_Channel0_IRQHandler(void);
void GPDMA1_Channel1_IRQHandler(void);
void TIM6_IRQHandler(void);
void OTG_HS_IRQHandler(void);
void UCPD1_IRQHandler(void);

#if defined(TCPP0203_SUPPORT)
void EXTI8_IRQHandler(void);
#endif /* TCPP0203_SUPPORT */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7RSxx_IT_H */
