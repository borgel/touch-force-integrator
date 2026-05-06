#include "main.h"
#include "stm32h7rsxx_it.h"
#include "usbpd.h"

extern HCD_HandleTypeDef hhcd_USB_OTG_HS;
extern TIM_HandleTypeDef htim6;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern LTDC_HandleTypeDef hlcd_ltdc;

// NOTE: If you are using CMSIS, the registers can also be
// accessed through CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk
#define HALT_IF_DEBUGGING()                              \
    do {                                                   \
      if ((*(volatile uint32_t *)0xE000EDF0) & (1 << 0)) { \
        __asm("bkpt 1");                                   \
      }                                                    \
    } while (0)

typedef struct __attribute__((packed)) ContextStateFrame {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t return_address;
  uint32_t xpsr;
} sContextStateFrame;

#define HARDFAULT_HANDLING_ASM(_x)               \
    __asm volatile(                                \
        "tst lr, #4 \n"                            \
        "ite eq \n"                                \
        "mrseq r0, msp \n"                         \
        "mrsne r0, psp \n"                         \
        "b my_fault_handler_c \n"                  \
    )

// Disable optimizations for this function so "frame" argument
// does not get optimized away
__attribute__((optimize("O0")))
void my_fault_handler_c(sContextStateFrame *frame) {
  // If and only if a debugger is attached, execute a breakpoint
  // instruction so we can take a look at what triggered the fault
  HALT_IF_DEBUGGING();

  // Logic for dealing with the exception. Typically:
  //  - log the fault which occurred for postmortem analysis
  //  - If the fault is recoverable,
  //    - clear errors and return back to Thread Mode
  //  - else
  //    - reboot system
}

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  HARDFAULT_HANDLING_ASM();
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void DebugMon_Handler(void)
{
}

void GPDMA1_Channel0_IRQHandler(void)
{
}

void GPDMA1_Channel1_IRQHandler(void)
{
}

void TIM6_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}

void OTG_HS_IRQHandler(void)
{
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_HS);
}

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void UCPD1_IRQHandler(void)
{
  USBPD_PORT0_IRQHandler();
}

/* Drives the line-event interrupt that signals VBLANK to the render
 * task — see HAL_LTDC_LineEventCallback in main.c. */
void LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&hlcd_ltdc);
}

#if defined(TCPP0203_SUPPORT)
/* EXTI line associated with the TCPP0203 FLGn line. */
void EXTI8_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) != RESET)
  {
    BSP_USBPD_PWR_EventCallback(USBPD_PWR_TYPE_C_PORT_1);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
  }
}
#endif /* TCPP0203_SUPPORT */
