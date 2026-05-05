/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include "usbpd.h"
#include "usb_host.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_host.h"
#include "usbd_cdc_if.h"
#include "stm32_lcd.h"
#include "stm32h7s78_discovery.h"
#include "stm32h7s78_discovery_lcd.h"

#include "usbh_hid_wisecoco.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;

/* USB host/device pumping task: runs MX_USB_HOST_Init + MX_USB_DEVICE_Init
 * once, then loops calling HID_Process to drive the wisecoco state machine.
 * Note: MX_USB_HOST_Process is *not* called here because USBH_USE_OS == 1
 * makes the host stack spawn its own internal thread that pumps
 * USBH_Process from the URB IRQ event queue. */
osThreadId_t USB_TaskHandle;
const osThreadAttr_t USB_Task_attributes = {
  .name = "USB_Task",
  .stack_size = 512 * 4,
  /* Stay at Normal — same as USBH_Thread (the USB stack's internal
   * thread, see USBH_PROCESS_PRIO in usbh_conf.h). Both call
   * USBH_Process, which mutates phost->gState and isn't reentrant.
   * Same priority means FreeRTOS round-robin only preempts on tick
   * boundaries (~1 ms), keeping the race window small. Higher priority
   * here would let our pre-empt USBH_Thread mid-USBH_Process and
   * corrupt the state machine. */
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t CDC_TaskHandle;
const osThreadAttr_t CDC_Task_attributes = {
  .name = "CDC_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t Render_TaskHandle;
const osThreadAttr_t Render_Task_attributes = {
  .name = "Render_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_UART4_Init(void);
static void MX_UCPD1_Init(void);
void _USB_Task(void *argument);
void _CDC_Task(void *argument);
void _Render_Task(void *argument);

/* USER CODE BEGIN PFP */
#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
size_t __write(int file, unsigned char const *ptr, size_t len);
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
/* ARM Compiler 5/6*/
#define PUTCHAR_PROTOTYPE int fputc(int ch)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif /* __ICCARM__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * VBLANK signal for the render task. HAL_LTDC_LineEventCallback below
 * fires once per frame (configured for line 0, the start of the visible
 * region after vertical front porch) and gives this semaphore. The
 * render task takes it between SwapVisibleBuffer (which queues an
 * address change for the LTDC to apply at next VBLANK) and
 * SwapDrawBuffer (which is only safe AFTER the LTDC has actually
 * picked up the new visible buffer — otherwise we'd start drawing
 * over a buffer the LTDC is still scanning out).
 */
static StaticSemaphore_t s_vblankSemMeta;
static SemaphoreHandle_t s_vblankSem;

void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc)
{
  if (s_vblankSem != NULL)
  {
    BaseType_t hpw = pdFALSE;
    (void)xSemaphoreGiveFromISR(s_vblankSem, &hpw);
    portYIELD_FROM_ISR(hpw);
  }
  /* Re-arm for the next frame; HAL_LTDC_ProgramLineEvent disables and
   * re-enables the line interrupt, so this is the canonical "tick me
   * again" call from inside the callback. */
  HAL_LTDC_ProgramLineEvent(hltdc, 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Update SystemCoreClock variable according to RCC registers values. */
  SystemCoreClockUpdate();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_UART4_Init();
  MX_UCPD1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Create the wisecoco frame-publication semaphore and the USB host
   * event-wakeup semaphore before any USB activity and before the
   * tasks that wait on them. Both are static, no scheduler needed. */
  USBH_HID_WisecocoAppInit();
  USBH_HostEvent_AppInit();

  /* USBPD initialisation ---------------------------------*/
  MX_USBPD_Init();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  USB_TaskHandle = osThreadNew(_USB_Task, NULL, &USB_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  CDC_TaskHandle = osThreadNew(_CDC_Task, NULL, &CDC_Task_attributes);
  Render_TaskHandle = osThreadNew(_Render_Task, NULL, &Render_Task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_DMA_InitTypeDef DMA_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOM);
  /** UCPD1 GPIO Configuration
  PM1   ------> UCPD1_CC2
  PM0   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOM, &GPIO_InitStruct);

  /* UCPD1 DMA Init */

  /* GPDMA1_REQUEST_UCPD1_RX Init */
  DMA_InitStruct.SrcAddress = 0x00000000U;
  DMA_InitStruct.DestAddress = 0x00000000U;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
  DMA_InitStruct.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;
  DMA_InitStruct.SrcBurstLength = 1;
  DMA_InitStruct.DestBurstLength = 1;
  DMA_InitStruct.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_BYTE;
  DMA_InitStruct.DestDataWidth = LL_DMA_DEST_DATAWIDTH_BYTE;
  DMA_InitStruct.SrcIncMode = LL_DMA_SRC_FIXED;
  DMA_InitStruct.DestIncMode = LL_DMA_DEST_FIXED;
  DMA_InitStruct.Priority = LL_DMA_LOW_PRIORITY_LOW_WEIGHT;
  DMA_InitStruct.BlkDataLength = 0x00000000U;
  DMA_InitStruct.TriggerMode = LL_DMA_TRIGM_BLK_TRANSFER;
  DMA_InitStruct.TriggerPolarity = LL_DMA_TRIG_POLARITY_MASKED;
  DMA_InitStruct.TriggerSelection = 0x00000000U;
  DMA_InitStruct.Request = LL_GPDMA1_REQUEST_UCPD1_RX;
  DMA_InitStruct.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
  DMA_InitStruct.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT0;
  DMA_InitStruct.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT0;
  DMA_InitStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT1;
  DMA_InitStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
  DMA_InitStruct.LinkedListBaseAddr = 0x00000000U;
  DMA_InitStruct.LinkedListAddrOffset = 0x00000000U;
  LL_DMA_Init(GPDMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);

  /* GPDMA1_REQUEST_UCPD1_TX Init */
  DMA_InitStruct.SrcAddress = 0x00000000U;
  DMA_InitStruct.DestAddress = 0x00000000U;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStruct.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
  DMA_InitStruct.DataAlignment = LL_DMA_DATA_ALIGN_ZEROPADD;
  DMA_InitStruct.SrcBurstLength = 1;
  DMA_InitStruct.DestBurstLength = 1;
  DMA_InitStruct.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_BYTE;
  DMA_InitStruct.DestDataWidth = LL_DMA_DEST_DATAWIDTH_BYTE;
  DMA_InitStruct.SrcIncMode = LL_DMA_SRC_FIXED;
  DMA_InitStruct.DestIncMode = LL_DMA_DEST_FIXED;
  DMA_InitStruct.Priority = LL_DMA_LOW_PRIORITY_LOW_WEIGHT;
  DMA_InitStruct.BlkDataLength = 0x00000000U;
  DMA_InitStruct.TriggerMode = LL_DMA_TRIGM_BLK_TRANSFER;
  DMA_InitStruct.TriggerPolarity = LL_DMA_TRIG_POLARITY_MASKED;
  DMA_InitStruct.TriggerSelection = 0x00000000U;
  DMA_InitStruct.Request = LL_GPDMA1_REQUEST_UCPD1_TX;
  DMA_InitStruct.TransferEventMode = LL_DMA_TCEM_BLK_TRANSFER;
  DMA_InitStruct.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT0;
  DMA_InitStruct.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT0;
  DMA_InitStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT1;
  DMA_InitStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
  DMA_InitStruct.LinkedListBaseAddr = 0x00000000U;
  DMA_InitStruct.LinkedListAddrOffset = 0x00000000U;
  LL_DMA_Init(GPDMA1, LL_DMA_CHANNEL_0, &DMA_InitStruct);

  /* UCPD1 interrupt Init */
  NVIC_SetPriority(UCPD1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(UCPD1_IRQn);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOM_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
#if defined(__ICCARM__)
size_t __write(int file, unsigned char const *ptr, size_t len)
{
  size_t idx;
  unsigned char const *pdata = ptr;

  for (idx = 0; idx < len; idx++)
  {
    iar_fputc((int)*pdata);
    pdata++;
  }
  return len;
}
#endif /* __ICCARM__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of putchar here */
  /* e.g. write a character to the UART4 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header__USB_Task */
/**
  * @brief  USB host/device pumping task. Brings up the host stack (which
  *         spawns its own internal USBH_Thread for USBH_Process) and the
  *         device stack, then loops driving HID_Process to consume parsed
  *         touch reports.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header__USB_Task */
void _USB_Task(void *argument)
{
  (void)argument;

  /* USER CODE BEGIN 5 */
  USBH_HID_WisecocoSetRotation(USBH_WC_ROTATE_90);

  MX_USB_HOST_Init();
  MX_USB_DEVICE_Init();

  for (;;) {
    /* USBH_USE_OS == 1 makes the host stack spawn its own internal thread
     * that pumps USBH_Process from URB IRQ events, but enumeration state
     * transitions still need this user-side pump to make progress —
     * matches the pattern in ST's HID_RTOS reference example. */
    MX_USB_HOST_Process();
    HID_Process();

    /* The HCD callbacks in usbh_conf.c give this semaphore on every
     * USB event of interest (URB completion, port attach/detach, port
     * enable/disable). When USB activity is happening, this take
     * returns immediately and we pump again — effectively event-driven.
     *
     * The timeout is a backstop, not a polling rate: some host state
     * transitions (timeouts inside USBH_Process, NAK retries that
     * don't surface as URB transitions, certain phase changes) advance
     * without firing any HAL callback, so the timeout makes sure we
     * eventually re-pump the state machine even if no IRQ ever arrives.
     * Return value is intentionally ignored — whether woken by event
     * or timeout, the action is the same. */
    (void)USBH_HostEvent_Wait(5U);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header__Render_Task */
/**
  * @brief  LCD rendering task. Brings up the LCD with two layers (static
  *         background on layer 0, animated dots on layer 1 with color
  *         keying for transparency), then waits on the wisecoco
  *         frame-published semaphore and redraws on each new frame.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header__Render_Task */
void _Render_Task(void *argument)
{
  (void)argument;
  int res;

  res = BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
  assert(res == 0);
  /* Drop the backlight: when the board is ST-Link-powered, peak
   * DMA2D + USB + 50%-backlight current sagged the 5V rail enough
   * to glitch the LCD interface (LD10 visibly flickered in sync
   * with display flicker). 20% is still readable indoors and
   * keeps the rail stable. */
  BSP_LCD_SetBrightness(0, 20);
  BSP_LCD_Reload(0, BSP_LCD_RELOAD_VERTICAL_BLANKING);
  BSP_LCD_EnableDoubleBuffering(0, BSP_LCD_LAYER_BACKGROUND);
  res = BSP_LCD_InitLayer1(0);
  assert(res == 0);
  BSP_LCD_EnableDoubleBuffering(0, BSP_LCD_LAYER_FOREGROUND);

  /* VBLANK signaling: create the semaphore, enable the LTDC IRQ, and
   * arm the first line-event interrupt. From here on the callback in
   * USER CODE 0 above keeps re-arming itself, so we get one
   * xSemaphoreGiveFromISR per frame indefinitely. */
  s_vblankSem = xSemaphoreCreateBinaryStatic(&s_vblankSemMeta);
  HAL_NVIC_SetPriority(LTDC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);
  HAL_LTDC_ProgramLineEvent(&hlcd_ltdc, 0);

  // connect our lower level LCD functions to the higher level driver
  UTIL_LCD_SetFuncDriver(&LCD_Driver);

  // 4:3 touch surface anchored top-left of the 800x480 landscape LCD (2880:2160 = 4:3),
  // leaving the right 160-px strip for future metadata.
  const uint16_t TOUCH_W = 640;
  const uint16_t TOUCH_H = 480;
  const uint16_t TOUCH_X0 = 0;
  const uint16_t TOUCH_Y0 = 0;

  // Layer 1 (foreground) uses color keying for transparency: any pixel of
  // KEY_565 on layer 1 lets the underlying layer 0 (background) show through.
  // KEY_888 is the same color in 24-bit form for the LTDC color-key register.
  const uint32_t KEY_565 = LCD_COLOR_RGB565_MAGENTA;
  const uint32_t KEY_888 = 0x00FF00FFU;
  BSP_LCD_SetColorKeying(0, 1, KEY_888);

  // ---- Layer 0 (background): drawn ONCE. ----
  BSP_LCD_SetActiveLayer(0, 0);
  for (int i = 0; i < 2; i++) {
    UTIL_LCD_Clear(UTIL_LCD_COLOR_BLACK);
    UTIL_LCD_FillRect(TOUCH_X0, TOUCH_Y0, TOUCH_W, TOUCH_H, 0xFFFFFFFF);
    BSP_LCD_SwapVisibleBuffer(0, BSP_LCD_LAYER_BACKGROUND);
    BSP_LCD_SwapDrawBuffer(0, BSP_LCD_LAYER_BACKGROUND);
  }

  // ---- Layer 1 (foreground): initialize both buffers to fully transparent. ----
  BSP_LCD_SetActiveLayer(0, 1);
  for (int i = 0; i < 2; i++) {
    UTIL_LCD_Clear(KEY_565);
    BSP_LCD_SwapVisibleBuffer(0, BSP_LCD_LAYER_FOREGROUND);
    BSP_LCD_SwapDrawBuffer(0, BSP_LCD_LAYER_FOREGROUND);
  }

  struct USBH_LatestWisecocoData snapshot;
  for (;;) {
    // Wait for the next published touch frame. When the touchscreen is
    // idle the producer doesn't publish, so this task sleeps until there's
    // actually something new to render.
    if (!USBH_HID_WisecocoWaitFrame(&snapshot, HAL_MAX_DELAY)) {
      continue;
    }

    // Clear only the touch area to KEY (transparent), erasing last frame's dots.
    UTIL_LCD_FillRect(TOUCH_X0, TOUCH_Y0, TOUCH_W, TOUCH_H, KEY_565);

    for (unsigned i = 0; i < snapshot.liveTouches; i++) {
      struct USBH_WCSingleFinger const * const f = &snapshot.fingers[i];
      if (f->touching) {
        UTIL_LCD_FillRect(TOUCH_X0 + f->xFrac * TOUCH_W,
                          TOUCH_Y0 + f->yFrac * TOUCH_H,
                          5, 5, 0xFF000000);
      }
    }

    /* SwapVisibleBuffer queues a layer-address change that the LTDC
     * applies at the NEXT VBLANK (RELOAD_VERTICAL_BLANKING was set at
     * init). Until that VBLANK, the LTDC is still scanning the OLD
     * buffer, so SwapDrawBuffer would redirect the next iteration's
     * draws to the still-being-scanned buffer and we'd tear.
     *
     * Drain any stale give from earlier in the frame, then wait for
     * the next VBLANK to confirm the LTDC has actually picked up the
     * new visible buffer. SwapDrawBuffer is then safe. */
    BSP_LCD_SwapVisibleBuffer(0, BSP_LCD_LAYER_FOREGROUND);
    (void)xSemaphoreTake(s_vblankSem, 0);
    (void)xSemaphoreTake(s_vblankSem, pdMS_TO_TICKS(20));
    BSP_LCD_SwapDrawBuffer(0, BSP_LCD_LAYER_FOREGROUND);
  }
}

/* USER CODE BEGIN Header__CDC_Task */
/**
  * @brief  Echo example for the USB CDC virtual COM port. Waits for the host
  *         to enumerate the device, sends a greeting, then echoes any text
  *         the host sends back as "echo: <text>\r\n".
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header__CDC_Task */
void _CDC_Task(void *argument)
{
  (void)argument;

  /* MX_USB_DEVICE_Init runs from _USB_Task and calls CDC_AppInit before
   * starting the USB stack, so by the time the host has enumerated the
   * device the FreeRTOS objects backing CDC_Read / CDC_Write definitely
   * exist.  hUsbDeviceFS lives in usb_device.c. */
  extern USBD_HandleTypeDef hUsbDeviceFS;

  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    osDelay(100);
  }

  static const char greeting[] = "[CDC ready - type something]\r\n";
  (void)CDC_Write((const uint8_t *)greeting, sizeof(greeting) - 1U, 100U);

  uint8_t  rxBuf[64];
  char     txBuf[80];

  for (;;)
  {
    size_t n = CDC_Read(rxBuf, sizeof(rxBuf), osWaitForever);
    if (n == 0U)
    {
      continue;
    }

    /* Cap the printed payload at what fits after "echo: " and "\r\n". */
    if (n > sizeof(txBuf) - 9U)
    {
      n = sizeof(txBuf) - 9U;
    }
    int len = snprintf(txBuf, sizeof(txBuf), "echo: %.*s\r\n", (int)n, (const char *)rxBuf);
    if (len > 0)
    {
      (void)CDC_Write((const uint8_t *)txBuf, (uint16_t)len, 100U);
    }
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("Error handler\n");
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
