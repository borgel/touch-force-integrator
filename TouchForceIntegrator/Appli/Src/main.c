#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include "usbpd.h"
#include "usb_host.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32_lcd.h"
#include "stm32h7s78_discovery.h"
#include "stm32h7s78_discovery_lcd.h"
#include "usbh_hid_wisecoco.h"
#include "protocol_task.h"
#include "haptic_area.h"
#include "cobs.h"
#include "pb_encode.h"
#include "touch_force.pb.h"
#include "usbd_def.h"  /* USBD_OK */

/* USB host enumeration state, defined in usb_host.c. APPLICATION_READY
 * once a device has enumerated; we treat that as "host link up" for the
 * status panel. */
extern ApplicationTypeDef Appli_state;
/* USB device handle (CDC-side), defined in usb_device.c. dev_state ==
 * USBD_STATE_CONFIGURED once the host PC has enumerated and configured
 * us. */
extern USBD_HandleTypeDef hUsbDeviceFS;

#include <stdio.h>
#include <string.h>

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

osThreadId_t CDC_TaskHandle;
const osThreadAttr_t CDC_Task_attributes = {
    .name = "CDC_Task",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Touch_TaskHandle;
const osThreadAttr_t Touch_Task_attributes = {
    .name = "Touch_Task",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_UART4_Init(void);
static void MX_UCPD1_Init(void);
void _USB_Task(void *argument);
void _CDC_Task(void *argument);
void _Touch_Task(void *argument);

#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
size_t __write(int file, unsigned char const *ptr, size_t len);
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION)
#define PUTCHAR_PROTOTYPE int fputc(int ch)
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif

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

/* Streaming state. Concurrency: .enabled is single-byte volatile
 * (atomic on Cortex-M); .eventsSent/.txFails may be torn for a few
 * microseconds when sampled, harmless for monotonic counters; the
 * buffers are touched only by _Touch_Task. Buffer size covers a
 * TouchFrameEvent with 10 fingers (~150 bytes encoded) wrapped in
 * a Frame, plus COBS overhead — not touchforce_v1_Frame_size, which
 * post-haptic-areas is sized to the worst-case oneof variant. */
#define PROTO_EVENT_BUF_BYTES   512U
#define PROTO_EVENT_TX_BYTES    512U

static struct {
  volatile bool     enabled;
  volatile uint32_t eventsSent;
  volatile uint32_t txFails;
  uint8_t           eventBuf[PROTO_EVENT_BUF_BYTES];
  uint8_t           eventTxBuf[PROTO_EVENT_TX_BYTES];
} s_streaming = {
  .enabled = true,
};

/* Right-side status panel: 160 px strip on layer 1, repainted only
 * on change. Layer 1 is double-buffered so a change repaints both
 * buffers before settling — s_statusDirty starts at 2 and decrements
 * per render. */
#define STATUS_X0       640U
#define STATUS_W        160U
#define STATUS_H        480U
#define STATUS_LED_R     12U

static struct {
  bool hostUp;
  bool devUp;
  bool hapticOn;
} s_displayedStatus;
static uint8_t s_statusDirty;

/* Layer 0: white touch area + haptic-area outlines. Repainted only
 * when HapticArea_GetGeneration() changes; like s_statusDirty, the
 * counter starts at 2 to cover both back buffers. */
static uint8_t  s_layer0Dirty;
static uint32_t s_lastHapticAreaGeneration;

/* Per-finger rising-edge tracker. wisecoco finger ids are stable
 * across a single touch (lift+redown gets a new id), so id equality
 * is enough to detect "this finger was already touching last frame". */
static uint8_t s_prevTouchingIds[MAX_TOUCHES];
static uint8_t s_prevTouchingCount;

/* HAL_GetTick value at which LD1 should turn off, or 0 when no
 * pulse is pending. Re-arming during an active pulse extends the
 * on-time so a burst reads as a single sustained pulse. */
static volatile uint32_t s_ledOffDeadline;

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

int main(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();

  SystemCoreClockUpdate();
  HAL_Init();

  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_UART4_Init();
  MX_UCPD1_Init();

  osKernelInitialize();

  /* Create the wisecoco frame-publication semaphore and the USB host
   * event-wakeup semaphore before any USB activity and before the
   * tasks that wait on them. Both are static, no scheduler needed. */
  USBH_HID_WisecocoAppInit();
  USBH_HostEvent_AppInit();
  HapticArea_Init();

  MX_USBPD_Init();

  USB_TaskHandle    = osThreadNew(_USB_Task,    NULL, &USB_Task_attributes);
  CDC_TaskHandle    = osThreadNew(_CDC_Task,    NULL, &CDC_Task_attributes);
  Touch_TaskHandle = osThreadNew(_Touch_Task, NULL, &Touch_Task_attributes);

  osKernelStart();

  /* Should never reach here — scheduler took over. */
  while (1)
  {
  }
}

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

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* USB host/device pumping task. Brings up the host stack (which
 * spawns its own internal USBH_Thread for USBH_Process) and the
 * device stack, then loops driving HID_Process to consume parsed
 * touch reports. */
void _USB_Task(void *argument)
{
  (void)argument;

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
}

/* Clear the touch area to KEY (transparent over layer 0) and draw
 * a 5x5 dot per currently-touching finger. Caller does the swap. */
static void Touch_RenderTouchDots(const struct USBH_LatestWisecocoData *snapshot)
{
  const uint16_t TOUCH_W  = 640;
  const uint16_t TOUCH_H  = 480;
  const uint16_t TOUCH_X0 = 0;
  const uint16_t TOUCH_Y0 = 0;
  const uint32_t KEY_565  = LCD_COLOR_RGB565_MAGENTA;

  UTIL_LCD_FillRect(TOUCH_X0, TOUCH_Y0, TOUCH_W, TOUCH_H, KEY_565);

  for (unsigned i = 0; i < snapshot->liveTouches; i++) {
    struct USBH_WCSingleFinger const * const f = &snapshot->fingers[i];
    if (f->touching) {
      UTIL_LCD_FillRect(TOUCH_X0 + f->xFrac * TOUCH_W,
          TOUCH_Y0 + f->yFrac * TOUCH_H,
          5, 5, 0xFF000000);
    }
  }
}

/* Sample status sources and bump s_statusDirty on change. Runs even
 * on the WaitFrame-timeout path so USB plug events refresh the panel
 * while the touchscreen is idle. */
static void Touch_UpdateStatus(void)
{
  bool hostUp   = (Appli_state == APPLICATION_READY);
  bool devUp    = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
  bool hapticOn = HapticArea_GetMode();

  if (hostUp   != s_displayedStatus.hostUp ||
      devUp    != s_displayedStatus.devUp  ||
      hapticOn != s_displayedStatus.hapticOn)
  {
    s_displayedStatus.hostUp   = hostUp;
    s_displayedStatus.devUp    = devUp;
    s_displayedStatus.hapticOn = hapticOn;
    s_statusDirty = 2U;  /* repaint both layer-1 buffers */
  }
}

/* Paint the status strip onto the current layer-1 draw buffer.
 * Caller does the swap. */
static void Touch_DrawStatus(void)
{
  const uint32_t KEY_565 = LCD_COLOR_RGB565_MAGENTA;

  UTIL_LCD_FillRect(STATUS_X0, 0, STATUS_W, STATUS_H, KEY_565);

  UTIL_LCD_SetFont(&Font24);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
  /* MAGENTA back-color matches KEY_565 once converted, so glyph
   * inter-pixels stay transparent over layer 0's black. */
  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_MAGENTA);

  UTIL_LCD_DisplayStringAt(STATUS_X0 + 12U, 20U,
                           (uint8_t *)"Touch", LEFT_MODE);
  UTIL_LCD_FillCircle(STATUS_X0 + 130U, 20+8U, STATUS_LED_R,
                      s_displayedStatus.hostUp
                        ? UTIL_LCD_COLOR_GREEN
                        : UTIL_LCD_COLOR_RED);

  UTIL_LCD_DisplayStringAt(STATUS_X0 + 12U, 60U,
                           (uint8_t *)"Serial", LEFT_MODE);
  UTIL_LCD_FillCircle(STATUS_X0 + 130U, 60+8, STATUS_LED_R,
                      s_displayedStatus.devUp
                        ? UTIL_LCD_COLOR_GREEN
                        : UTIL_LCD_COLOR_RED);

  UTIL_LCD_DisplayStringAt(STATUS_X0 + 12U, 180U,
                           (uint8_t *)"Areas", LEFT_MODE);
  UTIL_LCD_FillCircle(STATUS_X0 + 130U, 192U, STATUS_LED_R,
                      s_displayedStatus.hapticOn
                        ? UTIL_LCD_COLOR_GREEN
                        : UTIL_LCD_COLOR_RED);
}

static void Touch_DrawRectOutline(uint32_t x, uint32_t y,
                                  uint32_t w, uint32_t h, uint32_t color)
{
  UTIL_LCD_FillRect(x,           y,          w,  2U, color);
  UTIL_LCD_FillRect(x,           y + h - 2U, w,  2U, color);
  UTIL_LCD_FillRect(x,           y,          2U, h,  color);
  UTIL_LCD_FillRect(x + w - 2U,  y,          2U, h,  color);
}

static void Touch_DrawDashedRect(uint32_t x, uint32_t y,
                                 uint32_t w, uint32_t h, uint32_t color)
{
  const uint32_t ON  = 8U;
  const uint32_t OFF = 4U;
  for (uint32_t i = 0; i < w; ) {
    uint32_t seg = (i + ON <= w) ? ON : (w - i);
    UTIL_LCD_FillRect(x + i, y,          seg, 2U, color);
    UTIL_LCD_FillRect(x + i, y + h - 2U, seg, 2U, color);
    i += seg + OFF;
  }
  /* Skip 2 px corner caps already drawn by the top+bottom edges. */
  if (h > 4U) {
    for (uint32_t j = 2U; j < h - 2U; ) {
      uint32_t seg = (j + ON <= h - 2U) ? ON : (h - 2U - j);
      UTIL_LCD_FillRect(x,          y + j, 2U, seg, color);
      UTIL_LCD_FillRect(x + w - 2U, y + j, 2U, seg, color);
      j += seg + OFF;
    }
  }
}

/* Repaint layer 0: white touch area, then all area outlines. The
 * right-side strip stays untouched — it was painted black in
 * Touch_InitLCD and nothing else lands there, so a full-screen
 * clear is unnecessary (and on a brownout-prone board, undesirable
 * DMA2D current draw). Restores layer 1 as active on exit. */
static void Touch_RenderLayer0(void)
{
  const uint32_t TOUCH_W  = 640U;
  const uint32_t TOUCH_H  = 480U;
  const uint32_t TOUCH_X0 = 0U;
  const uint32_t TOUCH_Y0 = 0U;

  BSP_LCD_SetActiveLayer(0, 0);
  UTIL_LCD_FillRect(TOUCH_X0, TOUCH_Y0, TOUCH_W, TOUCH_H,
                    UTIL_LCD_COLOR_WHITE);

  /* .bss, not the 2 KB task stack — sizeof(entries) is ~24 KB. */
  static HapticArea_RenderEntry entries[HAPTIC_AREA_MAX_COUNT];
  uint32_t count = (uint32_t)HAPTIC_AREA_MAX_COUNT;
  HapticArea_RenderSnapshot(entries, &count);

  for (uint32_t i = 0; i < count; i++) {
    const HapticArea_RenderEntry *e = &entries[i];
    /* Panel -> LCD pixel: ROTATE_90 swaps the axes, so panel x
     * ranges over DISP_HEIGHT_PX and panel y over DISP_WIDTH_PX. */
    uint32_t lx = (e->rect.x0 * TOUCH_W) / (uint32_t)DISP_HEIGHT_PX;
    uint32_t ly = (e->rect.y0 * TOUCH_H) / (uint32_t)DISP_WIDTH_PX;
    uint32_t lw = ((e->rect.x1 - e->rect.x0) * TOUCH_W) / (uint32_t)DISP_HEIGHT_PX;
    uint32_t lh = ((e->rect.y1 - e->rect.y0) * TOUCH_H) / (uint32_t)DISP_WIDTH_PX;
    if (lw < 2U || lh < 2U) continue;

    if (!e->enabled) {
      Touch_DrawDashedRect(lx, ly, lw, lh, UTIL_LCD_COLOR_GRAY);
    } else if (e->kind == touchforce_v1_HapticAreaKind_HAPTIC_AREA_KIND_BLOCK) {
      Touch_DrawRectOutline(lx, ly, lw, lh, UTIL_LCD_COLOR_RED);
    } else {
      Touch_DrawRectOutline(lx, ly, lw, lh, UTIL_LCD_COLOR_GREEN);
    }
  }

  /* Layer-1 work later in the loop expects active layer == 1. */
  BSP_LCD_SetActiveLayer(0, 1);
}

/* Swap whichever layers were freshly drawn. Both layers reload at
 * the same VBLANK (RELOAD_VERTICAL_BLANKING was set at init), so
 * one wait covers both. */
static void Touch_PresentLayers(bool swapL0, bool swapL1)
{
  if (swapL0) BSP_LCD_SwapVisibleBuffer(0, BSP_LCD_LAYER_BACKGROUND);
  if (swapL1) BSP_LCD_SwapVisibleBuffer(0, BSP_LCD_LAYER_FOREGROUND);

  /* Drain stale gives, then wait for the next VBLANK to confirm
   * the LTDC has picked up the new visible buffer(s). */
  (void)xSemaphoreTake(s_vblankSem, 0);
  (void)xSemaphoreTake(s_vblankSem, pdMS_TO_TICKS(20));

  if (swapL0) BSP_LCD_SwapDrawBuffer(0, BSP_LCD_LAYER_BACKGROUND);
  if (swapL1) BSP_LCD_SwapDrawBuffer(0, BSP_LCD_LAYER_FOREGROUND);
}

/* Encode a TouchFrameEvent into a COBS-framed CDC packet and write
 * it with a 5 ms timeout (drop on busy: host slow, disconnected, or
 * encode bug all bump txFails for GetTelemetry). */
static void Touch_StreamFrame(const struct USBH_LatestWisecocoData *snap)
{
  /* Static (BSS) — touchforce_v1_Frame is ~6 KB worst-case per
   * the GetHapticAreaListResponse oneof variant, far past the 2 KB
   * task stack. Only _Touch_Task calls this, so no concurrency. */
  static touchforce_v1_Frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.which_kind = touchforce_v1_Frame_event_tag;

  touchforce_v1_Event *event = &frame.kind.event;
  event->which_payload = touchforce_v1_Event_touch_frame_tag;

  touchforce_v1_TouchFrameEvent *tfe = &event->payload.touch_frame;
  tfe->live_touches      = (uint32_t)snap->liveTouches;
  tfe->tips_touched_down = (uint32_t)snap->tipsTouchedDown;

  pb_size_t fc = (pb_size_t)snap->liveTouches;
  if (fc > (pb_size_t)10U) {
    fc = (pb_size_t)10U;
  }
  tfe->fingers_count = fc;
  for (pb_size_t i = 0U; i < fc; i++) {
    const struct USBH_WCSingleFinger *f = &snap->fingers[i];
    touchforce_v1_TouchFinger *tf = &tfe->fingers[i];
    tf->id             = (uint32_t)f->id;
    tf->touching       = f->touching;
    tf->x              = (uint32_t)f->x;
    tf->y              = (uint32_t)f->y;
    tf->patch_width    = (uint32_t)f->patchWidth;
    tf->patch_height   = (uint32_t)f->patchHeight;
    tf->touch_duration = (uint32_t)f->touchDuration;
  }

  pb_ostream_t os = pb_ostream_from_buffer(s_streaming.eventBuf, sizeof(s_streaming.eventBuf));
  if (!pb_encode(&os, touchforce_v1_Frame_fields, &frame)) {
    s_streaming.txFails++;
    return;
  }

  cobs_encode_result enc = cobs_encode(s_streaming.eventTxBuf, sizeof(s_streaming.eventTxBuf) - 1U,
                                        s_streaming.eventBuf, os.bytes_written);
  if (enc.status != COBS_ENCODE_OK) {
    s_streaming.txFails++;
    return;
  }
  s_streaming.eventTxBuf[enc.out_len] = 0x00U;

  uint8_t result = CDC_Write(s_streaming.eventTxBuf,
                             (uint16_t)(enc.out_len + 1U),
                             5U);  /* non-blocking-ish: drop on busy */
  if (result == USBD_OK) {
    s_streaming.eventsSent++;
  } else {
    s_streaming.txFails++;
  }
}

static void Touch_InitLCD(void)
{
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
   * arm the first line-event interrupt. From here on the line-event
   * callback at the top of this file keeps re-arming itself, so we
   * get one xSemaphoreGiveFromISR per frame indefinitely. */
  s_vblankSem = xSemaphoreCreateBinaryStatic(&s_vblankSemMeta);
  HAL_NVIC_SetPriority(LTDC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);
  HAL_LTDC_ProgramLineEvent(&hlcd_ltdc, 0);

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

  /* Layer 0 baseline. Touch_RenderLayer0 may repaint the touch-area
   * portion later; the right strip stays this initial black. */
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
}

/* Drive LD1 on and arm the shutoff deadline. The actual off happens
 * when the touch-task loop sees the deadline elapse. Non-blocking. */
static void Haptic_Fire(touchforce_v1_HapticEffect effect)
{
  if (effect != touchforce_v1_HapticEffect_HAPTIC_EFFECT_DEFAULT) {
    return;
  }
  BSP_LED_On(LD1);
  s_ledOffDeadline = HAL_GetTick() + 100U;
}

void _Touch_Task(void *argument)
{
  (void)argument;
  Touch_InitLCD();

  BSP_LED_Init(LD1);

  /* Force initial paints so both layers render on the first wake
   * even when the sampled state matches s_displayedStatus's all-zero
   * default and HapticArea_GetGeneration() is still 0. */
  s_statusDirty = 2U;
  s_layer0Dirty = 2U;

  /* Persisted across iterations so a status-only repaint (no new
   * touch frame) can still re-render dots from the last known state. */
  static struct USBH_LatestWisecocoData lastSnapshot;
  for (;;) {
    struct USBH_LatestWisecocoData snapshot;

    /* LED-off deadline. Checked before WaitFrame and used to shrink
     * the timeout so the LED can shut off near 100 ms even on a
     * single tap with no follow-up touch frames to wake us. */
    uint32_t timeoutMs = 250U;
    if (s_ledOffDeadline != 0U) {
      uint32_t now = HAL_GetTick();
      if ((int32_t)(now - s_ledOffDeadline) >= 0) {
        BSP_LED_Off(LD1);
        s_ledOffDeadline = 0U;
      } else {
        uint32_t remainingMs = s_ledOffDeadline - now;
        if (remainingMs < timeoutMs) timeoutMs = remainingMs;
      }
    }

    /* Finite timeout (not HAL_MAX_DELAY) so USB plug events refresh
     * the status panel while the touchscreen is idle. */
    bool gotFrame = USBH_HID_WisecocoWaitFrame(&snapshot, pdMS_TO_TICKS(timeoutMs));
    if (gotFrame) {
      lastSnapshot = snapshot;
    }

    Touch_UpdateStatus();

    /* Rising-edge detection: any id present now and absent in
     * s_prevTouchingIds is a touchdown event. */
    if (gotFrame) {
      for (unsigned i = 0; i < snapshot.liveTouches; i++) {
        const struct USBH_WCSingleFinger *f = &snapshot.fingers[i];
        if (!f->touching) continue;

        bool wasTouching = false;
        for (uint8_t j = 0; j < s_prevTouchingCount; j++) {
          if (s_prevTouchingIds[j] == f->id) {
            wasTouching = true;
            break;
          }
        }
        if (wasTouching) continue;

        if (!HapticArea_GetMode()) {
          /* mode=disabled: every rising edge fires the default. */
          Haptic_Fire(touchforce_v1_HapticEffect_HAPTIC_EFFECT_DEFAULT);
        } else {
          touchforce_v1_HapticEffect hit =
              HapticArea_TestRisingEdge((uint32_t)f->x, (uint32_t)f->y);
          if (hit != touchforce_v1_HapticEffect_HAPTIC_EFFECT_UNSPECIFIED) {
            Haptic_Fire(hit);
          }
        }
      }

      s_prevTouchingCount = 0U;
      for (unsigned i = 0; i < snapshot.liveTouches; i++) {
        const struct USBH_WCSingleFinger *f = &snapshot.fingers[i];
        if (f->touching && s_prevTouchingCount < MAX_TOUCHES) {
          s_prevTouchingIds[s_prevTouchingCount++] = f->id;
        }
      }
    }

    /* Generation change → repaint both layer-0 buffers. */
    uint32_t curGen = HapticArea_GetGeneration();
    if (curGen != s_lastHapticAreaGeneration) {
      s_lastHapticAreaGeneration = curGen;
      s_layer0Dirty = 2U;
    }

    bool needL1 = gotFrame || s_statusDirty > 0U;
    bool needL0 = s_layer0Dirty > 0U;
    if (!needL1 && !needL0) {
      continue;
    }

    if (needL1) {
      Touch_RenderTouchDots(&lastSnapshot);
      if (s_statusDirty > 0U) { Touch_DrawStatus(); s_statusDirty--; }
    }
    if (needL0) {
      Touch_RenderLayer0();
      s_layer0Dirty--;
    }
    Touch_PresentLayers(needL0, needL1);

    if (gotFrame && s_streaming.enabled) {
      Touch_StreamFrame(&snapshot);
    }
  }
}

/* Drives the touchforce.v1 serial protocol over USB CDC after the
 * host has enumerated the device. */
void _CDC_Task(void *argument)
{
  (void)argument;

  /* By the time dev_state == CONFIGURED, MX_USB_DEVICE_Init (run from
   * _USB_Task) has already called CDC_AppInit, so the FreeRTOS objects
   * backing CDC_Read / CDC_Write exist. */
  extern USBD_HandleTypeDef hUsbDeviceFS;

  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    osDelay(100);
  }

  Protocol_RunForever();
}

/* Drives the HAL tick from TIM6's update interrupt. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

/* Single-byte volatile write — atomic on Cortex-M. */
void Touch_SetStreaming(bool enabled)
{
  s_streaming.enabled = enabled;
}

/* Two non-atomic reads against concurrent increments in _Touch_Task.
 * A torn value reads as a slightly stale monotonic counter, fine. */
void Touch_GetTelemetry(uint32_t *sent, uint32_t *fails)
{
  *sent  = s_streaming.eventsSent;
  *fails = s_streaming.txFails;
}

static void MX_GPDMA1_Init(void)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
}

static void MX_UART4_Init(void)
{
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
}

static void MX_UCPD1_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_DMA_InitTypeDef DMA_InitStruct = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOM);

  /* PM1 -> UCPD1_CC2, PM0 -> UCPD1_CC1 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOM, &GPIO_InitStruct);

  /* GPDMA1_REQUEST_UCPD1_RX */
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

  /* GPDMA1_REQUEST_UCPD1_TX */
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

  NVIC_SetPriority(UCPD1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(UCPD1_IRQn);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOM_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

void Error_Handler(void)
{
  printf("Error handler\n");
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */
