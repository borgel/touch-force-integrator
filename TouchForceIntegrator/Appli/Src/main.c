#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
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

/* All touchforce.v1 streaming state in one place. Individual
 * fields keep their volatile qualifiers; the struct itself is
 * static.
 *
 * Concurrency:
 *   - .enabled      — written by _CDC_Task via Touch_SetStreaming,
 *                     read by _Touch_Task. Single-byte volatile
 *                     access is atomic on Cortex-M.
 *   - .eventsSent / .txFails — written by _Touch_Task in
 *                     Touch_StreamFrame, read by _CDC_Task via
 *                     Touch_GetTelemetry. Reads may be torn for a
 *                     few microseconds; harmless for monotonic
 *                     counters.
 *   - .eventBuf / .eventTxBuf — only touched by _Touch_Task inside
 *                     Touch_StreamFrame, no contention.
 *
 * Buffer sizes are derived from touchforce_v1_Frame_size (worst-case
 * encoded Frame carrying a TouchFrameEvent with 10 fingers, currently
 * 418 bytes) plus COBS overhead and the 0x00 delimiter. 512 covers
 * the worst case with margin. */
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

/* Right-side status panel. The touch render area uses the left
 * 640 px of the 800 px LCD; this strip fills the remaining 160 px
 * on layer 1. We track what's currently painted and only repaint
 * when something changes. Layer 1 is double-buffered, so a state
 * change has to be painted onto BOTH buffers before they agree —
 * s_statusDirty starts at 2 on a change and decrements per render. */
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

/* Layer 0 holds the static touch-area white plus the haptic-area
 * outlines. Repainted only when HapticArea_GetGeneration() changes.
 * Both background buffers must be repainted to agree, so the dirty
 * counter starts at 2 on a generation bump and decrements per
 * render. */
static uint8_t  s_layer0Dirty;
static uint32_t s_lastHapticAreaGeneration;

/* Per-finger rising-edge tracker. wisecoco finger ids are stable
 * for the duration of a single touch (lift+redown gets a new id),
 * so equality on id is enough to detect "this finger was already
 * touching last frame". MAX_TOUCHES = 10. */
static uint8_t s_prevTouchingIds[MAX_TOUCHES];
static uint8_t s_prevTouchingCount;

/* One-shot 100 ms timer that turns LD1 off after a fire. xTimerReset
 * during an active period extends the on-time, so a burst of fires
 * reads as a single sustained pulse. */
static StaticTimer_t s_ledTimerMeta;
static TimerHandle_t s_ledTimer;

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

/* LCD rendering task. Brings up the LCD with two layers (static
 * background on layer 0, animated dots on layer 1 with color keying
 * for transparency), then waits on the wisecoco frame-published
 * semaphore and redraws on each new frame. */
/* Clear the touch-preview region to the chroma-key color (so layer 0
 * shows through), then draw a 5x5 dot for each currently-touching
 * finger. Caller does the swap. */
static void Touch_RenderTouchDots(const struct USBH_LatestWisecocoData *snapshot)
{
  /* Constants are intentionally duplicated from _Touch_Task's init
   * block: KEY_565 also appears there for the layer-1 transparent
   * clear at startup. The handful of touch-area constants are local
   * to the per-frame work. */
  const uint16_t TOUCH_W  = 640;
  const uint16_t TOUCH_H  = 480;
  const uint16_t TOUCH_X0 = 0;
  const uint16_t TOUCH_Y0 = 0;
  const uint32_t KEY_565  = LCD_COLOR_RGB565_MAGENTA;

  // Clear only the touch area to KEY (transparent), erasing last frame's dots.
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

/* Sample USB host link, USB device link, and haptic mode, and bump
 * s_statusDirty if any of them have changed since the last paint.
 * Called every render-loop iteration, including the timeout path,
 * so USB plug events refresh the panel even when the touchscreen
 * is idle. */
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

/* Paint the right-side status strip onto the current layer-1 draw
 * buffer. Background is the color-key (transparent over layer 0's
 * black), with white labels and red/green LED circles. Caller does
 * the swap. */
static void Touch_DrawStatus(void)
{
  const uint32_t KEY_565 = LCD_COLOR_RGB565_MAGENTA;

  /* Clear strip to transparent, erasing last paint's values. */
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
                           (uint8_t *)"HAPTIC", LEFT_MODE);
  UTIL_LCD_FillCircle(STATUS_X0 + 130U, 192U, STATUS_LED_R,
                      s_displayedStatus.hapticOn
                        ? UTIL_LCD_COLOR_GREEN
                        : UTIL_LCD_COLOR_RED);
}

/* Solid 2-px rectangle outline at LCD pixel coords. */
static void Touch_DrawRectOutline(uint32_t x, uint32_t y,
                                  uint32_t w, uint32_t h, uint32_t color)
{
  UTIL_LCD_FillRect(x,           y,          w,  2U, color);
  UTIL_LCD_FillRect(x,           y + h - 2U, w,  2U, color);
  UTIL_LCD_FillRect(x,           y,          2U, h,  color);
  UTIL_LCD_FillRect(x + w - 2U,  y,          2U, h,  color);
}

/* 2-px dashed outline. 8 px on / 4 off. Used for disabled areas. */
static void Touch_DrawDashedRect(uint32_t x, uint32_t y,
                                 uint32_t w, uint32_t h, uint32_t color)
{
  const uint32_t ON  = 8U;
  const uint32_t OFF = 4U;
  /* Top + bottom edges. */
  for (uint32_t i = 0; i < w; ) {
    uint32_t seg = (i + ON <= w) ? ON : (w - i);
    UTIL_LCD_FillRect(x + i, y,          seg, 2U, color);
    UTIL_LCD_FillRect(x + i, y + h - 2U, seg, 2U, color);
    i += seg + OFF;
  }
  /* Left + right edges, skipping the 2 px corner caps already drawn. */
  if (h > 4U) {
    for (uint32_t j = 2U; j < h - 2U; ) {
      uint32_t seg = (j + ON <= h - 2U) ? ON : (h - 2U - j);
      UTIL_LCD_FillRect(x,          y + j, 2U, seg, color);
      UTIL_LCD_FillRect(x + w - 2U, y + j, 2U, seg, color);
      j += seg + OFF;
    }
  }
}

/* Repaint layer 0 from scratch: black everywhere, white touch area,
 * then all enabled/disabled haptic-area outlines. Caller does the
 * swap. Restores layer 1 as the active drawing target on exit so
 * subsequent layer-1 work in the loop draws into the right buffer. */
static void Touch_RenderLayer0(void)
{
  const uint32_t TOUCH_W  = 640U;
  const uint32_t TOUCH_H  = 480U;
  const uint32_t TOUCH_X0 = 0U;
  const uint32_t TOUCH_Y0 = 0U;

  BSP_LCD_SetActiveLayer(0, 0);
  UTIL_LCD_Clear(UTIL_LCD_COLOR_BLACK);
  UTIL_LCD_FillRect(TOUCH_X0, TOUCH_Y0, TOUCH_W, TOUCH_H,
                    UTIL_LCD_COLOR_WHITE);

  /* File-static so the snapshot lives in .bss, not on the stack
   * (HAPTIC_AREA_MAX_COUNT * sizeof(HapticArea_RenderEntry) is
   * ~24 KB — way too much for the 2 KB task stack). */
  static HapticArea_RenderEntry entries[HAPTIC_AREA_MAX_COUNT];
  uint32_t count = (uint32_t)HAPTIC_AREA_MAX_COUNT;
  HapticArea_RenderSnapshot(entries, &count);

  for (uint32_t i = 0; i < count; i++) {
    const HapticArea_RenderEntry *e = &entries[i];
    /* Panel coords -> LCD pixel coords. axes: with ROTATE_90 set
     * on the wisecoco, panel x ranges over DISP_HEIGHT_PX and panel
     * y over DISP_WIDTH_PX. */
    uint32_t lx = (e->rect.x0 * TOUCH_W) / (uint32_t)DISP_HEIGHT_PX;
    uint32_t ly = (e->rect.y0 * TOUCH_H) / (uint32_t)DISP_WIDTH_PX;
    uint32_t lw = ((e->rect.x1 - e->rect.x0) * TOUCH_W) / (uint32_t)DISP_HEIGHT_PX;
    uint32_t lh = ((e->rect.y1 - e->rect.y0) * TOUCH_H) / (uint32_t)DISP_WIDTH_PX;
    if (lw < 2U || lh < 2U) continue;  /* too small to outline */

    if (!e->enabled) {
      Touch_DrawDashedRect(lx, ly, lw, lh, UTIL_LCD_COLOR_GRAY);
    } else if (e->kind == touchforce_v1_HapticAreaKind_HAPTIC_AREA_KIND_BLOCK) {
      Touch_DrawRectOutline(lx, ly, lw, lh, UTIL_LCD_COLOR_RED);
    } else {
      Touch_DrawRectOutline(lx, ly, lw, lh, UTIL_LCD_COLOR_GREEN);
    }
  }

  /* Restore layer 1 as active so the rest of the loop's layer-1
   * draws (Touch_RenderTouchDots, Touch_DrawStatus) land in the
   * right buffer. */
  BSP_LCD_SetActiveLayer(0, 1);
}

/* Swap whichever layers were freshly drawn. Layer 0 swap is gated
 * by swapL0; layer 1 by swapL1. Both reload at the same VBLANK
 * (RELOAD_VERTICAL_BLANKING was set at init), so we wait once and
 * SwapDraw both afterwards. */
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

/* Build a Frame{Event{TouchFrameEvent}} from snap, encode it via
 * nanopb into s_streaming.eventBuf, COBS-encode that into s_streaming.eventTxBuf with
 * a trailing 0x00, and CDC_Write it with a 5ms timeout. Drops on
 * any non-OK status (host slow / disconnected / encode bug) and
 * bumps the corresponding counter so GetTelemetry can report it. */
static void Touch_StreamFrame(const struct USBH_LatestWisecocoData *snap)
{
  touchforce_v1_Frame frame = touchforce_v1_Frame_init_zero;
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
    /* Encoder hit a logic bug or insufficient buffer. The buffer
     * is sized from Frame_size + slack, so this should be a hard
     * sizing-bug signal — count it and drop. */
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

/* One-shot LCD setup: panel init, brightness, double-buffer enable
 * for both layers, VBLANK-line interrupt arm, color-key, layer-0
 * background draw, and layer-1 transparent clear. Called once from
 * _Touch_Task before its render loop begins. */
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
}

static void Haptic_LedTimerCallback(TimerHandle_t xTimer)
{
  (void)xTimer;
  BSP_LED_Off(LED1);
}

/* Async fire: drives LD1 on, (re)starts the 100 ms one-shot
 * timer. Subsequent fires within 100 ms extend the on-time so a
 * burst reads as a single sustained pulse. Non-blocking — the
 * touch loop never waits for the LED to turn off. */
static void Haptic_Fire(touchforce_v1_HapticEffect effect)
{
  if (effect != touchforce_v1_HapticEffect_HAPTIC_EFFECT_DEFAULT) {
    return;  /* unspecified or unknown effect: no fire */
  }
  BSP_LED_On(LED1);
  /* xTimerReset both starts the timer (if stopped) and resets
   * its countdown (if running). 0-tick block since we're in a
   * task context. */
  (void)xTimerReset(s_ledTimer, pdMS_TO_TICKS(0));
}

void _Touch_Task(void *argument)
{
  (void)argument;
  Touch_InitLCD();

  BSP_LED_Init(LED1);
  s_ledTimer = xTimerCreateStatic(
      "led-pulse",
      pdMS_TO_TICKS(100),     /* one-shot 100 ms */
      pdFALSE,                 /* uxAutoReload = pdFALSE */
      NULL,
      Haptic_LedTimerCallback,
      &s_ledTimerMeta);

  /* Force the initial status paint so the panel is visible on boot
   * even if the sampled state happens to match the all-zero default
   * of s_displayedStatus. */
  s_statusDirty = 2U;

  /* Force the initial layer-0 paint so the HAPTIC mode-state and
   * any pre-existing areas (none at boot, but consistent) render
   * on the first wake. */
  s_layer0Dirty = 2U;

  /* lastSnapshot is kept across iterations so the timeout path can
   * still resample the previous finger count for the status check
   * (no new frame == count unchanged). */
  static struct USBH_LatestWisecocoData lastSnapshot;
  for (;;) {
    struct USBH_LatestWisecocoData snapshot;
    /* 250 ms timeout, not HAL_MAX_DELAY, so USB plug events refresh
     * the status panel while the touchscreen is idle (no frames
     * published). */
    bool gotFrame = USBH_HID_WisecocoWaitFrame(&snapshot, pdMS_TO_TICKS(250));
    if (gotFrame) {
      lastSnapshot = snapshot;
    }

    Touch_UpdateStatus();

    /* Per-finger rising-edge detection: any finger id present now
     * and absent in s_prevTouchingIds is a touchdown event. Only
     * runs when we have a fresh frame — no frame means no new
     * events. */
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
        if (wasTouching) continue;  /* not a rising edge */

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

      /* Refresh prev set with currently-touching ids for next frame. */
      s_prevTouchingCount = 0U;
      for (unsigned i = 0; i < snapshot.liveTouches; i++) {
        const struct USBH_WCSingleFinger *f = &snapshot.fingers[i];
        if (f->touching && s_prevTouchingCount < MAX_TOUCHES) {
          s_prevTouchingIds[s_prevTouchingCount++] = f->id;
        }
      }
    }

    /* Detect haptic-area changes and arm a layer-0 repaint. Two
     * frames per change covers both background buffers. */
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

/* Drives the touchforce.v1 serial protocol over USB CDC. Runs the
 * COBS frame accumulator + nanopb dispatcher forever after the host
 * has enumerated the device. */
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

  Protocol_RunForever();
}

/* Drives the HAL tick from TIM6's update interrupt. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

/* Toggle touch streaming. Called from the protocol_task.c dispatch
 * for SetTouchStreaming. Single byte write, atomic on Cortex-M. */
void Touch_SetStreaming(bool enabled)
{
  s_streaming.enabled = enabled;
}

/* Read the streaming telemetry counters. Called from protocol_task.c
 * when handling GetTelemetry. The two reads are sequenced but not
 * atomic relative to _Touch_Task's increments — a torn value just
 * looks like a counter from a few microseconds earlier, which is fine
 * for monotonic counters. */
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
