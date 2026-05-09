#include "haptic_area.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7rsxx_hal.h"
#include "main.h"               /* Error_Handler */
#include "usbh_hid_wisecoco.h"  /* DISP_HEIGHT_PX, DISP_WIDTH_PX */

#include <string.h>

static RNG_HandleTypeDef s_rngHandle;

static StaticSemaphore_t s_mutexMeta;
static SemaphoreHandle_t s_mutex;

/* volatile so cross-task writes/reads aren't cached across calls. */
static volatile bool     s_modeEnabled;

/* Bumped under the mutex on every successful mutation; read
 * lock-free by the LCD task (uint32 reads are atomic on Cortex-M). */
static volatile uint32_t s_generation;

typedef struct {
  bool                     used;
  touchforce_v1_HapticArea area;
} HapticAreaSlot;

static HapticAreaSlot s_areas[HAPTIC_AREA_MAX_COUNT];
static uint32_t       s_areaCount;

static bool rectIsValid(const touchforce_v1_HapticAreaRect *r)
{
  if (r->x0 >= r->x1) return false;
  if (r->y0 >= r->y1) return false;
  /* ROTATE_90 swaps the wisecoco axes, so panel x ranges over
   * DISP_HEIGHT_PX and panel y over DISP_WIDTH_PX. */
  if (r->x1 > (uint32_t)DISP_HEIGHT_PX) return false;
  if (r->y1 > (uint32_t)DISP_WIDTH_PX)  return false;
  return true;
}

/* Caller holds the mutex. */
static int32_t findSlotById(uint32_t id)
{
  for (uint32_t i = 0; i < s_areaCount; i++) {
    if (s_areas[i].used && s_areas[i].area.id == id) {
      return (int32_t)i;
    }
  }
  return -1;
}

/* Caller holds the mutex. With 32-bit random draws and ≤1024
 * entries, collisions are vanishingly rare. */
static uint32_t generateUniqueId(void)
{
  for (;;) {
    uint32_t candidate = 0U;
    if (HAL_RNG_GenerateRandomNumber(&s_rngHandle, &candidate) != HAL_OK) {
      Error_Handler();
    }
    if (candidate == 0U) continue;
    if (findSlotById(candidate) >= 0) continue;
    return candidate;
  }
}

void HapticArea_Init(void)
{
  s_mutex = xSemaphoreCreateMutexStatic(&s_mutexMeta);

  /* RNG kernel clock on STM32H7S is hardwired to HSI48 (no
   * peripheral-clock mux), and the bootloader leaves HSI48 off.
   * USB FS later does the same enable in HAL_PCD_MspInit;
   * HAL_RCC_OscConfig is idempotent, so the duplicate is a no-op. */
  RCC_OscInitTypeDef oscInit = {0};
  oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  oscInit.HSI48State     = RCC_HSI48_ON;
  if (HAL_RCC_OscConfig(&oscInit) != HAL_OK) {
    Error_Handler();
  }

  __HAL_RCC_RNG_CLK_ENABLE();
  s_rngHandle.Instance = RNG;
  if (HAL_RNG_Init(&s_rngHandle) != HAL_OK) {
    Error_Handler();
  }
}

void     HapticArea_SetMode(bool enabled) { s_modeEnabled = enabled; }
bool     HapticArea_GetMode(void)         { return s_modeEnabled; }
uint32_t HapticArea_GetGeneration(void)   { return s_generation; }

bool HapticArea_Set(touchforce_v1_HapticArea *areaInOut, uint32_t *errCodeOut)
{
  if (!rectIsValid(&areaInOut->rect)) {
    *errCodeOut = HAPTIC_AREA_ERR_INVALID_RECT;
    return false;
  }

  (void)xSemaphoreTake(s_mutex, portMAX_DELAY);

  if (areaInOut->id != 0U) {
    int32_t idx = findSlotById(areaInOut->id);
    if (idx >= 0) {
      s_areas[idx].area = *areaInOut;
      s_generation++;
      xSemaphoreGive(s_mutex);
      return true;
    }
    /* id != 0 but not found — fall through to "create at this id"
     * so a host can always Set against an id it still believes in. */
  }

  if (s_areaCount >= HAPTIC_AREA_MAX_COUNT) {
    xSemaphoreGive(s_mutex);
    *errCodeOut = HAPTIC_AREA_ERR_TABLE_FULL;
    return false;
  }

  uint32_t newId = (areaInOut->id != 0U) ? areaInOut->id : generateUniqueId();

  /* Compaction on Delete keeps the unused region as a tail at
   * s_areaCount, so the next slot is always s_areas[s_areaCount]. */
  HapticAreaSlot *slot = &s_areas[s_areaCount];
  slot->used     = true;
  slot->area     = *areaInOut;
  slot->area.id  = newId;
  s_areaCount++;
  s_generation++;

  areaInOut->id = newId;

  xSemaphoreGive(s_mutex);
  return true;
}

bool HapticArea_Get(uint32_t id, touchforce_v1_HapticArea *areaOut)
{
  (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
  int32_t idx = findSlotById(id);
  bool found = (idx >= 0);
  if (found) {
    *areaOut = s_areas[idx].area;
  }
  xSemaphoreGive(s_mutex);
  return found;
}

bool HapticArea_Delete(uint32_t id)
{
  (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
  int32_t idx = findSlotById(id);
  if (idx < 0) {
    xSemaphoreGive(s_mutex);
    return false;
  }
  /* Compact: pull the last slot into the freed one so iteration
   * stays linear without tombstones. */
  uint32_t last = s_areaCount - 1U;
  if ((uint32_t)idx != last) {
    s_areas[idx] = s_areas[last];
  }
  s_areas[last].used = false;
  s_areaCount--;
  s_generation++;
  xSemaphoreGive(s_mutex);
  return true;
}

void HapticArea_DeleteAll(void)
{
  (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
  for (uint32_t i = 0; i < s_areaCount; i++) {
    s_areas[i].used = false;
  }
  s_areaCount = 0U;
  s_generation++;
  xSemaphoreGive(s_mutex);
}

void HapticArea_GetList(uint32_t *idsOut, uint32_t *countInOut)
{
  (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
  uint32_t cap = *countInOut;
  uint32_t n   = (s_areaCount < cap) ? s_areaCount : cap;
  for (uint32_t i = 0; i < n; i++) {
    idsOut[i] = s_areas[i].area.id;
  }
  *countInOut = n;
  xSemaphoreGive(s_mutex);
}

touchforce_v1_HapticEffect
HapticArea_TestRisingEdge(uint32_t panelX, uint32_t panelY)
{
  /* 0-tick try-take: drop the fire rather than block the render
   * loop on a concurrent CDC mutator. */
  if (xSemaphoreTake(s_mutex, 0) != pdTRUE) {
    return touchforce_v1_HapticEffect_HAPTIC_EFFECT_UNSPECIFIED;
  }

  /* Block beats fire on overlap — scan blocks first. */
  for (uint32_t i = 0; i < s_areaCount; i++) {
    const touchforce_v1_HapticArea *a = &s_areas[i].area;
    if (!a->enabled) continue;
    if (a->kind != touchforce_v1_HapticAreaKind_HAPTIC_AREA_KIND_BLOCK) continue;
    if (panelX <  a->rect.x0 || panelX >= a->rect.x1) continue;
    if (panelY <  a->rect.y0 || panelY >= a->rect.y1) continue;
    xSemaphoreGive(s_mutex);
    return touchforce_v1_HapticEffect_HAPTIC_EFFECT_UNSPECIFIED;
  }

  /* First fire-area hit wins. */
  touchforce_v1_HapticEffect chosen =
      touchforce_v1_HapticEffect_HAPTIC_EFFECT_UNSPECIFIED;
  for (uint32_t i = 0; i < s_areaCount; i++) {
    const touchforce_v1_HapticArea *a = &s_areas[i].area;
    if (!a->enabled) continue;
    if (a->kind != touchforce_v1_HapticAreaKind_HAPTIC_AREA_KIND_FIRE) continue;
    if (panelX <  a->rect.x0 || panelX >= a->rect.x1) continue;
    if (panelY <  a->rect.y0 || panelY >= a->rect.y1) continue;
    chosen = a->haptic;
    break;
  }

  xSemaphoreGive(s_mutex);
  return chosen;
}

void HapticArea_RenderSnapshot(HapticArea_RenderEntry *entriesOut,
                               uint32_t *countInOut)
{
  (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
  uint32_t cap = *countInOut;
  uint32_t n   = (s_areaCount < cap) ? s_areaCount : cap;
  for (uint32_t i = 0; i < n; i++) {
    const touchforce_v1_HapticArea *a = &s_areas[i].area;
    entriesOut[i].id      = a->id;
    entriesOut[i].rect    = a->rect;
    entriesOut[i].enabled = a->enabled;
    entriesOut[i].kind    = a->kind;
  }
  *countInOut = n;
  xSemaphoreGive(s_mutex);
}
