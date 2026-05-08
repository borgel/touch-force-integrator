#include "haptic_area.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7rsxx_hal.h"
#include "main.h"               /* Error_Handler */
#include "usbh_hid_wisecoco.h"  /* DISP_HEIGHT_PX, DISP_WIDTH_PX */

#include <string.h>

/* RNG peripheral handle. RNG isn't initialized in any MX_*_Init
 * block today, so we set it up locally in HapticArea_Init. */
static RNG_HandleTypeDef s_rngHandle;

static StaticSemaphore_t s_mutexMeta;
static SemaphoreHandle_t s_mutex;

/* mode default: false (== HAPTIC_AREA_MODE_DISABLED). volatile
 * because writes from _CDC_Task and reads from _Touch_Task must
 * not be cached across calls. */
static volatile bool     s_modeEnabled;

/* Bumped under the mutex on every successful mutation. Read
 * lock-free by the LCD task. volatile + uint32 reads are atomic
 * on Cortex-M, so the cached/current comparison is safe. */
static volatile uint32_t s_generation;

typedef struct {
  bool                     used;
  touchforce_v1_HapticArea area;
} HapticAreaSlot;

static HapticAreaSlot s_areas[HAPTIC_AREA_MAX_COUNT];
static uint32_t       s_areaCount;

/* --- Helpers (file-static) ------------------------------------------------ */

static bool rectIsValid(const touchforce_v1_HapticAreaRect *r)
{
  if (r->x0 >= r->x1) return false;
  if (r->y0 >= r->y1) return false;
  /* x ranges over DISP_HEIGHT_PX, y over DISP_WIDTH_PX after the
   * ROTATE_90 set in main.c. Match wisecoco's frame conventions. */
  if (r->x1 > (uint32_t)DISP_HEIGHT_PX) return false;
  if (r->y1 > (uint32_t)DISP_WIDTH_PX)  return false;
  return true;
}

/* Linear scan; returns slot index or -1. Caller holds the mutex. */
static int32_t findSlotById(uint32_t id)
{
  for (uint32_t i = 0; i < s_areaCount; i++) {
    if (s_areas[i].used && s_areas[i].area.id == id) {
      return (int32_t)i;
    }
  }
  return -1;
}

/* Generate a non-zero id not already in use. With 32-bit random
 * draws and <=1024 entries, collisions are vanishingly rare; the
 * loop terminates almost certainly on the first iteration. Caller
 * holds the mutex. */
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

/* --- Init ----------------------------------------------------------------- */

void HapticArea_Init(void)
{
  s_mutex = xSemaphoreCreateMutexStatic(&s_mutexMeta);

  __HAL_RCC_RNG_CLK_ENABLE();
  s_rngHandle.Instance = RNG;
  if (HAL_RNG_Init(&s_rngHandle) != HAL_OK) {
    Error_Handler();
  }
}

/* --- Mode getter / setter ------------------------------------------------- */

void     HapticArea_SetMode(bool enabled) { s_modeEnabled = enabled; }
bool     HapticArea_GetMode(void)         { return s_modeEnabled; }
uint32_t HapticArea_GetGeneration(void)   { return s_generation; }

/* --- Set ------------------------------------------------------------------ */

bool HapticArea_Set(touchforce_v1_HapticArea *areaInOut, uint32_t *errCodeOut)
{
  if (!rectIsValid(&areaInOut->rect)) {
    *errCodeOut = HAPTIC_AREA_ERR_INVALID_RECT;
    return false;
  }

  (void)xSemaphoreTake(s_mutex, portMAX_DELAY);

  /* Update path: id != 0 and matches an existing slot. */
  if (areaInOut->id != 0U) {
    int32_t idx = findSlotById(areaInOut->id);
    if (idx >= 0) {
      s_areas[idx].area = *areaInOut;
      s_generation++;
      xSemaphoreGive(s_mutex);
      return true;
    }
    /* id != 0 but not found — fall through to "create at this id".
     * This makes the API symmetric: a host caching an id can
     * always Set with that id, whether or not we still have it. */
  }

  /* Create path: append into the first unused slot. */
  if (s_areaCount >= HAPTIC_AREA_MAX_COUNT) {
    xSemaphoreGive(s_mutex);
    *errCodeOut = HAPTIC_AREA_ERR_TABLE_FULL;
    return false;
  }

  uint32_t newId = (areaInOut->id != 0U) ? areaInOut->id : generateUniqueId();

  /* First unused slot in [0, HAPTIC_AREA_MAX_COUNT). With compaction
   * on Delete, the unused region is a contiguous tail at and beyond
   * s_areaCount, so we can just take s_areas[s_areaCount]. */
  HapticAreaSlot *slot = &s_areas[s_areaCount];
  slot->used     = true;
  slot->area     = *areaInOut;
  slot->area.id  = newId;
  s_areaCount++;
  s_generation++;

  /* Reflect the assigned id back to the caller. */
  areaInOut->id = newId;

  xSemaphoreGive(s_mutex);
  return true;
}

/* --- Get ------------------------------------------------------------------ */

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

/* --- Delete --------------------------------------------------------------- */

bool HapticArea_Delete(uint32_t id)
{
  (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
  int32_t idx = findSlotById(id);
  if (idx < 0) {
    xSemaphoreGive(s_mutex);
    return false;
  }
  /* Compact: move the last populated slot into the freed slot.
   * Keeps iteration linear without tombstones. */
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

/* --- GetList -------------------------------------------------------------- */

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

/* --- Hit-test ------------------------------------------------------------- */

touchforce_v1_HapticEffect
HapticArea_TestRisingEdge(uint32_t panelX, uint32_t panelY)
{
  /* 0-tick try-take: if a CDC mutator holds the mutex, return
   * "no fire" rather than blocking the touch loop. One dropped
   * fire is preferable to a render-side stall. */
  if (xSemaphoreTake(s_mutex, 0) != pdTRUE) {
    return touchforce_v1_HapticEffect_HAPTIC_EFFECT_UNSPECIFIED;
  }

  /* Block beats fire on overlap: scan blocks first; any hit
   * suppresses. */
  for (uint32_t i = 0; i < s_areaCount; i++) {
    const touchforce_v1_HapticArea *a = &s_areas[i].area;
    if (!a->enabled) continue;
    if (a->kind != touchforce_v1_HapticAreaKind_HAPTIC_AREA_KIND_BLOCK) continue;
    if (panelX <  a->rect.x0 || panelX >= a->rect.x1) continue;
    if (panelY <  a->rect.y0 || panelY >= a->rect.y1) continue;
    xSemaphoreGive(s_mutex);
    return touchforce_v1_HapticEffect_HAPTIC_EFFECT_UNSPECIFIED;
  }

  /* No block hit — first fire-area hit wins. */
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

/* --- Render snapshot ------------------------------------------------------ */

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
