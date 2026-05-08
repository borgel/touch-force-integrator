#ifndef HAPTIC_AREA_H
#define HAPTIC_AREA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "touch_force.pb.h"

#include <stdbool.h>
#include <stdint.h>

/* Maximum simultaneous areas. Storage is a flat array of records;
 * lookups are O(N) linear scans, fine at this size. */
#define HAPTIC_AREA_MAX_COUNT   1024U

/* Phase 0 error codes returned via ErrorResponse.code. Code 1 is
 * reserved upstream for "unknown command" in protocol_task.c. */
#define HAPTIC_AREA_ERR_NOT_FOUND     2U
#define HAPTIC_AREA_ERR_TABLE_FULL    3U
#define HAPTIC_AREA_ERR_INVALID_RECT  4U

/* One-shot init: brings up RNG, creates the storage mutex, zeroes
 * the table. Called from main() before the scheduler starts. */
void HapticArea_Init(void);

/* Mode bool — when false (boot default), hit-testing is bypassed
 * by callers and every touch rising edge fires
 * HAPTIC_EFFECT_DEFAULT. When true, only matching areas fire. */
void HapticArea_SetMode(bool enabled);
bool HapticArea_GetMode(void);

/* Mutating ops, called from _CDC_Task via protocol dispatch. All
 * acquire the storage mutex with portMAX_DELAY. Each successful
 * mutation bumps the generation counter. On failure, *errCodeOut
 * is set to one of HAPTIC_AREA_ERR_*. */
bool HapticArea_Set(touchforce_v1_HapticArea *areaInOut,
                    uint32_t *errCodeOut);
bool HapticArea_Get(uint32_t id, touchforce_v1_HapticArea *areaOut);
bool HapticArea_Delete(uint32_t id);
void HapticArea_DeleteAll(void);

/* Snapshot the populated ids into idsOut[]. *countInOut is the
 * array capacity on entry; the actual number copied on exit
 * (= min(*countInOut, current populated count)). */
void HapticArea_GetList(uint32_t *idsOut, uint32_t *countInOut);

/* Generation counter, bumped under the mutex on every successful
 * Set / Delete / DeleteAll. The render loop caches the last-seen
 * value and triggers a layer-0 repaint when it changes. Wraps
 * once every ~4 billion mutations. */
uint32_t HapticArea_GetGeneration(void);

/* Hit-test entry point called from _Touch_Task per rising-edge
 * finger when mode == enabled. Block-kind areas always win on
 * overlap with fire-kind. Returns HAPTIC_EFFECT_UNSPECIFIED for
 * "no fire" (no area covers the point, or a block area does).
 * Uses a 0-tick mutex try-take: if a CDC mutator holds it, this
 * returns UNSPECIFIED (one missed fire) rather than blocking the
 * render loop. */
touchforce_v1_HapticEffect
HapticArea_TestRisingEdge(uint32_t panelX, uint32_t panelY);

/* Read-only snapshot for LCD rendering. Caller iterates
 * [0, *countInOut). Acquires the mutex with portMAX_DELAY. */
typedef struct {
  uint32_t                     id;
  touchforce_v1_HapticAreaRect rect;
  bool                         enabled;
  touchforce_v1_HapticAreaKind kind;
} HapticArea_RenderEntry;

void HapticArea_RenderSnapshot(HapticArea_RenderEntry *entriesOut,
                               uint32_t *countInOut);

#ifdef __cplusplus
}
#endif

#endif /* HAPTIC_AREA_H */
