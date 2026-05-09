#ifndef HAPTIC_AREA_H
#define HAPTIC_AREA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "touch_force.pb.h"

#include <stdbool.h>
#include <stdint.h>

#define HAPTIC_AREA_MAX_COUNT   1024U

/* ErrorResponse.code values. 1 is reserved for "unknown command"
 * in protocol_task.c. */
#define HAPTIC_AREA_ERR_NOT_FOUND     2U
#define HAPTIC_AREA_ERR_TABLE_FULL    3U
#define HAPTIC_AREA_ERR_INVALID_RECT  4U

void HapticArea_Init(void);

/* Mode bool. When false (boot default) callers should bypass
 * hit-testing and fire HAPTIC_EFFECT_DEFAULT on every rising edge. */
void HapticArea_SetMode(bool enabled);
bool HapticArea_GetMode(void);

/* Mutating ops. All acquire the storage mutex with portMAX_DELAY
 * and bump the generation counter on success. On failure,
 * *errCodeOut is set to one of HAPTIC_AREA_ERR_*. */
bool HapticArea_Set(touchforce_v1_HapticArea *areaInOut,
                    uint32_t *errCodeOut);
bool HapticArea_Get(uint32_t id, touchforce_v1_HapticArea *areaOut);
bool HapticArea_Delete(uint32_t id);
void HapticArea_DeleteAll(void);

/* Snapshot populated ids into idsOut[]. *countInOut is array
 * capacity on entry, count copied on exit. */
void HapticArea_GetList(uint32_t *idsOut, uint32_t *countInOut);

/* Bumped on every successful mutation. Lets readers detect changes
 * without taking the storage mutex. Wraps every ~4 B mutations. */
uint32_t HapticArea_GetGeneration(void);

/* Hit-test for rising-edge touches in mode=enabled. Block beats fire
 * on overlap. Returns HAPTIC_EFFECT_UNSPECIFIED for "no fire". Uses
 * a 0-tick mutex try-take: if a CDC mutator holds it the call
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
