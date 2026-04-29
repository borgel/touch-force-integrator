#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "usbh_hid.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * REPORT_ID_OFFSET controls where field 0 of the payload lives within
 * each buffer. USB HID delivers each report prefixed with its 1-byte
 * Report ID. Some stacks leave that byte in the buffer they hand to
 * the app (offset = 1); others strip it first (offset = 0). Change
 * this single #define if your stack differs.
 */
#ifndef REPORT_ID_OFFSET
#define REPORT_ID_OFFSET    1U
#endif

#define DISP_WIDTH_PX       2160
#define DISP_HEIGHT_PX      2880

/*
 * Maximum payload size across all reports this device defines. Useful
 * if you want a single RX staging buffer that can hold whatever comes
 * in, or to sanity-check USB transfer lengths.
 *
 *   ReportID  Payload  Total (with REPORT_ID_OFFSET=1)
 *   --------  -------  -------------------------------
 *      1      115       116
 *     10        1         2
 *     68      256       257   <-- largest
 *      2       64        65
 *      3       63        64
 *      4       19        20
 */
#define USBH_WISECOCO_REPORT_SIZE    (REPORT_ID_OFFSET + 256U)

// touchscreen can report up to this many touches (index 0-9)
#define MAX_TOUCHES     10

struct USBH_WCSingleFinger {
  // arbitrary ID, stays constant to track this finger DURING THIS TOUCH
  uint8_t id;
  // if this finger is touching (will be a constant 1, except on the last frame where it goes to 0 a single time)
  bool touching;
  // coordinates of the touch
  uint16_t x, y;
  // x/y coordinated phrased as a fraction of the display size (IE 10% of the way across it)
  double xFrac, yFrac;
  // size of the contact patch
  uint8_t patchWidth, patchHeight;
  // how many microseconds since this touch started
  uint32_t touchDuration;
};

struct USBH_LatestWisecocoData {
  // how many fingers were down the last time we got a frame
  unsigned liveTouches;
  // array of latest data for each touch. Take care to only access index 0-liveTouches,
  // as anything above that will be unitialized and not meaningful
  struct USBH_WCSingleFinger fingers[MAX_TOUCHES];
};

USBH_StatusTypeDef USBH_HID_WisecocoInit(USBH_HandleTypeDef *phost);
// return true if there is new data
void USBH_HID_PumpTouchReports(USBH_HandleTypeDef *phost);
// get the set of touches from the latest report
// Note this function call cannot fail (it's just returns a pointer to the internal struct),
// but there may be 0 active touches, and it will be invalidated the next time a USB frame is processed
struct USBH_LatestWisecocoData const * const USBH_HID_WisecocoGetLatestTouches(void);

#ifdef __cplusplus
}
#endif
