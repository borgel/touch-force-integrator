#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "usbh_hid.h"

#include <stdbool.h>

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

/*
 * Maximum payload size across all reports this device defines. Useful
 * if you want a single RX staging buffer that can hold whatever comes
 * in, or to sanity-check USB transfer lengths.
 *
 *   ReportID  Payload  Total (with REPORT_ID_OFFSET=1)
 *   --------  -------  -------------------------------
 *      1       75        76
 *     10        1         2
 *     68      256       257   <-- largest
 *      2       64        65
 *      3       63        64
 *      4       19        20
 */
#define USBH_WISECOCO_REPORT_SIZE    (REPORT_ID_OFFSET + 256U)

struct USBH_LatestWisecocoData {
  // TODO add array for all fingers, pressure, etc
  int x, y;     // touch coords for 0th touch
};

USBH_StatusTypeDef USBH_HID_WisecocoInit(USBH_HandleTypeDef *phost);
// return true if there is new data
bool USBH_HID_GetTouchReport(USBH_HandleTypeDef *phost, struct USBH_LatestWisecocoData * const newReport);

#ifdef __cplusplus
}
#endif
