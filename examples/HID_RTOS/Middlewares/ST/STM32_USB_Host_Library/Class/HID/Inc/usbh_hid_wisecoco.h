#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "usbh_hid.h"

#ifndef USBH_HID_WISECOCO_REPORT_SIZE
// FIXME what's the actual report size? (more than one I assume?)
#define USBH_HID_WISECOCO_REPORT_SIZE                       0x8U
#endif /* USBH_HID_KEYBD_REPORT_SIZE */

struct USBH_LatestWisecocoData {
  int x, y;     // touch coords for 0th touch
};

USBH_StatusTypeDef USBH_HID_WisecocoInit(USBH_HandleTypeDef *phost);
// TODO return latest touch info to print at a higher layer
int USBH_HID_GetTouchReport(USBH_HandleTypeDef *phost);

#ifdef __cplusplus
}
#endif
