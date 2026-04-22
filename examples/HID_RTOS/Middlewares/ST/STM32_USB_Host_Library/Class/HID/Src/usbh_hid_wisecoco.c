/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_lcd.c"
- "stm32xxxxx_{eval}{discovery}_sdram.c"
EndBSPDependencies */

#include "usbh_hid_wisecoco.h"

static const HID_Report_ItemTypedef imp_0_lctrl =
{
  keybd_report_data, /*data*/
  1,     /*size*/
  0,     /*shift*/
  0,     /*count (only for array items)*/
  0,     /*signed?*/
  0,     /*min value read can return*/
  1,     /*max value read can return*/
  0,     /*min vale device can report*/
  1,     /*max value device can report*/
  1      /*resolution*/
};

// hold the most recent update state data we've gotten from the device
static struct USBH_LatestWisecocoData latestData;

USBH_StatusTypeDef USBH_HID_WisecocoInit(USBH_HandleTypeDef *phost)
{
  uint32_t x;
  HID_HandleTypeDef * const HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  memset(&latestData, 0, sizeof(latestData));

  // TODO get report size, to setup FIFO later

  /*
  for (x = 0U; x < sizeof(keybd_report_data); x++)
  {
    keybd_report_data[x] = 0U;
    keybd_rx_report_buf[x] = 0U;
  }

  if (HID_Handle->length > (sizeof(keybd_report_data)))
  {
    HID_Handle->length = (uint16_t)(sizeof(keybd_report_data));
  }

  HID_Handle->pData = keybd_rx_report_buf;

  if ((HID_QUEUE_SIZE * sizeof(keybd_report_data)) > sizeof(phost->device.Data))
  {
    return USBH_FAIL;
  }
  else
  {
    USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, (uint16_t)(HID_QUEUE_SIZE * sizeof(keybd_report_data)));
  }
  */

  return USBH_OK;
}

// return the latest? touch report info to be printed above
int USBH_HID_GetTouchInfo(USBH_HandleTypeDef *phost) {
  // TODO called with a new message from the USB device for us to decode
  return 5;
}
