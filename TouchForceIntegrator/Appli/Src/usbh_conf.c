#include "usbh_core.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

HCD_HandleTypeDef hhcd_USB_OTG_HS;

/*
 * Event semaphore for the USB host pumping task. The HCD callbacks below
 * give this from IRQ context whenever something useful happens (URB
 * completion, port connect/disconnect, port enable/disable). The user
 * task takes it with a long timeout, so most state-machine progress is
 * IRQ-driven and only stalls fall back on the timeout.
 */
static StaticSemaphore_t s_usbHostEventSemMeta;
static SemaphoreHandle_t s_usbHostEventSem;

void USBH_HostEvent_AppInit(void)
{
  if (s_usbHostEventSem == NULL)
  {
    s_usbHostEventSem = xSemaphoreCreateBinaryStatic(&s_usbHostEventSemMeta);
  }
}

bool USBH_HostEvent_Wait(uint32_t timeoutMs)
{
  if (s_usbHostEventSem == NULL)
  {
    return false;
  }
  TickType_t ticks = (timeoutMs == HAL_MAX_DELAY) ? portMAX_DELAY
      : pdMS_TO_TICKS(timeoutMs);
  return xSemaphoreTake(s_usbHostEventSem, ticks) == pdTRUE;
}

static inline void wakeUSBHostTaskFromISR(void)
{
  if (s_usbHostEventSem != NULL)
  {
    BaseType_t higherPrioWoken = pdFALSE;
    (void)xSemaphoreGiveFromISR(s_usbHostEventSem, &higherPrioWoken);
    portYIELD_FROM_ISR(higherPrioWoken);
  }
}

USBH_StatusTypeDef USBH_Get_USB_Status(HAL_StatusTypeDef hal_status);

void HAL_HCD_MspInit(HCD_HandleTypeDef* hcdHandle)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hcdHandle->Instance==USB_OTG_HS)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USBPHYC;
    PeriphClkInit.UsbPhycClockSelection = RCC_USBPHYCCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_PWREx_EnableUSBVoltageDetector();

    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
    __HAL_RCC_USBPHYC_CLK_ENABLE();

    HAL_NVIC_SetPriority(OTG_HS_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
  }
}

void HAL_HCD_MspDeInit(HCD_HandleTypeDef* hcdHandle)
{
  if(hcdHandle->Instance==USB_OTG_HS)
  {
    __HAL_RCC_USB_OTG_HS_CLK_DISABLE();
    __HAL_RCC_USBPHYC_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
  }
}

void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd)
{
  USBH_LL_IncTimer(hhcd->pData);
}

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
  USBH_LL_Connect(hhcd->pData);
  wakeUSBHostTaskFromISR();
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
  USBH_LL_Disconnect(hhcd->pData);
  wakeUSBHostTaskFromISR();
}

void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum, HCD_URBStateTypeDef urb_state)
{
#if (USBH_USE_OS == 1)
  USBH_LL_NotifyURBChange(hhcd->pData);
#endif
  wakeUSBHostTaskFromISR();
}

void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd)
{
  USBH_LL_PortEnabled(hhcd->pData);
  wakeUSBHostTaskFromISR();
}

void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd)
{
  USBH_LL_PortDisabled(hhcd->pData);
  wakeUSBHostTaskFromISR();
}

USBH_StatusTypeDef USBH_LL_Init(USBH_HandleTypeDef *phost)
{
  if (phost->id == HOST_HS) {
    hhcd_USB_OTG_HS.pData = phost;
    phost->pData = &hhcd_USB_OTG_HS;

    hhcd_USB_OTG_HS.Instance = USB_OTG_HS;
    hhcd_USB_OTG_HS.Init.Host_channels = 16;
    hhcd_USB_OTG_HS.Init.speed = HCD_SPEED_HIGH;
    hhcd_USB_OTG_HS.Init.dma_enable = DISABLE;
    hhcd_USB_OTG_HS.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
    hhcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
    hhcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
    hhcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
    hhcd_USB_OTG_HS.Init.use_external_vbus = ENABLE;
    if (HAL_HCD_Init(&hhcd_USB_OTG_HS) != HAL_OK)
    {
      Error_Handler();
    }

    USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(&hhcd_USB_OTG_HS));
  }
  return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_DeInit(USBH_HandleTypeDef *phost)
{
  return USBH_Get_USB_Status(HAL_HCD_DeInit(phost->pData));
}

USBH_StatusTypeDef USBH_LL_Start(USBH_HandleTypeDef *phost)
{
  return USBH_Get_USB_Status(HAL_HCD_Start(phost->pData));
}

USBH_StatusTypeDef USBH_LL_Stop(USBH_HandleTypeDef *phost)
{
  return USBH_Get_USB_Status(HAL_HCD_Stop(phost->pData));
}

USBH_SpeedTypeDef USBH_LL_GetSpeed(USBH_HandleTypeDef *phost)
{
  switch (HAL_HCD_GetCurrentSpeed(phost->pData))
  {
  case 0:  return USBH_SPEED_HIGH;
  case 1:  return USBH_SPEED_FULL;
  case 2:  return USBH_SPEED_LOW;
  default: return USBH_SPEED_FULL;
  }
}

USBH_StatusTypeDef USBH_LL_ResetPort(USBH_HandleTypeDef *phost)
{
  return USBH_Get_USB_Status(HAL_HCD_ResetPort(phost->pData));
}

uint32_t USBH_LL_GetLastXferSize(USBH_HandleTypeDef *phost, uint8_t pipe)
{
  return HAL_HCD_HC_GetXferCount(phost->pData, pipe);
}

USBH_StatusTypeDef USBH_LL_OpenPipe(USBH_HandleTypeDef *phost, uint8_t pipe_num, uint8_t epnum,
    uint8_t dev_address, uint8_t speed, uint8_t ep_type, uint16_t mps)
{
  return USBH_Get_USB_Status(
      HAL_HCD_HC_Init(phost->pData, pipe_num, epnum, dev_address, speed, ep_type, mps));
}

USBH_StatusTypeDef USBH_LL_ClosePipe(USBH_HandleTypeDef *phost, uint8_t pipe)
{
  return USBH_Get_USB_Status(HAL_HCD_HC_Halt(phost->pData, pipe));
}

USBH_StatusTypeDef USBH_LL_SubmitURB(USBH_HandleTypeDef *phost, uint8_t pipe, uint8_t direction,
    uint8_t ep_type, uint8_t token, uint8_t *pbuff, uint16_t length,
    uint8_t do_ping)
{
  return USBH_Get_USB_Status(
      HAL_HCD_HC_SubmitRequest(phost->pData, pipe, direction, ep_type, token, pbuff, length, do_ping));
}

USBH_URBStateTypeDef USBH_LL_GetURBState(USBH_HandleTypeDef *phost, uint8_t pipe)
{
  return (USBH_URBStateTypeDef)HAL_HCD_HC_GetURBState(phost->pData, pipe);
}

USBH_StatusTypeDef USBH_LL_DriverVBUS(USBH_HandleTypeDef *phost, uint8_t state)
{
  if (phost->id == HOST_HS)
  {
    if (state == 0)
    {
      /* TODO: Drive high charge pump (add IOE driver control). */
    }
    else
    {
      /* TODO: Drive low charge pump (add IOE driver control). */
    }
  }
  HAL_Delay(200);
  return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_SetToggle(USBH_HandleTypeDef *phost, uint8_t pipe, uint8_t toggle)
{
  HCD_HandleTypeDef *pHandle = phost->pData;
  if (pHandle->hc[pipe].ep_is_in)
  {
    pHandle->hc[pipe].toggle_in = toggle;
  }
  else
  {
    pHandle->hc[pipe].toggle_out = toggle;
  }
  return USBH_OK;
}

uint8_t USBH_LL_GetToggle(USBH_HandleTypeDef *phost, uint8_t pipe)
{
  HCD_HandleTypeDef *pHandle = phost->pData;
  return pHandle->hc[pipe].ep_is_in ? pHandle->hc[pipe].toggle_in
      : pHandle->hc[pipe].toggle_out;
}

void USBH_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

USBH_StatusTypeDef USBH_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
  switch (hal_status)
  {
  case HAL_OK:    return USBH_OK;
  case HAL_BUSY:  return USBH_BUSY;
  case HAL_ERROR:
  case HAL_TIMEOUT:
  default:        return USBH_FAIL;
  }
}
