/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "semphr.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* RX byte stream from USB ISR to consumer thread. */
#define CDC_RX_STREAM_LEN  1024U
static StaticStreamBuffer_t cdcRxStreamMeta;
static uint8_t              cdcRxStreamStorage[CDC_RX_STREAM_LEN + 1];
static StreamBufferHandle_t cdcRxStream;

/* TX-done signal from USB ISR to producer thread.  Initially "given" so the
 * first CDC_Write() can take it without waiting. */
static StaticSemaphore_t cdcTxDoneMeta;
static SemaphoreHandle_t cdcTxDone;
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB FS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  /* Push received bytes into the RX stream BEFORE re-arming the next receive,
   * since the next receive will overwrite Buf. */
  if (cdcRxStream != NULL && Len != NULL && *Len > 0U)
  {
    BaseType_t higherPrioWoken = pdFALSE;
    (void)xStreamBufferSendFromISR(cdcRxStream, Buf, *Len, &higherPrioWoken);
    portYIELD_FROM_ISR(higherPrioWoken);
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  if (cdcTxDone != NULL)
  {
    BaseType_t higherPrioWoken = pdFALSE;
    (void)xSemaphoreGiveFromISR(cdcTxDone, &higherPrioWoken);
    portYIELD_FROM_ISR(higherPrioWoken);
  }
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @brief  Create the FreeRTOS objects backing CDC_Read / CDC_Write.
 *         Must be called once, after the kernel is initialized and before
 *         the USB device stack starts.
 */
void CDC_AppInit(void)
{
  if (cdcRxStream == NULL)
  {
    cdcRxStream = xStreamBufferCreateStatic(sizeof(cdcRxStreamStorage),
                                            1U,
                                            cdcRxStreamStorage,
                                            &cdcRxStreamMeta);
  }
  if (cdcTxDone == NULL)
  {
    cdcTxDone = xSemaphoreCreateBinaryStatic(&cdcTxDoneMeta);
    /* No transmit in flight at startup, so the first writer can proceed. */
    (void)xSemaphoreGive(cdcTxDone);
  }
}

/**
 * @brief  Block until up to maxLen bytes are available from the host, or
 *         until timeoutMs elapses with nothing received.
 * @retval Number of bytes copied into buf (0 on timeout or before init).
 */
size_t CDC_Read(uint8_t *buf, size_t maxLen, uint32_t timeoutMs)
{
  if (cdcRxStream == NULL || buf == NULL || maxLen == 0U)
  {
    return 0U;
  }
  TickType_t ticks = (timeoutMs == HAL_MAX_DELAY) ? portMAX_DELAY
                                                  : pdMS_TO_TICKS(timeoutMs);
  return xStreamBufferReceive(cdcRxStream, buf, maxLen, ticks);
}

/**
 * @brief  Send len bytes to the host, waiting up to timeoutMs for any
 *         in-flight transmit to complete first.
 * @retval USBD_OK on success, USBD_BUSY if timed out waiting for the
 *         previous transmit, USBD_FAIL on lower-layer error.
 */
uint8_t CDC_Write(const uint8_t *buf, uint16_t len, uint32_t timeoutMs)
{
  if (cdcTxDone == NULL || buf == NULL || len == 0U)
  {
    return USBD_FAIL;
  }
  TickType_t ticks = (timeoutMs == HAL_MAX_DELAY) ? portMAX_DELAY
                                                  : pdMS_TO_TICKS(timeoutMs);
  if (xSemaphoreTake(cdcTxDone, ticks) != pdTRUE)
  {
    return USBD_BUSY;
  }
  uint8_t result = CDC_Transmit_FS((uint8_t *)buf, len);
  if (result != USBD_OK)
  {
    /* No completion callback will fire — return the token so we don't
     * deadlock the next caller. */
    (void)xSemaphoreGive(cdcTxDone);
  }
  return result;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
