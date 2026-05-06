#ifndef __USBH_CONF_H
#define __USBH_CONF_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "stm32h7rsxx.h"
#include "stm32h7rsxx_hal.h"

/*
 * USB-host event semaphore plumbing. The HCD callbacks in usbh_conf.c
 * give a binary semaphore from IRQ context whenever something useful
 * happens (URB completion, port connect/disconnect/enable/disable),
 * which the user-side USB pumping task takes to wake up.
 *
 * Call USBH_HostEvent_AppInit() once at app startup before the task
 * starts taking the semaphore. Static allocation, can be called before
 * the scheduler runs.
 */
void USBH_HostEvent_AppInit(void);
bool USBH_HostEvent_Wait(uint32_t timeoutMs);

#define USBH_MAX_NUM_ENDPOINTS         4U
#define USBH_MAX_NUM_INTERFACES        4U
#define USBH_MAX_NUM_CONFIGURATION     4U
#define USBH_KEEP_CFG_DESCRIPTOR       1U
#define USBH_MAX_NUM_SUPPORTED_CLASS   4U
#define USBH_MAX_SIZE_CONFIGURATION    2048U
#define USBH_MAX_DATA_BUFFER           2048U
#define USBH_DEBUG_LEVEL               3U
#define USBH_USE_OS                    1U

#define HOST_HS                        0
#define HOST_FS                        1

#if (USBH_USE_OS == 1)
  #include "cmsis_os.h"
  #define USBH_PROCESS_PRIO          osPriorityNormal
  #define USBH_PROCESS_STACK_SIZE    ((uint16_t)4096)
#endif /* (USBH_USE_OS == 1) */

#define USBH_malloc         malloc
#define USBH_free           free
#define USBH_memset         memset
#define USBH_memcpy         memcpy

#if (USBH_DEBUG_LEVEL > 0U)
#define  USBH_UsrLog(...)   do { \
                            printf(__VA_ARGS__); \
                            printf("\n"); \
} while (0)
#else
#define USBH_UsrLog(...) do {} while (0)
#endif

#if (USBH_DEBUG_LEVEL > 1U)
#define  USBH_ErrLog(...) do { \
                            printf("ERROR: "); \
                            printf(__VA_ARGS__); \
                            printf("\n"); \
} while (0)
#else
#define USBH_ErrLog(...) do {} while (0)
#endif

#if (USBH_DEBUG_LEVEL > 2U)
#define  USBH_DbgLog(...)   do { \
                            printf("DEBUG : "); \
                            printf(__VA_ARGS__); \
                            printf("\n"); \
} while (0)
#else
#define USBH_DbgLog(...) do {} while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __USBH_CONF_H */
