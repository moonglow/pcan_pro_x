#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#define USBD_MAX_NUM_INTERFACES         2U
#define USBD_MAX_NUM_CONFIGURATION      1U
#define USBD_MAX_STR_DESC_SIZ           512U
#define USBD_SUPPORT_USER_STRING_DESC   (1)
#define USBD_DEBUG_LEVEL                0U
#define USBD_LPM_ENABLED                0U
#define USBD_SELF_POWERED               1U


#define DEVICE_FS                       0
#define DEVICE_HS                       1

#define USBD_malloc         malloc
#define USBD_free           free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

/* DEBUG macros */
#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)    printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)

#define USBD_ErrLog(...)    printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)    printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_DbgLog(...)
#endif
