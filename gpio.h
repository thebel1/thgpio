/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio.h --
 */

#ifndef GPIO_H
#define GPIO_H

#define VMKAPI_ONLY
#include "vmkapi.h"

/***********************************************************************/

#define GPIO_DEBUG

#define GPIO_DRIVER_NAME "thgpio"

// Probably overkill
#define GPIO_HEAP_INITIAL_SIZE (1024 * 1024)
#define GPIO_HEAP_MAX_SIZE (2 * 1024 * 1024)

#define GPIO_MIN(a, b) (a > b ? b : a)
#define GPIO_MAX(a, b) (a > b ? a : b)

#define GPIO_INT_MAX ((vmk_uint32)~0)

/***********************************************************************/

#endif /* GPIO_H */