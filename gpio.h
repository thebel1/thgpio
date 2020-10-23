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

/*
 * Debug options
 */

#define GPIO_DEBUG
#ifdef GPIO_DEBUG
#include "gpio_debug.h"
#endif /* GPIO_DEBUG */

/***********************************************************************/

#define GPIO_DRIVER_NAME "thgpio"

// Probably overkill
#define GPIO_HEAP_INITIAL_SIZE (1024 * 1024)
#define GPIO_HEAP_MAX_SIZE (2 * 1024 * 1024)

#define GPIO_MIN(a, b) (a > b ? b : a)
#define GPIO_MAX(a, b) (a > b ? a : b)

#define GPIO_INT_MAX ((vmk_uint32)~0)

/***********************************************************************/

/*
 * GPIO device-specific constants.
 * 
 * See: https://github.com/RPi-Distro/raspi-gpio/blob/master/raspi-gpio.c
 */
#define GPIO_PIN_MIN 0
#define GPIO_PIN_MAX 53

/***********************************************************************/

#endif /* GPIO_H */