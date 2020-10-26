/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_debug.h --
 */

#ifndef GPIO_DEBUG_H
#define GPIO_DEBUG_H

#include "gpio.h"
#include "gpio_types.h"
#include "gpio_drv.h"

/***********************************************************************/

VMK_ReturnStatus gpioDebug_dumpMMIOMem(gpio_Device_t *adapter);
VMK_ReturnStatus gpioDebug_dumpRegisters(gpio_Device_t *adapter);

VMK_ReturnStatus gpioDebug_testPins(gpio_Device_t *adapter);
VMK_ReturnStatus gpioDebug_dumpPins(gpio_Device_t *adapter);

/***********************************************************************/

#endif /* GPIO_DEBUG_H */