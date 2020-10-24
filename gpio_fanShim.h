/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_fanShim.h --
 */

#ifndef GPIO_FANSHIM_H
#define GPIO_FANSHIM_H

#include "gpio.h"
#include "gpio_types.h"
#include "gpio_drv.h"

/***********************************************************************/

VMK_ReturnStatus gpio_fanShimFanWorldFunc(void *clientData);

VMK_ReturnStatus gpio_fanShimLEDWorldFunc(void *clientData);

VMK_ReturnStatus gpio_fanShimBtnWorldFunc(void *clientData);

VMK_ReturnStatus gpio_fanShimSetLED(gpio_Device_t *adapter,
                                    vmk_uint8 red,
                                    vmk_uint8 green,
                                    vmk_uint8 blue,
                                    vmk_uint8 brightness);

VMK_ReturnStatus gpio_fanShimFlashLED(gpio_Device_t *adapter,
                                      vmk_uint8 red,
                                      vmk_uint8 green,
                                      vmk_uint8 blue,
                                      vmk_uint8 brightness,
                                      int intervalMs);

VMK_ReturnStatus gpio_fanShimGradientLED(gpio_Device_t *adapter, int periodMs);

/***********************************************************************/

#endif /* GPIO_FANSHIM_H */