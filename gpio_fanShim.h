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

#define GPIO_FANSHIM_PIN_CLOCK  14
#define GPIO_FANSHIM_PIN_DATA   15
#define GPIO_FANSHIM_PIN_BUTTON 17
#define GPIO_FANSHIM_PIN_FAN    18

#define GPIO_FANSHIM_CMD_MAX_LEN 32

/***********************************************************************/

VMK_ReturnStatus gpio_fanShimInit(gpio_Device_t *adapter);

VMK_ReturnStatus gpio_fanShimCharDevOpenCB(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus gpio_fanShimCharDevCloseCB(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus gpio_fanShimCharDevReadCB(char *buffer,
                                           vmk_ByteCount nbytes);

VMK_ReturnStatus gpio_fanShimCharDevWriteCB(char *buffer,
                                            vmk_ByteCountSigned nwritten);

VMK_ReturnStatus gpio_fanShimFanToggle();

VMK_ReturnStatus gpio_fanShimSetLED(vmk_uint8 red,
                                    vmk_uint8 green,
                                    vmk_uint8 blue,
                                    vmk_uint8 brightness);

VMK_ReturnStatus gpio_fanShimFlashLED(vmk_uint8 red,
                                      vmk_uint8 green,
                                      vmk_uint8 blue,
                                      vmk_uint8 brightness,
                                      int durationMs,
                                      int intervalMs);

VMK_ReturnStatus gpio_fanShimGradientLED(int durationMs, int periodMs);

/***********************************************************************/

#endif /* GPIO_FANSHIM_H */