/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_drv.h --
 *
 *    Definition for gpio device layer interface
 */

#ifndef GPIO_DRV_H
#define GPIO_DRV_H

#include "gpio.h"
#include "gpio_types.h"

/***********************************************************************/

/*
 * As documented in:
 * https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2711/rpi_DATA_2711_1p0.pdf
 */
#define GPIO_MMIO_MAX_OFFSET 0xf4

/* Each register is 4 bytes in size */
#define GPIO_REG_SIZE 4

/***********************************************************************/

VMK_ReturnStatus gpio_drvInit(gpio_Device_t *adapter);

VMK_INLINE VMK_ReturnStatus gpio_mmioDirectRead(vmk_uint32 offset,
                                                vmk_uint32 *value);

VMK_INLINE VMK_ReturnStatus gpio_mmioDirectWrite(vmk_uint32 offset,
                                                 vmk_uint32 value);

VMK_INLINE VMK_ReturnStatus gpio_mmioOpenCB(vmk_CharDevFdAttr *attr);

VMK_INLINE VMK_ReturnStatus gpio_mmioCloseCB(vmk_CharDevFdAttr *attr);

VMK_INLINE VMK_ReturnStatus gpio_mmioReadCB(char *buffer,
                                            vmk_ByteCount nbytes,
                                            vmk_loff_t *ppos,
                                            vmk_ByteCountSigned *nread);

VMK_INLINE VMK_ReturnStatus gpio_mmioWriteCB(char *buffer,
                                             vmk_ByteCount nbytes,
                                             vmk_loff_t *ppos,
                                             vmk_ByteCountSigned *nwritten);

/***********************************************************************/

#endif /* GPIO_DRV_H */