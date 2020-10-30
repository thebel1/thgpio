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
#define GPIO_MMIO_SIZE 0xf4

/* Each register is 4 bytes in size */
#define GPIO_REG_SIZE 4

#define GPIO_NUM_REGS (GPIO_MMIO_SIZE / GPIO_REG_SIZE)

#define GPIO_POLL_TIMEOUT_SEC 1

/***********************************************************************/

/*
 * Packing the struct to avoid alignment issues when passing between UW and
 * driver. Must be exactly 64 bits in size!
 */
#pragma pack(push, 1)
typedef struct gpio_IoctlData_t {
   vmk_uint16 offset;
   union {
      vmk_uint32 mask;
      vmk_uint32 data;
   };
   /* We might use this for something at some point */
   vmk_uint16 __padding;
} gpio_IoctlData_t;
#pragma pack(pop)

/*
 * An opaque version of gpio_IoctlData_t, to be used outside of gpio_drv.c.
 */
typedef vmk_uint64 gpio_IoctlCookie_t;

typedef enum gpio_IoctlCommands_t {
   GPIO_IOCTL_INVALID   = 0,
   /* Poll a GPIO register until masked bits change */
   GPIO_IOCTL_POLL      = 1,
   /* Used for sanity */
   GPIO_IOCTL_MAX       = GPIO_IOCTL_POLL,
} gpio_IoctlCommands_t;

/***********************************************************************/

VMK_ReturnStatus gpio_drvInit(gpio_Driver_t *driver,
                              gpio_Device_t *adapter);

VMK_ReturnStatus gpio_mmioPoll(vmk_uint32 offset,
                               vmk_uint32 mask,
                               vmk_uint32 *data);

VMK_ReturnStatus gpio_mmioDirectRead(vmk_uint32 offset,
                                     vmk_uint32 *value);

VMK_ReturnStatus gpio_mmioDirectWrite(vmk_uint32 offset,
                                      vmk_uint32 value);

VMK_ReturnStatus gpio_mmioOpenCB(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus gpio_mmioCloseCB(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus gpio_mmioIoctlCB(unsigned int cmd,
                                  gpio_IoctlCookie_t *data);

VMK_ReturnStatus gpio_mmioReadCB(char *buffer,
                                 vmk_ByteCount nbytes,
                                 vmk_loff_t *ppos,
                                 vmk_ByteCountSigned *nread);

VMK_ReturnStatus gpio_mmioWriteCB(char *buffer,
                                  vmk_ByteCount nbytes,
                                  vmk_loff_t *ppos,
                                  vmk_ByteCountSigned *nwritten);

/***********************************************************************/

#endif /* GPIO_DRV_H */