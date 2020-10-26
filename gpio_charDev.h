/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_charDev.h --
 */

#ifndef GPIO_CHARDEV_H
#define GPIO_CHARDEV_H

#include "gpio.h"
#include "gpio_types.h"
#include "gpio_drv.h"

/***********************************************************************/

#define GPIO_CHARDEV_BUFFER_SIZE       4096 /* Probably overkill */
#define GPIO_CHARDEV_POLL_TIMEOUT_US   1000000

/***********************************************************************/

/*
 * Callbacks for gluing the char dev driver to the GPIO driver
 */
typedef VMK_ReturnStatus (*gpio_CharDevOpenCB_t)(vmk_CharDevFdAttr *attr);
typedef VMK_ReturnStatus (*gpio_CharDevCloseCB_t)(vmk_CharDevFdAttr *attr);
typedef VMK_ReturnStatus (*gpio_CharDevReadCB_t)(char *buffer,
                                                 vmk_ByteCount nbytes);
typedef VMK_ReturnStatus (*gpio_CharDevWriteCB_t)(char *buffer,
                                                  vmk_ByteCountSigned nwritten);
typedef struct gpio_CharDevCallbacks_t {
   gpio_CharDevOpenCB_t open;
   gpio_CharDevCloseCB_t close;
   gpio_CharDevReadCB_t read;
   gpio_CharDevWriteCB_t write;
} gpio_CharDevCallbacks_t;

typedef struct gpio_CharDev_t {
   vmk_Device vmkDevice;
   vmk_CharDevRegData regData;
   gpio_CharDevCallbacks_t devCBs;
} gpio_CharDev_t;

typedef struct gpio_CharFileData_t {
   vmk_Lock lock;
   char *data;
   vmk_Bool timerPending;
   vmk_Bool deathPending;
   vmk_Timer timer;
   vmk_int32 timeoutUS;
   vmk_uint32 pollMask;
} gpio_CharFileData_t;

/***********************************************************************/

VMK_ReturnStatus gpio_charDevRegister(gpio_Driver_t *gpioDriver,
                                      vmk_BusType logicalBusType,
                                      vmk_Device parentDevice,
                                      gpio_CharDev_t *charDev,
                                      vmk_uint32 logicalPort,
                                      gpio_CharDevCallbacks_t *devCBs);

VMK_ReturnStatus gpio_charVmkDevRemove(vmk_Device logicalDev);

VMK_ReturnStatus gpio_charDevAssoc(vmk_AddrCookie charDevPriv,
                                   vmk_CharDevHandle charDevHandle);

VMK_ReturnStatus gpio_charDevDisassoc(vmk_AddrCookie charDevPriv);

VMK_ReturnStatus gpio_charDevOpen(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus gpio_charDevClose(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus gpio_charDevIoctl(vmk_CharDevFdAttr *attr,
                                   vmk_uint32 cmd,
                                   vmk_uintptr_t userData,
                                   vmk_IoctlCallerSize callerSize,
                                   vmk_int32 *result);

VMK_ReturnStatus gpio_charDevRead(vmk_CharDevFdAttr *attr,
                                  char *buffer,
                                  vmk_ByteCount nbytes,
                                  vmk_loff_t *ppos,
                                  vmk_ByteCountSigned *nread);

VMK_ReturnStatus gpio_charDevWrite(vmk_CharDevFdAttr *attr,
                                   char *buffer,
                                   vmk_ByteCount nbytes,
                                   vmk_loff_t *ppos,
                                   vmk_ByteCountSigned *nwritten);

VMK_ReturnStatus gpio_charDevPoll(vmk_CharDevFdAttr *attr,
                                  vmk_PollContext pollCtx,
                                  vmk_uint32 *pollMask);

void gpio_charDevTimerCB(vmk_TimerCookie data);

void gpio_charDevFileDestroy(gpio_CharFileData_t *fileData);

/***********************************************************************/

#endif /* GPIO_CHARDEV_H */