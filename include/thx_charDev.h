/******************************************************************************\
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * thx_charDev.h --
 */

#ifndef THX_CHARDEV_H
#define THX_CHARDEV_H

/***********************************************************************/

#define VMKAPI_ONLY
#include "vmkapi.h"

/***********************************************************************/

#define THX_DRIVER_NAME "thx_char_dev"
#define THX_CHARDEV_BUFFER_SIZE       4096 /* Probably overkill */
#define THX_CHARDEV_POLL_TIMEOUT_US   1000000

/***********************************************************************/

/*
 * Opaque cookie for passing ioctl argument.
 */
typedef vmk_uint64 thx_IoctlCookie_t;

/*
 * Callbacks for gluing the char dev driver to the THX driver
 */
typedef VMK_ReturnStatus (*thx_CharDevOpenCB_t)(vmk_CharDevFdAttr *attr);
typedef VMK_ReturnStatus (*thx_CharDevCloseCB_t)(vmk_CharDevFdAttr *attr);

typedef VMK_ReturnStatus (*thx_CharDevIoctlCB_t)(unsigned int cmd,
                                                 thx_IoctlCookie_t *ioctlData);
typedef VMK_ReturnStatus (*thx_CharDevReadCB_t)(char *buffer,
                                                vmk_ByteCount nbytes,
                                                vmk_loff_t *ppos,
                                                vmk_ByteCountSigned *nread);
typedef VMK_ReturnStatus (*thx_CharDevWriteCB_t)(char *buffer,
                                                 vmk_ByteCount nbytes,
                                                 vmk_loff_t *ppos,
                                                 vmk_ByteCountSigned *nread);
typedef struct thx_CharDevCallbacks_t {
   thx_CharDevOpenCB_t open;
   thx_CharDevCloseCB_t close;
   thx_CharDevIoctlCB_t ioctl;
   thx_CharDevReadCB_t read;
   thx_CharDevWriteCB_t write;
} thx_CharDevCallbacks_t;

typedef struct thx_CharDev_t {
   vmk_Device vmkDevice;
   vmk_CharDevRegData regData;
   thx_CharDevCallbacks_t devCBs;
} thx_CharDev_t;

typedef struct thx_CharFileData_t {
   vmk_Lock lock;
   char *data;
   vmk_Bool timerPending;
   vmk_Bool deathPending;
   vmk_Timer timer;
   vmk_int32 timeoutUS;
   vmk_uint32 pollMask;
} thx_CharFileData_t;

typedef struct thx_CharDevProps_t {
   vmk_Driver driverHandle;
   vmk_BusType logicalBusType;
   vmk_Device parentDevice;
   thx_CharDev_t *charDev;
   vmk_uint32 logicalPort;
   thx_CharDevCallbacks_t *callbacks;
} thx_CharDevProps_t;

/***********************************************************************/

VMK_ReturnStatus thx_charDevInit(vmk_ModuleID moduleID,
                                 vmk_HeapID heapID,
                                 vmk_LogComponent logger);

VMK_ReturnStatus thx_charDevRegister(thx_CharDevProps_t *props);

VMK_ReturnStatus thx_charVmkDevRemove(vmk_Device logicalDev);

VMK_ReturnStatus thx_charDevAssoc(vmk_AddrCookie charDevPriv,
                                  vmk_CharDevHandle charDevHandle);

VMK_ReturnStatus thx_charDevDisassoc(vmk_AddrCookie charDevPriv);

VMK_ReturnStatus thx_charDevOpen(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus thx_charDevClose(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus thx_charDevIoctl(vmk_CharDevFdAttr *attr,
                                  vmk_uint32 cmd,
                                  vmk_uintptr_t userData,
                                  vmk_IoctlCallerSize callerSize,
                                  vmk_int32 *result);

VMK_ReturnStatus thx_charDevRead(vmk_CharDevFdAttr *attr,
                                 char *buffer,
                                 vmk_ByteCount nbytes,
                                 vmk_loff_t *ppos,
                                 vmk_ByteCountSigned *nread);

VMK_ReturnStatus thx_charDevWrite(vmk_CharDevFdAttr *attr,
                                  char *buffer,
                                  vmk_ByteCount nbytes,
                                  vmk_loff_t *ppos,
                                  vmk_ByteCountSigned *nwritten);

VMK_ReturnStatus thx_charDevIO(vmk_CharDevFdAttr *attr,
                               char *buffer,
                               vmk_ByteCount nbytes,
                               vmk_loff_t *ppos,
                               vmk_ByteCountSigned *ndone,
                               vmk_Bool isWrite);

VMK_ReturnStatus thx_charDevPoll(vmk_CharDevFdAttr *attr,
                                 vmk_PollContext pollCtx,
                                 vmk_uint32 *pollMask);

void thx_charDevTimerCB(vmk_TimerCookie data);

void thx_charDevFileDestroy(thx_CharFileData_t *fileData);

/***********************************************************************/

#endif /* THX_CHARDEV_H */