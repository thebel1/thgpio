/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 *    References:
 *       - /bora/modules/vmkernel/tests/vmkapitest/vmkapitest_ndd/chardev/ndd_chardev.c
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_charDev.c --
 */

#include "gpio_charDev.h"

/***********************************************************************/

static vmk_CharDevOps gpio_charDevFileOps = {
   .open = gpio_charDevOpen,
   .close = gpio_charDevClose,
   .ioctl = gpio_charDevIoctl,
   .poll = gpio_charDevPoll,
   .read = gpio_charDevRead,
   .write = gpio_charDevWrite,
};

static vmk_CharDevRegOps gpio_CharDevOps = {
   .associate = gpio_charDevAssoc,
   .disassociate = gpio_charDevDisassoc,
   .fileOps = &gpio_charDevFileOps,
};

/*
 * Dev ops for the new vmk_Device associated with the char dev
 */
static vmk_DeviceOps gpio_CharVmkDevOps = {
   .removeDevice = gpio_charVmkDevRemove,
};

/*
 * Call backs that glue the char dev to the GPIO driver
 */
static gpio_CharDevCallbacks_t *gpio_CharDevCBs;

/***********************************************************************/

/*
 * Char dev has its own pointer to gpio_os's driver struct, so we don't
 * have a global shared by several files.
 */
static gpio_Driver_t *gpio_Driver;

static vmk_TimerQueue gpio_charDevTimerQueue;

/*
 ***********************************************************************
 * gpio_charDevRegister --
 * 
 *    Register a character device with the vmkernel chardev layer.
 * 
 *    References:
 *       - /bora/modules/vmkernel/tests/vmkapitest/vmkapitest_ndd/chardev/ndd_chardev.c
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevRegister(gpio_Driver_t *gpioDriver,
                     vmk_BusType logicalBusType,
                     vmk_Device parentDevice,
                     gpio_CharDev_t *charDev,
                     vmk_uint32 logicalPort,
                     gpio_CharDevCallbacks_t *devCBs)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_Device newDevice;
   vmk_DeviceID devID;
   vmk_AddrCookie registeringDriverData;
   vmk_AddrCookie registrationData;
   vmk_DeviceProps deviceProps;
   vmk_AddrCookie charDevPriv;

   /* Make available globally */
   gpio_Driver = gpioDriver;

   devID.busType = logicalBusType;
   status = vmk_LogicalCreateBusAddress(gpioDriver->driverHandle,
                                        parentDevice, logicalPort,
                                        &devID.busAddress,
                                        &devID.busAddressLen);
   if (status != VMK_OK) {
      goto logical_bus_failed;
   }

   /*
    * As per vmkapi_char.h it can only be graphics or a test device
    */
   devID.busIdentifier = VMK_CHARDEV_IDENTIFIER_GRAPHICS;
   devID.busIdentifierLen = vmk_Strnlen(devID.busIdentifier, VMK_MISC_NAME_MAX);

   charDevPriv.ptr = charDev;

   /*
    * Set up char dev registration
    */

   charDev->regData.moduleID = gpioDriver->moduleID;
   charDev->regData.deviceOps = &gpio_CharDevOps;
   charDev->regData.devicePrivate = charDevPriv;

   registrationData.ptr = &charDev->regData;
   registeringDriverData.ptr = charDev;

   deviceProps.registeringDriver = gpioDriver->driverHandle;
   deviceProps.deviceID = &devID;
   deviceProps.deviceOps = &gpio_CharVmkDevOps;
   deviceProps.registeringDriverData = registeringDriverData;
   deviceProps.registrationData = registrationData;

   status = vmk_DeviceRegister(&deviceProps, parentDevice, &newDevice);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to register device: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto register_device_failed;
   }

   status = vmk_LogicalFreeBusAddress(gpioDriver->driverHandle,
                                      devID.busAddress);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to free logical bus: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto free_bus_failed;
   }

   /*
    * Set CBs
    */
   gpio_CharDevCBs = devCBs;

   return VMK_OK;

free_bus_failed:
vmk_LogicalFreeBusAddress(gpioDriver->driverHandle,
                          devID.busAddress);
register_device_failed:
logical_bus_failed:
   return status;
}

/*
 ***********************************************************************
 * gpio_charVmkDevRemove --
 * 
 *    Remove character device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charVmkDevRemove(vmk_Device logicalDev)
{
   VMK_ReturnStatus status = VMK_OK;

   status = vmk_DeviceUnregister(logicalDev);
   if (status != VMK_OK)
   {
      vmk_WarningMessage("%s: %s: failed to unregister device: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
   }

   return status;
}

/*
 ***********************************************************************
 * gpio_charDevAssoc --
 * 
 *    Associate a char device with a device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevAssoc(vmk_AddrCookie charDevPriv,
                  vmk_CharDevHandle charDevHandle)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_Name charDevAlias;
   int aliasLen;

   status = vmk_CharDeviceGetAlias(charDevHandle, &charDevAlias);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to obtain logical device alias: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto get_alias_failed;
   }

#ifdef GPIO_DEBUG
   {
      vmk_LogMessage("%s: %s: obtained logical device alias %s",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     charDevAlias.string);
   }
#endif /* GPIO_DEBUG */

   aliasLen = vmk_Strnlen(charDevAlias.string, VMK_MISC_NAME_MAX);

   // TODO: checks to determine whether the chardev is valid

   return VMK_OK;

get_alias_failed:
   return status;
}

/*
 ***********************************************************************
 * gpio_charDevDisassoc --
 * 
 *    Disassociate a char device with a device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevDisassoc(vmk_AddrCookie charDevPriv)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_charDevOpen --
 * 
 *    Opens a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevOpen(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_CharFileData_t *fileData;
   vmk_SpinlockCreateProps lockProps;

   /*
    * Init private file data
    */

   fileData = vmk_HeapAlloc(gpio_Driver->heapID, sizeof(*fileData));
   if (fileData == NULL) {
      status = VMK_NO_MEMORY;
      vmk_WarningMessage("%s: %s: failed to create file private data: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto file_priv_alloc_failed;
   }
   vmk_Memset(fileData, 0, sizeof(*fileData));

   /*
    * Init lock
    */

   lockProps.moduleID = gpio_Driver->moduleID;
   lockProps.heapID = gpio_Driver->heapID;
   status = vmk_NameInitialize(&lockProps.name, GPIO_DRIVER_NAME);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to init lock name: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto lock_init_failed;
   }

   lockProps.type = VMK_SPINLOCK;
   lockProps.domain = VMK_LOCKDOMAIN_INVALID;
   lockProps.rank = VMK_SPINLOCK_UNRANKED;
   status = vmk_SpinlockCreate(&lockProps, &fileData->lock);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to create spinlock: %s",
         	             GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto lock_init_failed;
   }

   /*
    * Alloc I/O data
    */
   fileData->data = vmk_HeapAlloc(gpio_Driver->heapID,
                                  GPIO_CHARDEV_BUFFER_SIZE);
   if (fileData->data == NULL) {
      status = VMK_NO_MEMORY;
      vmk_WarningMessage("%s: %s: failed to alloc I/O buffer: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto buffer_alloc_failed;
   }

   /*
    * Zero out buffer
    */
   vmk_Memset(fileData->data, 0, GPIO_CHARDEV_BUFFER_SIZE);

   /*
    * Prep file for I/O
    */
   fileData->timerPending = VMK_FALSE;
   fileData->deathPending = VMK_FALSE;
   fileData->pollMask = VMKAPI_POLL_WRITE;
   fileData->timeoutUS = GPIO_CHARDEV_POLL_TIMEOUT_US;
   attr->clientInstanceData.ptr = fileData;

#ifdef GPIO_DEBUG
   {
      vmk_LogMessage("%s: %s: opened file; priv %p, data %p, lock %p",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     fileData,
                     fileData->data,
                     fileData->lock);
   }
#endif /* GPIO_DEBUG */

   /* Call CB */
   gpio_CharDevCBs->open(attr);

   return VMK_OK;

buffer_alloc_failed:
   vmk_SpinlockDestroy(fileData->lock);

lock_init_failed:
   vmk_HeapFree(gpio_Driver->heapID, fileData);

file_priv_alloc_failed:
   return status;
}

/*
 ***********************************************************************
 * gpio_charDevClose --
 * 
 *    Closes a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevClose(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_CharFileData_t *fileData = attr->clientInstanceData.ptr;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_WarningMessage("%s: %s: file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   fileData->deathPending = VMK_TRUE;

   if (fileData->timerPending == VMK_FALSE) {
      vmk_SpinlockUnlock(fileData->lock);
      gpio_charDevFileDestroy(fileData);
   }

   /* Call CB */
   gpio_CharDevCBs->close(attr);

file_data_null:
   return status;
}

/*
 ***********************************************************************
 * gpio_charDevIoctl --
 * 
 *    GPIO chardev-specific I/O ops. Currently not supported.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevIoctl(vmk_CharDevFdAttr *attr,
                  vmk_uint32 cmd,
                  vmk_uintptr_t userData,
                  vmk_IoctlCallerSize callerSize,
                  vmk_int32 *result)
{
   return VMK_NOT_SUPPORTED;
}

/*
 ***********************************************************************
 * gpio_charDevRead --
 * 
 *    Reads from a file. Not supported for now, as we only allow writing
 *    commands of sorts to the chardev file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevRead(vmk_CharDevFdAttr *attr,
                 char *buffer,
                 vmk_ByteCount nbytes,
                 vmk_loff_t *ppos,
                 vmk_ByteCountSigned *nread)
{
   return VMK_NOT_SUPPORTED;
}

/*
 ***********************************************************************
 * gpio_charDevWrite --
 * 
 *    Writes to a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevWrite(vmk_CharDevFdAttr *attr,
                  char *buffer,
                  vmk_ByteCount nbytes,
                  vmk_loff_t *ppos,
                  vmk_ByteCountSigned *nwritten)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_CharFileData_t *fileData;
   vmk_ByteCount curByte;
   vmk_ByteCountSigned doneBytes = 0;
   vmk_loff_t offset;
   char *localBuffer;

   if (attr == NULL || ppos == NULL || nwritten == NULL) {
      status = VMK_BAD_PARAM;
      vmk_WarningMessage("%s: %s: invalid write command received;"
                         " attr %p ppos %p ndone %p",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         attr, ppos, nwritten);
      goto invalid_params;
   }

#ifdef GPIO_DEBUG
   {
      vmk_LogMessage("%s: %s: nbytes %lu offset %ld buffer %p",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     nbytes,
                     *ppos,
                     buffer);
   }
#endif /* GPIO_DEBUG */

   localBuffer = vmk_HeapAlloc(gpio_Driver->heapID, GPIO_CHARDEV_BUFFER_SIZE);
   if (localBuffer == NULL) {
      status = VMK_NO_MEMORY;
      vmk_WarningMessage("%s: %s: unable to allocate local buffer: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto localBuffer_alloc_failed;
   }

   offset = *ppos;
   fileData = (gpio_CharFileData_t *)attr->clientInstanceData.ptr;
   
   /*
    * Copy data from user space
    */
   for (curByte = 0; curByte < nbytes; ++curByte) {
      status = vmk_CopyFromUser(
         (vmk_VA)&localBuffer[(curByte + offset) % GPIO_CHARDEV_BUFFER_SIZE],
         (vmk_VA)&buffer[curByte],
         sizeof(char)
      );
      if (status != VMK_OK) {
         break;
      }
      ++doneBytes;
   }

   vmk_SpinlockLock(fileData->lock);
   vmk_Memcpy(fileData->data, localBuffer, GPIO_CHARDEV_BUFFER_SIZE);
   vmk_SpinlockUnlock(fileData->lock);

   vmk_HeapFree(gpio_Driver->heapID, localBuffer);

   *nwritten = doneBytes;

   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: I/O failed to file %p: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto write_failed;
   }

   /* Call CB */
   gpio_CharDevCBs->write(localBuffer, doneBytes);

   return VMK_OK;

write_failed:
localBuffer_alloc_failed:
invalid_params:
   return status;
}

/*
 ***********************************************************************
 * gpio_charDevPoll --
 * 
 *    Polls a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevPoll(vmk_CharDevFdAttr *attr,
                 vmk_PollContext pollCtx,
                 vmk_uint32 *pollMask)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_CharFileData_t *fileData = attr->clientInstanceData.ptr;
   vmk_TimerCookie tmrCookie;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_WarningMessage("%s: %s: file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   if (fileData->pollMask == VMK_FALSE
       && fileData->timerPending == VMK_FALSE
       && fileData->deathPending == VMK_FALSE) {
          tmrCookie.ptr = fileData;
          status = vmk_TimerSchedule(gpio_charDevTimerQueue,
                                     gpio_charDevTimerCB,
                                     tmrCookie,
                                     GPIO_CHARDEV_POLL_TIMEOUT_US,
                                     VMK_TIMER_DEFAULT_TOLERANCE,
                                     VMK_TIMER_ATTR_NONE,
                                     VMK_LOCKDOMAIN_INVALID,
                                     VMK_SPINLOCK_UNRANKED,
                                     &fileData->timer);
      if (status == VMK_OK) {
         fileData->timerPending = VMK_TRUE;
      }
      else {
         vmk_WarningMessage("%s: %s: failed to create poll timer: %s",
                            GPIO_DRIVER_NAME,
                            __FUNCTION__,
                            vmk_StatusToString(status));
         goto create_poll_timer_failed;
      }
   }

   vmk_CharDevSetPollContext(pollCtx, (void *)fileData);
   *pollMask = fileData->pollMask;

   vmk_SpinlockUnlock(fileData->lock);

   return VMK_OK;

create_poll_timer_failed:
   vmk_SpinlockUnlock(fileData->lock);

file_data_null:
   return status;
}

/*
 ***********************************************************************
 * gpio_charDevTimerCB --
 * 
 *    Callback for char device poll timer.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
void
gpio_charDevTimerCB(vmk_TimerCookie data)
{
   gpio_CharFileData_t *fileData = (gpio_CharFileData_t *)data.ptr;

   if (fileData == NULL) {
      vmk_WarningMessage("%s: %s: file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   fileData->timerPending = VMK_FALSE;
   fileData->pollMask = VMKAPI_POLL_WRITE;

   if (fileData->deathPending == VMK_FALSE) {
      /* Wake up pollers in UW */
      vmk_CharDevWakePollers(fileData);
   }
   else {
      vmk_SpinlockUnlock(fileData->lock);
      gpio_charDevFileDestroy(fileData);
      goto death_pending;
   }

   vmk_SpinlockUnlock(fileData->lock);

death_pending:
file_data_null:
   return;
}

/*
 ***********************************************************************
 * gpio_charDevFileDestroy --
 * 
 *    Destroy a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
void
gpio_charDevFileDestroy(gpio_CharFileData_t *fileData)
{
#ifdef GPIO_DEBUG
   {
      vmk_LogMessage("%s: %s: destroying file %p",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     fileData);
   }
#endif /* GPIO_DEBUG */

   vmk_SpinlockDestroy(fileData->lock);
   vmk_HeapFree(gpio_Driver->heapID, fileData->data);
   vmk_HeapFree(gpio_Driver->heapID, fileData);
}