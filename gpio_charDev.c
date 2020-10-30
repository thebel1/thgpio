/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
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
      vmk_Warning(gpio_Driver->logger,
                  "failed to register device: %s",
                  vmk_StatusToString(status));
      goto register_device_failed;
   }

   status = vmk_LogicalFreeBusAddress(gpioDriver->driverHandle,
                                      devID.busAddress);
   if (status != VMK_OK) {
      vmk_Warning(gpio_Driver->logger,
                  "failed to free logical bus: %s",
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
      vmk_Warning(gpio_Driver->logger,
                  "failed to unregister device: %s",
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

   status = vmk_CharDeviceGetAlias(charDevHandle, &charDevAlias);
   if (status != VMK_OK) {
      vmk_Warning(gpio_Driver->logger,
                  "failed to obtain logical device alias: %s",
                  vmk_StatusToString(status));
      goto get_alias_failed;
   }

#ifdef GPIO_DEBUG
   {
      vmk_Log(gpio_Driver->logger,
              "obtained logical device alias %s",
              charDevAlias.string);
   }
#endif /* GPIO_DEBUG */

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
      vmk_Warning(gpio_Driver->logger,
                  "failed to create file private data: %s",
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
      vmk_Warning(gpio_Driver->logger,
                  "failed to init lock name: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

   lockProps.type = VMK_SPINLOCK;
   lockProps.domain = VMK_LOCKDOMAIN_INVALID;
   lockProps.rank = VMK_SPINLOCK_UNRANKED;
   status = vmk_SpinlockCreate(&lockProps, &fileData->lock);
   if (status != VMK_OK) {
      vmk_Warning(gpio_Driver->logger,
                  "failed to create spinlock: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

   /*
    * We don't use this buffer, so set it to NULL
    */
   fileData->data = NULL;

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
      vmk_Log(gpio_Driver->logger,
              "opened file; priv %p lock %p",
              fileData,
              fileData->lock);
   }
#endif /* GPIO_DEBUG */

   /* Call CB */
   status = gpio_CharDevCBs->open(attr);

   return status;

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
      vmk_Warning(gpio_Driver->logger, "file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   fileData->deathPending = VMK_TRUE;

   if (fileData->timerPending == VMK_FALSE) {
      vmk_SpinlockUnlock(fileData->lock);
      gpio_charDevFileDestroy(fileData);
   }

   /* Call CB */
   status = gpio_CharDevCBs->close(attr);

file_data_null:
   return status;
}

/*
 ***********************************************************************
 * gpio_charDevIoctl --
 * 
 *    GPIO chardev-specific I/O ops. Used for programming reads to input
 *    pins.
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
                  unsigned int cmd,
                  vmk_uintptr_t userData,
                  vmk_IoctlCallerSize callerSize,
                  vmk_int32 *result)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_IoctlCookie_t ioctlData;
   gpio_CharFileData_t *fileData = attr->clientInstanceData.ptr;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(gpio_Driver->logger, "file data null");
      goto file_data_null;
   }

   /* Copy ioctl data from UW */
   status = vmk_CopyFromUser((vmk_VA)&ioctlData,
                             (vmk_VA)userData,
                             sizeof(ioctlData));
   if (status != VMK_OK) {
      vmk_Warning(gpio_Driver->logger,
                  "unable to copy ioctl data from UW ptr %p: %s",
                  userData,
                  vmk_StatusToString(status));
      goto ioctl_uw2vmk_failed;
   }

#ifdef GPIO_DEBUG
   {
      vmk_Log(gpio_Driver->logger,
              "executing ioctl cmd %d with data %p",
              cmd,
              userData);
   }
#endif

   /* Call CB */
   vmk_SpinlockLock(fileData->lock);
   status = gpio_CharDevCBs->ioctl(cmd, &ioctlData);
   vmk_SpinlockUnlock(fileData->lock);
   if (status != VMK_OK) {
      vmk_Warning(gpio_Driver->logger,
                  "ioctl cmd %d with data %p failed: %s",
                  cmd,
                  userData,
                  vmk_StatusToString(status));
      goto ioctl_cmd_failed;
   }

   /* Copy iotl data back to UW */
   status = vmk_CopyToUser((vmk_VA)userData,
                           (vmk_VA)&ioctlData,
                           sizeof(ioctlData));
   if (status != VMK_OK) {
      vmk_Warning(gpio_Driver->logger,
                  "unable to copy ioctl data back to UW: %s",
                  vmk_StatusToString(status));
      goto ioctl_vmk2uw_failed;
   }

   return VMK_OK;

ioctl_vmk2uw_failed:
ioctl_cmd_failed:
ioctl_uw2vmk_failed:
file_data_null:
   return status;
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
   return gpio_charDevIO(attr, buffer, nbytes, ppos, nread, VMK_FALSE);
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
   return gpio_charDevIO(attr, buffer, nbytes, ppos, nwritten, VMK_TRUE);
}

/*
 ***********************************************************************
 * gpio_charDevIO --
 * 
 *    Read/write to file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_charDevIO(vmk_CharDevFdAttr *attr,
               char *buffer,
               vmk_ByteCount nbytes,
               vmk_loff_t *ppos,
               vmk_ByteCountSigned *ndone,
               vmk_Bool isWrite)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_CharFileData_t *fileData;
   vmk_ByteCount curByte;
   vmk_ByteCountSigned doneBytes = 0;
   vmk_loff_t offset;
   char *localBuffer;

   if (attr == NULL || ppos == NULL || ndone == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(gpio_Driver->logger,
                  "invalid write command received;"
                  " attr %p ppos %p ndone %p",
                  attr, ppos, ndone);
      goto invalid_params;
   }

#ifdef GPIO_DEBUG
   {
      vmk_Log(gpio_Driver->logger,
              "nbytes %lu offset %ld buffer %p",
              nbytes,
              *ppos,
              buffer);
   }
#endif /* GPIO_DEBUG */

   localBuffer = vmk_HeapAlloc(gpio_Driver->heapID, GPIO_CHARDEV_BUFFER_SIZE);
   if (localBuffer == NULL) {
      status = VMK_NO_MEMORY;
      vmk_Warning(gpio_Driver->logger,
                  "unable to allocate local buffer: %s",
                  vmk_StatusToString(status));
      goto localBuffer_alloc_failed;
   }

   offset = *ppos;
   fileData = (gpio_CharFileData_t *)attr->clientInstanceData.ptr;
   
   /*
    * Perform I/O and copy data to/from user space
    */
   if (isWrite) { /* Write */
      for (curByte = 0; curByte < nbytes; ++curByte) {
         status = vmk_CopyFromUser(
            (vmk_VA)&localBuffer[curByte % GPIO_CHARDEV_BUFFER_SIZE],
            (vmk_VA)&buffer[curByte % GPIO_CHARDEV_BUFFER_SIZE],
            sizeof(char)
         );

         if (status != VMK_OK) {
            break;
         }
         ++doneBytes;
      }
      vmk_SpinlockLock(fileData->lock);
      status = gpio_CharDevCBs->write(localBuffer, doneBytes, ppos, &doneBytes);
      vmk_SpinlockUnlock(fileData->lock);
   }
   else { /* Read */
      status = gpio_CharDevCBs->read(localBuffer, nbytes, ppos, &doneBytes);
      for (curByte = 0; curByte < doneBytes; ++curByte) {
         status = vmk_CopyToUser(
            (vmk_VA)&buffer[curByte % GPIO_CHARDEV_BUFFER_SIZE],
            (vmk_VA)&localBuffer[curByte % GPIO_CHARDEV_BUFFER_SIZE],
            sizeof(char)
         );
         if (status != VMK_OK) {
            break;
         }
      }
      doneBytes = curByte + 1;
   }

   vmk_HeapFree(gpio_Driver->heapID, localBuffer);

   *ndone = doneBytes;

   if (status != VMK_OK) {
      vmk_Warning(gpio_Driver->logger,
                  "I/O failed to file %p: %s",
                  vmk_StatusToString(status));
      goto io_failed;
   }

   return VMK_OK;

io_failed:
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
      vmk_Warning(gpio_Driver->logger, "file data null");
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
         vmk_Warning(gpio_Driver->logger,
                     "failed to create poll timer: %s",
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
      vmk_Warning(gpio_Driver->logger, "file data null");
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
      vmk_Log(gpio_Driver->logger,
              "destroying file %p",
              fileData);
   }
#endif /* GPIO_DEBUG */

   vmk_SpinlockDestroy(fileData->lock);
   vmk_HeapFree(gpio_Driver->heapID, fileData);
}