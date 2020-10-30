/******************************************************************************\
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * thx_charDev.c --
 * 
 *    Generic character device driver implementation.
 */

#include "thx_charDev.h"

/***********************************************************************/

static vmk_ModuleID thx_moduleID;
static vmk_HeapID thx_heapID;
static vmk_LogComponent thx_logger;

static vmk_CharDevOps thx_charDevFileOps = {
   .open = thx_charDevOpen,
   .close = thx_charDevClose,
   .ioctl = thx_charDevIoctl,
   .poll = thx_charDevPoll,
   .read = thx_charDevRead,
   .write = thx_charDevWrite,
};

static vmk_CharDevRegOps thx_CharDevOps = {
   .associate = thx_charDevAssoc,
   .disassociate = thx_charDevDisassoc,
   .fileOps = &thx_charDevFileOps,
};

/*
 * Dev ops for the new vmk_Device associated with the char dev
 */
static vmk_DeviceOps thx_CharVmkDevOps = {
   .removeDevice = thx_charVmkDevRemove,
};

/*
 * Call backs that glue the char dev to the GPIO driver
 */
static thx_CharDevCallbacks_t *thx_CharDevCBs;

/***********************************************************************/

static vmk_TimerQueue thx_charDevTimerQueue;

/*
 ***********************************************************************
 * thx_charDevInit --
 * 
 *    Initialize the character device driver. Onyl called once, as it sets
 *    a number of global variables.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
thx_charDevInit(vmk_ModuleID moduleID,
                vmk_HeapID heapID,
                vmk_LogComponent logger)
{
   VMK_ReturnStatus status = VMK_OK;

   thx_moduleID = moduleID;
   thx_heapID = heapID;
   thx_logger = logger;

   return status;
}

/*
 ***********************************************************************
 * thx_charDevRegister --
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
thx_charDevRegister(thx_CharDevProps_t *props)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_Device newDevice;
   vmk_DeviceID devID;
   vmk_AddrCookie registeringDriverData;
   vmk_AddrCookie registrationData;
   vmk_DeviceProps deviceProps;
   thx_CharDev_t *charDev;
   vmk_AddrCookie charDevPriv;

   devID.busType = props->logicalBusType;
   status = vmk_LogicalCreateBusAddress(props->driverHandle,
                                        props->parentDevice,
                                        props->logicalPort,
                                        &devID.busAddress,
                                        &devID.busAddressLen);
   if (status != VMK_OK) {
      goto logical_bus_failed;
   }

   charDev = props->charDev;

   /*
    * As per vmkapi_char.h it can only be graphics or a test device
    */
   devID.busIdentifier = VMK_CHARDEV_IDENTIFIER_GRAPHICS;
   devID.busIdentifierLen = vmk_Strnlen(devID.busIdentifier, VMK_MISC_NAME_MAX);

   charDevPriv.ptr = props->charDev;

   /*
    * Set up char dev registration
    */

   charDev->regData.moduleID = thx_moduleID;
   charDev->regData.deviceOps = &thx_CharDevOps;
   charDev->regData.devicePrivate = charDevPriv;

   registrationData.ptr = &charDev->regData;
   registeringDriverData.ptr = props->charDev;

   deviceProps.registeringDriver = props->driverHandle;
   deviceProps.deviceID = &devID;
   deviceProps.deviceOps = &thx_CharVmkDevOps;
   deviceProps.registeringDriverData = registeringDriverData;
   deviceProps.registrationData = registrationData;

   status = vmk_DeviceRegister(&deviceProps, props->parentDevice, &newDevice);
   if (status != VMK_OK) {
      vmk_Warning(thx_logger,
                  "failed to register device: %s",
                  vmk_StatusToString(status));
      goto register_device_failed;
   }

   status = vmk_LogicalFreeBusAddress(props->driverHandle,
                                      devID.busAddress);
   if (status != VMK_OK) {
      vmk_Warning(thx_logger,
                  "failed to free logical bus: %s",
                  vmk_StatusToString(status));
      goto free_bus_failed;
   }

   /*
    * Set CBs
    */
   thx_CharDevCBs = props->callbacks;

   return VMK_OK;

free_bus_failed:
   vmk_LogicalFreeBusAddress(props->driverHandle,
                             devID.busAddress);
register_device_failed:
logical_bus_failed:
   return status;
}

/*
 ***********************************************************************
 * thx_charVmkDevRemove --
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
thx_charVmkDevRemove(vmk_Device logicalDev)
{
   VMK_ReturnStatus status = VMK_OK;

   status = vmk_DeviceUnregister(logicalDev);
   if (status != VMK_OK)
   {
      vmk_Warning(thx_logger,
                  "failed to unregister device: %s",
                  vmk_StatusToString(status));
   }

   return status;
}

/*
 ***********************************************************************
 * thx_charDevAssoc --
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
thx_charDevAssoc(vmk_AddrCookie charDevPriv,
                 vmk_CharDevHandle charDevHandle)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_Name charDevAlias;

   status = vmk_CharDeviceGetAlias(charDevHandle, &charDevAlias);
   if (status != VMK_OK) {
      vmk_Warning(thx_logger,
                  "failed to obtain logical device alias: %s",
                  vmk_StatusToString(status));
      goto get_alias_failed;
   }

#ifdef THX_DEBUG
   {
      vmk_Log(thx_logger,
              "obtained logical device alias %s",
              charDevAlias.string);
   }
#endif /* THX_DEBUG */

   return VMK_OK;

get_alias_failed:
   return status;
}

/*
 ***********************************************************************
 * thx_charDevDisassoc --
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
thx_charDevDisassoc(vmk_AddrCookie charDevPriv)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * thx_charDevOpen --
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
thx_charDevOpen(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;
   thx_CharFileData_t *fileData;
   vmk_SpinlockCreateProps lockProps;

   /*
    * Init private file data
    */

   fileData = vmk_HeapAlloc(thx_heapID, sizeof(*fileData));
   if (fileData == NULL) {
      status = VMK_NO_MEMORY;
      vmk_Warning(thx_logger,
                  "failed to create file private data: %s",
                  vmk_StatusToString(status));
      goto file_priv_alloc_failed;
   }
   vmk_Memset(fileData, 0, sizeof(*fileData));

   /*
    * Init lock
    */

   lockProps.moduleID = thx_moduleID;
   lockProps.heapID = thx_heapID;
   status = vmk_NameInitialize(&lockProps.name, THX_DRIVER_NAME);
   if (status != VMK_OK) {
      vmk_Warning(thx_logger,
                  "failed to init lock name: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

   lockProps.type = VMK_SPINLOCK;
   lockProps.domain = VMK_LOCKDOMAIN_INVALID;
   lockProps.rank = VMK_SPINLOCK_UNRANKED;
   status = vmk_SpinlockCreate(&lockProps, &fileData->lock);
   if (status != VMK_OK) {
      vmk_Warning(thx_logger,
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
   fileData->timeoutUS = THX_CHARDEV_POLL_TIMEOUT_US;
   attr->clientInstanceData.ptr = fileData;

#ifdef THX_DEBUG
   {
      vmk_Log(thx_logger,
              "opened file; priv %p lock %p",
              fileData,
              fileData->lock);
   }
#endif /* THX_DEBUG */

   /* Call CB */
   status = thx_CharDevCBs->open(attr);

   return status;

lock_init_failed:
   vmk_HeapFree(thx_heapID, fileData);

file_priv_alloc_failed:
   return status;
}

/*
 ***********************************************************************
 * thx_charDevClose --
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
thx_charDevClose(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;
   thx_CharFileData_t *fileData = attr->clientInstanceData.ptr;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(thx_logger, "file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   fileData->deathPending = VMK_TRUE;

   if (fileData->timerPending == VMK_FALSE) {
      vmk_SpinlockUnlock(fileData->lock);
      thx_charDevFileDestroy(fileData);
   }

   /* Call CB */
   status = thx_CharDevCBs->close(attr);

file_data_null:
   return status;
}

/*
 ***********************************************************************
 * thx_charDevIoctl --
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
thx_charDevIoctl(vmk_CharDevFdAttr *attr,
                  unsigned int cmd,
                  vmk_uintptr_t userData,
                  vmk_IoctlCallerSize callerSize,
                  vmk_int32 *result)
{
   VMK_ReturnStatus status = VMK_OK;
   thx_IoctlCookie_t ioctlData;
   thx_CharFileData_t *fileData = attr->clientInstanceData.ptr;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(thx_logger, "file data null");
      goto file_data_null;
   }

   /* Copy ioctl data from UW */
   status = vmk_CopyFromUser((vmk_VA)&ioctlData,
                             (vmk_VA)userData,
                             sizeof(ioctlData));
   if (status != VMK_OK) {
      vmk_Warning(thx_logger,
                  "unable to copy ioctl data from UW ptr %p: %s",
                  userData,
                  vmk_StatusToString(status));
      goto ioctl_uw2vmk_failed;
   }

#ifdef THX_DEBUG
   {
      vmk_Log(thx_logger,
              "executing ioctl cmd %d with data %p",
              cmd,
              userData);
   }
#endif

   /* Call CB */
   vmk_SpinlockLock(fileData->lock);
   status = thx_CharDevCBs->ioctl(cmd, &ioctlData);
   vmk_SpinlockUnlock(fileData->lock);
   if (status != VMK_OK) {
      vmk_Warning(thx_logger,
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
      vmk_Warning(thx_logger,
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
 * thx_charDevRead --
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
thx_charDevRead(vmk_CharDevFdAttr *attr,
                 char *buffer,
                 vmk_ByteCount nbytes,
                 vmk_loff_t *ppos,
                 vmk_ByteCountSigned *nread)
{
   return thx_charDevIO(attr, buffer, nbytes, ppos, nread, VMK_FALSE);
}

/*
 ***********************************************************************
 * thx_charDevWrite --
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
thx_charDevWrite(vmk_CharDevFdAttr *attr,
                  char *buffer,
                  vmk_ByteCount nbytes,
                  vmk_loff_t *ppos,
                  vmk_ByteCountSigned *nwritten)
{
   return thx_charDevIO(attr, buffer, nbytes, ppos, nwritten, VMK_TRUE);
}

/*
 ***********************************************************************
 * thx_charDevIO --
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
thx_charDevIO(vmk_CharDevFdAttr *attr,
               char *buffer,
               vmk_ByteCount nbytes,
               vmk_loff_t *ppos,
               vmk_ByteCountSigned *ndone,
               vmk_Bool isWrite)
{
   VMK_ReturnStatus status = VMK_OK;
   thx_CharFileData_t *fileData;
   vmk_ByteCount curByte;
   vmk_ByteCountSigned doneBytes = 0;
   vmk_loff_t offset;
   char *localBuffer;

   if (attr == NULL || ppos == NULL || ndone == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(thx_logger,
                  "invalid write command received;"
                  " attr %p ppos %p ndone %p",
                  attr, ppos, ndone);
      goto invalid_params;
   }

#ifdef THX_DEBUG
   {
      vmk_Log(thx_logger,
              "nbytes %lu offset %ld buffer %p",
              nbytes,
              *ppos,
              buffer);
   }
#endif /* THX_DEBUG */

   localBuffer = vmk_HeapAlloc(thx_heapID, THX_CHARDEV_BUFFER_SIZE);
   if (localBuffer == NULL) {
      status = VMK_NO_MEMORY;
      vmk_Warning(thx_logger,
                  "unable to allocate local buffer: %s",
                  vmk_StatusToString(status));
      goto localBuffer_alloc_failed;
   }

   offset = *ppos;
   fileData = (thx_CharFileData_t *)attr->clientInstanceData.ptr;
   
   /*
    * Perform I/O and copy data to/from user space
    */
   if (isWrite) { /* Write */
      for (curByte = 0; curByte < nbytes; ++curByte) {
         status = vmk_CopyFromUser(
            (vmk_VA)&localBuffer[curByte % THX_CHARDEV_BUFFER_SIZE],
            (vmk_VA)&buffer[curByte % THX_CHARDEV_BUFFER_SIZE],
            sizeof(char)
         );

         if (status != VMK_OK) {
            break;
         }
         ++doneBytes;
      }
      vmk_SpinlockLock(fileData->lock);
      status = thx_CharDevCBs->write(localBuffer, doneBytes, ppos, &doneBytes);
      vmk_SpinlockUnlock(fileData->lock);
   }
   else { /* Read */
      status = thx_CharDevCBs->read(localBuffer, nbytes, ppos, &doneBytes);
      for (curByte = 0; curByte < doneBytes; ++curByte) {
         status = vmk_CopyToUser(
            (vmk_VA)&buffer[curByte % THX_CHARDEV_BUFFER_SIZE],
            (vmk_VA)&localBuffer[curByte % THX_CHARDEV_BUFFER_SIZE],
            sizeof(char)
         );
         if (status != VMK_OK) {
            break;
         }
      }
      doneBytes = curByte + 1;
   }

   vmk_HeapFree(thx_heapID, localBuffer);

   *ndone = doneBytes;

   if (status != VMK_OK) {
      vmk_Warning(thx_logger,
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
 * thx_charDevPoll --
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
thx_charDevPoll(vmk_CharDevFdAttr *attr,
                 vmk_PollContext pollCtx,
                 vmk_uint32 *pollMask)
{
   VMK_ReturnStatus status = VMK_OK;
   thx_CharFileData_t *fileData = attr->clientInstanceData.ptr;
   vmk_TimerCookie tmrCookie;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(thx_logger, "file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   if (fileData->pollMask == VMK_FALSE
       && fileData->timerPending == VMK_FALSE
       && fileData->deathPending == VMK_FALSE) {
          tmrCookie.ptr = fileData;
          status = vmk_TimerSchedule(thx_charDevTimerQueue,
                                     thx_charDevTimerCB,
                                     tmrCookie,
                                     THX_CHARDEV_POLL_TIMEOUT_US,
                                     VMK_TIMER_DEFAULT_TOLERANCE,
                                     VMK_TIMER_ATTR_NONE,
                                     VMK_LOCKDOMAIN_INVALID,
                                     VMK_SPINLOCK_UNRANKED,
                                     &fileData->timer);
      if (status == VMK_OK) {
         fileData->timerPending = VMK_TRUE;
      }
      else {
         vmk_Warning(thx_logger,
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
 * thx_charDevTimerCB --
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
thx_charDevTimerCB(vmk_TimerCookie data)
{
   thx_CharFileData_t *fileData = (thx_CharFileData_t *)data.ptr;

   if (fileData == NULL) {
      vmk_Warning(thx_logger, "file data null");
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
      thx_charDevFileDestroy(fileData);
      goto death_pending;
   }

   vmk_SpinlockUnlock(fileData->lock);

death_pending:
file_data_null:
   return;
}

/*
 ***********************************************************************
 * thx_charDevFileDestroy --
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
thx_charDevFileDestroy(thx_CharFileData_t *fileData)
{
#ifdef THX_DEBUG
   {
      vmk_Log(thx_logger,
              "destroying file %p",
              fileData);
   }
#endif /* THX_DEBUG */

   vmk_SpinlockDestroy(fileData->lock);
   vmk_HeapFree(thx_heapID, fileData);
}