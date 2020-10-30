/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_drv.c --
 *
 *    Implementation of gpio hardware interface.
 * 
 * TODO:
 *    - fix up interrupts to work as described here:
 *       https://www.raspberrypi.org/documentation/hardware/raspberrypi/gpio/README.md
 */

#include "gpio_drv.h"

/***********************************************************************/

static gpio_Device_t *gpio_Device;

/*
 ***********************************************************************
 * gpio_drvInit --
 * 
 *    Initializes the GPIO driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_drvInit(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;

   gpio_Device = adapter;

   return status;
}

/*
 ***********************************************************************
 * gpio_mmioPoll --
 * 
 *    Polls a GPIO register and returns once the maked bits have changed.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_mmioPoll(vmk_uint32 offset, // IN
              vmk_uint32 mask,   // IN
              vmk_uint32 *data)  // OUT
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_uint32 prevData, curData;
   vmk_uint32 prevMasked, curMasked;
   vmk_TimerCycles monoCounterFreq = vmk_GetMonotonicCounterFrequency();
   unsigned long startTime, curTime;

#ifdef GPIO_DEBUG
   {
      vmk_LogMessage("%s: %s: polling GPIO MMIO offset 0x%x with mask 0x%x",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     offset,
                     mask);
   }
#endif /* GPIO_DEBUG */

   status = vmk_MappedResourceRead32(&gpio_Device->mmioMappedAddr,
                                     offset,
                                     &prevData);
   if (status != VMK_OK) {
      goto mmio_read_failed;
   }

   /* Mask on only the bits we want to poll */
   prevMasked = prevData & mask;

   /*
    * Loop until the read value changes or we hit the timeout
    */
   startTime = vmk_TimerMonotonicCounter;
   do {
      status = vmk_MappedResourceRead32(&gpio_Device->mmioMappedAddr,
                                        offset,
                                        &curData);
      if (status != VMK_OK) {
         goto mmio_read_failed;
      }

      curMasked = curData & mask;
      
      curTime = vmk_TimerMonotonicCounter;
      if ((curTime - startTime) / monoCounterFreq >= GPIO_POLL_TIMEOUT_SEC) {
         goto poll_timed_out;
      }
   } while (curMasked == prevMasked);

   *data = curData;

   return VMK_OK;

poll_timed_out:
   return VMK_TIMEOUT;

mmio_read_failed:
   vmk_WarningMessage("%s: %s: read from GPIO MMIO offset 0x%x failed: %s",
                      GPIO_DRIVER_NAME,
                      __FUNCTION__,
                      offset,
                      vmk_StatusToString(status));
   return status;
}

/*
 ***********************************************************************
 * gpio_mmioDirectRead --
 * 
 *    Read directly from the GPIO MMIO mapped area.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_INLINE
VMK_ReturnStatus
gpio_mmioDirectRead(vmk_uint32 offset,
                    vmk_uint32 *value)
{
   VMK_ReturnStatus status = VMK_OK;

#ifdef GPIO_DEBUG
   {
      vmk_LogMessage("%s: %s: reading value from GPIO MMIO offset 0x%x",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     offset);
   }
#endif /* GPIO_DEBUG */

   status = vmk_MappedResourceRead32(&gpio_Device->mmioMappedAddr,
                                     offset,
                                     value);

   return status;
}

/*
 ***********************************************************************
 * gpio_mmioDirectWrite --
 * 
 *    Write directly to the GPIO MMIO mapped area.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_INLINE
VMK_ReturnStatus
gpio_mmioDirectWrite(vmk_uint32 offset,
                     vmk_uint32 value)
{
   VMK_ReturnStatus status = VMK_OK;

#ifdef GPIO_DEBUG
   {
      vmk_LogMessage("%s: %s: writing value 0x%x to GPIO MMIO offset 0x%x",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     value,
                     offset);
   }
#endif /* GPIO_DEBUG */

   status = vmk_MappedResourceWrite32(&gpio_Device->mmioMappedAddr,
                                      offset,
                                      value);

   return status;
}

/*
 ***********************************************************************
 * gpio_mmioOpenCB --
 * 
 *    Callback used by char dev driver when the /dev/vmgfx32 file is
 *    opened.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_mmioOpenCB(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_mmioCloseCB --
 * 
 *    Callback used by char dev driver when the /dev/vmgfx32 file is
 *    closed.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_mmioCloseCB(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_mmioIoctlCB --
 * 
 *    Callback used by char dev driver for I/O control. I/O control is
 *    used to allow to perform alternate reads/writes that aren't
 *    supported by the corresponding syscalls. For example, for polling
 *    a pin value until it changes.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_mmioIoctlCB(unsigned int cmd,           // IN
                 gpio_IoctlCookie_t *data)   // IN/OUT
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_IoctlData_t ioctlData;
   vmk_uint32 outData;

   vmk_Memcpy(&ioctlData, data, sizeof(ioctlData));

   /* Sanity checks */
   if (ioctlData.offset % GPIO_REG_SIZE != 0
       || ioctlData.offset > GPIO_MMIO_SIZE
       || cmd <= GPIO_IOCTL_INVALID
       || cmd > GPIO_IOCTL_MAX) {
      status = VMK_BAD_PARAM;
      vmk_WarningMessage("%s: %s: invalid ioctl: cmd %d data %p"
                         " (0x%x, 0x%x)",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         cmd,
                         *data,
                         ioctlData.offset,
                         ioctlData.mask);
      goto ioctl_invalid;
   }

   /* Run ioctl command */
   switch (cmd) {
      case GPIO_IOCTL_POLL:
         status = gpio_mmioPoll(ioctlData.offset, ioctlData.mask, &outData);
         break;
   }
   if (status != VMK_OK) {
      goto ioctl_cmd_failed;
   }

   /* Prep data for output */
   ioctlData.data = outData;
   vmk_Memcpy(data, &ioctlData, sizeof(ioctlData));

   return VMK_OK;

ioctl_cmd_failed:
ioctl_invalid:
   return status;
}

/*
 ***********************************************************************
 * gpio_mmioReadCB --
 * 
 *    Callback used by char dev driver when the /dev/vmgfx32 file is
 *    read.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_mmioReadCB(char *buffer,
                vmk_ByteCount nbytes,
                vmk_loff_t *ppos,
                vmk_ByteCountSigned *nread)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_loff_t offset = *ppos;
   vmk_uint32 value = 0;

   /*
    * Offset sanity checks
    */
   if (offset % GPIO_REG_SIZE != 0
       || offset > GPIO_MMIO_SIZE - GPIO_REG_SIZE) {
      status = VMK_FAILURE;
      vmk_WarningMessage("%s: %s: attempted read from invalid MMIO offset 0x%x",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         offset);
      goto invalid_mmio_offset;
   }

   /*
    * Read from GPIO MMIO space
    */
   status = gpio_mmioDirectRead(offset, &value);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to read from GPIO MMIO region: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      *nread = 0;
      goto mmio_read_failed;
   }

   /* Write register value to UW buffer */
   *(vmk_uint32 *)buffer = value;

#ifdef GPIO_DEBUG
   {
      vmk_LogMessage("%s: %s: read 0x%x from offset 0x%x",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     value,
                     offset);
   }
#endif /* GPIO_DEBUG */

   /* Each read is the size of a register */
   *nread = GPIO_REG_SIZE;

   return VMK_OK;

mmio_read_failed:
invalid_mmio_offset:
   return status;
}

/*
 ***********************************************************************
 * gpio_mmioWrite --
 * 
 *    Callback used by char dev driver when the /dev/vmgfx32 file is
 *    written to. We can write without locking, since locking is handled
 *    by the char dev driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_mmioWriteCB(char *buffer,
                 vmk_ByteCount nbytes,
                 vmk_loff_t *ppos,
                 vmk_ByteCountSigned *nwritten)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_loff_t offset = *ppos;
   long value;

   /*
    * Offset sanity checks
    */
   if (offset % GPIO_REG_SIZE != 0
       || offset + nbytes > GPIO_MMIO_SIZE) {
      status = VMK_FAILURE;
      vmk_WarningMessage("%s: %s: attempted write to invalid MMIO offset 0x%x",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         offset);
      goto invalid_mmio_offset;
   }

   value = *(vmk_uint32*)buffer;
   status = gpio_mmioDirectWrite(offset, (vmk_uint32)value);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to write to GPIO MMIO region: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      *nwritten = 0;
      goto mmio_write_failed;
   }

   /* Each write is the size of a register */
   *nwritten = GPIO_REG_SIZE;

mmio_write_failed:
invalid_mmio_offset:
   return status;
}