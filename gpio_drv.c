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
      vmk_LogMessage("%s: %s: reading from GPIO MMIO offset 0x%x",
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

   if (offset > GPIO_MMIO_MAX_OFFSET) {
      status = VMK_FAILURE;
      vmk_WarningMessage("%s: %s: attempt to read past GPIO MMIO max offset"
                         " 0x%x; offset attempted: 0x%x",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         GPIO_MMIO_MAX_OFFSET,
                         offset);
      goto read_past_mmio_max;
   }

   status = gpio_mmioDirectRead(offset, &value);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to read from GPIO MMIO region: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      *nread = 0;
      goto mmio_read_failed;
   }

   status = vmk_Sprintf(buffer, "%d", value);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to convert value to string: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__);
      *nread = 0;
      goto convert_failed;
   }

   /* Each read is the size of a register */
   *nread = GPIO_REG_SIZE;

convert_failed:
mmio_read_failed:
read_past_mmio_max:
   return status;
}

/*
 ***********************************************************************
 * gpio_mmioWrite --
 * 
 *    Callback used by char dev driver when the /dev/vmgfx32 file is
 *    written to.
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

   if (offset > GPIO_MMIO_MAX_OFFSET) {
      status = VMK_FAILURE;
      vmk_WarningMessage("%s: %s: attempt to write past GPIO MMIO max offset"
                         " 0x%x; offset attempted: 0x%x",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         GPIO_MMIO_MAX_OFFSET,
                         offset);
      goto write_past_mmio_max;
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
write_past_mmio_max:
   return status;
}