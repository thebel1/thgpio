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

static vmk_uint32 gpioPinToSelReg[] = {
   GPFSEL0, GPFSEL1, GPFSEL2, GPFSEL3, GPFSEL4, GPFSEL5
};

static vmk_uint32 gpioPinToPudReg[] = {
   GPIO_PUP_PDN_CNTRL_REG0,
   GPIO_PUP_PDN_CNTRL_REG1,
   GPIO_PUP_PDN_CNTRL_REG2,
   GPIO_PUP_PDN_CNTRL_REG3,
};

/*
 ***********************************************************************
 * gpio_readReg --
 * 
 *    Reads a 4-byte word from a GPIO register.
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
gpio_readReg(gpio_Device_t *adapter,   // IN
             vmk_uint32 reg,           // IN
             vmk_uint32 *ptr)          // OUT
{
   VMK_ReturnStatus status = VMK_OK;

   status = vmk_MappedResourceRead32(&adapter->mmioMappedAddr, reg, ptr);

   return status;
}

/*
 ***********************************************************************
 * gpio_writedReg --
 * 
 *    Writes a 4-byte word into a GPIO register.
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
gpio_writeReg(gpio_Device_t *adapter,  // IN
              vmk_uint32 reg,          // IN
              vmk_uint32 val)          // IN
{
   VMK_ReturnStatus status = VMK_OK;

   status = vmk_MappedResourceWrite32(&adapter->mmioMappedAddr, reg, val);

   return status;
}

/*
 ***********************************************************************
 * gpio_funcSelPin --
 * 
 *    Selects the function of a GPIO pin.
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
gpio_funcSelPin(gpio_Device_t *adapter,   // IN
                vmk_uint32 pin,           // IN
                vmk_uint32 func)          // IN
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_uint32 origVal, newVal, reg, shift;

   /* RPi 4B only supports 0-57 */
   if (pin < 58) {
      reg = gpioPinToSelReg[pin / 10];
   }
   else {
      status = VMK_NOT_IMPLEMENTED;
      goto invalid_pin_number;
   }

   /*
    * (1) Get the original value of the GPFSEL reg
    * (2) Overwrite only the bits belonging to the pin with 1's
    * (3) Flip the bits belonging to the pin
    * (4) Write the desired bits using bitwise OR
    */
   shift = ((pin % 10) * 3);
   status = gpio_readReg(adapter, reg, &origVal);
   newVal = origVal | (0b111 << shift);
   newVal ^= (0b111 << shift);
   newVal |= (func << shift);

   status = vmk_MappedResourceWrite32(&adapter->mmioMappedAddr,
                                      reg,
                                      newVal);

invalid_pin_number:
   return status;
}

/*
 ***********************************************************************
 * gpio_setPin --
 * 
 *    Set the value of a GPIO pin to 1.
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
gpio_setPin(gpio_Device_t *adapter, // IN
            vmk_uint32 pin)         // IN
{
   VMK_ReturnStatus status = VMK_OK;

   if (pin < 32) {
      status = vmk_MappedResourceWrite32(&adapter->mmioMappedAddr,
                                         GPSET0,
                                         1 << pin);
   }
   else {
      status = vmk_MappedResourceWrite32(&adapter->mmioMappedAddr,
                                         GPSET1,
                                         1 << (pin & GPIO_PIN_MASK));
   }

   return status;
}

/*
 ***********************************************************************
 * gpio_clrPin --
 * 
 *    Clear the value of a GPIO pin.
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
gpio_clrPin(gpio_Device_t *adapter, // IN
            vmk_uint32 pin)         // IN
{
   VMK_ReturnStatus status = VMK_OK;

   if (pin < 32) {
      status = vmk_MappedResourceWrite32(&adapter->mmioMappedAddr,
                                         GPCLR0,
                                         1 << pin);
   }
   else {
      status = vmk_MappedResourceWrite32(&adapter->mmioMappedAddr,
                                         GPCLR1,
                                         1 << (pin & GPIO_PIN_MASK));
   }

   return status;
}

/*
 ***********************************************************************
 * gpio_getPin --
 * 
 *    Outputs the value of a GPIO pin.
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
gpio_levPin(gpio_Device_t *adapter, // IN
            vmk_uint32 pin,         // IN
            vmk_uint32 *ptr)        // OUT
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_uint32 tmp;

   if (pin < 32) {
      status = vmk_MappedResourceRead32(&adapter->mmioMappedAddr,
                                        GPLEV0,
                                        &tmp);
   }
   else {
      status = vmk_MappedResourceRead32(&adapter->mmioMappedAddr,
                                        GPLEV1,
                                        &tmp);
   }

   /* Extract the value of the nth pin from the bitfield */
   *ptr = (tmp & (1 << (pin & GPIO_PIN_MASK))) != 0;

   return status;
}

/*
 ***********************************************************************
 * gpio_setPull --
 * 
 *    Sets the pull-up/pull-down state for a specific gpio pin. This is
 *    only valid for BCM2711.
 * 
 *    Reference:
 *       - https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2711/rpi_DATA_2711_1p0.pdf
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
gpio_setPull(gpio_Device_t *adapter,   // IN
             vmk_uint32 pin,           // IN
             vmk_uint8 pud)            // IN
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_uint32 reg, shift, oldPud, newPud;

   shift = (pin % 16) * 2;
   reg = gpioPinToPudReg[pin / 16];
   gpio_readReg(adapter, reg, &oldPud);

   /*
    * (1) Set both bits belonging to this pin to 1
    * (2) Flip the same two bits
    * (3) Write the desired value
    */
   newPud = oldPud | (0b11 << shift);
   newPud ^= (0b11 << shift);
   newPud |= (pud << shift);

   gpio_writeReg(adapter, reg, newPud);

   return status;
}