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

#include "gpio.h"
#include "gpio_drv.h"

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
   vmk_uint32 origVal, newVal, reg, offset;

   if (pin < 10) {
      reg = GPFSEL0;
   }
   else if (pin < 20) {
      reg = GPFSEL1;
   }
   else if (pin < 30) {
      reg = GPFSEL2;
   }
   else if (pin < 40) {
      reg = GPFSEL3;
   }
   else if (pin < 50) {
      reg = GPFSEL4;
   }
   else if (pin < 58) { /* RPi 4B only supports 0-57 */
      reg = GPFSEL5;
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
   offset = ((pin % 10) * 3);
   status = gpio_readReg(adapter, reg, &origVal);
   newVal = origVal | (0b111 << offset);
   newVal ^= (0b111 << offset);
   newVal |= (func << offset);

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
 * gpio_changeIntrState --
 * 
 *    Changes the interrupt state. UNUSED.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 * 
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_changeIntrState(gpio_Device_t *adapter, // IN
                     vmk_uint64 curState,    // IN
                     vmk_uint64 newState)    // IN
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_uint64 actualState;

   actualState = vmk_AtomicReadIfEqualWrite64(&adapter->intState,
                                              curState,
                                              newState);
   if (actualState != curState) {
      status = VMK_BUSY;
   }

   return status;
}

/*
 ***********************************************************************
 * gpio_setupIntr --
 * 
 *    Sets up interrupts. UNUSED.
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_setupIntr(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_IntrProps intrProps;
   int numIntrsAllocated;

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_NONE,
                                 GPIO_INT_BUSY_UP);
   if (status != VMK_OK) {
      goto change_intr_failed;
   }

   /*
    * Set up interrupts for this device
    */
   status = vmk_ACPIAllocIntrCookie(vmk_ModuleCurrentID,
                                    adapter->acpiDevice,
                                    1, 1,
                                    &adapter->intrCookie,
                                    &numIntrsAllocated);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to allocate interrupts for acpi"
                         " device %p: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto alloc_intr_cookie_failed;
   }

   /*
    * Register interrupt. Assume only one interrupt allocated.
    */
   vmk_NameCopy(&intrProps.deviceName, &gpio_Driver.driverName);
   intrProps.device = adapter->vmkDevice;
   intrProps.acknowledgeInterrupt = gpio_ackIntr;
   intrProps.handler = gpio_intrHandler;
   intrProps.handlerData = adapter;
   intrProps.attrs = VMK_INTR_ATTRS_ENTROPY_SOURCE;
   status = vmk_IntrRegister(vmk_ModuleCurrentID,
                             adapter->intrCookie,
                             &intrProps);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to register interrupt: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto reg_intr_failed;
   }

   vmk_LogMessage("%s: %s: got cookie 0x%x for acpi device %p",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  adapter->intrCookie,
                  &intrProps);

   /*
    * Enable interrupt
    */
   status = vmk_IntrEnable(adapter->intrCookie);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to enable interrupt: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto enable_intr_failed;
   }

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_BUSY_UP,
                                 GPIO_INT_READY);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to revert interrupt state: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
   }

   return status;

enable_intr_failed:
reg_intr_failed:
alloc_intr_cookie_failed:
   gpio_changeIntrState(adapter,
                        GPIO_INT_BUSY_UP,
                        GPIO_INT_NONE);

change_intr_failed:
   return status;
}

/*
 ***********************************************************************
 * gpio_ackIntr --
 * 
 *    Acknowledges an interrupt. UNUSED.
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_ackIntr(void *clientData,            // IN
             vmk_IntrCookie intrCookie)   // IN
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_SIGNAL,
                                 GPIO_INT_HANDLE_START);
   if (status != VMK_OK) {
      return VMK_NOT_THIS_DEVICE;
   }

   return status;
}

/*
 ***********************************************************************
 * gpio_intrHandler --
 * 
 *    Handles an interrupt. UNUSED.
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 * 
 * Results:
 *    None.
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
void
gpio_intrHandler(void *clientData,            // IN
                 vmk_IntrCookie intrCookie)   // IN
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;
   vmk_WorldID worldID;

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_HANDLE_START,
                                 GPIO_INT_HANDLE_END);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to change interrupt state: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto change_intr_failed;
   }

   vmk_AtomicInc64(&adapter->intCount);

   worldID = adapter->wakeWorld;

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_HANDLE_END,
                                 GPIO_INT_READY);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to revert interrupt state: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto revert_intr_failed;
   }

   vmk_WorldForceWakeup(worldID);

revert_intr_failed:
change_intr_failed:
   return;
}

/*
 ***********************************************************************
 * gpio_waitIntr --
 * 
 *    Waits for an interrupt to occur. UNUSED.
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_waitIntr(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_READY,
                                 GPIO_INT_SIGNAL);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to change interrupt state: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto change_intr_failed;
   }
   
   adapter->wakeWorld = vmk_WorldGetID();

   /*
    * Trigger interrupt
    */
   status = vmk_MappedResourceWrite32(&adapter->mmioMappedAddr, 0, 0);
   if (status == VMK_OK) {
      vmk_LogMessage("%s: %s: write to mmio mem at %p successful",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     adapter->mmioBase,
                     vmk_StatusToString(status));
   }
   else {
      vmk_WarningMessage("%s: %s: write failed to mmio mem at %p: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         adapter->mmioBase,
                         vmk_StatusToString(status));
      goto mmio_write_failed;
   }
   
   /*
    * So bottom line is, nothing happens here and no interrupt is triggered.
    * This is because the interrupt state remains at GPIO_INT_SIGNAL rather than
    * transitioning back to GPIO_INT_READY. In other words, the hardware doesn't
    * respond to the interrupt trigger.
    */

   status = vmk_WorldWait(VMK_EVENT_NONE,
                          VMK_LOCK_INVALID,
                          10 * VMK_MSEC_PER_SEC, // Default 10
                          "wait for interrupt");
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unexpected interrupt state;"
                         " cur 0x%x exp 0x%x: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_AtomicRead64(&adapter->intState),
                         GPIO_INT_READY,
                         vmk_StatusToString(status));
      goto unexpected_intr_state;
   }

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_SIGNAL,
                                 GPIO_INT_READY);
   vmk_WarningMessage("%s: %s: unable to revert interrupt state: %s",
                      GPIO_DRIVER_NAME,
                      __FUNCTION__,
                      vmk_StatusToString(status));

   return status;

unexpected_intr_state:
mmio_write_failed:
change_intr_failed:
   return status;
}

/*
 ***********************************************************************
 * gpio_destroyIntr --
 * 
 *    Destroys an interrupt. UNUSED.
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 * 
 * Results:
 *    None.
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
void
gpio_destroyIntr(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status;

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_READY,
                                 GPIO_INT_BUSY_DOWN);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to change interrupt state: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto change_intr_failed;
   }

   status = vmk_IntrDisable(adapter->intrCookie);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to disable interrupt: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto disable_intr_failed;
   }

   status = vmk_IntrUnregister(vmk_ModuleCurrentID,
                               adapter->intrCookie,
                               adapter);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to unregister interrupt: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto unreg_intr_failed;
   }

   status = vmk_ACPIFreeIntrCookie(vmk_ModuleCurrentID, adapter->acpiDevice);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to free interrupt cookied: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto free_cookie_failed;
   }

   status = gpio_changeIntrState(adapter,
                                 GPIO_INT_BUSY_DOWN,
                                 GPIO_INT_NONE);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to revert interrupt state: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
   }

free_cookie_failed:
unreg_intr_failed:
disable_intr_failed:
change_intr_failed:
   return;
}