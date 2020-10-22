/*
 * gpio_device.c --
 *
 *    Implementation of gpio interface
 *
 *    Some ideas taken from /bora/modules/vmkernel/arm64/armtestacpivmkapi/armtestacpivmkapi.c
 */

#include "gpio.h"
#include "gpio_device.h"

/***********************************************************************/

gpio_DevOps_t gpio_DevOps = {
   .dumpMem = gpio_dumpMMIOMem,
   .dumpRegs = gpio_dumpRegisters,
   .changeIntrState = gpio_changeIntrState,
   .setupIntr = gpio_setupIntr,
   .ackIntr = gpio_ackIntr,
   .intrHandler = gpio_intrHandler,
   .waitIntr = gpio_waitIntr,
   .destroyIntr = gpio_destroyIntr,
};

/*
 ***********************************************************************
 * gpio_dumpMMIOMem --
 *
 *    Dump the mapped mmio memory to log.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_dumpMMIOMem(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   char *buf;
   int bufLen;
   int i, j;
   char val;

   vmk_LogMessage("%s: %s: dumping io space mapped to %p of length %d",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  adapter->mmioBase,
                  adapter->mmioLen);

   bufLen = 24;
   buf = vmk_HeapAlloc(gpio_Driver.heapID, bufLen);
   vmk_Memset(buf, 0, bufLen);
   for (i = 0, j = 0; i < adapter->mmioLen; ++i, j += 3) {
      if (j != 0 && j % 8 == 0) {
         /* Print line */
         j = 0;
         buf[bufLen - 1] = '\0';
         vmk_LogMessage("%s: %s: %s", GPIO_DRIVER_NAME, __FUNCTION__, buf);
      }
      val = *(adapter->mmioBase + i);
      vmk_Sprintf(&buf[j], "%02x ", val);
   }

   /* Dump remaining bytes */
   buf[j] = '\0';
   vmk_LogMessage("%s: %s: %s", GPIO_DRIVER_NAME, __FUNCTION__, buf);

   /*
    * This PSODs the machine somehow, so for now I'll just live with the memory
    * leak.
    */
   //vmk_HeapFree(gpio_Driver.heapID, buf);

   return status;
}

/*
 ***********************************************************************
 * gpio_DumgRegisters --
 * 
 *    Dumps the gpio registers to log.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_dumpRegisters(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_changeIntrState --
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
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_setupIntr(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_IntrProps intrProps;
   int numIntrsAllocated;

   status = adapter->devOps->changeIntrState(adapter,
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
   intrProps.acknowledgeInterrupt = adapter->devOps->ackIntr;
   intrProps.handler = adapter->devOps->intrHandler;
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

   status = adapter->devOps->changeIntrState(adapter,
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
   adapter->devOps->changeIntrState(adapter,
                                    GPIO_INT_BUSY_UP,
                                    GPIO_INT_NONE);

change_intr_failed:
   return status;
}

/*
 ***********************************************************************
 * gpio_ackIntr --
 * 
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_ackIntr(void *clientData,            // IN
             vmk_IntrCookie intrCookie)   // IN
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;

   status = adapter->devOps->changeIntrState(adapter,
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
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 ***********************************************************************
 */
void
gpio_intrHandler(void *clientData,            // IN
                                  vmk_IntrCookie intrCookie)   // IN
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;
   vmk_WorldID worldID;

   status = adapter->devOps->changeIntrState(adapter,
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

   status = adapter->devOps->changeIntrState(adapter,
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
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_waitIntr(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;

   status = adapter->devOps->changeIntrState(adapter,
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

   //DEBUG
   return status;

   status = adapter->devOps->changeIntrState(adapter,
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
 *    Reference: armtestacpivmkapi.c#ATDSetupInt
 ***********************************************************************
 */
void
gpio_destroyIntr(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status;

   status = adapter->devOps->changeIntrState(adapter,
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

   status = adapter->devOps->changeIntrState(adapter,
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