/*
 * gpio_debug.c --
 */

#include "gpio_debug.h"

/*
 ***********************************************************************
 * gpioDebug_worldFunc --
 * 
 * 
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_worldFunc(void *clientData) // IN: adapter
{
   VMK_ReturnStatus status = VMK_OK;
   //gpio_Device_t *adapter = (gpio_Device_t *)clientData;

   do {
      status = vmk_WorldWait(VMK_EVENT_NONE,
                             VMK_LOCK_INVALID,
                             GPIO_DEBUG_WORLD_SLEEP_MS,
                             __FUNCTION__);
      
      /* Adapter must be initialized by now */
      //gpio_dumpMMIOMem(adapter);
      //gpioDebug_dumpPins(adapter);
   } while (status != VMK_DEATH_PENDING);

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_zeroOutMMIOMem --
 * 
 * 
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_zeroOutMMIOMem(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   int i;
   
   vmk_LogMessage("%s: %s: resetting mmio mem",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__);

   for (i = 0; i < adapter->mmioLen; ++i) {
      vmk_MappedResourceWrite8((void *)adapter->mmioBase, i, 0);
   }

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_forceIntr --
 * 
 * 
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_forceIntr(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   
   vmk_LogMessage("%s: %s: forcing interrupt",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__);

   status = gpio_setupIntr(adapter);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to configure interrupts: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto intr_config_failed;
   }

   status = gpio_waitIntr(adapter);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: timeout waiting for interrupt",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__);
   }

   /*
    * Currently, destroying the interrupt fails because the interrupt state
    * doesn't change, meaning we're stuck in GPIO_INT_SIGNAL.
    */
   gpio_destroyIntr(adapter);

intr_config_failed:
   return status;
}

/*
 ***********************************************************************
 * gpioDebug_testPins --
 * 
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_testPins(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   int bank0, bank1;
   int i, oldVal, newVal;

   GPIO_READ_BANK(adapter, 0, &bank0);
   GPIO_READ_BANK(adapter, 1, &bank1);

   vmk_LogMessage("%s: %s: gpio bank0 0x%x bank1 0x%x",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  bank0,
                  bank1);

   for (i = 0; i < GPIO_NUM_PINS; ++i) {
      GPIO_GET_PIN(adapter, i, &oldVal);
      
      /* Select the pin */
      GPIO_SEL(adapter, i, GPIO_SEL_OUT);

      /* Flip pin */
      if (oldVal == 0) {
         GPIO_SET_PIN(adapter, i);
      }
      else {
         GPIO_CLR_PIN(adapter, i);
      }
      
      GPIO_GET_PIN(adapter, i, &newVal);
      
      /* Flip pin back */
      if (oldVal == 1) {
         GPIO_SET_PIN(adapter, i);
      }
      else {
         GPIO_CLR_PIN(adapter, i);
      }

      vmk_LogMessage("%s: %s: gpio pin %d oldVal 0x%x newVal 0x%x",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     i,
                     oldVal,
                     newVal);
   }

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_dumpPins --
 * 
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_dumpPins(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   int bank0, bank1;
   int i, oldVal, newVal;

   GPIO_READ_BANK(adapter, 0, &bank0);
   GPIO_READ_BANK(adapter, 1, &bank1);

   vmk_LogMessage("%s: %s: gpio bank0 0x%x bank1 0x%x",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  bank0,
                  bank1);

   for (i = 0; i < GPIO_NUM_PINS; ++i) {
      GPIO_GET_PIN(adapter, i, &oldVal);
      
      /* Select the pin */
      GPIO_SEL(adapter, i, GPIO_SEL_OUT);

      /* Flip pin */
      /*if (oldVal == 0) {
         GPIO_SET_PIN(adapter, i);
      }
      else {
         GPIO_CLR_PIN(adapter, i);
      }*/
      
      GPIO_GET_PIN(adapter, i, &newVal);

      vmk_LogMessage("%s: %s: gpio pin %d oldVal 0x%x newVal 0x%x",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     i,
                     oldVal,
                     newVal);
   }

   return status;
}