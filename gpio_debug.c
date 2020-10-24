/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_debug.c --
 * 
 *    Various debugging and testing functions.
 */

#include "gpio_debug.h"

/*
 ***********************************************************************
 * gpioDebug_dumpMMIOMem --
 *
 *    Dump the mapped mmio memory to log.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_dumpMMIOMem(gpio_Device_t *adapter)
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

   vmk_HeapFree(gpio_Driver.heapID, buf);

   return status;
}

/*
 ***********************************************************************
 * gpio_DumgRegisters --
 * 
 *    Dumps the gpio registers to log.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_dumpRegisters(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;

   // TODO

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_dumpPins --
 * 
 *    Outputs the value of each gpio pin.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_dumpPins(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   int i, oldVal, newVal;

   for (i = 0; i < GPIO_NUM_PINS; ++i) {
      gpio_levPin(adapter, i, &oldVal);
      
      /* Select the pin */
      gpio_funcSelPin(adapter, i, GPIO_SEL_OUT);
      
      gpio_levPin(adapter, i, &newVal);

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
 * gpioDebug_testPins --
 * 
 *    Flips the value of all gpio pins, i.e. from 0 to 1 and vice versa.
 *    Some pins are initialized with 0 and some with 1 by the hardware.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_testPins(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   int i, oldVal, newVal;

   for (i = 0; i < GPIO_NUM_PINS; ++i) {
      gpio_levPin(adapter, i, &oldVal);
      
      /* Select the pin */
      gpio_funcSelPin(adapter, i, GPIO_SEL_OUT);

      /* Flip pin */
      if (oldVal == 0) {
         gpio_setPin(adapter, i);
      }
      else {
         gpio_clrPin(adapter, i);
      }
      
      gpio_levPin(adapter, i, &newVal);
      
      /* Flip pin back */
      if (oldVal == 1) {
         gpio_setPin(adapter, i);
      }
      else {
         gpio_clrPin(adapter, i);
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