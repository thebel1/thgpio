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
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;

   /* So the compiler stops complaining when it's not used */
   adapter = adapter;

   do {
      status = vmk_WorldWait(VMK_EVENT_NONE,
                             VMK_LOCK_INVALID,
                             5000,
                             __FUNCTION__);
      
      /* Adapter must be initialized by now */
      //gpioDebug_dumpMMIOMem(adapter);
      //gpioDebug_dumpPins(adapter);
      //gpioDebug_piHutFanShimToggle(adapter);
      //gpioDebug_blinkEachPinOnce(adapter, GPIO_DEBUG_WORLD_SLEEP_MS);
      //gpioDebug_turnOffEachPinAndWait(adapter, 5000);
      //gpioDebug_turnOnEachPinAndWait(adapter, 5000);
   } while (status != VMK_DEATH_PENDING);

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_dumpMMIOMem --
 *
 *    Dump the mapped mmio memory to log.
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
gpioDebug_dumpRegisters(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_dumpPins --
 * 
 *    Outputs the value of each gpio pin.
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
      GPIO_SEL_PIN(adapter, i, GPIO_SEL_OUT);

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

/*
 ***********************************************************************
 * gpioDebug_testPins --
 * 
 *    Flips the value of all gpio pins, i.e. from 0 to 1 and vice versa.
 *    Some pins are initialized with 0 and some with 1 by the hardware.
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
      GPIO_SEL_PIN(adapter, i, GPIO_SEL_OUT);

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
 * gpioDebug_piHutFanShimToggle --
 * 
 *    Turns the PiHut FanShim's fan on or off. This is used as a test
 *    device to figure out how controlling the pins works.
 * 
 *    See:
 *       - https://github.com/flobernd/raspi-fanshim/blob/master/src/Fanshim.c
 *       - http://wiringpi.com/wp-content/uploads/2013/03/pins.pdf
 ***********************************************************************
 */
#define PIHUT_FANSHIM_FAN_PIN 29
VMK_ReturnStatus
gpioDebug_piHutFanShimToggle(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   int curVal;

   GPIO_SEL_PIN(adapter, PIHUT_FANSHIM_FAN_PIN, GPIO_SEL_OUT);
   GPIO_GET_PIN(adapter, PIHUT_FANSHIM_FAN_PIN, &curVal);

   vmk_LogMessage("%s: %s: toggling PiHut FanShim (pin %d) from %d to %d",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  PIHUT_FANSHIM_FAN_PIN,
                  curVal,
                  GPIO_PIN_HI - curVal);

   GPIO_TOGGLE_PIN(adapter, PIHUT_FANSHIM_FAN_PIN);

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_blinkEachPinOnce --
 * 
 *    Flips each pin once.
 * 
 *    See:
 *       - https://github.com/flobernd/raspi-fanshim/blob/master/src/Fanshim.c
 *       - http://wiringpi.com/wp-content/uploads/2013/03/pins.pdf
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_blinkEachPinOnce(gpio_Device_t *adapter, int waitMs)
{
   VMK_ReturnStatus status = VMK_OK;
   int i, curVal, newVal;

   for (i = 0; i < GPIO_NUM_PINS; ++i) {
      GPIO_SEL_PIN(adapter, i, GPIO_SEL_OUT);
      GPIO_GET_PIN(adapter, i, &curVal);
      GPIO_TOGGLE_PIN(adapter, i);
      GPIO_GET_PIN(adapter, i, &newVal);

      vmk_LogMessage("%s: %s: toggling pin %d from %d to %d and back",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     i,
                     curVal,
                     newVal);

      status = vmk_WorldWait(VMK_EVENT_NONE,
                             VMK_LOCK_INVALID,
                             waitMs,
                             __FUNCTION__);

      GPIO_TOGGLE_PIN(adapter, i);
   }

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_turnOnEachPinAndWait --
 *
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_turnOnEachPinAndWait(gpio_Device_t *adapter, int waitMs)
{
   VMK_ReturnStatus status = VMK_OK;
   int i, val;

   for (i = 0; i < GPIO_NUM_PINS; ++i) {
      GPIO_SEL_PIN(adapter, i, GPIO_SEL_OUT);
      GPIO_GET_PIN(adapter, i, &val);
      GPIO_SET_PIN(adapter, i);
      
      vmk_LogMessage("%s: %s: turned ON pin %d (orig val %d)",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     i,
                     val);

      status = vmk_WorldWait(VMK_EVENT_NONE,
                             VMK_LOCK_INVALID,
                             waitMs,
                             __FUNCTION__);
   }

   return status;
}

/*
 ***********************************************************************
 * gpioDebug_turnOffEachPinAndWait --
 *
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_turnOffEachPinAndWait(gpio_Device_t *adapter, int waitMs)
{
   VMK_ReturnStatus status = VMK_OK;
   int i, val;

   for (i = 0; i < GPIO_NUM_PINS; ++i) {
      GPIO_SEL_PIN(adapter, i, GPIO_SEL_OUT);
      GPIO_GET_PIN(adapter, i, &val);
      GPIO_CLR_PIN(adapter, i);
      
      vmk_LogMessage("%s: %s: turned OFF pin %d (orig val %d)",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__,
                     i,
                     val);

      status = vmk_WorldWait(VMK_EVENT_NONE,
                             VMK_LOCK_INVALID,
                             waitMs,
                             __FUNCTION__);
   }

   return status;
}