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
#define FANSHIM_FAN_PIN 18
VMK_ReturnStatus
gpioDebug_worldFunc(void *clientData) // IN: adapter
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;
   int val;

   /* So the compiler stops complaining when it's not used */
   adapter = adapter;

   do {
      status = vmk_WorldWait(VMK_EVENT_NONE,
                             VMK_LOCK_INVALID,
                             5000,
                             __FUNCTION__);

      vmk_LogMessage("%s: %s: toggling fan",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__);

      gpio_funcSelPin(adapter, FANSHIM_FAN_PIN, GPIO_SEL_OUT);
      gpio_levPin(adapter, FANSHIM_FAN_PIN, &val);
      if (val) {
         gpio_clrPin(adapter, FANSHIM_FAN_PIN);
      }
      else {
         gpio_setPin(adapter, FANSHIM_FAN_PIN);
      }
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

   // TODO

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

/*
 ***********************************************************************
 * gpioDebug_fanShimTurnOnLED --
 * 
 * References:
 *    - https://github.com/flobernd/raspi-apa102
 *    - https://github.com/pimoroni/apa102-python/blob/master/library/apa102/__init__.py
 ***********************************************************************
 */
#define RASPI_FANSHIM_PIN_SPI_SCLK  14
#define RASPI_FANSHIM_PIN_SPI_MOSI  15
#define RASPI_APA102_CLCK_STRETCH 5
VMK_ReturnStatus
gpioDebug_fanShimTurnOnLED(gpio_Device_t *adapter,
                           vmk_uint8 red,
                           vmk_uint8 green,
                           vmk_uint8 blue,
                           vmk_uint8 brightness)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_uint8 buf[] = {0, 0, 0, 0,                     /* SOF */
                      0b11100000 | (brightness * 31), /* brightness */
                      blue,
                      green,
                      red,
                      ~0, ~0, ~0, ~0};                /* EOF */
   int bufLen = sizeof(buf);
   char logBuf[16];
   int i, j;

   vmk_LogMessage("%s: %s: turning on fan shim LED using buffer",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__);

   status = gpio_funcSelPin(adapter, RASPI_FANSHIM_PIN_SPI_SCLK, GPIO_SEL_OUT);
   status = gpio_funcSelPin(adapter, RASPI_FANSHIM_PIN_SPI_MOSI, GPIO_SEL_OUT);
   status = gpio_clrPin(adapter, RASPI_FANSHIM_PIN_SPI_MOSI);
   status = gpio_clrPin(adapter, RASPI_FANSHIM_PIN_SPI_SCLK);

   /* Set LED color */
   for (i = 0; i < bufLen; ++i) {
      vmk_uint8 byte = buf[i];
      vmk_Memset(logBuf, 0, sizeof(logBuf));

      vmk_LogMessage("%s: %s: hex 0x%x", GPIO_DRIVER_NAME, __FUNCTION__, byte);

      for (j = 0; j < 8; ++j) {
         /* Slice off left-most bit */
         vmk_Bool bit = (byte & 0x80) > 0;

         if (bit) {
            logBuf[j] = '\x31'; /* "1" */
            logBuf[j + 1] = '\x20'; /* space */
            status = gpio_setPin(adapter, RASPI_FANSHIM_PIN_SPI_MOSI);
         }
         else {
            logBuf[j] = '\x30'; /* "0" */
            logBuf[j + 1] = '\x20'; /* space */
            status = gpio_clrPin(adapter, RASPI_FANSHIM_PIN_SPI_MOSI);
         }

         status = gpio_setPin(adapter, RASPI_FANSHIM_PIN_SPI_SCLK);
         status = vmk_WorldSleep(100);
         status = gpio_clrPin(adapter, RASPI_FANSHIM_PIN_SPI_SCLK);
         status = vmk_WorldSleep(100);

         /* Shift left one bit so we can slice it like a salami */
         byte <<= 1;
      }
      logBuf[15] = '\0';
      vmk_LogMessage("%s: %s: bin %s", GPIO_DRIVER_NAME, __FUNCTION__, logBuf);
   }

   return VMK_OK;
}

/*
 ***********************************************************************
 * gpioDebug_fanShimFlashLED --
 * 
 ***********************************************************************
 */
VMK_ReturnStatus
gpioDebug_fanShimFlashLED(gpio_Device_t *adapter,
                          int intervalMs)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}