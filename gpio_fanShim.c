/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_fanShim.c --
 * 
 *    Code for interacting with Pimoroni's Fan Shim.
 * 
 *    Pinout: https://pinout.xyz/pinout/fan_shim
 *    APA102 LED specs: https://datasheetspdf.com/pdf-file/905213/GreeledElectronic/APA102/1
 *    UW libs:
 *       - https://github.com/pimoroni/fanshim-python/
 *       - https://github.com/flobernd/raspi-fanshim/
 *       - https://github.com/joaomoreno/fanshim-rust
 */

#include "gpio_fanShim.h"

/***********************************************************************/

static gpio_Device_t *gpio_Device;

/*
 ***********************************************************************
 * gpio_fanShimInit --
 * 
 *    Initializes the data required for the FanShim.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimInit(gpio_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;

   gpio_Device = adapter;

   /*
    * For some reason, the initial pin state does not reflect the actual state
    */
   gpio_fanShimFanToggle();

   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimCharDevOpenCB --
 * 
 *    Callback for GPIO character device file open.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimCharDevOpenCB(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimCharDevCloseCB --
 * 
 *    Callback for GPIO character device file close.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimCharDevCloseCB(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimCharDevReadCB --
 * 
 *    Callback for GPIO character device file read.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimCharDevReadCB(char *buffer,
                          vmk_ByteCount nbytes)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimCharDevWriteCB --
 * 
 *    Callback for GPIO character device file write.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimCharDevWriteCB(char *buffer,
                           vmk_ByteCountSigned nwritten)
{
   VMK_ReturnStatus status = VMK_OK;
   char cmd[GPIO_FANSHIM_CMD_MAX_LEN];
   int i, n;
   long col;
   vmk_uint8 red, green, blue;

   n = nwritten % GPIO_FANSHIM_CMD_MAX_LEN;
   
   /* Make sure it's zero-terminated */
   if (n < GPIO_FANSHIM_CMD_MAX_LEN) {
      buffer[n - 1] = '\0';
   }
   else {
      buffer[GPIO_FANSHIM_CMD_MAX_LEN - 1] = '\0';
   }

   /*
    * Only use for debugging! Not safe!
    */
#ifdef GPIO_DEBUG
   vmk_LogMessage("%s: %s: buffer: %s",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  buffer);
#endif /* GPIO_DEBUG */

   for (i = 0; i < n; ++i) {
      cmd[i] = buffer[i];
   }

   /*
    * Only use for debugging! Not safe!
    */
#ifdef GPIO_DEBUG
   vmk_LogMessage("%s: %s: cmd: %s",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  cmd);
#endif /* GPIO_DEBUG */

   /*
    * Parse commands
    */
   if (vmk_Strcmp(cmd, "gradient") == 0) {
      gpio_fanShimGradientLED(5000, 2000);
      gpio_fanShimSetLED(0, 0, 0, 255);
   }
   else if (vmk_Strcmp(cmd, "flash red") == 0) {
      gpio_fanShimFlashLED(255, 0, 0, 255, 5000, 500);
      gpio_fanShimSetLED(0, 0, 0, 255);
   }
   else if (vmk_Strcmp(cmd, "flash green") == 0) {
      gpio_fanShimFlashLED(0, 255, 0, 255, 5000, 500);
      gpio_fanShimSetLED(0, 0, 0, 255);
   }
   else if (vmk_Strcmp(cmd, "flash blue") == 0) {
      gpio_fanShimFlashLED(0, 0, 255, 255, 5000, 500);
      gpio_fanShimSetLED(0, 0, 0, 255);
   }
   else if (vmk_Strcmp(cmd, "fan") == 0) {
      gpio_fanShimFanToggle();
   }
   else if (nwritten == 7) {
      col = vmk_Strtol(cmd, NULL, 16);
      red = (vmk_uint8)((col & (0xff << 16)) >> 16);
      green = (vmk_uint8)((col & (0xff << 8)) >> 8);
      blue = (vmk_uint8)(col & 0xff);
      gpio_fanShimSetLED(red, green, blue, 255);
   }
   else {
      vmk_WarningMessage("%s: %s: unsupported command",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__);
   }

   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimFanToggle --
 * 
 *    Turns the fan on or off.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimFanToggle()
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = gpio_Device;
   vmk_uint32 val;

   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_FAN, GPIO_SEL_OUT);
   gpio_levPin(adapter, GPIO_FANSHIM_PIN_FAN, &val);
   if (val) {
      gpio_clrPin(adapter, GPIO_FANSHIM_PIN_FAN);
   }
   else {
      gpio_setPin(adapter, GPIO_FANSHIM_PIN_FAN);
   }

   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimSetLED --
 * 
 *    Turns on the LED on the FanShim accessory.   
 * 
 *    References:
 *       - https://github.com/flobernd/raspi-apa102
 *       - https://github.com/pimoroni/apa102-python/blob/master/library/apa102/__init__.py
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimSetLED(vmk_uint8 red,
                   vmk_uint8 green,
                   vmk_uint8 blue,
                   vmk_uint8 brightness)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = gpio_Device;
   vmk_uint8 buf[] = {0, 0, 0, 0,                     /* SOF */
                      0b11100000 | (brightness * 31), /* brightness */
                      blue,
                      green,
                      red,
                      ~0, ~0, ~0, ~0};                /* EOF */
   int bufLen = sizeof(buf);
   char logBuf[16];
   int i, j;
   int clockVal, dataVal;
   vmk_uint8 byte;
   vmk_Bool bit;

   vmk_LogMessage("%s: %s: turning on fan shim LED using clock pin %d,"
                  " data pin %d, and buffer...",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  GPIO_FANSHIM_PIN_CLOCK,
                  GPIO_FANSHIM_PIN_DATA);

   status = gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_CLOCK, GPIO_SEL_OUT);
   status = gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_DATA, GPIO_SEL_OUT);
   status = gpio_clrPin(adapter, GPIO_FANSHIM_PIN_DATA);
   status = gpio_clrPin(adapter, GPIO_FANSHIM_PIN_CLOCK);

   gpio_levPin(adapter, GPIO_FANSHIM_PIN_CLOCK, &clockVal);
   gpio_levPin(adapter, GPIO_FANSHIM_PIN_DATA, &dataVal);

   vmk_LogMessage("%s: %s: clock %d data %d",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  clockVal,
                  dataVal);

   /*
    * Set LED color (model APA102).
    * 
    * Transfer data format documented in:
    * https://datasheetspdf.com/pdf-file/905213/GreeledElectronic/APA102/1
    */
   for (i = 0; i < bufLen; ++i) {
      byte = buf[i];
      vmk_Memset(logBuf, 0, sizeof(logBuf));

      vmk_LogMessage("%s: %s: hex 0x%x", GPIO_DRIVER_NAME, __FUNCTION__, byte);

      for (j = 0; j < 8; ++j) {
         /* Slice off left-most bit */
         bit = (byte & 0x80) > 0;

         if (bit) {
            logBuf[j] = '\x31'; /* "1" */
            logBuf[j + 1] = '\x20'; /* space */
            
            status = gpio_setPin(adapter, GPIO_FANSHIM_PIN_DATA);
            
            gpio_levPin(adapter, 15, &dataVal);
            vmk_LogMessage("%s: %s: out %d",
                           GPIO_DRIVER_NAME,
                           __FUNCTION__,
                           dataVal);
         }
         else {
            logBuf[j] = '\x30'; /* "0" */
            logBuf[j + 1] = '\x20'; /* space */
            
            status = gpio_clrPin(adapter, GPIO_FANSHIM_PIN_DATA);
            
            gpio_levPin(adapter, 15, &dataVal);
            vmk_LogMessage("%s: %s: out %d",
                           GPIO_DRIVER_NAME,
                           __FUNCTION__,
                           dataVal);
         }

         status = gpio_setPin(adapter, GPIO_FANSHIM_PIN_CLOCK);
         gpio_levPin(adapter, 14, &clockVal);
         vmk_LogMessage("%s: %s: clk %d",
                        GPIO_DRIVER_NAME,
                        __FUNCTION__,
                        clockVal);
         status = vmk_WorldSleep(1);
         status = gpio_clrPin(adapter, GPIO_FANSHIM_PIN_CLOCK);
         gpio_levPin(adapter, 14, &clockVal);
         vmk_LogMessage("%s: %s: clk %d",
                        GPIO_DRIVER_NAME,
                        __FUNCTION__,
                        clockVal);
         
         /* Shift left one bit so we can slice it like a salami */
         byte <<= 1;
         
         status = vmk_WorldSleep(1);
      }
      logBuf[15] = '\0';
      vmk_LogMessage("%s: %s: bin %s", GPIO_DRIVER_NAME, __FUNCTION__, logBuf);
   }

   return VMK_OK;
}

/*
 ***********************************************************************
 * gpio_fanShimFlashLED --
 * 
 *    Flashes the LED on the FanShim accessory.   
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimFlashLED(vmk_uint8 red,
                     vmk_uint8 green,
                     vmk_uint8 blue,
                     vmk_uint8 brightness,
                     int durationMs,
                     int intervalMs)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = gpio_Device;
   vmk_uint8 bufOn[] = {0, 0, 0, 0,
                        brightness,
                        blue,
                        green,
                        red,
                        ~0, ~0, ~0, ~0};
   vmk_uint8 bufOff[] = {0, 0, 0, 0,
                         brightness,
                         0,
                         0,
                         0,
                         ~0, ~0, ~0, ~0};
   vmk_uint8 *curBuf = bufOff;
   int bufLen = sizeof(bufOn);
   int i, j;
   vmk_uint8 byte;
   vmk_Bool bit;
   int durationSec = durationMs / 1000;
   int intervalUs = intervalMs * 1000;
   vmk_TimerCycles monoCounterFreq = vmk_GetMonotonicCounterFrequency();
   unsigned long startTime, curTime;

   /* Avoid button jitter */
   vmk_WorldSleep(1000);

   /*
    * Set up data & clock pins
    */
   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_CLOCK, GPIO_SEL_OUT);
   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_DATA, GPIO_SEL_OUT);
   gpio_clrPin(adapter, GPIO_FANSHIM_PIN_CLOCK);
   gpio_clrPin(adapter, GPIO_FANSHIM_PIN_DATA);

   startTime = vmk_TimerMonotonicCounter;
   while (1) {
      /*
       * Check whether duration has elapsed
       */
      curTime = vmk_TimerMonotonicCounter;
      if ((curTime - startTime) / monoCounterFreq >= durationSec) {
         goto duration_elapsed;
      }

      /* Flip between color buffer and black buffer */
      if (curBuf == bufOff) {
         curBuf = bufOn;
      }
      else {
         curBuf = bufOff;
      }

      /* Transmit buffer to LED */
      for (i = 0; i < bufLen; ++i) {
         byte = curBuf[i];

         for (j = 0; j < 8; ++j) {
            bit = (byte & 0x80) > 0;
            
            if (bit) {
               gpio_setPin(adapter, GPIO_FANSHIM_PIN_DATA);
            }
            else {
               gpio_clrPin(adapter, GPIO_FANSHIM_PIN_DATA);
            }

            gpio_setPin(adapter, GPIO_FANSHIM_PIN_CLOCK);
            vmk_WorldSleep(1);
            gpio_clrPin(adapter, GPIO_FANSHIM_PIN_CLOCK);
            vmk_WorldSleep(1);

            byte <<= 1;
         }
      }

      vmk_WorldSleep(intervalUs);
   }

duration_elapsed:
   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimGradientLED --
 * 
 *    Gradually transitions the LED from red to green to blue to red.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimGradientLED(int durationMs, int periodMs)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = gpio_Device;
   vmk_uint8 buf[] = {0, 0, 0, 0,
                      255, 0, 0, 0,
                      ~0, ~0, ~0, ~0};
   int bufLen = sizeof(buf);
   vmk_uint8 byte;
   vmk_Bool bit;
   int i, j, k;
   vmk_uint8 rgb[] = {255, 0, 0};
   int intervalUs = (periodMs * 1000) / 256;
   int durationSec = durationMs / 1000;
   vmk_TimerCycles monoCounterFreq = vmk_GetMonotonicCounterFrequency();
   unsigned long startTime, curTime;

   /* Wait to avoid initial button pin jitter */
   vmk_WorldSleep(1000);

   /*
    * Set up FanShim clock & data pins
    */

   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_DATA, GPIO_SEL_OUT);
   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_CLOCK, GPIO_SEL_OUT);
   gpio_clrPin(adapter, GPIO_FANSHIM_PIN_DATA);
   gpio_clrPin(adapter, GPIO_FANSHIM_PIN_CLOCK);

   /*
    * Control the color gradient
    */

   k = 1;
   startTime = vmk_TimerMonotonicCounter;
   while (1) {
      /*
       * Check whether duration has elapsed
       */
      curTime = vmk_TimerMonotonicCounter;
      if ((curTime - startTime) / monoCounterFreq >= durationSec) {
         goto duration_elapsed;
      }

      for (i = 0; i < bufLen; ++i) {
         byte = buf[i];

         for (j = 0; j < 8; ++j) {
            bit = (byte & 0x80) > 0;

            if (bit) {
               gpio_setPin(adapter, GPIO_FANSHIM_PIN_DATA);
            }
            else {
               gpio_clrPin(adapter, GPIO_FANSHIM_PIN_DATA);
            }

            gpio_setPin(adapter, GPIO_FANSHIM_PIN_CLOCK);
            vmk_WorldSleep(1);
            gpio_clrPin(adapter, GPIO_FANSHIM_PIN_CLOCK);
            vmk_WorldSleep(1);

            byte <<= 1;
         }
      }

      /*
       * Move cursor one color to the right & wrap around
       */
      if (rgb[k] == 255) {
         k = (k + 1) % 3;
      }
      
      /*
       * Transition the colors based on the position of cursor k
       */
      switch (k) {
         case 0:
            --rgb[2];
            ++rgb[0];
            break;
         case 1:
            --rgb[0];
            ++rgb[1];
            break;
         case 2:
            --rgb[1];
            ++rgb[2];
      }
      buf[5] = rgb[0];
      buf[6] = rgb[1];
      buf[7] = rgb[2];

      vmk_WorldSleep(intervalUs);
   }

duration_elapsed:
   return status;
}