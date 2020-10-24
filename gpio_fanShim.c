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

#define GPIO_FANSHIM_PIN_CLOCK  14
#define GPIO_FANSHIM_PIN_DATA   15
#define GPIO_FANSHIM_PIN_BUTTON 17
#define GPIO_FANSHIM_PIN_FAN    18

#define GPIO_FANSHIM_FAN_MS        5000  /* Toggle fan every x seconds */
#define GPIO_FANSHIM_LED_PERIOD_MS 2000  /* 2sec period for pulse */

/*
 ***********************************************************************
 * gpio_fanShimFanWorldFunc --
 * 
 *    Entry point for debug world that turns the Pimoroni FanShim fan on
 *    and off every X seconds.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimFanWorldFunc(void *clientData) // IN: adapter
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;
   int val;

   do {
      status = vmk_WorldWait(VMK_EVENT_NONE,
                             VMK_LOCK_INVALID,
                             GPIO_FANSHIM_FAN_MS,
                             __FUNCTION__);

      vmk_LogMessage("%s: %s: toggling fan",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__);

      gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_FAN, GPIO_SEL_OUT);
      gpio_levPin(adapter, GPIO_FANSHIM_PIN_FAN, &val);
      if (val) {
         gpio_clrPin(adapter, GPIO_FANSHIM_PIN_FAN);
      }
      else {
         gpio_setPin(adapter, GPIO_FANSHIM_PIN_FAN);
      }
   } while (status != VMK_DEATH_PENDING);

   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimLEDWorldFunc --
 * 
 *    Entry point for debug world that transitions the Pimoroni FanShim
 *    LED from red to green to blue to red.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimLEDWorldFunc(void *clientData) // IN: adapter
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;
   int mode, btnPrev, btnCur;

   /* Switch mode between gradient, flashing, and off */
   mode = 0;

   /*
    * Set up button
    */
   
   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_BUTTON, GPIO_SEL_IN);
   gpio_setPull(adapter, GPIO_FANSHIM_PIN_BUTTON, GPIO_PUD_UP);

   /* Avoid initial button jitter */
   vmk_WorldSleep(1000 * 1000);

   gpio_levPin(adapter, GPIO_FANSHIM_PIN_BUTTON, &btnPrev);
   while (1) {

      switch (mode) {
         case 0:
            gpio_fanShimGradientLED(adapter, GPIO_FANSHIM_LED_PERIOD_MS);
            mode = (mode + 1) % 5;
            break;
         case 1:
            gpio_fanShimFlashLED(adapter, 255, 0, 0, 255, 500);
            mode = (mode + 1) % 5;
            break;
         case 2:
            gpio_fanShimFlashLED(adapter, 0, 255, 0, 255, 500);
            mode = (mode + 1) % 5;
            break;
         case 3:
            gpio_fanShimFlashLED(adapter, 0, 0, 255, 255, 500);
            mode = (mode + 1) % 5;
            break;
         case 4:
            gpio_fanShimSetLED(adapter, 0, 0, 0, 0);
            vmk_WorldSleep(1000);
            
            gpio_levPin(adapter, GPIO_FANSHIM_PIN_BUTTON, &btnCur);
            if (btnPrev < btnCur) { /* Button released */
               mode = (mode + 1) % 5;
            }
            btnPrev = btnCur;

            break;
      }
   }

   return status;
}

/*
 ***********************************************************************
 * gpio_fanShimBtnWorldFunc --
 * 
 *    Entry point for debug world that panics the system when the button
 *    on the FanShim is pressed.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    Panics the system.
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_fanShimBtnWorldFunc(void *clientData) // IN: adapter
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_Device_t *adapter = (gpio_Device_t *)clientData;
   int btnPrev, btnCur;

   /* So we can use the button to control the LED */
   if (1) {
      return status;
   }

   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_BUTTON, GPIO_SEL_IN);
   gpio_setPull(adapter, GPIO_FANSHIM_PIN_BUTTON, GPIO_PUD_UP);

   /* Wait to avoid initial button pin jitter */
   vmk_WorldSleep(1000 * 1000);

   /* Loop so we can respond to button presses */
   gpio_levPin(adapter, GPIO_FANSHIM_PIN_BUTTON, &btnPrev);
   while (1) {
      gpio_levPin(adapter, GPIO_FANSHIM_PIN_BUTTON, &btnCur);
      if (btnPrev != btnCur) {
         vmk_Panic("Panic button pressed!");
      }
      btnPrev = btnCur;
      vmk_WorldSleep(1000);
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
gpio_fanShimSetLED(gpio_Device_t *adapter,
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
gpio_fanShimFlashLED(gpio_Device_t *adapter,
                          vmk_uint8 red,
                          vmk_uint8 green,
                          vmk_uint8 blue,
                          vmk_uint8 brightness,
                          int intervalMs)
{
   VMK_ReturnStatus status = VMK_OK;
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
   int intervalUs = intervalMs * 1000;
   int btnPrev, btnCur;

   /*
    * Set up button
    */
   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_BUTTON, GPIO_SEL_IN);
   gpio_setPull(adapter, GPIO_FANSHIM_PIN_BUTTON, GPIO_PUD_UP);

   /* Avoid button jitter */
   vmk_WorldSleep(1000);

   /*
    * Set up data & clock pins
    */
   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_CLOCK, GPIO_SEL_OUT);
   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_DATA, GPIO_SEL_OUT);
   gpio_clrPin(adapter, GPIO_FANSHIM_PIN_CLOCK);
   gpio_clrPin(adapter, GPIO_FANSHIM_PIN_DATA);

   gpio_levPin(adapter, GPIO_FANSHIM_PIN_BUTTON, &btnPrev);
   while (1) {

      gpio_levPin(adapter, GPIO_FANSHIM_PIN_BUTTON, &btnCur);
      if (btnPrev < btnCur) { /* Button released */
         goto button_pressed;
      }
      btnPrev = btnCur;

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

button_pressed:
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
gpio_fanShimGradientLED(gpio_Device_t *adapter, int periodMs)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_uint8 buf[] = {0, 0, 0, 0,
                      255, 0, 0, 0,
                      ~0, ~0, ~0, ~0};
   int bufLen = sizeof(buf);
   vmk_uint8 byte;
   vmk_Bool bit;
   int i, j, k;
   vmk_uint8 rgb[] = {255, 0, 0};
   int intervalUs = (periodMs * 1000) / 256;
   int btnPrev, btnCur;

   /*
    * Set up FanShim button
    */

   gpio_funcSelPin(adapter, GPIO_FANSHIM_PIN_BUTTON, GPIO_SEL_IN);
   gpio_setPull(adapter, GPIO_FANSHIM_PIN_BUTTON, GPIO_PUD_UP);

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
   gpio_levPin(adapter, GPIO_FANSHIM_PIN_BUTTON, &btnPrev);
   while (1) {

      for (i = 0; i < bufLen; ++i) {
         byte = buf[i];

         /* Register button press */
         gpio_levPin(adapter, GPIO_FANSHIM_PIN_BUTTON, &btnCur);
         if (btnPrev < btnCur) { /* Button released */
            goto button_pressed;
         }
         btnPrev = btnCur;

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

button_pressed:
   return status;
}