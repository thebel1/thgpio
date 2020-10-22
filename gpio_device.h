/*
 * gpio_device.h --
 *
 *    Definition for gpio device layer interface
 * 
 * References:
 * 
 *    - https://github.com/RPi-Distro/raspi-gpio/blob/master/raspi-gpio.c
 *    - 
 * 
 * TODO:
 *    - rename to gpio_hw.h to better reflect this file's purpose
 *    - create a gpio_dpi.c/.h to control 
 */

#ifndef GPIO_DEVICE_H
#define GPIO_DEVICE_H

#include "gpio_types.h"

/***********************************************************************/

/*
 * GPIO register defs and macros for BCM2835 SoC.
 * 
 * TODO:
 *    - implement vmk status reporting
 * 
 * Reference: https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf
 */

#define GPIO_NUM_PINS 54

#define GPIO_READ_REG(_adapter, _reg, _ptr)                                    \
   (vmk_MappedResourceRead32(&_adapter->mmioMappedAddr, _reg, _ptr))

#define GPIO_WRITE_REG(_adapter, _reg, _val)                                   \
   (vmk_MappedResourceWrite32(&_adapter->mmioMappedAddr, _reg, _val))

#define GPIO_WRITE_PIN_REG(_adapter, _regPfx, _pin, _val)                      \
   do {                                                                        \
      if (_pin < 32) {  /* bank 0 */                                           \
         GPIO_WRITE_REG(_adapter, _regPfx##0, _val);                           \
      } else { /* bank1 */                                                     \
         GPIO_WRITE_REG(_adapter, _regPfx##1, _val);                           \
      }                                                                        \
   } while(0)

#define GPIO_SET_PIN(_adapter, _pin) GPIO_WRITE_PIN_REG(_adapter,              \
                                                        GPSET,                 \
                                                        _pin,                  \
                                                        (1 << (_pin % 32)))

#define GPIO_CLR_PIN(_adapter, _pin) GPIO_WRITE_PIN_REG(_adapter,              \
                                                        GPCLR,                 \
                                                        _pin,                  \
                                                        (1 << (_pin % 32)))

#define GPIO_GET_PIN(_adapter, _pin, _ptr)                                     \
   do {                                                                        \
      if (_pin < 32) {  /* bank 0 */                                           \
         GPIO_READ_REG(_adapter, GPLEV0, _ptr);                                \
      } else { /* bank1 */                                                     \
         GPIO_READ_REG(_adapter, GPLEV1, _ptr);                  \
      }                                                                        \
      *_ptr = *_ptr & (1 << (_pin % 32));                                      \
   } while(0)

/*
 * Function selector registers which control the operation of the 54 gpio pins.
 * 
 * See p. 91.
 */
#define GPFSEL0   0x00     /* gpio function select 0 */
#define GPFSEL1   0x04     /* gpio function select 1 */
#define GPFSEL2   0x08     /* gpio function select 2 */
#define GPFSEL3   0x0c     /* gpio function select 3 */
#define GPFSEL4   0x10     /* gpio function select 4 */
#define GPFSEL5   0x14     /* gpio function select 5 */

/*
 * Used to set a specific gpio pin. Setting to 0 has no effect.
 */
#define GPSET0    0x1c     /* gpio pin output set 0 */
#define GPSET1    0x20     /* gpio pin output set 1 */

/*
 * Used to clear gpio pin. Setting to 0 has no effect.
 */
#define GPCLR0    0x28     /* gpio pin output clear 0 */
#define GPCLR1    0x2c     /* gpio pin output clear 1 */

/*
 * Return value of a pin.
 */
#define GPLEV0    0x34     /* gpio pin level 0 */
#define GPLEV1    0x38     /* gpio pin level 1 */

/*
 * Used to retrieve signal edge detection results. 
 */
#define GPEDS0    0x40     /* gpio pin event detect status 0 */
#define GPEDS1    0x44     /* gpio pin event detect status 1 */
#define GPREN0    0x4c     /* gpio pin rising edge detect enable 0 */
#define GPREN1    0x50     /* gpio pin rising edge detect enable 1 */
#define GPFEN0    0x58     /* gpio pin falling edge detect enable 0 */
#define GPFEN1    0x5c     /* gpio pin falling edge detect enable 1 */
#define GPHEN0    0x64     /* gpio pin high detect enable 0 */
#define GPHEN1    0x68     /* gpio pin high detect enable 1 */
#define GPLEN0    0x70     /* gpio pin low detect enable 0 */
#define GPLEN1    0x74     /* gpio pin low detect enable 1 */
#define GPAREN0   0x7c     /* gpio pin async rising edge detect 0 */
#define GPAREN1   0x80     /* gpio pin async rising edge detect 1 */
#define GPAFEN0   0x88     /* gpio pin async falling edge detect 0*/
#define GPAFEN1   0x8c     /* gpio ping async falling edge detect 1 */

/*
 * Pull-up/down config.
 */
#define GPPUD     0x94     /* gpio pin pull-up/down enable */
#define GPPUDCLK0 0x98     /* gpio pin pull-up/down enable clock 0 */
#define GPPUDCLK1 0x9c     /* gpio pin pull-up/down enable clock 1 */

/***********************************************************************/

VMK_ReturnStatus gpio_dumpMMIOMem(gpio_Device_t *adapter);

VMK_ReturnStatus gpio_dumpRegisters(gpio_Device_t *adapter);

VMK_ReturnStatus gpio_changeIntrState(gpio_Device_t *adapter,
                                      vmk_uint64 curState,
                                      vmk_uint64 newState);

VMK_ReturnStatus gpio_setupIntr(gpio_Device_t *adapter);

VMK_ReturnStatus gpio_ackIntr(void *clientData,
                              vmk_IntrCookie intrCookie);

void gpio_intrHandler(void *clientData,
                      vmk_IntrCookie intrCookie);

VMK_ReturnStatus gpio_waitIntr(gpio_Device_t *adapter);

void gpio_destroyIntr(gpio_Device_t *adapter);

/***********************************************************************/

#endif /* GPIO_DEVICE_H */