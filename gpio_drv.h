/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_drv.h --
 *
 *    Definition for gpio device layer interface
 */

#ifndef GPIO_DRV_H
#define GPIO_DRV_H

#include "gpio_types.h"

/***********************************************************************/

/*
 * GPIO register defs and macros for BCM2835 SoC.
 * 
 * TODO:
 *    - implement vmk status reporting
 * 
 * References:
 * - https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2711/rpi_DATA_2711_1p0.pdf
 * - https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf
 */

/* For the RPi 4B */
#define GPIO_NUM_PINS 40

#define GPIO_PIN_HI 1
#define GPIO_PIN_LO 0

#define GPIO_PIN_MASK 31

/*
 * GPIO pin function selectors
 */

#define GPIO_SEL_IN  0b000
#define GPIO_SEL_OUT 0b001
#define GPIO_SEL_F0  0b100
#define GPIO_SEL_F1  0b101
#define GPIO_SEL_F2  0b110
#define GPIO_SEL_F3  0b111
#define GPIO_SEL_F4  0b011
#define GPIO_SEL_F5  0b010

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

#define GPIO_PUP_PDN_CNTRL_REG0  0xe4
#define GPIO_PUP_PDN_CNTRL_REG1  0xe8
#define GPIO_PUP_PDN_CNTRL_REG2  0xec
#define GPIO_PUP_PDN_CNTRL_REG3  0xf0

#define GPIO_PUD_OFF 0b00
#define GPIO_PUD_DN  0b10
#define GPIO_PUD_UP  0b01

/***********************************************************************/

VMK_INLINE VMK_ReturnStatus gpio_readReg(gpio_Device_t *adapter,
                                         vmk_uint32 reg,
                                         vmk_uint32 *ptr);

VMK_INLINE VMK_ReturnStatus gpio_writeReg(gpio_Device_t *adapter,
                                          vmk_uint32 reg,
                                          vmk_uint32 val);

VMK_INLINE VMK_ReturnStatus gpio_funcSelPin(gpio_Device_t *adapter,
                                            vmk_uint32 pin,
                                            vmk_uint32 func);

VMK_INLINE VMK_ReturnStatus gpio_setPin(gpio_Device_t *adapter,
                                        vmk_uint32 pin);

VMK_INLINE VMK_ReturnStatus gpio_clrPin(gpio_Device_t *adapter,
                                        vmk_uint32 pin);

VMK_INLINE VMK_ReturnStatus gpio_levPin(gpio_Device_t *adapter,
                                        vmk_uint32 pin,
                                        vmk_uint32 *ptr);

VMK_INLINE VMK_ReturnStatus gpio_setPull(gpio_Device_t *adapter,
                                         vmk_uint32 pin,
                                         vmk_uint8 pud);

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

#endif /* GPIO_DRV_H */