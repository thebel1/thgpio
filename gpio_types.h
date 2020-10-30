/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_types.h --
 *
 *    Type definitions
 */

#ifndef GPIO_TYPES_H
#define GPIO_TYPES_H

/***********************************************************************/

typedef struct gpio_Driver_t {
   vmk_Name driverName;
   vmk_ModuleID moduleID;
   vmk_HeapID heapID;
   vmk_Driver driverHandle;
   vmk_IOResource resHandle;
   vmk_LogComponent logger;
} gpio_Driver_t;

/*
 * The gpio adapter. It should only ever be allocated once since there is only
 * one gpio interface on a Raspberry Pi.
 */
typedef struct gpio_Device_t {
   /* Object */
   vmk_Bool initialized;
   vmk_atomic64 refCount;
   /* Device */
   vmk_Device vmkDevice;
   vmk_ACPIDevice acpiDevice;
   vmk_ACPIInfo acpiInfo;
   /* Interrupts */
   vmk_IntrCookie intrCookie;
   vmk_atomic64 intCount;
   vmk_atomic64 intState;
   vmk_WorldID wakeWorld;
   /* MMIO */
   vmk_MappedResourceAddress mmioMappedAddr;
   char *mmioBase;
   vmk_ByteCount mmioLen;
} gpio_Device_t;

/*
 * Struct holding information about a world to launch while starting the GPIO
 * device in gpio_startDevice().
 */
typedef struct gpio_StartUpWorld_t {
   char name[32];
   vmk_WorldID worldID;
   vmk_WorldStartFunc startFunc;
} gpio_StartUpWorld_t;

/***********************************************************************/

#endif /* GPIO_TYPES_H */