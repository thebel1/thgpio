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
} gpio_Driver_t;

/*
 * The gpio adapter. It should only ever be allocated once since there is only
 * one gpio interface on a Raspberry Pi.
 */
typedef struct gpio_Device_t {
   /* since we only have one device, we store its init state */
   vmk_Bool initialized;
   /* vmk device handle */
   vmk_Device vmkDevice;
   /* vmk acpi device */
   vmk_ACPIDevice acpiDevice;
   /* vmk acpi device info */
   vmk_ACPIInfo acpiInfo;
   /* interrupt cookie */
   vmk_IntrCookie intrCookie;
   /* number of ints received */
   vmk_atomic64 intCount;
   /* refcount for this struct */
   vmk_atomic64 refCount;
   /* state for int handling */
   vmk_atomic64 intState;
   /* world ID to woke on int handler */
   vmk_WorldID wakeWorld;
   /* mapped io address struct */
   vmk_MappedResourceAddress mmioMappedAddr;
   /* base addr for mmio */
   char *mmioBase;
   /* size of mmio space */
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