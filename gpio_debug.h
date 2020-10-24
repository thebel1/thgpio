/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * gpio_debug.h --
 */

#ifndef GPIO_DEBUG_H
#define GPIO_DEBUG_H

#include "gpio.h"
#include "gpio_types.h"
#include "gpio_drv.h"

/***********************************************************************/

/* Run debug world every second */
#define GPIO_DEBUG_WORLD_SLEEP_MS 5000

/***********************************************************************/

/*
 * Define debug data types for vmk opaque types so we don't have to do
 * any weird pointer arithmetic.
 */

typedef struct Debug_vmkBusType {
   vmk_ListLinks links;
   vmk_Name name;
   int refCount;
   vmk_ModuleID moduleID;
} Debug_vmkBusType;

typedef void *ACPI_HANDLE;

typedef void *VAArray_Handle;

typedef struct Debug_vmkIOResource {
    vmk_ListLinks links;
    vmk_IOResourceInfo info;
    vmk_Device device;
    vmk_ListLinks devNext;
    vmk_ListLinks reservations;
    int refCount;
} Debug_vmkIOResource;

typedef struct Debug_vmkIOResMap {
    Debug_vmkIOResource *res;
    vmk_IOReservation resv;
    vmk_VA va;
    VAArray_Handle vaArray;
} Debug_vmkIOResMap;

typedef struct Debug_VMKAcpiPnpDevice {
   vmk_uint32 magic;
   vmk_ListLinks links;
   vmk_Device vmkDev;
   ACPI_HANDLE acpiHandle;
   ACPI_HANDLE parentHandle;
   char *deviceName;
   Debug_vmkIOResMap *iores;
   vmk_ACPIResDesc *res;
   vmk_uint32 *irqs;
   vmk_uint32 numRes;
   vmk_uint32 numIrqs;
   vmk_uint64 flags;
   vmk_IntrCookie *intrArray;
   vmk_uint32 intrArrayLen;
   vmk_ListLinks busResList;
   vmk_IOA addressMask;
   vmk_DMADevice dmaInfo;
} Debug_VMKAcpiPnpDevice;

/***********************************************************************/

typedef struct gpio_DebugWorld_t {
   char name[32];
   vmk_WorldID worldID;
   vmk_WorldStartFunc startFunc;
} gpio_DebugWorld_t;

/***********************************************************************/

VMK_ReturnStatus gpioDebug_fanWorldFunc(void *clientData);
VMK_ReturnStatus gpioDebug_ledWorldFunc(void *clientData);
VMK_ReturnStatus gpioDebug_btnWorldFunc(void *clientData);

VMK_ReturnStatus gpioDebug_dumpMMIOMem(gpio_Device_t *adapter);
VMK_ReturnStatus gpioDebug_dumpRegisters(gpio_Device_t *adapter);

VMK_ReturnStatus gpioDebug_testPins(gpio_Device_t *adapter);
VMK_ReturnStatus gpioDebug_dumpPins(gpio_Device_t *adapter);

VMK_ReturnStatus gpioDebug_fanShimTurnOnLED(gpio_Device_t *adapter,
                                            vmk_uint8 red,
                                            vmk_uint8 green,
                                            vmk_uint8 blue,
                                            vmk_uint8 brightness);

VMK_ReturnStatus gpioDebug_fanShimFlashLED(gpio_Device_t *adapter,
                                           vmk_uint8 red,
                                           vmk_uint8 green,
                                           vmk_uint8 blue,
                                           vmk_uint8 brightness,
                                           int intervalMs);

VMK_ReturnStatus gpioDebug_fanShimGradientLED(gpio_Device_t *adapter,
                                              int periodMs);

/***********************************************************************/

#endif /* GPIO_DEBUG_H */