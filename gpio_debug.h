/*
 * gpio_debug.h --
 */

#ifndef GPIO_DEBUG_H
#define GPIO_DEBUG_H

#include "gpio.h"
#include "gpio_types.h"
#include "gpio_device.h"

/***********************************************************************/

/* Run debug world every second */
#define GPIO_DEBUG_WORLD_SLEEP_MS 1000

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

VMK_ReturnStatus gpioDebug_worldFunc(void *clientData);

VMK_ReturnStatus gpioDebug_zeroOutMMIOMem(gpio_Device_t *adapter);

VMK_ReturnStatus gpioDebug_forceIntr(gpio_Device_t *adapter);

VMK_ReturnStatus gpio_debugTestPins(gpio_Device_t *adapter);

/***********************************************************************/

#endif /* GPIO_DEBUG_H */