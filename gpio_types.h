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
extern gpio_Driver_t gpio_Driver;

/*
 * The supported model types. UNUSED.
 */
typedef enum {
   GPIO_MODEL_2708,
   GPIO_MODEL_2711,
} gpio_Model_t;

/*
 * Shamelessly stolen from /bora/modules/vmkernel/arm64/armtestacpivmkapi/armtestacpivmkapi.c
 * 
 * Adapter intState. These states record the driver
 * going through different phases of configuring interrupts
 * on the ATD device, triggering an int, and handling an interrupt.
 *
 * Flow is:
 *
 *    /------<  GPIO_INT_BUSY_DOWN <-----------------\
 *    |                                         |
 * GPIO_INT_NONE >-> GPIO_INT_BUSY_UP >---> GPIO_INT_READY >-------> GPIO_INT_SIGNAL >-------\
 *                            /                                          |
 *                            |                                          |
 *                            \--< GPIO_INT_HANDLE_END <-- GPIO_INT_HANDLE_START <-/
 */
enum {
   GPIO_INT_NONE,           /* No interrupts are configured, we are quescient and
                             * ready to service requests via the VSI interface .*/
   GPIO_INT_BUSY_UP,        /* We are in the process for configuring ints for the
                             * device, as part of acting on a VSI command to test
                             * interrupt delivery. */
   GPIO_INT_BUSY_DOWN,      /* We are in the process of tearing down ints after
                             * testing interrupt delivery. */
   GPIO_INT_READY,          /* We are are ready to set the ACPI device to trigger
                             * an interrupt and wait for its delivery. */
   GPIO_INT_SIGNAL,         /* We have set the ACPI device to trigger an interrupt,
                             * and are now waiting for its delivery. */
   GPIO_INT_HANDLE_START,   /* We have been called by the acknowledgeInterrupt
                             * callback and are now ready to handle the interrupt. */
   GPIO_INT_HANDLE_END,     /* We have handled the interrupt and are now ready
                             * to acknowledge the interrupt, wake up the waiter
                             * in the VSI interface, and transition to GPIO_INT_READY */
};

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
   /* device model */
   gpio_Model_t model;
   /* mapped io address struct */
   vmk_MappedResourceAddress mmioMappedAddr;
   /* base addr for mmio */
   char *mmioBase;
   /* size of mmio space */
   vmk_ByteCount mmioLen;
} gpio_Device_t;
extern gpio_Device_t gpio_Device;

/***********************************************************************/

#endif /* GPIO_TYPES_H */