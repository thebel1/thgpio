/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * TODO:
 *    - Set up logger
 */

/*
 * gpio_os.c --
 * 
 *    Module initialization and other OS stuff.
 */

#include "gpio.h"
#include "gpio_types.h"
#include "gpio_drv.h"
#include "gpio_charDev.h"

/***********************************************************************/

static gpio_Driver_t gpio_Driver;
static gpio_Device_t gpio_Device;

/* For chardev */
static gpio_CharDev_t gpio_CharDev;
static vmk_BusType gpio_logicalBusType;
static vmk_uint32 gpio_logicalPort = 0;

VMK_ReturnStatus gpio_attachDevice(vmk_Device device);
VMK_ReturnStatus gpio_scanDevice(vmk_Device device);
VMK_ReturnStatus gpio_detachDevice(vmk_Device device);
VMK_ReturnStatus gpio_quiesceDevice(vmk_Device device);
VMK_ReturnStatus gpio_startDevice(vmk_Device device);
void gpio_forgetDevice(vmk_Device device);

static vmk_DriverOps gpio_DriverOps = {
   .attachDevice = gpio_attachDevice,
   .scanDevice = gpio_scanDevice,
   .detachDevice = gpio_detachDevice,
   .quiesceDevice = gpio_quiesceDevice,
   .startDevice = gpio_startDevice,
   .forgetDevice = gpio_forgetDevice,
};

/*
 * Callbacks gluing the chardev driver to the GPIO driver "back-end".
 */
static gpio_CharDevCallbacks_t gpio_CharDevCBs = {
   .open = gpio_mmioOpenCB,
   .close = gpio_mmioCloseCB,
   .ioctl = gpio_mmioIoctlCB,
   .read = gpio_mmioReadCB,
   .write = gpio_mmioWriteCB,
};

/*
 ***********************************************************************
 * init_module --
 * 
 *    Entry point for driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
int init_module(void)
{
   VMK_ReturnStatus status;
   vmk_HeapID heapID;
   vmk_HeapCreateProps heapProps;
   vmk_DriverProps driverProps;
   //vmk_LogProperties logProps;
   vmk_Name logicalBusName; /* For the chardev */

   status = vmk_NameInitialize(&gpio_Driver.driverName, GPIO_DRIVER_NAME);
   VMK_ASSERT(status == VMK_OK);

   gpio_Driver.moduleID = vmk_ModuleCurrentID;

   gpio_Device.initialized = VMK_FALSE;

   /*
    * Create heap
    */

   heapProps.type = VMK_HEAP_TYPE_SIMPLE;
   status = vmk_NameInitialize(&heapProps.name, GPIO_DRIVER_NAME);
   VMK_ASSERT(status == VMK_OK);
   heapProps.module = gpio_Driver.moduleID;
   heapProps.initial = GPIO_HEAP_INITIAL_SIZE;
   heapProps.creationTimeoutMS = VMK_TIMEOUT_NONBLOCKING;
   heapProps.max = GPIO_HEAP_MAX_SIZE;
   
   status = vmk_HeapCreate(&heapProps, &heapID);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: vmk_HeapCreate failed: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto heap_create_failed;
   }

   vmk_ModuleSetHeapID(gpio_Driver.moduleID, heapID);
   VMK_ASSERT(vmk_ModuleGetHeapID(gpio_Driver.moduleID) == heapID);
   gpio_Driver.heapID = heapID;

   /*
    * Init logical bus
    */

   status = vmk_NameInitialize(&logicalBusName, VMK_LOGICAL_BUS_NAME);
   status = vmk_BusTypeFind(&logicalBusName, &gpio_logicalBusType);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: vmk_BusTypeFind for logical bus failed: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto logical_bus_failed;
   }

   /*
    * Register driver
    */

   vmk_Memset(&driverProps, 0, sizeof(vmk_DriverProps));
   status = vmk_NameInitialize(&driverProps.name, GPIO_DRIVER_NAME);
   VMK_ASSERT(status == VMK_OK);
   driverProps.moduleID = gpio_Driver.moduleID;
   driverProps.ops = &gpio_DriverOps;

   status = vmk_DriverRegister(&driverProps,
                               &(gpio_Driver.driverHandle));
   if (status == VMK_OK) {
      vmk_LogMessage("%s: %s: vmk_DriverRegister successful",
                     GPIO_DRIVER_NAME,
                     __FUNCTION__);
   }
   else {
      vmk_WarningMessage("%s: %s: vmk_DriverRegister failed: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto driver_register_failed;
   }

   /*
    * Init GPIO driver
    */
   gpio_drvInit(&gpio_Device);

   return VMK_OK;

driver_register_failed:
   vmk_HeapDestroy(gpio_Driver.heapID);

logical_bus_failed:
heap_create_failed:
   vmk_WarningMessage("%s: %s failed", GPIO_DRIVER_NAME, __FUNCTION__);

   return status;
}

/*
 ***********************************************************************
 * cleanup_module --
 * 
 *    Cleans up the module.
 * 
 * Results:
 *    None.
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
void cleanup_module(void)
{
   vmk_BusTypeRelease(gpio_logicalBusType);
   vmk_HeapDestroy(gpio_Driver.heapID);
   vmk_ACPIUnmapIOResource(gpio_Driver.moduleID, gpio_Device.acpiDevice, 0);
   vmk_DriverUnregister(gpio_Driver.driverHandle);
}

/*
 ***********************************************************************
 * gpio_attachDevice --
 * 
 *    Driver callback that attaches gpio logical device to the driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_attachDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_ACPIDevice acpiDev;
   vmk_ACPIInfo acpiInfo;
   vmk_MappedResourceAddress mappedAddr;
   gpio_Device_t *adapter = &gpio_Device;

   /*
    * Since there is only one gpio device on the system board, we're making
    * sure we're not initializing more.
    */
   if (adapter->initialized != VMK_FALSE) {
      vmk_WarningMessage("%s: %s: gpio device %p already initialized as %p",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         device,
                         adapter->vmkDevice);
      status = VMK_EXISTS;
      goto device_already_exists;
   }

   /*
    * Get vmk acpi dev
    */
   status = vmk_DeviceGetRegistrationData(device, (vmk_AddrCookie *)&acpiDev);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to get acpi dev for vmk dev %p: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         device,
                         vmk_StatusToString(status));
      goto get_acpi_dev_failed;
   }

   /*
    * Find out if this is an acpi dev
    */
   status = vmk_ACPIQueryInfo(acpiDev, &acpiInfo);
   if (status != VMK_OK) {
      goto not_an_acpi_dev;
   }

   vmk_LogMessage("%s: %s: retrieved acpi dev %p for vmk dev %p",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  acpiDev,
                  device);

   /*
    * Map io resources
    */

   status = vmk_ACPIMapIOResource(gpio_Driver.moduleID,
                        	       acpiDev,
                                  0,
                                  &mappedAddr);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to acquire io resources"
                         " for device %p: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         device,
                         vmk_StatusToString(status));
      goto iores_map_failed;
   }
   else if (mappedAddr.type != VMK_IORESOURCE_MEM) {
      vmk_WarningMessage("%s: %s: mapped io space %p of type %d, expecting %d",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         mappedAddr,
                         mappedAddr.type,
                         VMK_IORESOURCE_MEM);
      goto iores_map_failed;
   }

   vmk_LogMessage("%s: %s: mapped io space at %p of length %d for device %p",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  (void*)mappedAddr.address.vaddr,
                  mappedAddr.len,
                  device);

   /*
    * Init adapter
    */

   vmk_Memset(adapter, 0, sizeof(*adapter));
   adapter->vmkDevice = device;
   adapter->acpiDevice = acpiDev;
   adapter->acpiInfo = acpiInfo;
   vmk_AtomicWrite64(&adapter->refCount, 0);
   adapter->wakeWorld = VMK_INVALID_WORLD_ID;
   vmk_Memcpy(&adapter->mmioMappedAddr,
              &mappedAddr,
              sizeof(adapter->mmioMappedAddr));
   adapter->mmioBase = (void *)mappedAddr.address.vaddr;
   adapter->mmioLen = mappedAddr.len;

   /* Currently unused */
   vmk_AtomicWrite64(&adapter->intCount, 0);
   vmk_AtomicWrite64(&adapter->intState, 0);

   adapter->initialized = VMK_TRUE;

   /*
    * Attach device
    */
   status = vmk_DeviceSetAttachedDriverData(adapter->vmkDevice, adapter);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to attach device %p: %s",
                         GPIO_DRIVER_NAME,
                         __FUNCTION__,
                         adapter->vmkDevice,
                         vmk_StatusToString(status));
      goto device_attach_failed;
   }

   return status;

device_attach_failed:
iores_map_failed:
not_an_acpi_dev:
get_acpi_dev_failed:
device_already_exists:
   vmk_WarningMessage("%s: %s: no device attached",
                      GPIO_DRIVER_NAME,
                      __FUNCTION__);
   return status;
}

/*
 ***********************************************************************
 * gpio_scanDevice --
 * 
 *    Register char dev.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_scanDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;
   gpio_CharDev_t *charDev = &gpio_CharDev;

   /*
    * Register char device
    */
   status = gpio_charDevRegister(&gpio_Driver,
                                 gpio_logicalBusType,
                                 device,
                                 charDev,
                                 gpio_logicalPort,
                                 &gpio_CharDevCBs);

   return status;
}

/*
 ***********************************************************************
 * gpio_detachDevice --
 * 
 *    Detaches the device from the driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_detachDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_quiesceDevice --
 * 
 *    Do nothing.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_quiesceDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_startDevice --
 *
 *    Start the GPIO device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
gpio_startDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * gpio_forgetDevice --
 * 
 *    Do nothing.
 * 
 * Results:
 *    None.
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
void
gpio_forgetDevice(vmk_Device device)
{
   return;
}