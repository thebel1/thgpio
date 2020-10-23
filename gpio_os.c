/******************************************************************************\
 * Native ESXi driver for the RPi's GPIO interface.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * TODO:
 * - set up logger
 * - set up locking / lock domain
 */

/*
 * gpio_os.c --
 * 
 *    Module initialization and other OS stuff.
 */

#include "gpio.h"
#include "gpio_types.h"
#include "gpio_drv.h"

/***********************************************************************/

gpio_Driver_t gpio_Driver;
gpio_Device_t gpio_Device;

VMK_ReturnStatus gpio_attachDevice(vmk_Device device);
VMK_ReturnStatus gpio_scanDevice(vmk_Device device);
VMK_ReturnStatus gpio_detachDevice(vmk_Device device);
VMK_ReturnStatus gpio_quiesceDevice(vmk_Device device);
VMK_ReturnStatus gpio_startDevice(vmk_Device device);
void gpio_forgetDevice(vmk_Device device);

vmk_DriverOps gpio_DriverOps = {
   .attachDevice = gpio_attachDevice,
   .scanDevice = gpio_scanDevice,
   .detachDevice = gpio_detachDevice,
   .quiesceDevice = gpio_quiesceDevice,
   .startDevice = gpio_startDevice,
   .forgetDevice = gpio_forgetDevice,
};

/***********************************************************************/

/* Used for dumption mmio mem and registers */
vmk_WorldID gpioDebugWorldID = VMK_INVALID_WORLD_ID;

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
      goto heap_create_fail;
   }

   vmk_ModuleSetHeapID(gpio_Driver.moduleID, heapID);
   VMK_ASSERT(vmk_ModuleGetHeapID(gpio_Driver.moduleID) == heapID);
   gpio_Driver.heapID = heapID;

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
      goto driver_register_fail;
   }

   return VMK_OK;

driver_register_fail:
   vmk_HeapDestroy(gpio_Driver.heapID);

heap_create_fail:
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
    * Debug message to determine whether we're attaching to the correct device.
    */
#ifdef GPIO_DEBUG
   vmk_DeviceID *debug_devID;
   Debug_vmkBusType* debug_busType;
   status = vmk_DeviceGetDeviceID(gpio_Driver.heapID, device, &debug_devID);
   VMK_ASSERT(status == VMK_OK);
   debug_busType = ((Debug_vmkBusType*)debug_devID->busType);
   vmk_LogMessage("%s: %s: attaching device %p"
                  " busType %s busAddr %s busIdent %s",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  device,
                  &debug_busType->name.string,
                  debug_devID->busAddress,
                  debug_devID->busIdentifier);
   status = vmk_DevicePutDeviceID(gpio_Driver.heapID, debug_devID);
   VMK_ASSERT(status == VMK_OK);
#endif /* GPIO_DEBUG */

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
    * Debug info about io resources
    */
#ifdef GPIO_DEBUG
   Debug_VMKAcpiPnpDevice *debug_acpiPnpDev = ((Debug_VMKAcpiPnpDevice*)acpiDev);
   vmk_LogMessage("%s: %s: device %p at addr %p has %d io resource(s)",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  device,
                  debug_acpiPnpDev->iores->res->info.start.address.memory,
                  debug_acpiPnpDev->numRes);
#endif /* GPIO_DEBUG */

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
   vmk_AtomicWrite64(&adapter->intCount, 0);
   vmk_AtomicWrite64(&adapter->refCount, 0);
   vmk_AtomicWrite64(&adapter->intState, GPIO_INT_NONE);
   adapter->wakeWorld = VMK_INVALID_WORLD_ID;
   vmk_Memcpy(&adapter->mmioMappedAddr,
              &mappedAddr,
              sizeof(adapter->mmioMappedAddr));
   adapter->mmioBase = (void *)mappedAddr.address.vaddr;
   adapter->mmioLen = mappedAddr.len;
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

   /*
    * Launch debug world for gpio adapter
    */
#ifdef GPIO_DEBUG
   vmk_WorldProps debugWorldProps;
   debugWorldProps.moduleID = vmk_ModuleCurrentID;
   debugWorldProps.name = "gpio_debug_world";
   debugWorldProps.startFunction = gpioDebug_worldFunc;
   debugWorldProps.data = (void *)adapter;
   debugWorldProps.schedClass = VMK_WORLD_SCHED_CLASS_DEFAULT;
   debugWorldProps.heapID = gpio_Driver.heapID;
   status = vmk_WorldCreate(&debugWorldProps, &gpioDebugWorldID);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to create debug world: %s",
                           GPIO_DRIVER_NAME,
                           __FUNCTION__,
                           vmk_StatusToString(status));
      vmk_WorldDestroy(gpioDebugWorldID);
      vmk_WorldWaitForDeath(gpioDebugWorldID);
   }
   
   /* Dump mmio mem */
   //gpioDebug_dumpMMIOMem(adapter);
#endif /* GPIO_DEBUG */

#ifdef GPIO_DEBUG
   //gpioDebug_testPins(adapter);
   //gpioDebug_turnOffEachPinAndWait(adapter, 5000);
   //gpioDebug_turnOnEachPinAndWait(adapter, 5000);

   /*vmk_LogMessage("%s: %s: attempting to turn off fan",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__);*/

   //gpioDebug_fanShimToggle(adapter);

   /* Select gpio pin 18 */
   //*((int *)adapter->mmioBase + 1) = 0b001 << 24;
   /* Clear gpio pin 18 */
   //*((int *)adapter->mmioBase + 10) = 1 << 18;

   //gpio_funcSelPin(adapter, 18, GPIO_SEL_OUT);
   //gpio_clrPin(adapter, 18);

   //gpioDebug_fanShimFlashLED(adapter, 500);
   
   //gpioDebug_fanShimTurnOnLED(adapter, 0, 255, 0, 1);
#endif /* GPIO_DEBUG */

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
 *    We don't do anything here since the gpio device doesn't have any
 *    child devices.
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

   vmk_LogMessage("%s: %s: scanning device %p",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  device);

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

   vmk_LogMessage("%s: %s: Detaching device %p",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  device);

   /*
    * Destroy the debug world
    */
#ifdef GPIO_DEBUG
   if (gpioDebugWorldID != VMK_INVALID_WORLD_ID) {
      vmk_WorldDestroy(gpioDebugWorldID);
      vmk_WorldWaitForDeath(gpioDebugWorldID);
   }
#endif /* GPIO_DEBUG */

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

   vmk_LogMessage("%s: %s: quiescing device %p",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  device);

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
   gpio_Device_t *adapter = &gpio_Device;

   vmk_LogMessage("%s: %s: base addr %p for device %p",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  adapter->mmioBase,
                  device);

//#ifdef GPIO_DEBUG
   //gpioDebug_fanShimTurnOnLED(adapter, 0, 255, 0, 1);
   gpioDebug_fanShimFlashLED(adapter, 0, 255, 0, 255, 500);
//#endif /* GPIO_DEBUG */

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

   vmk_LogMessage("%s: %s: forgetting device %p",
                  GPIO_DRIVER_NAME,
                  __FUNCTION__,
                  device);

   return;
}