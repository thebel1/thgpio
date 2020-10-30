/******************************************************************************\
 * Native ESXi driver for the RPi's SPI interfaces.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * spi_os.c --
 *
 *    Driver's OS interface.
 */

#include "spi.h"
#include "spi_types.h"
//#include "spi_drv.h"
#include "../include/thx_charDev.h"

/***********************************************************************/

static spi_Driver_t spi_Driver;

/* For chardev */
static spi_CharDev_t spi_CharDev;
static vmk_BusType spi_logicalBusType;
static vmk_uint32 spi_logicalPort = 0;

VMK_ReturnStatus spi_attachDevice(vmk_Device device);
VMK_ReturnStatus spi_scanDevice(vmk_Device device);
VMK_ReturnStatus spi_detachDevice(vmk_Device device);
VMK_ReturnStatus spi_quiesceDevice(vmk_Device device);
VMK_ReturnStatus spi_startDevice(vmk_Device device);
void spi_forgetDevice(vmk_Device device);

static vmk_DriverOps spi_DriverOps = {
   .attachDevice = spi_attachDevice,
   .scanDevice = spi_scanDevice,
   .detachDevice = spi_detachDevice,
   .quiesceDevice = spi_quiesceDevice,
   .startDevice = spi_startDevice,
   .forgetDevice = spi_forgetDevice,
};

/*
 * Callbacks gluing the chardev driver to the SPI driver "back-end".
 */
static spi_CharDevCallbacks_t spi_CharDevCBs = {
   /*.open = spi_mmioOpenCB,
   .close = spi_mmioCloseCB,
   .ioctl = spi_mmioIoctlCB,
   .read = spi_mmioReadCB,
   .write = spi_mmioWriteCB,*/
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
VMK_ReturnStatus
init_module(void)
{
   vmk_ReturnStatus status = VMK_OK;
   vmk_HeapID heapID;
   vmk_HeapCreateProps heapProps;
   vmk_DriverProps driverProps;
   vmk_Name charDevLogicalBus;

   status = vmk_NameInitialize(&spi_Driver.driverName, SPI_DRIVER_NAME);
   ASSERT(status == VMK_OK);
   spi_Driver.moduleID = vmk_ModuleCurrentID;

   /*
    * Create heap
    */

   heapProps.type = VMK_HEAP_TYPE_SIMPLE;
   status = vmk_NameInitialize(&heapProps.name SPI_DRIVER_NAME);
   ASSERT(status == VMK_OK);
   heapProps.module = spi_Driver.moduleID;
   heapProps.initial = SPI_HEAP_INITIAL_SIZE;
   heapProps.creationTimeoutUS = VMK_TIMEOUT_NONBLOCKING;
   heapProps.max = SPI_HEAP_MAX_SIZE;

   status = vmk_HeapCreate(&heapProps, &heapID);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to create heap: %s",
                         SPI_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto heap_create_failed;
   }

   vmk_ModuleSetHeapID(spi_Driver.moduleID, heapID);
   if (vmk_ModuleGetHeapID(spi_Driver.moduleID) != heapID) {
      status = VMK_FAILURE;
      vmk_WarningMessage("%s: %s: unable to assign heap to driver",
                         SPI_DRIVER_NAME,
                         __FUNCTION__);
      goto heap_assign_failed;
   }
   spi_Driver.heapID = heapID;

   /*
    * Set up logger
    */

   status = vmk_NameInitialize(&logProps.name, SPI_DRIVER_NAME);
   VMK_ASSERT(status == VMK_OK);
   logProps.module = spi_Driver.moduleID;
   logProps.heap = spi_Driver.heapID;
   logProps.defaultLevel = 0;
   logProps.throttle = NULL;
   
   status = vmk_LogRegister(&logProps, &spi_Driver.logger);
   if (status != VMK_OK) {
      vmk_WarningMessage("failed to register logger: %s",
                         vmk_StatusToString(status));
      goto logger_reg_failed;
   }
   vmk_LogSetCurrentLogLevel(spi_Driver.logger, SPI_LOG_INIT);

   /*
    * Init logical bus
    */
   
   vmk_NameInitialize(&charDevLogicalBus, VMK_LOGICAL_BUS_NAME);
   ASSERT(status == VMK_OK);
   status = vmk_BusTypeFind(&logicalBusName, &spi_logicalBusType);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: unable to find logical bus: %s",
                         SPI_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto logical_bus_failed;
   }

   /*
    * Register driver
    */

   vmk_Memset(&driverProps, 0, sizeof(vmk_DriverProps));
   vmk_NameInitialize(&driverProps.name, SPI_DRIVER_NAME);
   ASSERT(status == VMK_OK);
   driverProps.moduleID = spi_Driver.moduleID;
   driverProps.ops = &spi_DriverOps;

   status = vmk_DriverRegister(&driverProps,
                               &(spi_Driver.driverHandle));
      if (status == VMK_OK) {
      vmk_LogMessage("%s: %s: successfully registered driver",
                     SPI_DRIVER_NAME,
                     __FUNCTION__);
   }
   else {
      vmk_WarningMessage("%s: %s: failed to register driver: %s",
                         SPI_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto driver_register_failed;
   }

   // Set up logger

   return VMK_OK;

heap_assign_failed:
   vmk_HeapDestroy(heapID);

heap_name_init_failed:
heap_name_init_failed:
drv_name_init_failed:
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
   vmk_LogUnregister(spi_Driver.logger);
   vmk_BusTypeRelease(spi_logicalBusType);
   vmk_HeapDestroy(spi_Driver.heapID);
   vmk_ACPIUnmapIOResource(spi_Driver.moduleID, spi_Device.acpiDevice, 0);
   vmk_DriverUnregister(spi_Driver.driverHandle);
}

/*
 ***********************************************************************
 * spi_attachDevice --
 * 
 *    Driver callback that attaches spi logical device to the driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
spi_attachDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_ACPIDevice acpiDev;
   vmk_ACPIInfo acpiInfo;
   vmk_MappedResourceAddress mappedAddr;
   spi_Device_t *adapter = &spi_Device;

   /*
    * Since there is only one spi device on the system board, we're making
    * sure we're not initializing more.
    */
   if (adapter->initialized != VMK_FALSE) {
      vmk_Warning(spi_Driver.logger,
                  "spi device %p already initialized as %p",
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
      vmk_Warning(spi_Driver.logger,
                  "failed to get acpi dev for vmk dev %p: %s",
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

   vmk_Log(spi_Driver.logger,
           "retrieved acpi dev %p for vmk dev %p",
           acpiDev,
           device);

   /*
    * Map io resources
    */

   status = vmk_ACPIMapIOResource(spi_Driver.moduleID,
                                  acpiDev,
                                  0,
                                  &mappedAddr);
   if (status != VMK_OK) {
      vmk_Warning(spi_Driver.logger,
                  "unable to acquire io resources"
                  " for device %p: %s",
                  device,
                  vmk_StatusToString(status));
      goto iores_map_failed;
   }
   else if (mappedAddr.type != VMK_IORESOURCE_MEM) {
      vmk_Warning(spi_Driver.logger,
                  "mapped io space %p of type %d, expecting %d",
                  mappedAddr,
                  mappedAddr.type,
                  VMK_IORESOURCE_MEM);
      goto iores_map_failed;
   }

   vmk_Log(spi_Driver.logger,
           "mapped io space at %p of length %d for device %p",
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
      vmk_Warning(spi_Driver.logger,
                  "failed to attach device %p: %s",
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
   vmk_Warning(spi_Driver.logger,
               "no device attached",
               SPI_DRIVER_NAME,
               __FUNCTION__);
   return status;
}

/*
 ***********************************************************************
 * spi_scanDevice --
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
spi_scanDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;
   spi_CharDev_t *charDev = &spi_CharDev;

   /*
    * Register char device
    */
   status = spi_charDevRegister(&spi_Driver,
                                 spi_logicalBusType,
                                 device,
                                 charDev,
                                 spi_logicalPort,
                                 &spi_CharDevCBs);

   return status;
}

/*
 ***********************************************************************
 * spi_detachDevice --
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
spi_detachDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * spi_quiesceDevice --
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
spi_quiesceDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * spi_startDevice --
 *
 *    Start the SPI device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
spi_startDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * spi_forgetDevice --
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
spi_forgetDevice(vmk_Device device)
{
   return;
}