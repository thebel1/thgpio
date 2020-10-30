/******************************************************************************\
 * Native ESXi driver for the RPi's SPI interfaces.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * spi_types.h --
 *
 *    Type definitions
 */

#ifndef SPI_TYPES_H
#define SPI_TYPES_H

/***********************************************************************/

typedef struct spi_Driver_t {
   vmk_Name driverName;
   vmk_ModuleID moduleID;
   vmk_HeapID heapID;
   vmk_Driver driverHandle;
   vmk_IOResource resHandle;
   vmk_LogComponent logger;
} spi_Driver_t;

typedef struct spi_Device_t {
   /* Object */
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
} spi_Device_t;

/***********************************************************************/

#endif /* SPI_TYPES_H */