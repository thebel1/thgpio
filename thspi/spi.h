/******************************************************************************\
 * Native ESXi driver for the RPi's SPI interfaces.
 * 
 * Tom Hebel, 2020
\******************************************************************************/

/*
 * spi.h --
 */

#ifndef SPI_H
#define SPI_H

#define VMKAPI_ONLY
#include "vmkapi.h"

/***********************************************************************/

#define SPI_DEBUG

#define SPI_DRIVER_NAME "thspi"

// Probably overkill
#define SPI_HEAP_INITIAL_SIZE (1024 * 1024)
#define SPI_HEAP_MAX_SIZE (2 * 1024 * 1024)

#define SPI_MIN(a, b) (a > b ? b : a)
#define SPI_MAX(a, b) (a > b ? a : b)

#define SPI_INT_MAX ((vmk_uint32)~0)

#define SPI_LOG_ERR       1
#define SPI_LOG_WARN      2
#define SPI_LOG_NOTE      3
#define SPI_LOG_INIT      4
#define SPI_LOG_DISC      5
#define SPI_LOG_INFO      6
#define SPI_LOG_FUNC      7
#define SPI_LOG_TRACEIO   8

/***********************************************************************/

#endif /* SPI_H */