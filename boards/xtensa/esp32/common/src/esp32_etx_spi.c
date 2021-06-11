/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_etx_spi.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include "esp32-devkitc.h"
#include "esp32_spi.h"
#include <nuttx/spi/spi_transfer.h>

#ifdef CONFIG_ESP32_ETX_SPI

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_SPI2

#define ETX_SPI_2   ( 2 )

struct spi_dev_s *etx_spi2;

#endif

#ifdef CONFIG_ESP32_SPI3

#define ETX_SPI_3   ( 3 )

struct spi_dev_s *etx_spi3;

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etx_spi_driver_init
 *
 * Description:
 *   Initialize the EmbeTronicX SPI device Driver. 
 *   This will create the device file as "/dev/spiX"
 *			X = 2 or 3
 *
 * 
 *   return negative number on failure.
 *   return 0 on success.
 *
 ****************************************************************************/

int etx_spi_driver_init( void )
{
  int ret = 0;
  
  // do while( false ) loop to break if any error.
  do
  {
#ifdef CONFIG_ESP32_SPI2

    // Initialize the SPI2 bus and register the driver
    etx_spi2 = esp32_spibus_initialize( ETX_SPI_2 );
    if ( !etx_spi2 )
    {
      ret =  -ENODEV;
      _err("ERROR: esp32_spibus_initialize() failed : SPI%d\n", ETX_SPI_2);
      break;
    }
    
    ret = spi_register( etx_spi2, ETX_SPI_2 );
    if (ret < 0)
    {
      _err("ERROR: spi_register() failed: SPI%d - %d\n", ETX_SPI_2, ret);
      break;
    }
#endif

#ifdef CONFIG_ESP32_SPI3
    // Initialize the SPI3 bus and register the driver
    etx_spi3 = esp32_spibus_initialize( ETX_SPI_3 );
    if ( !etx_spi3 )
    {
      ret =  -ENODEV;
      _err("ERROR: esp32_spibus_initialize() failed : SPI%d\n", ETX_SPI_3);
      break;
    }
    
    ret = spi_register( etx_spi2, ETX_SPI_3 );
    if (ret < 0)
    {
      _err("ERROR: spi_register() failed: SPI%d - %d\n", ETX_SPI_3, ret);
      break;
    }
#endif

  } while( false );
  
  return ( ret );
}

#endif /* CONFIG_ESP32_ETX_SPI */
