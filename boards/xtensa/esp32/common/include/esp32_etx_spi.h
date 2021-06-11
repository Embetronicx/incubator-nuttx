/****************************************************************************
 * boards/xtensa/esp32/common/include/esp32_etx_spi.h
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

#ifndef __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ETX_SPI_H
#define __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ETX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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

#ifdef CONFIG_ESP32_ETX_SPI
int etx_spi_driver_init( void );
#endif



#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ETX_SPI_H */
