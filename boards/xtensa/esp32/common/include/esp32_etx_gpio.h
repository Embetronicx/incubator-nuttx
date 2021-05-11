/****************************************************************************
 * boards/xtensa/esp32/common/include/esp32_etx_gpio.h
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

#ifndef __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ETX_GPIO_H
#define __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ETX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#define GPIOC_REGISTER   _GPIOC(1)              // IOCTL command to register
#define GPIOC_UNREGISTER _GPIOC(2)              // IOCTL command to unregister

#define ETX_GPIO_SIGNO   SIGUSR1                // Signal number 

typedef enum
{
  ETX_GPIO_IN,        // GPIO as input
  ETX_GPIO_OUT,       // GPIO as ouput
  ETX_GPIO_IN_INT     // GPIO as input and enable the interrupt
} GPIO_TYPE;

typedef struct
{
  GPIO_TYPE gpio_type;    // GPIO type
  uint8_t   gpio_num;     // GPIO number
  uint8_t  *gpio_value;   // GPIO value
  void     *data;         // Data
}etx_gpio;

/****************************************************************************
 * Name: etx_gpio_driver_init
 *
 * Description:
 *   Initialize the EmbeTronicX gpio device Driver. 
 *   This will create the device file as "/dev/etx_gpio"
 *
 *   return negative number on failure.
 *   return 0 on success.
 * 
 ****************************************************************************/

#ifdef CONFIG_ESP32_ETX_GPIO
int etx_gpio_driver_init( void );
#endif



#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ETX_GPIO_H */
