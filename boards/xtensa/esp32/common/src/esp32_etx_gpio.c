/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_etx_gpio.c
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
#include <assert.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <nuttx/signal.h>
#include "esp32-devkitc.h"
#include "esp32_gpio.h"
#include "esp32_etx_gpio.h"

#ifdef CONFIG_ESP32_ETX_GPIO

#define INVALID_GPIO 0xFF     //invalid GPIO

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     etx_gpio_open(FAR struct file *filep);
static int     etx_gpio_close(FAR struct file *filep);
static ssize_t etx_gpio_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
static ssize_t etx_gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t len);
static int etx_gpio_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sigevent event  = { 0 };
pid_t  pid;

static const struct file_operations etx_gpio_fops =
{
  etx_gpio_open,   /* open   */
  etx_gpio_close,  /* close  */
  etx_gpio_read,   /* read   */
  etx_gpio_write,  /* write  */
  NULL,            /* seek   */
  etx_gpio_ioctl,  /* ioctl  */
  NULL,            /* poll   */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL             /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
 /****************************************************************************
 * Name: etx_gpio_ISR
 *
 * Description:
 *   Interrupt Service Routine.
 ****************************************************************************/

static int etx_gpio_ISR(int irq, void *context, void *arg)
{
  if ( pid )
  {
    struct sigevent signal;
    
    signal.sigev_notify = SIGEV_SIGNAL;
    signal.sigev_signo  = ETX_GPIO_SIGNO;
    signal.sigev_value.sival_int = 1;
    
    /* send signal to the application */
    nxsig_notification(pid, &signal, SI_QUEUE, NULL);
  }
  
  return OK;
}

/****************************************************************************
 * Name: etx_gpio_open
 *
 * Description:
 *   Standard character driver open method.
 *
 *	This function will be getting called when the application
 *	opens the "/dev/etx_gpio" file.
 ****************************************************************************/

static int etx_gpio_open(FAR struct file *filep)
{
  int ret = 0;

  return ( ret );
}

/****************************************************************************
 * Name: etx_gpio_close
 *
 * Description:
 *   Standard character driver close method.
 *
 *	This function will be getting called when the application
 *	closess the "/dev/etx_gpio" file.
 ****************************************************************************/

static int etx_gpio_close(FAR struct file *filep)
{
  int ret = 0;

  return ( ret );
}

/****************************************************************************
 * Name: etx_gpio_write
 *
 * Description:
 *   Standard character driver write method.
 *
 *	This function will be getting called when the application
 *	writes data the "/dev/etx_gpio" file.
 ****************************************************************************/

static ssize_t etx_gpio_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  DEBUGASSERT(buffer != NULL);
  
  etx_gpio *gpio = (FAR etx_gpio *)((uintptr_t)buffer);
  
  if( gpio->gpio_num < 0 || gpio->gpio_num > ESP32_NGPIOS )
  {
    syslog(LOG_ERR, "Write: Invalid GPIO Number - %d\n", gpio->gpio_num);
    return ( len );
  }
  
  // Write to the GPIO
  esp32_gpiowrite( gpio->gpio_num, *gpio->gpio_value );
    
  return ( len );
}

/****************************************************************************
 * Name: etx_gpio_read
 *
 * Description:
 *  Standard character driver read method.
 *
 *	This function will be getting called when the application
 *	reads data from the "/dev/etx_gpio" file.
 *
 ****************************************************************************/

static ssize_t etx_gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t len)
{
  int ret = 0;
  
  DEBUGASSERT(buffer != NULL);

  etx_gpio *gpio = (FAR etx_gpio *)( (uintptr_t)buffer );
  
  if( gpio->gpio_num < 0 || gpio->gpio_num > ESP32_NGPIOS )
  {
    syslog(LOG_ERR, "Read: Invalid GPIO Number - %d\n", gpio->gpio_num);
    return ( ret );
  }
   
  *gpio->gpio_value  = esp32_gpioread( gpio->gpio_num );
    
  return ( ret );
}

/****************************************************************************
 * Name: etx_gpio_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 *	This function will be getting called when the application
 *	do IOCTL call to the "/dev/etx_gpio" file.
 *
 ****************************************************************************/

static int etx_gpio_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = 0;
  etx_gpio gpio;
  
  DEBUGASSERT( (FAR void *)arg != NULL );
  
  //copy the GPIO struct
  memcpy( &gpio, (FAR void *)arg, sizeof(etx_gpio) );
  
  if( gpio.gpio_num < 0 || gpio.gpio_num > ESP32_NGPIOS )
  {
    syslog(LOG_ERR, "IOCTL: Invalid GPIO Number - %d\n", gpio.gpio_num);
    return ( -1 );
  }
  
  switch( cmd )
  {    
    case GPIOC_REGISTER:
    {      
      if( gpio.gpio_type == ETX_GPIO_IN )
      {
        // Configure the GPIO as output
        esp32_configgpio( gpio.gpio_num , INPUT );
      }
      else if( gpio.gpio_type == ETX_GPIO_OUT )
      {
        // Configure the GPIO as output
        esp32_configgpio( gpio.gpio_num , OUTPUT );
      }
      else if( gpio.gpio_type == ETX_GPIO_IN_INT )
      {
      	pid = getpid();
      	
        // Configure the GPIO as interrupt pin
        int irq = ESP32_PIN2IRQ(gpio.gpio_num);
        esp32_configgpio( gpio.gpio_num , INPUT_FUNCTION_3 | PULLUP );
        ret = irq_attach(irq, etx_gpio_ISR, NULL);
        if( ret >= 0 )
        {
          esp32_gpioirqenable(irq, ONHIGH);
        }
      }
      else
      {
        // Invalid operation
        ret = -1;
      }
      
    }
    break;
    
    case GPIOC_UNREGISTER:
    {
      /* Unregister the GPIO */
      if( gpio.gpio_type == ETX_GPIO_IN_INT )
      {
        // unregister the interrupt pin
        int irq = ESP32_PIN2IRQ(gpio.gpio_num);
        esp32_gpioirqdisable(irq);
        pid = 0;
      }
    }
    break;
    
    default:
      ret = -1;
      break;
  }

  return ( ret );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: etx_gpio_driver_init
 *
 * Description:
 *   Initialize the EmbeTronicX GPIO device Driver. 
 *   This will create the device file as "/dev/etx_gpio"
 * 
 *   return negative number on failure.
 *   return 0 on success.
 *
 ****************************************************************************/

int etx_gpio_driver_init( void )
{
  int ret = 0;
  ret = register_driver("/dev/etx_gpio", &etx_gpio_fops, 0666, NULL);
  if (ret < 0)
  {
    _err("ERROR: register_driver failed : /dev/etx_gpio : %d\n", ret);
    ret = -1;;
  }

  return ( ret );

}

#endif /* CONFIG_ESP32_ETX_GPIO */
