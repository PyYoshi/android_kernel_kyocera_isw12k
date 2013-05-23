/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
drivers/kwimax_gpio_wrapper/kwimax_gpio_wrapper.c

DESCRIPTION
  Driver for WiMAX gpio wrapper.

EXTERNALIZED FUNCTIONS
  Operete gpio.

This software is contributed or developed by KYOCERA Corporation.
(C) 2011 KYOCERA Corporation
(C) 2012 KYOCERA Corporation
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/* This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>

#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include "kwimax_gpio_wrapper.h"

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KWiMAX gpio wrapper");

#ifdef FEATURE_KCC_WIMAX_DBG_MSG_GPIOWPR
	#define GPIO_WRAPPER_DEBUG(string, args...) \
		printk("%s: " string "\n", __FUNCTION__, ##args)
#else
	#define GPIO_WRAPPER_DEBUG(string, args...)
#endif

/*===========================================================================
 FUNCTION kwimax_gpio_to_irq

 DESCRIPTION
   Wrapper to __gpio_to_irq.

 DEPENDENCIES
   none

 PARAMETER
   gpio:  Gpio No.

 RETURN VALUE
   IRQ corresponding to a GPIO.

 SIDE EFFECTS
   none

===========================================================================*/
int kwimax_gpio_to_irq(unsigned gpio)
{
	int result=0;
	
	GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Function Start");

	result = __gpio_to_irq(gpio);
	
	GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Rrsult = %d", result);

	return result;
}

EXPORT_SYMBOL(kwimax_gpio_to_irq);

/*===========================================================================
 FUNCTION kwimax_request_any_context_irq

 DESCRIPTION
      Wrapper to request_any_context_irq.

 DEPENDENCIES
   none

 PARAMETER
   irq			Interrupt line to allocate.
   handler		Function to be called when the IRQ occurs.
   flags		Interrupt type flags.
   name			An ascii name for the claiming device
   dev_id		A cookie passed back to the handler function.

 RETURN VALUE
   On failure, it returns a negative value. On success,
   it returns either IRQC_IS_HARDIRQ or IRQC_IS_NESTED.

 SIDE EFFECTS
   none

===========================================================================*/
int kwimax_request_any_context_irq(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev_id)
{
	int result=0;
	
	GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Function Start");
	
	result = request_any_context_irq(irq, handler, flags, name, dev_id);
	
	GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Rrsult = %d", result);
	
	return result;
}

EXPORT_SYMBOL(kwimax_request_any_context_irq);

/*===========================================================================
 FUNCTION kwimax_request_irq

 DESCRIPTION
      Wrapper to request_irq.

 DEPENDENCIES
   none

 PARAMETER
   irq			Interrupt line to allocate.
   handler		Function to be called when the IRQ occurs.
   flags		Interrupt type flags.
   name			An ascii name for the claiming device
   dev_id		A cookie passed back to the handler function.

 RETURN VALUE
   On failure, it returns a negative value. On success,
   it returns either IRQC_IS_HARDIRQ or IRQC_IS_NESTED.

 SIDE EFFECTS
   none

===========================================================================*/
int kwimax_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev_id)
{
	int result=0;
	
	GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Function Start");
	
	result = request_irq(irq, handler, flags, name, dev_id);
	
	GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Rrsult = %d", result);
	
	return result;
}

EXPORT_SYMBOL(kwimax_request_irq);

/*===========================================================================
 FUNCTION kwimax_enable_irq_wake

 DESCRIPTION
   

 DEPENDENCIES
   none

 PARAMETER
   irq			Interrupt line to allocate.

 RETURN VALUE
   0:      Success

 SIDE EFFECTS
   none

===========================================================================*/
int kwimax_enable_irq_wake(unsigned int irq)
{
	GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Function Start");
	return enable_irq_wake(irq);
}

EXPORT_SYMBOL(kwimax_enable_irq_wake);

/*===========================================================================
 FUNCTION kwimax_gpio_wrapper_init

 DESCRIPTION
   Initialization. To register as a character device to the Kernel.

 DEPENDENCIES
   none

 PARAMETER
   none

 RETURN VALUE
   0:      Success
   Not 0:   Failed

 SIDE EFFECTS
   none

===========================================================================*/
static int32_t kwimax_gpio_wrapper_init( void )
{
    GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Function Start");

    return( 0 );
}

/*===========================================================================
 FUNCTION kwimax_gpio_wrapper_exit

 DESCRIPTION
   Shoutdown processing. Resource releaase.

 DEPENDENCIES
   none

 PARAMETER
   none

 RETURN VALUE
   0:      Success

 SIDE EFFECTS
   none

===========================================================================*/
static void kwimax_gpio_wrapper_exit( void )
{
    GPIO_WRAPPER_DEBUG("[GPIO_WRAPPER] Function Start");

}

module_init(kwimax_gpio_wrapper_init);
module_exit(kwimax_gpio_wrapper_exit);

