/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
drivers/kwimax_gpio_wrapper/kwimax_gpio_wrapper.h

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

#ifndef __KWIMAX_GPIO_WRAPPER__
#define __KWIMAX_GPIO_WRAPPER__

#include <linux/interrupt.h>

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
extern int kwimax_gpio_to_irq(unsigned gpio);

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
extern int kwimax_request_any_context_irq(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev_id);

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
int kwimax_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags, const char *name, void *dev_id);

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
extern int kwimax_enable_irq_wake(unsigned int irq);

#endif //__KWIMAX_GPIO_WRAPPER__

