/*
 *  arch/arm/mach-apple_iphone/include/mach/irqs.h
 *
 *  Copyright (C) 2008 Yiduo Wang
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <mach/platform.h>

#define IPHONE_NR_VIC_IRQS 0x40
#define IPHONE_NR_GPIO_IRQS (7 * 32)
#define IPHONE_GPIO_IRQS IPHONE_NR_VIC_IRQS
#define NR_IRQS	(IPHONE_NR_VIC_IRQS + IPHONE_NR_GPIO_IRQS)

#define S5L_PA_UART		(0x3CC00000)
#define S5L_PA_UART0		(S5L_PA_UART + 0x00)
#define S5L_PA_UART1		(S5L_PA_UART + 0x400)
#define S5L_PA_UART2		(S5L_PA_UART + 0x800)
#define S5L_PA_UART3		(S5L_PA_UART + 0xC00)
#define S5L_PA_UART4		(S5L_PA_UART + 0x1000)
#define S5L_SZ_UART		SZ_256
#define S5L_UART_OFFSET		(0x400)

/* See notes on UART VA mapping in debug-macro.S */
#define S5L_VA_UARTx(x)	(S5L_VA_UART + (S5L_PA_UART & 0xfffff) + ((x) * S5L_UART_OFFSET))

#define S5L_VA_UART0		S5L_VA_UARTx(0)
#define S5L_VA_UART1		S5L_VA_UARTx(1)
#define S5L_VA_UART2		S5L_VA_UARTx(2)
#define S5L_VA_UART3		S5L_VA_UARTx(3)
#define S5L_VA_UART4		S5L_VA_UARTx(4)

