/*
 *  arch/arm/mach-apple_iphone/include/mach/uart.h
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

#ifndef _IPHONE_MACH_UART_H
#define _IPHONE_MACH_UART_H

// Devices
#define UART IO_ADDRESS(0x3CC00000)

// Registers
#define UART0 0x0
#define UART1 0x4000
#define UART2 0x8000
#define UART3 0xC000
#define UART4 0x10000

#define UART_ULCON 0x0
#define UART_UCON 0x4
#define UART_UFCON 0x8
#define UART_UMCON 0xC
#define UART_UTRSTAT 0x10
#define UART_UERSTAT 0x14
#define UART_UFSTAT 0x18
#define UART_UMSTAT 0x1C
#define UART_UTXH 0x20
#define UART_URXH 0x24
#define UART_UBAUD 0x28
#define UART_UINTP 0x2C

// Values
#define NUM_UARTS 5
#define UART_CLOCKGATE 0x29

#define UART_CLOCK_SELECTION_MASK (0x3 << 10) // Bit 10-11
#define UART_CLOCK_SELECTION_SHIFT 10 // Bit 10-11
#define UART_UCON_UNKMASK 0x7000
#define UART_UCON_LOOPBACKMODE (0x1 << 5)
#define UART_UCON_RXMODE_SHIFT 0
#define UART_UCON_RXMODE_MASK (0x3 << UART_UCON_RXMODE_SHIFT)
#define UART_UCON_TXMODE_SHIFT 2
#define UART_UCON_TXMODE_MASK (0x3 << UART_UCON_TXMODE_SHIFT)

#define UART_FIFO_RESET_TX 0x4
#define UART_FIFO_RESET_RX 0x2
#define UART_FIFO_ENABLE 0x1

#define UART_DIVVAL_MASK 0x0000FFFF
#define UART_SAMPLERATE_MASK 0x00030000 // Bit 16-17
#define UART_SAMPLERATE_SHIFT 16

#define UART_UCON_MODE_DISABLE 0
#define UART_UCON_MODE_IRQORPOLL 1
#define UART_UCON_MODE_DMA0 2
#define UART_UCON_MODE_DMA1 3

#define UART_CLOCK_PCLK 0
#define UART_CLOCK_EXT_UCLK0 1
#define UART_CLOCK_EXT_UCLK1 3

#define UART_SAMPLERATE_4 2
#define UART_SAMPLERATE_8 1
#define UART_SAMPLERATE_16 0

#define UART_UMCON_AFC_BIT 0x10
#define UART_UMCON_NRTS_BIT 0x1

#define UART_UTRSTAT_TRANSMITTEREMPTY 0x4
#define UART_UTRSTAT_RECEIVEDATAREADY 0x1

#define UART_UMSTAT_CTS 0x1

#define UART_UFSTAT_TXFIFO_FULL (0x1 << 9)
#define UART_UFSTAT_RXFIFO_FULL (0x1 << 8)
#define UART_UFSTAT_RXCOUNT_MASK 0xF

#define UART_5BITS 0
#define UART_6BITS 1
#define UART_7BITS 2
#define UART_8BITS 3

#endif

