/*
 * This file contains driver for the Apple iPhone Timer Counter IP.
 *
 *  Copyright (C) 2013 Ramon Rebersak <fergy@idroidproject.org>
 *
 * based on arch/arm/mach-s5l8900/timer.c timer driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <asm/system.h>
#include <asm/mach-types.h>

#include <asm/irq.h>
#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <mach/iphone-clock.h>

// Constants
#define EventTimer 4
#define TicksPerSec 12000000

// Devices

#define TIMER IO_ADDRESS(0x3E200000)

// Registers

#define TIMER_0 0x0
#define TIMER_1 0x20
#define TIMER_2 0x40
#define TIMER_3 0x60
#define TIMER_4 0xA0
#define TIMER_5 0xC0
#define TIMER_6 0xE0
#define TIMER_CONFIG 0
#define TIMER_STATE 0x4
#define TIMER_COUNT_BUFFER 0x8
#define TIMER_COUNT_BUFFER2 0xC
#define TIMER_PRESCALER 0x10
#define TIMER_UNKNOWN3 0x14
#define TIMER_TICKSHIGH 0x80
#define TIMER_TICKSLOW 0x84
#define TIMER_UNKREG0 0x88
#define TIMER_UNKREG1 0x8C
#define TIMER_UNKREG2 0x90
#define TIMER_UNKREG3 0x94
#define TIMER_UNKREG4 0x98
#define TIMER_IRQSTAT 0x10000
#define TIMER_IRQLATCH 0xF8

// Timer
#define NUM_TIMERS 7
#define TIMER_CLOCKGATE 0x25
#define TIMER_IRQ 0x7
#define TIMER_STATE_START 1
#define TIMER_STATE_STOP 0
#define TIMER_STATE_MANUALUPDATE 2
#define TIMER_UNKREG0_RESET1 0xA
#define TIMER_UNKREG0_RESET2 0x18010
#define TIMER_UNKREG1_RESET 0xFFFFFFFF
#define TIMER_UNKREG2_RESET 0xFFFFFFFF
#define TIMER_UNKREG3_RESET 0xFFFFFFFF
#define TIMER_UNKREG4_RESET 0xFFFFFFFF
#define TIMER_DIVIDER1 4
#define TIMER_DIVIDER2 0
#define TIMER_DIVIDER4 1
#define TIMER_DIVIDER16 2
#define TIMER_DIVIDER64 3
#define TIMER_SPECIALTIMER_BIT0 0x1000000
#define TIMER_SPECIALTIMER_BIT1 0x2000000

#define TIMER_Separator 4

typedef void (*TimerHandler)(void);

typedef struct TimerRegisters {
	u32	config;
	u32	state;
	u32	count_buffer;
	u32	count_buffer2;
	u32	prescaler;
	u32	cur_count;
} TimerRegisters;

typedef struct TimerInfo {
	int	option6;
	u32	divider;
	u32	unknown1;
	TimerHandler	handler1;
	TimerHandler	handler2;
	TimerHandler	handler3;
} TimerInfo;

const TimerRegisters HWTimers[] = {
		{	TIMER + TIMER_0 + TIMER_CONFIG, TIMER + TIMER_0 + TIMER_STATE, TIMER + TIMER_0 + TIMER_COUNT_BUFFER,
			TIMER + TIMER_0 + TIMER_COUNT_BUFFER2, TIMER + TIMER_0 + TIMER_PRESCALER, TIMER + TIMER_0 + TIMER_UNKNOWN3 },
		{	TIMER + TIMER_1 + TIMER_CONFIG, TIMER + TIMER_1 + TIMER_STATE, TIMER + TIMER_1 + TIMER_COUNT_BUFFER,
			TIMER + TIMER_1 + TIMER_COUNT_BUFFER2, TIMER + TIMER_1 + TIMER_PRESCALER, TIMER + TIMER_1 + TIMER_UNKNOWN3 },
		{	TIMER + TIMER_2 + TIMER_CONFIG, TIMER + TIMER_2 + TIMER_STATE, TIMER + TIMER_2 + TIMER_COUNT_BUFFER,
			TIMER + TIMER_2 + TIMER_COUNT_BUFFER2, TIMER + TIMER_2 + TIMER_PRESCALER, TIMER + TIMER_2 + TIMER_UNKNOWN3 },
		{	TIMER + TIMER_3 + TIMER_CONFIG, TIMER + TIMER_3 + TIMER_STATE, TIMER + TIMER_3 + TIMER_COUNT_BUFFER,
			TIMER + TIMER_3 + TIMER_COUNT_BUFFER2, TIMER + TIMER_3 + TIMER_PRESCALER, TIMER + TIMER_3 + TIMER_UNKNOWN3 },
		{	TIMER + TIMER_4 + TIMER_CONFIG, TIMER + TIMER_4 + TIMER_STATE, TIMER + TIMER_4 + TIMER_COUNT_BUFFER,
			TIMER + TIMER_4 + TIMER_COUNT_BUFFER2, TIMER + TIMER_4 + TIMER_PRESCALER, TIMER + TIMER_4 + TIMER_UNKNOWN3 },
		{	TIMER + TIMER_5 + TIMER_CONFIG, TIMER + TIMER_5 + TIMER_STATE, TIMER + TIMER_5 + TIMER_COUNT_BUFFER,
			TIMER + TIMER_5 + TIMER_COUNT_BUFFER2, TIMER + TIMER_5 + TIMER_PRESCALER, TIMER + TIMER_5 + TIMER_UNKNOWN3 },
		{	TIMER + TIMER_6 + TIMER_CONFIG, TIMER + TIMER_6 + TIMER_STATE, TIMER + TIMER_6 + TIMER_COUNT_BUFFER,
			TIMER + TIMER_6 + TIMER_COUNT_BUFFER2, TIMER + TIMER_6 + TIMER_PRESCALER, TIMER + TIMER_6 + TIMER_UNKNOWN3 }
	};

TimerInfo Timers[7];

static void timer_init_rtc(void)
{
	__raw_writel(TIMER_UNKREG0_RESET1, TIMER + TIMER_UNKREG0);
	__raw_writel(TIMER_UNKREG2_RESET, TIMER + TIMER_UNKREG2);
	__raw_writel(TIMER_UNKREG1_RESET, TIMER + TIMER_UNKREG1);
	__raw_writel(TIMER_UNKREG4_RESET, TIMER + TIMER_UNKREG4);
	__raw_writel(TIMER_UNKREG3_RESET, TIMER + TIMER_UNKREG3);
	__raw_writel(TIMER_UNKREG0_RESET2, TIMER + TIMER_UNKREG0);
}

int timer_on_off(int timer_id, int on_off) {
	if(timer_id < NUM_TIMERS) {
		if(on_off == 1) {
			__raw_writel(TIMER_STATE_START, HWTimers[timer_id].state);
		} else {
			__raw_writel(TIMER_STATE_STOP, HWTimers[timer_id].state);
		}

		return 0;
	} else if(timer_id == NUM_TIMERS) {
		if(on_off == 1) {
			// clear bits 0, 1, 2, 3
			__raw_writel(__raw_readl(TIMER + TIMER_UNKREG0) & ~(0xF), TIMER + TIMER_UNKREG0);
		} else {
			// set bits 1, 3
			__raw_writel(__raw_readl(TIMER + TIMER_UNKREG0) | 0xA, TIMER + TIMER_UNKREG0);
		}
		return 0;
	} else {
		/* invalid timer id */
		return -1;
	}
}

static int timer_stop_all(void)
{
	int i;
	for(i = 0; i < NUM_TIMERS; i++) {
		timer_on_off(i, 0);
	}
	timer_on_off(NUM_TIMERS, 0);

	return 0;
}

static int timer_setup_clk(int timer_id, int type, int divider, u32 unknown1) {
	if(type == 2) {
		Timers[timer_id].option6 = 0;
		Timers[timer_id].divider = 6;
	} else {
		if(type == 1) {
			Timers[timer_id].option6 = 1;
		} else {
			Timers[timer_id].option6 = 0;
		}

		/* translate divider into divider code */
		switch(divider) {
			case 1:
				Timers[timer_id].divider = TIMER_DIVIDER1;
				break;
			case 2:
				Timers[timer_id].divider = TIMER_DIVIDER2;
				break;
			case 4:
				Timers[timer_id].divider = TIMER_DIVIDER4;
				break;
			case 16:
				Timers[timer_id].divider = TIMER_DIVIDER16;
				break;
			case 64:
				Timers[timer_id].divider = TIMER_DIVIDER64;
				break;
			default:
				/* invalid divider */
				return -1;
		}
	}

	Timers[timer_id].unknown1 = unknown1;

	return 0;
}

int timer_init(int timer_id, u32 interval, u32 interval2, u32 z, int option24, int option28, int option11) {
	u32 config;

	if(timer_id >= NUM_TIMERS || timer_id < 0) {
		return -1;
	}

	/* need to turn it off, since we're messing with the settings */
	timer_on_off(timer_id, 0);

//	if(interrupts)
		config = 0x7000; /* set bits 12, 13, 14 */
//	else
//		config = 0;

	/* these two options are only supported on timers 4, 5, 6 */
	if(timer_id >= TIMER_Separator) {
		config |= (option24 ? (1 << 24) : 0) | (option28 ? 1 << 28: 0);
	}

	/* set the rest of the options */
	config |= (Timers[timer_id].divider << 8)
			| (z << 3)
//			| (option5 ? (1 << 5) : 0)
			| (Timers[timer_id].option6 ? (1 << 6) : 0)
			| (option11 ? (1 << 11) : 0);

	__raw_writel(config, HWTimers[timer_id].config);
	__raw_writel(interval, HWTimers[timer_id].count_buffer);
	__raw_writel(interval2, HWTimers[timer_id].count_buffer2);
//	__raw_writel(interval2, HWTimers[timer_id].interval2);

	// apply the settings
	__raw_writel(TIMER_STATE_MANUALUPDATE, HWTimers[timer_id].state);

	return 0;
}

static void iphone_timer_get_rtc_ticks(u64* ticks) {
	register u32 ticksHigh;
	register u32 ticksLow;
	register u32 ticksHigh2;

	/* try to get a good read where the lower bits remain the same after reading the higher bits */
	do {
		ticksHigh = __raw_readl(TIMER + TIMER_TICKSHIGH);
		ticksLow = __raw_readl(TIMER + TIMER_TICKSLOW);
		ticksHigh2 = __raw_readl(TIMER + TIMER_TICKSHIGH);
	} while(ticksHigh != ticksHigh2);

	*ticks = (((u64)ticksHigh) << 32) | ticksLow;
}

u64 iphone_microtime(void) {
        u64 ticks;

        iphone_timer_get_rtc_ticks(&ticks);
	// FIXME: Unreliable for large tick values
        return ((u32)(ticks >> 2))/3;
}

int iphone_has_elapsed(u64 startTime, u64 elapsedTime) {
	if((iphone_microtime() - startTime) >= elapsedTime)
		return 1;
	else
		return 0;
}

static void callTimerHandler(int timer_id, uint32_t flags) {
	if((flags & (1 << 2)) != 0) {
		if(Timers[timer_id].handler1)
			Timers[timer_id].handler1();
	}

	if((flags & (1 << 1)) != 0) {
		if(Timers[timer_id].handler3)
			Timers[timer_id].handler3();
	}

	if((flags & (1 << 0)) != 0) {
		if(Timers[timer_id].handler2)
			Timers[timer_id].handler2();
	}
}

static irqreturn_t iphone_timer_interrupt(int irq, void* dev_id) {
	int i;
	/* this function does not implement incrementing a counter at dword_18022B28 like Apple's */
	uint32_t stat = __raw_readl(TIMER + TIMER_IRQSTAT);

	/* signal timer is being handled */
	volatile register uint32_t discard = __raw_readl(TIMER + TIMER_IRQLATCH); discard --;

	if(stat & TIMER_SPECIALTIMER_BIT0) {
		__raw_writel(__raw_readl(TIMER + TIMER_UNKREG0) | TIMER_SPECIALTIMER_BIT0, TIMER + TIMER_UNKREG0);
	}

	if(stat & TIMER_SPECIALTIMER_BIT1) {
		__raw_writel(__raw_readl(TIMER + TIMER_UNKREG0) | TIMER_SPECIALTIMER_BIT1, TIMER + TIMER_UNKREG0);
	}

	for(i = TIMER_Separator; i < NUM_TIMERS; i++) {
		callTimerHandler(i, stat >> (8 * (NUM_TIMERS - i - 1)));
	}

	/* signal timer has been handled */
	__raw_writel(stat, TIMER + TIMER_IRQLATCH);

	return IRQ_HANDLED;
}

static void iphone_timer_setup(void)
{
	/* stop/cleanup any existing timers */
	timer_stop_all();

	/* do some voodoo */
	timer_init_rtc();

	Timers[EventTimer].handler2 = timer_tick;
	// Initialize the timer hardware for something that goes off once every 100 Hz.
	// The handler for the timer will reset it so it's periodic
	timer_init(EventTimer, TicksPerSec/HZ, 0, 0, 0, 0, 0);

	timer_on_off(EventTimer, 1);
}

#define TIMER_USEC_SHIFT 16

/* timer_mask_usec_ticks
 *
 * given a clock and divisor, make the value to pass into timer_ticks_to_usec
 * to scale the ticks into usecs
*/

static inline unsigned long
timer_mask_usec_ticks(unsigned long scaler, unsigned long pclk)
{
	unsigned long den = pclk / 1000;

	return ((1000 << TIMER_USEC_SHIFT) * scaler + (den >> 1)) / den;
}

static unsigned long timer_usec_ticks;

/* timer_ticks_to_usec
 *
 * convert timer ticks to usec.
*/

static inline unsigned long timer_ticks_to_usec(unsigned long ticks)
{
	unsigned long res;

	res = ticks * timer_usec_ticks;
	res += 1 << (TIMER_USEC_SHIFT - 4);	/* round up slightly */

	return res >> TIMER_USEC_SHIFT;
}

static unsigned long iphone_gettimeoffset(void)
{
	unsigned long tdone;
	u32 stat;

	tdone = __raw_readl(HWTimers[EventTimer].cur_count);
	
	stat = __raw_readl(TIMER + TIMER_IRQSTAT) >> (8 * (NUM_TIMERS - EventTimer - 1));
	if((stat & (1 << 0)) != 0) {
		tdone = __raw_readl(HWTimers[EventTimer].cur_count);
		tdone += __raw_readl(HWTimers[EventTimer].count_buffer);
	}

	return timer_ticks_to_usec(tdone);
}

static struct irqaction iphone_timer_irq = {
	.name		= "iPhone Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= iphone_timer_interrupt,
};

static void __init iphone_timer_init(void)
{
	int i;

	printk("timer: initializing\n");

	for(i = 0; i < NUM_TIMERS; i++) {
		timer_setup_clk(i, 1, 2, 0);
	}

	timer_usec_ticks = timer_mask_usec_ticks(1, 12000000);
	iphone_timer_setup();
	setup_irq(TIMER_IRQ, &iphone_timer_irq);

	printk("timer: finished initialization\n");
}

struct sys_timer iphone_timer = {
	.init		= iphone_timer_init,
//	.offset		= iphone_gettimeoffset,
	.resume		= iphone_timer_setup
};

