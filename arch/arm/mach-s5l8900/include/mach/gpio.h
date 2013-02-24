#ifndef IPHONE_HW_GPIO_H
#define IPHONE_HW_GPIO_H

#include <linux/interrupt.h>
#include <mach/hardware.h>

// Device
#define GPIOIC IO_ADDRESS(0x39A00000)	/* probably a part of the system controller */
#define GPIO IO_ADDRESS(0x3E400000)

// Registers
#define GPIO_INTLEVEL 0x80
#define GPIO_INTSTAT 0xA0
#define GPIO_INTEN 0xC0
#define GPIO_INTTYPE 0xE0
#define GPIO_FSEL 0x320

#define GPIO_IO 0x320

// Values
#define GPIO_NUMINTGROUPS 7
#define GPIO_INTSTAT_RESET 0xFFFFFFFF
#define GPIO_INTEN_RESET 0

#define GPIO_IO_MAJSHIFT 16
#define GPIO_IO_MAJMASK 0x1F
#define GPIO_IO_MINSHIFT 8
#define GPIO_IO_MINMASK 0x7
#define GPIO_IO_USHIFT 0
#define GPIO_IO_UMASK 0xF

#define GPIO_CLOCKGATE 0x2C

#define GPIO_DETECT1 0xE04
#define GPIO_DETECT2 0xE05
#define GPIO_DETECT3 0xE06

#define GPIO_BUTTONS_HOME_IPHONE 0x1600
#define GPIO_BUTTONS_HOME_IPOD 0x1606
//#define GPIO_BUTTONS_HOME_IRQ_IPOD 0x2E //WTF IS THIS??

#ifdef CONFIG_IPODTOUCH_1G
#define GPIO_BUTTONS_HOME GPIO_BUTTONS_HOME_IPOD
#define GPIO_BUTTONS_HOME_IRQ 0x2E
#else
#define GPIO_BUTTONS_HOME GPIO_BUTTONS_HOME_IPHONE
#define GPIO_BUTTONS_HOME_IRQ 0x28
#endif

#define GPIO_BUTTONS_HOLD 0x1605
#define GPIO_BUTTONS_HOLD_IRQ 0x2D

#ifndef CONFIG_IPODTOUCH_1G
#define GPIO_BUTTONS_VOLUP 0x1601
#define GPIO_BUTTONS_VOLUP_IRQ 0x29

#define GPIO_BUTTONS_VOLDOWN 0x1602
#define GPIO_BUTTONS_VOLDOWN_IRQ 0x2A

#define GPIO_BUTTONS_RINGERAB 0x1603
#define GPIO_BUTTONS_RINGERAB_IRQ 0x2B
#endif

typedef struct GPIORegisters {
	volatile uint32_t CON;
	volatile uint32_t DAT;
	volatile uint32_t PUD;
	volatile uint32_t CONSLP;
	volatile uint32_t PUDSLP;
	volatile uint32_t unused1;
	volatile uint32_t unused2;
	volatile uint32_t unused3;
} GPIORegisters;

int iphone_gpio_pin_state(int port);
void iphone_gpio_custom_io(int port, int bits);
void iphone_gpio_pin_reset(int port);
void iphone_gpio_pin_output(int port, int bit);
int iphone_gpio_detect_configuration(void);
int gpio_to_irq(unsigned gpio);
int irq_to_gpio(unsigned irq);

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{
}

static inline int gpio_get_value(unsigned gpio)
{
	return iphone_gpio_pin_state(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	iphone_gpio_pin_output(gpio, value);
}

static inline int gpio_cansleep(unsigned gpio)
{
	return 0;
}

static inline int gpio_get_value_cansleep(unsigned gpio)
{
	might_sleep();
	return gpio_get_value(gpio);
}

static inline void gpio_set_value_cansleep(unsigned gpio, int value)
{
	might_sleep();
	gpio_set_value(gpio, value);
}

static inline int gpio_is_valid(int number)
{
	if(number > 0 && number < 0x1808)
		return 1;
	else
		return 0;
}

static inline int gpio_direction_input(unsigned gpio)
{
	iphone_gpio_pin_reset(gpio);
	return 0;
}

static inline int gpio_direction_output(unsigned gpio, int value)
{
	iphone_gpio_pin_output(gpio, value);
	return 0;
}

#endif

