/*
 * lis331dlh.h		support STMicroelectronics LIS331DLH
 *			3d 2/4/8g Linear Accelerometers
 *
 * Copyright (c) 2007 Jonathan Cameron <jic23@xxxxxxxxx>
 * Copyright (c) 2013 Manuel Stahl <manuel.stahl@xxxxxxxxxxxxxxxxx>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef SPI_LIS331DLH_H_
#define SPI_LIS331DLH_H_

#include <asm/byteorder.h>
#include <linux/iio/iio.h>

#define LIS331DLH "lis331dlh"

#define LIS331DLH_READ_REG(a) ((a) | 0x80)
#define LIS331DLH_WRITE_REG(a) a
#define LIS331DLH_I2C_AUTO_INC(a) ((a) | 0x80)
#define LIS331DLH_SPI_AUTO_INC(a) ((a) | 0x40)

#define LIS331DLH_REG_WHO_AM_I_ADDR			0x0F
#define LIS331DLH_REG_WHO_AM_I_DEFAULT			0x32

/* Control Register (1 of 5) */
#define LIS331DLH_REG_CTRL_1_ADDR			0x20
/* Power ctrl - either bit set corresponds to on*/
#define LIS331DLH_REG_CTRL_1_POWER_OFF			0x00
#define LIS331DLH_REG_CTRL_1_POWER_ON			0x20
#define LIS331DLH_REG_CTRL_1_LOW_POWER_05		0x40

/* Data Rate */
#define LIS331DLH_REG_CTRL_1_DR_50			0x00
#define LIS331DLH_REG_CTRL_1_DR_100			0x08
#define LIS331DLH_REG_CTRL_1_DR_400			0x10
#define LIS331DLH_REG_CTRL_1_DR_1000			0x18
#define LIS331DLH_REG_CTRL_1_DR_MASK			0x18

/* Axes enable ctrls */
#define LIS331DLH_REG_CTRL_1_AXES_Z_ENABLE		0x04
#define LIS331DLH_REG_CTRL_1_AXES_Y_ENABLE		0x02
#define LIS331DLH_REG_CTRL_1_AXES_X_ENABLE		0x01

/* Control Register (2 of 5) */
#define LIS331DLH_REG_CTRL_2_ADDR			0x21

#define LIS331DLH_REG_CTRL_2_BOOT			0x80

#define LIS331DLH_REG_CTRL_2_HP_NORM			0x00
#define LIS331DLH_REG_CTRL_2_HP_REF			0x40

#define LIS331DLH_REG_CTRL_2_FDS_INT			0x10
#define LIS331DLH_REG_CTRL_2_HPEN2			0x08
#define LIS331DLH_REG_CTRL_2_HPEN1			0x04
#define LIS331DLH_REG_CTRL_2_HPCF1			0x02
#define LIS331DLH_REG_CTRL_2_HPCF0			0x01

/* Control Register (3 of 5) */
#define LIS331DLH_REG_CTRL_3_ADDR			0x22

#define LIS331DLH_REG_CTRL_3_IHL			0x80
#define LIS331DLH_REG_CTRL_3_PP_OD			0x40
#define LIS331DLH_REG_CTRL_3_LIR2			0x20
#define LIS331DLH_REG_CTRL_3_I2_I			0x00
#define LIS331DLH_REG_CTRL_3_I2_1OR2			0x08
#define LIS331DLH_REG_CTRL_3_I2_DRDY			0x10
#define LIS331DLH_REG_CTRL_3_I2_RUN			0x18
#define LIS331DLH_REG_CTRL_3_I2_MASK			0x18
#define LIS331DLH_REG_CTRL_3_LIR1			0x04
#define LIS331DLH_REG_CTRL_3_I1_I			0x00
#define LIS331DLH_REG_CTRL_3_I1_1OR2			0x01
#define LIS331DLH_REG_CTRL_3_I1_DRDY			0x02
#define LIS331DLH_REG_CTRL_3_I1_RUN			0x03
#define LIS331DLH_REG_CTRL_3_I1_MASK			0x03

/* Control Register (4 of 5) */
#define LIS331DLH_REG_CTRL_4_ADDR			0x23

/* Block Data Update only after MSB and LSB read */
#define LIS331DLH_REG_CTRL_4_BLOCK_UPDATE		0x80

/* Set to big endian output */
#define LIS331DLH_REG_CTRL_4_BIG_ENDIAN			0x40

/* Full scale selection */
#define LIS331DLH_REG_CTRL_4_FS_2G			0x00
#define LIS331DLH_REG_CTRL_4_FS_4G			0x10
#define LIS331DLH_REG_CTRL_4_FS_8G			0x30
#define LIS331DLH_REG_CTRL_4_FS_MASK			0x30

/* Self Test Sign */
#define LIS331DLH_REG_CTRL_4_ST_SIGN			0x08

/* Self Test Enable */
#define LIS331DLH_REG_CTRL_4_ST_ON			0x02

/* SPI 3 wire mode */
#define LIS331DLH_REG_CTRL_4_THREE_WIRE_SPI_MODE	0x01

/* Control Register (5 of 5) */
#define LIS331DLH_REG_CTRL_5_ADDR			0x24

#define LIS331DLH_REG_CTRL_5_SLEEP_WAKE_OFF		0x00
#define LIS331DLH_REG_CTRL_5_LOW_POWER			0x03

/* Dummy register */
#define LIS331DLH_REG_HP_FILTER_RESET			0x25

/* Reference register */
#define LIS331DLH_REG_REFERENCE				0x26

/* Status register */
#define LIS331DLH_REG_STATUS_ADDR			0x27
/* XYZ axis data overrun - first is all overrun? */
#define LIS331DLH_REG_STATUS_XYZ_OVERRUN		0x80
#define LIS331DLH_REG_STATUS_Z_OVERRUN			0x40
#define LIS331DLH_REG_STATUS_Y_OVERRUN			0x20
#define LIS331DLH_REG_STATUS_X_OVERRUN			0x10
/* XYZ new data available - first is all 3 available? */
#define LIS331DLH_REG_STATUS_XYZ_NEW_DATA		0x08
#define LIS331DLH_REG_STATUS_Z_NEW_DATA			0x04
#define LIS331DLH_REG_STATUS_Y_NEW_DATA			0x02
#define LIS331DLH_REG_STATUS_X_NEW_DATA			0x01

/* The accelerometer readings - low and high bytes.
Form of high byte dependant on justification set in ctrl reg */
#define LIS331DLH_REG_OUT_X_L_ADDR			0x28
#define LIS331DLH_REG_OUT_X_H_ADDR			0x29
#define LIS331DLH_REG_OUT_Y_L_ADDR			0x2A
#define LIS331DLH_REG_OUT_Y_H_ADDR			0x2B
#define LIS331DLH_REG_OUT_Z_L_ADDR			0x2C
#define LIS331DLH_REG_OUT_Z_H_ADDR			0x2D

/* Interrupt 1 config register */
#define LIS331DLH_REG_INT1_CFG_ADDR			0x30
#define LIS331DLH_REG_INT1_SRC_ADDR			0x31
#define LIS331DLH_REG_INT1_THS_ADDR			0x32
#define LIS331DLH_REG_INT1_DURATION_ADDR		0x33

/* Interrupt 2 config register */
#define LIS331DLH_REG_INT2_CFG_ADDR			0x34
#define LIS331DLH_REG_INT2_SRC_ADDR			0x35
#define LIS331DLH_REG_INT2_THS_ADDR			0x36
#define LIS331DLH_REG_INT2_DURATION_ADDR		0x37

#define LIS331DLH_REG_INT_AOI				0x80
#define LIS331DLH_REG_INT_6D				0x40
#define LIS331DLH_REG_INT_Z_HIGH			0x20
#define LIS331DLH_REG_INT_Z_LOW				0x10
#define LIS331DLH_REG_INT_Y_HIGH			0x08
#define LIS331DLH_REG_INT_Y_LOW				0x04
#define LIS331DLH_REG_INT_X_HIGH			0x02
#define LIS331DLH_REG_INT_X_LOW				0x01


/* Default control settings */
#define LIS331DLH_DEFAULT_CTRL1  (LIS331DLH_REG_CTRL_1_POWER_ON	     \
				| LIS331DLH_REG_CTRL_1_DR_50         \
				| LIS331DLH_REG_CTRL_1_AXES_Z_ENABLE \
				| LIS331DLH_REG_CTRL_1_AXES_Y_ENABLE \
				| LIS331DLH_REG_CTRL_1_AXES_X_ENABLE)

#define LIS331DLH_DEFAULT_CTRL2	 LIS331DLH_REG_CTRL_2_HP_NORM

#define LIS331DLH_DEFAULT_CTRL3  0

#ifdef __BIG_ENDIAN
#define LIS331DLH_REG_CTRL_4_CPU_ENDIAN LIS331DLH_REG_CTRL_4_BIG_ENDIAN
#else
#define LIS331DLH_REG_CTRL_4_CPU_ENDIAN 0
#endif

#define LIS331DLH_DEFAULT_CTRL4  (LIS331DLH_REG_CTRL_4_BLOCK_UPDATE \
				| LIS331DLH_REG_CTRL_4_CPU_ENDIAN \
				| LIS331DLH_REG_CTRL_4_FS_2G)

#define LIS331DLH_MAX_RX 6

/**
 * struct lis331dlh - device instance specific data
 * @us:			pointer to actual bus (spi/i2c)
 * @trig:		data ready trigger registered with iio
 * @irq:		irq line
 * @read/write		bus specific functions to access registers
 **/
struct lis331dlh {
	void			*us;
	struct iio_trigger	*trig;
	int			irq;

	int (*read_reg_8)  (struct lis331dlh *st, u8 addr, u8 *val);
	int (*write_reg_8) (struct lis331dlh *st, u8 addr, u8 val);
	int (*read_reg_16) (struct lis331dlh *st, u8 low_addr, u16 *val);
	int (*read_all)    (struct lis331dlh *st, s16 *buf);
};

enum LIS331DLH_SCAN_INDEX {
	LIS331DLH_SCAN_ACCEL_X,
	LIS331DLH_SCAN_ACCEL_Y,
	LIS331DLH_SCAN_ACCEL_Z,
	LIS331DLH_SCAN_ELEMENTS,
};

int lis331dlh_probe(struct iio_dev *indio_dev);
int lis331dlh_remove(struct iio_dev *indio_dev);

int lis331dlh_stop_device(struct iio_dev *indio_dev);
int lis331dlh_set_irq_data_rdy(struct iio_dev *indio_dev, bool enable);

#ifdef CONFIG_IIO_BUFFER

void lis331dlh_remove_trigger(struct iio_dev *indio_dev);
int lis331dlh_probe_trigger(struct iio_dev *indio_dev);

int lis331dlh_buffer_configure(struct iio_dev *indio_dev);
void lis331dlh_buffer_unconfigure(struct iio_dev *indio_dev);

#else /* CONFIG_IIO_BUFFER */

static inline void lis331dlh_remove_trigger(struct iio_dev *indio_dev)
{
}
static inline int lis331dlh_probe_trigger(struct iio_dev *indio_dev)
{
	return 0;
}

static inline int lis331dlh_buffer_configure(struct iio_dev *indio_dev)
{
	return 0;
}
static inline void lis331dlh_buffer_unconfigure(struct iio_dev *indio_dev)
{
}

#endif /* CONFIG_IIO_BUFFER */

#endif /* SPI_LIS331DLH_H_ */
