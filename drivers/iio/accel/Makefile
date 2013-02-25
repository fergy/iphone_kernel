#
# Makefile for industrial I/O accelerometer drivers
#

obj-$(CONFIG_HID_SENSOR_ACCEL_3D) += hid-sensor-accel-3d.o

lis331dlh-y			:= lis331dlh_core.o
lis331dlh-$(CONFIG_IIO_BUFFER)	+= lis331dlh_buffer.o
obj-$(CONFIG_LIS331DLH_SPI)	+= lis331dlh.o lis331dlh_spi.o
obj-$(CONFIG_LIS331DLH_I2C)	+= lis331dlh.o lis331dlh_i2c.o
