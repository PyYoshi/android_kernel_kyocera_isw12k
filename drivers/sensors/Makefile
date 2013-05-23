#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2012 KYOCERA Corporation
#

#
# Makefile for miscellaneous Sensors drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-y += apds990x.o
#obj-$(CONFIG_MACH_MSM8655_KC_APDS990X) += apds990x.o
obj-m                                          += alps-input.o hscd_i2c.o ml610q792.o

