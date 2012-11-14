#
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

USE_STANDARD_AS_RULE := true

# Object file lists.

obj-y			:= irq.o dma.o mach.o mipidle.o devices.o gpio.o time.o
obj-$(CONFIG_COMMON_CLKDEV) += clock.o
#obj-y			:= pllc.o
obj-$(CONFIG_PCI)	+= pci.o
#obj-$(CONFIG_PM)	+= pm.o
