#
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

USE_STANDARD_AS_RULE := true

# Object file lists.

obj-y			:= irq.o dma.o mach.o pllc.o mipidle.o devices.o gpio.o time.o
obj-m			:=
obj-n			:=
obj-			:=

#export-objs		:= mach.o

obj-$(CONFIG_PCI)	+= pci.o

#include $(TOPDIR)/Rules.make
