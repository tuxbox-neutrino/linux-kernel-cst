# Makefile for the Linux IP_2070 NAND driver

_TMDIVERSITY=nobsl
HWAPI_FLAGS-y := -Iinclude/HwAPI
HWAPI_FLAGS-y +=  -Idrivers/mtd/nand/nx_nand/tmhwEfmc/cfg \
                  -Idrivers/mtd/nand/nx_nand/tmhwEfmc/inc \
                  -Idrivers/mtd/nand/nx_nand/tmhwEfmc/src \
                  -Iarch/mips/include/asm

nx_nand_flags-$(CONFIG_PLAT_STB)	:= -DMIPSEL

EXTRA_CFLAGS   := $(HWAPI_FLAGS-y)
EXTRA_CFLAGS += $(nx_nand_flags-y) -DLINUX_BUILD -DLINUX -DMONTAVISTA_GNU -DTMFL_OS_IS_HPUNIX=0

nand_nx-objs += $(nx_nand-y) nx_nand.o \
                    tmhwEfmc/src/tmhwEfmc.o \
                    tmhwEfmc/cfg/tmhwEfmc_Cfg.o

obj-$(CONFIG_MTD_NX_NAND_PART_PARSE) += nx_part.o
obj-$(CONFIG_MTD_NX_NAND) += nand_nx.o
