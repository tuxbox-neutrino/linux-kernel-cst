#
# Makefile for the i2c algorithms
#

HWAPI_FLAGS-y := -Iinclude/HwAPI

HWAPI_FLAGS-y += -Idrivers/i2c/algos/i2c_phlm/tmhwI2c/inc \
                 -Idrivers/i2c/algos/i2c_phlm/tmhwI2c/src \
                -Idrivers/i2c/algos/i2c_phlm/ItmhwI2c/inc \
                -Idrivers/i2c/algos/i2c_phlm/tmhwI2c/cfg
                
nx_i2c_flags-$(CONFIG_PLAT_STB) := -DMINUS_SPEED_PATCH

EXTRA_CFLAGS-$(CONFIG_I2C_ALGO3203) := $(HWAPI_FLAGS-y) -DTMFL_CPU_IS_MIPS=1 \
			$(nx_i2c_flags-y) -DTMHWI2C_BSL_INDEPENDANT -DTMFL_OS_IS_HPUNIX=0 \
			-DMIPSEL -DDMA_MOD -DLINUX_BUILD -DLINUX -DMONTAVISTA_GNU

EXTRA_CFLAGS = $(EXTRA_CFLAGS-y) $(EXTRA_CFLAGS-m)
obj-$(CONFIG_I2C_ALGO3203)  += i2c-phlm.o
i2c-phlm-y := tmhwI2c/cfg/tmhwI2c_DvpNoBslCfg.o tmhwI2c/src/tmhwI2c.o i2c_phlm.o i2c_phlm_cfg.o

ifeq ($(CONFIG_I2C_DEBUG_ALGO),y)
EXTRA_CFLAGS += -DDEBUG
endif
