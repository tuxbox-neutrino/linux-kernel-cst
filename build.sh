#!/bin/sh
make -j4 ARCH=arm CROSS_COMPILE=arm-cx2450x-linux-gnueabi- zImage
make -j4 ARCH=arm CROSS_COMPILE=arm-cx2450x-linux-gnueabi- modules
./mkimage -A arm -O linux -T kernel -C none -a 0x17048000 -e 0x17048000 -n "Coolstream HDx Kernel" -d arch/arm/boot/zImage uImage
