#!/bin/sh
make -j8 ARCH=arm CROSS_COMPILE=arm-cx2450x-linux-gnueabi- zImage
make -j8 ARCH=arm CROSS_COMPILE=arm-cx2450x-linux-gnueabi- modules
./mkimage -A arm -O linux -T kernel -C none -a  0x48000 -e  0x48000 -n "Coolstream HDx Kernel" -d arch/arm/boot/zImage uImage
