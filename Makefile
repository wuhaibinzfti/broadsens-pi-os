PHONY := u-boot u-boot-distclean kernel busybox modules

PWD := $(shell pwd)
UBOOT_SRC_DIR = $(PWD)/u-boot/u-boot-2017.04
KERNEL_SRC_DIR = $(PWD)/kernel/linux-rpi-4.9.y-stable
BUSYBOX_SRC_DIR = $(PWD)/busybox/busybox-1.27.2
TOOLCHAIN_DIR = $(PWD)/tools/toolchain/arm-bcm2708/arm-linux-gnueabihf/bin
CROSS_COMPILE = $(TOOLCHAIN_DIR)/arm-linux-gnueabihf-
KERNEL_MOD_BUILD_DIR = $(PWD)/build/modules

BUSYBOX_INSTALL_DIR = $(PWD)/rootfs
KERNEL_DTC_TOOL = $(PWD)/kernel/linux-4.9/scripts/dtc/dtc
KERNEL_DTS_DIR = $(PWD)/kernel/linux-4.9/arch/arm/boot/dts/zynq-ax7010.dts

uboot:
	make -C $(UBOOT_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) zynq_ax7010_defconfig
	make -C $(UBOOT_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)
	cp $(UBOOT_SRC_DIR)/u-boot $(PWD)/u-boot.elf

uboot-menuconfig:
	make -C $(UBOOT_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) menuconfig
	
uboot-distclean:
	make -C $(UBOOT_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) distclean
	rm $(PWD)/u-boot.elf -rf

kernel-config:
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 bcm2709_defconfig
	
kernel:
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 zImage -j4
	rm ./kernel7.img -rf
	$(KERNEL_SRC_DIR)/scripts/mkknlimg $(KERNEL_SRC_DIR)/arch/arm/boot/zImage ./kernel7.img
	
kernel-distclean:
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 distclean

kernel-menuconfig:
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 menuconfig

kernel-dtb:
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 dtbs
	
modules:
	rm $(KERNEL_MOD_BUILD_DIR) -rf
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 modules -j4
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 modules_install INSTALL_MOD_PATH=$(KERNEL_MOD_BUILD_DIR)
	
busybox-config:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) bcm2709_defconfig

busybox-menuconfig:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) menuconfig
	
busybox:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)

busybox-install:
	rm -rf $(BUSYBOX_INSTALL_DIR)
	mkdir $(BUSYBOX_INSTALL_DIR)
	make -C $(BUSYBOX_SRC_DIR) install ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) CONFIG_PREFIX=$(BUSYBOX_INSTALL_DIR)

busybox-distclean:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) distclean

help:
	@echo  ''
	@echo  '  ------------ XILINX ------------'
	@echo  '  uboot            - Build for u-boot'
	@echo  '  uboot-distclean  - Distclean for u-boot'
	@echo  '  BOOT             - BOOT image'
	@echo  '  BOOT-clean       - BOOT image clean'
	@echo  '  kernel           - Build for kernel'
	@echo  ''
	
.PHONY: $(PHONY)
