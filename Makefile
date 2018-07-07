PHONY := u-boot u-boot-distclean kernel busybox modules rootfs

PWD := $(shell pwd)
UBOOT_SRC_DIR = $(PWD)/u-boot/u-boot-2017.04
KERNEL_SRC_DIR = $(PWD)/kernel/linux-rpi-4.9.y-stable
BUSYBOX_SRC_DIR = $(PWD)/busybox/busybox-1.27.2
TOOLS_DIR = $(PWD)/tools
LIB_DIR = $(PWD)/lib
TOOLCHAIN_DIR = $(PWD)/tools/toolchain/arm-bcm2708/arm-linux-gnueabihf/bin
CROSS_COMPILE = $(TOOLCHAIN_DIR)/arm-linux-gnueabihf-
KERNEL_MOD_BUILD_DIR = $(PWD)/build/modules
WIRELESS_TOOL_DIR = $(TOOLS_DIR)/wireless_tools.29

ROOTFS_SRC_DIR = $(PWD)/fs/rootfs
ROOTFS_BUILD_DIR = $(PWD)/build/rootfs
KERNEL_DTC_TOOL = $(PWD)/kernel/linux-4.9/scripts/dtc/dtc
KERNEL_DTS_DIR = $(PWD)/kernel/linux-4.9/arch/arm/boot/dts/zynq-ax7010.dts
KDIR = $(PWD)/build/modules/lib/modules/4.9.80-v7/build

# library
# openssl library
LIB_OPENSSL_DIR = $(LIB_DIR)/openssl/openssl-1.0.2o

# wpa_supplicant tool
# depend on : openssl
LIB_WPA_SUPPLICANT_DIR = $(LIB_DIR)/wpa_supplicant/wpa_supplicant-2.6/wpa_supplicant

# library install
LIB_INSTALL_DIR = $(PWD)/build/lib

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
	rm -rf $(PWD)/boot/kernel7.img
	$(KERNEL_SRC_DIR)/scripts/mkknlimg $(KERNEL_SRC_DIR)/arch/arm/boot/zImage $(PWD)/boot/kernel7.img
	
kernel-distclean:
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 distclean

kernel-menuconfig:
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 menuconfig

kernel-dtb:
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 dtbs
	
modules:
#	rm $(KERNEL_MOD_BUILD_DIR) -rf
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 modules -j4
#	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 modules_install INSTALL_MOD_PATH=$(KERNEL_MOD_BUILD_DIR)

modules_install:
	rm $(KERNEL_MOD_BUILD_DIR) -rf
	make -C $(KERNEL_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNEL=kernel7 modules_install INSTALL_MOD_PATH=$(KERNEL_MOD_BUILD_DIR)
	
busybox-config:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) bcm2709_defconfig

busybox-menuconfig:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) menuconfig
	
busybox:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)

busybox-install:
	make -C $(BUSYBOX_SRC_DIR) install ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) CONFIG_PREFIX=$(ROOTFS_BUILD_DIR)

busybox-distclean:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) distclean
	
busybox-uninstall:
	make -C $(BUSYBOX_SRC_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) uninstall CONFIG_PREFIX=$(ROOTFS_BUILD_DIR)

rootfs:
	rm -rf $(ROOTFS_BUILD_DIR)
	make -C $(BUSYBOX_SRC_DIR) install ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) CONFIG_PREFIX=$(ROOTFS_BUILD_DIR)
	mkdir -p $(ROOTFS_BUILD_DIR)/home/pi $(ROOTFS_BUILD_DIR)/root $(ROOTFS_BUILD_DIR)/mnt $(ROOTFS_BUILD_DIR)/var/run/wpa_supplicant \
	         $(ROOTFS_BUILD_DIR)/dev $(ROOTFS_BUILD_DIR)/proc $(ROOTFS_BUILD_DIR)/sys $(ROOTFS_BUILD_DIR)/root $(ROOTFS_BUILD_DIR)/tmp
	cp -af $(ROOTFS_SRC_DIR)/etc $(ROOTFS_BUILD_DIR)
	cp -af $(LIB_DIR)/libtoolchain/* $(ROOTFS_BUILD_DIR)/lib
	cp -af $(LIB_DIR)/firmware $(ROOTFS_BUILD_DIR)/lib/
	cp -af $(LIB_DIR)/output/libnl/lib/libnl-3.so* $(ROOTFS_BUILD_DIR)/lib
	cp -af $(LIB_DIR)/output/libnl/lib/libnl-genl-3.so* $(ROOTFS_BUILD_DIR)/lib
	cp -af $(LIB_DIR)/output/openssl/lib/*.so* $(ROOTFS_BUILD_DIR)/lib
	mkdir -p $(ROOTFS_BUILD_DIR)/lib/modules/4.9.80-v7
	cp -af $(KERNEL_MOD_BUILD_DIR)/lib/modules/4.9.80-v7/kernel/* $(ROOTFS_BUILD_DIR)/lib/modules/4.9.80-v7
	cp -af $(LIB_DIR)/output/wpa_supplicant/* $(ROOTFS_BUILD_DIR)/sbin
	rm -rf $(ROOTFS_BUILD_DIR)/linuxrc

os_drv:
	$(MAKE) -C KERNEL_DIR=$(KDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)
                                     
help:
	@echo  ''
	@echo  '  ------------ rpi ------------'
	@echo  '  uboot            - Build for u-boot'
	@echo  '  uboot-distclean  - Distclean for u-boot'
	@echo  '  BOOT             - BOOT image'
	@echo  '  BOOT-clean       - BOOT image clean'
	@echo  '  kernel           - Build for kernel'
	@echo  '  openssl-config - '
	@echo  '  wpa_supplicant   - '
	@echo  '  wpa_supplicant-config'
	@echo  ''
	
.PHONY: $(PHONY)
