KERNEL_DIR:= ../../build/modules/lib/modules/4.9.80-v7/build
TOOLCHAIN_DIR = ../../tools/toolchain/arm-bcm2708/arm-linux-gnueabihf/bin
CROSS_COMPILE = $(TOOLCHAIN_DIR)/arm-linux-gnueabihf-
obj-m:=bno055.o
default:
	$(MAKE) -C $(KERNEL_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) SUBDIRS=$(PWD) modules
clean:
	$(RM) .*.cmd *.mod.c *.o *.ko -r .tmp
