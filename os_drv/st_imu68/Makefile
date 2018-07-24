KERNEL_DIR:= ../../build/modules/lib/modules/4.9.80-v7/build
TOOLCHAIN_DIR = ../../tools/toolchain/arm-bcm2708/arm-linux-gnueabihf/bin
CROSS_COMPILE = $(TOOLCHAIN_DIR)/arm-linux-gnueabihf-


st_lsm9ds1_i2c-objs := st_imu68_buffer.o st_imu68_core.o st_imu68_i2c.o
obj-m := st_lsm9ds1_i2c.o

default:
	$(MAKE) -C $(KERNEL_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) SUBDIRS=$(PWD)
clean:
	$(RM) .*.cmd *.mod.c *.o *.ko *.symvers *.order -r .tmp
