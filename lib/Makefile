PHONY := openssl wpa_supplicant libnl libxml2 libiio

PWD := $(shell pwd)
TOP_DIR := $(shell dirname $(PWD))

TOOLCHAIN_DIR = $(TOP_DIR)/tools/toolchain/arm-bcm2708/arm-linux-gnueabihf/bin
CROSS_COMPILE = $(TOOLCHAIN_DIR)/arm-linux-gnueabihf-

# library
# openssl library
LIB_OPENSSL_DIR = $(PWD)/openssl/openssl-1.0.2o

# wpa_supplicant tool
# depend on : openssl
LIB_WPA_SUPPLICANT_DIR = $(PWD)/wpa_supplicant/wpa_supplicant-2.6/wpa_supplicant

# libnl
LIBNL_DIR = $(PWD)/libnl/libnl-3.4.0

# library install
LIB_INSTALL_DIR = $(PWD)/output

LIBXML2_DIR = $(PWD)/libxml2/libxml2-2.9.0
LIBIIO_DIR = $(PWD)/libiio/libiio-0.15

wireless_tool:
	make -C $(WIRELESS_TOOL_DIR) CC=$(CROSS_COMPILE)gcc

libnl-config:
	cd $(LIBNL_DIR) && ./configure --host=arm-linux --disable-static --enable-shared CC=$(CROSS_COMPILE)gcc --prefix=$(LIB_INSTALL_DIR)/libnl
libnl:
	make -C $(LIBNL_DIR)
libnl-install:
	rm -rf $(LIB_INSTALL_DIR)/libnl
	make -C $(LIBNL_DIR) install
libnl-distclean:
	make -C $(LIBNL_DIR) distclean

wpa_supplicant-config:
	cp $(LIB_WPA_SUPPLICANT_DIR)/defconfig $(LIB_WPA_SUPPLICANT_DIR)/.config
	echo LIBS += -L$(LIB_INSTALL_DIR)/libnl/lib >> $(LIB_WPA_SUPPLICANT_DIR)/.config
	echo LIBS += -L$(LIB_INSTALL_DIR)/openssl/lib >> $(LIB_WPA_SUPPLICANT_DIR)/.config
	echo LIBS_p += -L$(LIB_INSTALL_DIR)/openssl/lib >> $(LIB_WPA_SUPPLICANT_DIR)/.config
	echo CFLAGS += -I$(LIB_INSTALL_DIR)/libnl/include >> $(LIB_WPA_SUPPLICANT_DIR)/.config
	echo CFLAGS += -I$(LIB_INSTALL_DIR)/openssl/include >> $(LIB_WPA_SUPPLICANT_DIR)/.config
	echo CONFIG_LIBNL32=y >> $(LIB_WPA_SUPPLICANT_DIR)/.config
	echo CC=$(CROSS_COMPILE)gcc >> $(LIB_WPA_SUPPLICANT_DIR)/.config
wpa_supplicant:
	make -C $(LIB_WPA_SUPPLICANT_DIR) CC=$(CROSS_COMPILE)gcc
	mkdir -p $(LIB_INSTALL_DIR)/wpa_supplicant
	cp -af $(LIB_WPA_SUPPLICANT_DIR)/wpa_cli $(LIB_INSTALL_DIR)/wpa_supplicant/
	cp -af $(LIB_WPA_SUPPLICANT_DIR)/wpa_passphrase $(LIB_INSTALL_DIR)/wpa_supplicant/
	cp -af $(LIB_WPA_SUPPLICANT_DIR)/wpa_supplicant $(LIB_INSTALL_DIR)/wpa_supplicant/
wpa_supplicant-clean:
	make -C $(LIB_WPA_SUPPLICANT_DIR) CC=$(CROSS_COMPILE)gcc clean
	
# For openssl build
# 1. make openssl-config; 2. make openssl; 3. openssl-install
openssl-config:
	cd $(LIB_OPENSSL_DIR) && ./config -march=armv7-a no-asm shared --prefix=$(LIB_INSTALL_DIR)/openssl
openssl:
	make -C $(LIB_OPENSSL_DIR) CC=$(CROSS_COMPILE)gcc
openssl-install:
	make -C $(LIB_OPENSSL_DIR) CC=$(CROSS_COMPILE)gcc install
openssl-clean:
	make -C $(LIB_OPENSSL_DIR) clean
openssl-distclean:
	make -C $(LIB_OPENSSL_DIR) distclean

# For libxml2 build
libxml2-config:
	cd $(LIBXML2_DIR) && ./configure CC=$(CROSS_COMPILE)gcc --host=arm-linux-gnueabihf --prefix=$(LIB_INSTALL_DIR)/libxml2 --enable-shared --with-python=no
libxml2:
	make -C $(LIBXML2_DIR)
libxml2-install:
	rm -rf $(LIB_INSTALL_DIR)/libxml2
	make -C $(LIBXML2_DIR) install

# For libiio build
libiio-config:
	cd $(LIBIIO_DIR) && cmake \
	-DCMAKE_TOOLCHAIN_FILE=$(PWD)/libiio/toolchainfile.cmake \
	-DCMAKE_INSTALL_PREFIX=$(LIB_INSTALL_DIR)/libiio \
	-DCMAKE_EXE_LINKER_FLAGS="-L$(LIB_INSTALL_DIR)/libxml2/lib -lxml2 -lpthread"
libiio:
	make -C $(LIBIIO_DIR) all
libiio-install:
	rm -rf $(LIB_INSTALL_DIR)/libiio
	make -C $(LIBIIO_DIR) install
	
help:
	@echo  ''
	@echo  '  ----------------- lib ----------------'
	@echo  '  libnl-config      - libnl config      '
	@echo  '  libnl             - libnl             '
	@echo  '  libnl-install     - libnl install     '
	@echo  '  libnl-distclean   - libnl distclean   '
	@echo  '  openssl-config    - openssl config    '
	@echo  '  openssl           - openssl build     '
	@echo  '  openssl-install   - openssl install   '
	@echo  '  openssl-clean     - openssl clean     '
	@echo  '  openssl-distclean - openssl distclean '
	
	@echo  '  wpa_supplicant    -                   '
	@echo  '  wpa_supplicant-config                 '
	@echo  ''
	
.PHONY: $(PHONY)
