# Path for Quantenna Wifi Ax module

# QCS
QCS_SDK_VER ?= v50.0.56.8
QCS_IMAGE_PID ?= 562
QCS_IMAGE_FILE := pearl-linux-c-$(QCS_SDK_VER)p$(QCS_IMAGE_PID).lzma.img
QCS_IMAGE := pearl-linux.lzma.img

MODULE_NAME = qsr10g-pcie.ko

QCS_DRIVER_DIR := qsr10g-driver-$(QCS_SDK_VER)
QCS_FIRMWARE_DIR := qsr10g-firmware

CPU_ARCH := arm

PWD := $(shell pwd)
MODULE_PATH = $(PWD)/$(QCS_DRIVER_DIR)/arch/$(CPU_ARCH)
FIRMWARE_PATH = $(PWD)/$(QCS_FIRMWARE_DIR)/

all:
	$(MAKE) -C $(MODULE_PATH) all

install: install_driver install_firmware

install_driver:
	mkdir -p $(DESTDIR)/lib/modules
	cp -af $(MODULE_PATH)/$(MODULE_NAME) $(DESTDIR)/lib/modules

install_firmware:
	mkdir -p $(DESTDIR)/lib/firmware
	cp -af start-qsr10g.sh $(DESTDIR)/lib/firmware/
	cp -af $(FIRMWARE_PATH)/$(QCS_IMAGE_FILE) $(DESTDIR)/lib/firmware/
	pushd $(DESTDIR)/lib/firmware && \
	    rm -f $(QCS_IMAGE) && \
	    ln -sf $(QCS_IMAGE_FILE) $(QCS_IMAGE) && \
	    popd

clean:
	$(MAKE) -C $(MODULE_PATH) clean
	rm -rf $(DESTDIR)/lib/modules/$(MODULE_NAME)
	rm -rf $(DESTDIR)/lib/firmware/$(QCS_IMAGE_FILE)
	rm -rf $(DESTDIR)/lib/firmware/$(QCS_IMAGE)
