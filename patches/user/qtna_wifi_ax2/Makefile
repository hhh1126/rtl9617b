# Path for kernel module
#.EXPORT_ALL_VARIABLES:

# QCS
QCS_SDK_VER ?= v7.0.0.23

BUILD_MODULE_PATH = modules
BUILD_QCSAPI_PATH = qcsapi

MODULE_NAME = qsr10g-pcie.ko

RC_LEVEL_QTN="/etc/init.d/rc33"

ifeq ("$(ROMFSDIR)", "")
ROMFSDIR=`pwd`/.install
endif

all:
	$(MAKE) -C $(BUILD_MODULE_PATH) all
	$(MAKE) -C $(BUILD_QCSAPI_PATH) all
	
clean:
	$(MAKE) -C $(BUILD_MODULE_PATH) clean DESTDIR=$(ROMFSDIR)
	$(MAKE) -C $(BUILD_QCSAPI_PATH) clean DEST=$(ROMFSDIR)

rcX:
	$(ROMFSINST) -a "mkdir -p /tmp/qtn" $(RC_LEVEL_QTN)
	$(ROMFSINST) -a "insmod /lib/modules/$(MODULE_NAME)" $(RC_LEVEL_QTN)
	$(ROMFSINST) -a "brctl addif br0 host0" $(RC_LEVEL_QTN)
	$(ROMFSINST) -a "ifconfig host0 up" $(RC_LEVEL_QTN)
	$(ROMFSINST) -a "echo wlan 0 mode 1 cpu 3 > /proc/fc/ctrl/smp_dispatch_wifi_tx" $(RC_LEVEL_QTN)
	$(ROMFSINST) -a "echo wlan 0 mode 1 cpu 2 > /proc/fc/ctrl/smp_dispatch_wifi_rx" $(RC_LEVEL_QTN)
	$(ROMFSINST) -a "sleep 1" $(RC_LEVEL_QTN)
	$(ROMFSINST) -a "echo 2 > /proc/irq/13/smp_affinity" $(RC_LEVEL_QTN)
	$(ROMFSINST) -a "echo 1 > /proc/irq/18/smp_affinity" $(RC_LEVEL_QTN)
	
romfs:
	$(MAKE) -C $(BUILD_MODULE_PATH) install DESTDIR=$(ROMFSDIR) 
	$(MAKE) -C $(BUILD_QCSAPI_PATH) install DEST=$(ROMFSDIR)
	$(ROMFSINST) -s /tmp/qtn /lib/firmware/qtn
