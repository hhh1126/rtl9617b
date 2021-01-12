# Path for kernel module
#.EXPORT_ALL_VARIABLES:

# QCS
QCS_SDK_VER ?= v50.0.56.8

BUILD_MODULE_PATH = module
BUILD_QCSAPI_PATH = qcsapi-client-ax-$(QCS_SDK_VER)

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
	
romfs:
	$(MAKE) -C $(BUILD_MODULE_PATH) install DESTDIR=$(ROMFSDIR) 
	$(MAKE) -C $(BUILD_QCSAPI_PATH) install DEST=$(ROMFSDIR)
	$(ROMFSINST) -s /tmp/qtn /lib/firmware/qtn
