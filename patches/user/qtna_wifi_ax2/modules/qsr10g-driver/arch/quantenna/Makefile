#
# Makefile for Quantenna RC paltform
#
#

EXTRA_CFLAGS	+= -Wall -Werror -Wno-unknown-pragmas \
		   -I$(src)		\
		   -I$(src)/../../include \
		   -I../../drivers/include/shared \
		   -I../../drivers/include/kernel \
		   -I$(src)/../../common

EXTRA_CFLAGS    += -mlong-calls
 ifneq ($(CONFIG_HOTPLUG_PCI_PCIE),)
 EXTRA_CFLAGS    += -DPCIE_HOTPLUG_SUPPORTED
 endif

ifeq (${PCIE_HOST_CRUMBS},1)
EXTRA_CFLAGS += -finstrument-functions
endif

#EXTRA_CFLAGS	+= -DDEBUG

ifneq ($(KERNELRELEASE),)
COMMON_DIR	:= ../../common
EXTRA_CFLAGS += -I.
qdpc-host-objs   := $(COMMON_DIR)/pearl_vnet.o $(COMMON_DIR)/qdpc_init.o \
		$(COMMON_DIR)/qdpc_pcie.o $(COMMON_DIR)/qtn_pcie_dbg.o \
		qdpc_platform.o topaz_pcie_tqe.o

obj-m           :=  qdpc-host.o

else

KERNELDIR	?= ../../../../linux
INSTALL		= INSTALL_MOD_PATH=../linux/modules
CROSS		= ARCH=arc CROSS_COMPILE=$(tools_path)/arc-linux-uclibc-
PWD		:= $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) $(CROSS) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNELDIR) $(CROSS) $(INSTALL) M=$(PWD) modules_install

endif

clean:
	rm -rf *.o  *~  core  .depend  .*.cmd  *.ko  *.mod.c  .tmp_versions  Module.symvers  modules.order
	rm -rf $(COMMON_DIR)/.*.cmd $(COMMON_DIR)/.tmp_versions $(COMMON_DIR)/*.o

depend .depend dep:
	$(CC) $(CFLAGS) -M *.c > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif

