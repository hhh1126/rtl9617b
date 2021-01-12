#
# Makefile for Intel platform
#

EXTRA_CFLAGS	+= -Wall		\
		   -I$(src)		\
		   -I$(src)/../../include \
		   -I$(src)/../../common

ifneq ($(CONFIG_HOTPLUG_PCI_PCIE),)
EXTRA_CFLAGS    += -DPCIE_HOTPLUG_SUPPORTED
endif

KERNEL_BUILD ?= /lib/modules/$(shell uname -r)/build

default: all

COMMON_DIR	:= ../../common
qsr10g-pcie-objs   := $(COMMON_DIR)/qdpc_init.o $(COMMON_DIR)/qdpc_pcie.o $(COMMON_DIR)/pearl_vnet.o \
		$(COMMON_DIR)/qtn_pcie_dbg.o qdpc_platform.o
obj-m           :=  qsr10g-pcie.o

qsr10g_pcie.o: $(qsr10g-pcie-objs)
	ld -r $^ -o $@

all:
	make -C $(KERNEL_BUILD) M=$(PWD) modules

clean:
	make -C $(KERNEL_BUILD) M=$(PWD) clean
	rm -rf $(COMMON_DIR)/.*.cmd $(COMMON_DIR)/.tmp_versions
	rm -rf Module.markers  modules.order *~ $(qsr10g-pcie-objs)

