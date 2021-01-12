#
# Makefile for arm platform
#

EXTRA_CFLAGS	+= -Wall -Wno-deprecated-declarations	\
		   -I$(src)		\
		   -I$(src)/../../include \
		   -I$(src)/../../common

PWD	:= $(shell pwd)

default: all

COMMON_DIR	:= ../../common
qsr10g-pcie-objs   := $(COMMON_DIR)/qdpc_init.o $(COMMON_DIR)/qdpc_pcie.o $(COMMON_DIR)/pearl_vnet.o $(COMMON_DIR)/qtn_pcie_dbg.o qdpc_platform.o
obj-m           :=  qsr10g-pcie.o

qsr10g_pcie.o: $(qsr10g-pcie-objs)
	ld -r $^ -o $@

all:
	make -C $(KERNELDIR) $(CROSS) M=$(PWD) modules


clean:
	rm -rf $(COMMON_DIR)/.*.cmd $(COMMON_DIR)/.tmp_versions
	rm -rf Module.markers  Module.symvers modules.order *~ $(qsr10g-pcie-objs) *.o *.ko *.mod.o *.mod.c

