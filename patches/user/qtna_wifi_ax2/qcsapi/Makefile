
CFLAGS += -I. -I./common -fPIC -O -g -Wall -Werror -Wno-address-of-packed-member -Wno-unused-variable -Wno-unused-parameter -fstrict-aliasing
CFLAGS += -D_GNU_SOURCE

ifeq ($(USE_LIBTIRPC),1)
TIRPC_CFLAGS=$(shell pkg-config --cflags libtirpc)
TIRPC_LDFLAGS=$(shell pkg-config --libs libtirpc)
CFLAGS += -DUSE_TIRPC $(TIRPC_CFLAGS)
LDFLAGS += $(TIRPC_LDFLAGS)
endif

COMMON_PROG_OBJS = \
	qcsapi_bldinfo.o \
	call_qcsapi.o \
	qcsapi_param.o \
	qcsapi_output.o \
	qcsapi_util.o

SOCKET_PROG_OBJS = \
	$(COMMON_PROG_OBJS)				\
	qcsapi_rpc/client/socket/qcsapi_socket_rpc_client.o	\
	qcsapi_rpc_common/client/find_host_addr.o	\

SOCKET_C_SAMPLE_OBJS = \
	qcsapi_rpc_sample/c_rpc_qcsapi_sample.o	\
	qcsapi_rpc_common/client/find_host_addr.o	\

PCIE_PROG_OBJS = \
	$(COMMON_PROG_OBJS)				\
	qcsapi_rpc/client/pcie/qcsapi_pcie_rpc_client.o	\

SOCKET_RAW_PROG_OBJS = \
	$(COMMON_PROG_OBJS)				\
	qcsapi_rpc_common/client/qftc.o		\
	qcsapi_rpc/client/socket_raw/qcsapi_socketraw_rpc_client.o	\

LIB_OBJS = \
	qcsapi_rpc/generated/qcsapi_rpc_xdr.o	\
	qcsapi_rpc/generated/qcsapi_rpc_clnt_adapter.o	\
	qcsapi_sem.o	\
	qcsapi_rpc_common/client/rpc_pci_clnt.o	\
	qcsapi_rpc_common/client/rpc_raw_clnt.o	\
	qcsapi_rpc_common/common/rpc_raw.o \
	qcsapi_grabber.o

TARGETS = \
	qcsapi_sockrpc \
	qcsapi_sockrpc_static \
	qcsapi_pcie \
	qcsapi_pcie_static \
	qcsapi_sockraw \
	qcsapi_sockraw_static \
	$(LIB_REALNAME)

CFLAGS += -DPCIE_RPC_TYPE=RPC_TYPE_QCSAPI_PCIE
CFLAGS += -DPCIE_RPC_LOCK_ENABLE=1

all: $(TARGETS)

-include $(shell find . -name \*.d)

LIB_NAME = qcsapi_client
LIB_LDNAME = lib$(LIB_NAME).so
LIB_SONAME = $(LIB_LDNAME).1
LIB_REALNAME = $(LIB_LDNAME).1.0.1

c_rpc_qcsapi_sample: ${SOCKET_C_SAMPLE_OBJS:%=build/%} $(LIB_REALNAME)
	${CC} $(filter %.o, $^) -L. -lpthread  $(LDFLAGS) -l$(LIB_NAME) -o $@

qcsapi_pcie: ${PCIE_PROG_OBJS:%=build/%} $(LIB_REALNAME)
	${CC} $(filter %.o, $^) -L. -lpthread $(LDFLAGS) -l$(LIB_NAME) -o $@

qcsapi_pcie_static: ${PCIE_PROG_OBJS:%=build/%} ${LIB_OBJS}
	${CC} $(filter %.o, $^) -lpthread $(LDFLAGS) -o $@

qcsapi_sockrpc: ${SOCKET_PROG_OBJS:%=build/%} $(LIB_REALNAME)
	${CC} $(filter %.o, $^) -L. -lpthread $(LDFLAGS) -l$(LIB_NAME) -o $@

qcsapi_sockrpc_static: ${SOCKET_PROG_OBJS:%=build/%} ${LIB_OBJS}
	${CC} $(filter %.o, $^) -lpthread $(LDFLAGS) -o $@

qcsapi_sockraw: ${SOCKET_RAW_PROG_OBJS:%=build/%} $(LIB_REALNAME)
	${CC} $(filter %.o, $^) -L. -lpthread $(LDFLAGS) -l$(LIB_NAME) -o $@

qcsapi_sockraw_static: ${SOCKET_RAW_PROG_OBJS:%=build/%} ${LIB_OBJS}
	${CC} $(filter %.o, $^) -lpthread $(LDFLAGS) -o $@

$(LIB_REALNAME): ${LIB_OBJS:%=build/%}
	${CC} -shared -s -o $@ $(LDFLAGS) -Wl,-soname,$(LIB_SONAME) -lc $^
	cd ${@D} ; ln -fs $(LIB_REALNAME) $(LIB_SONAME)
	cd ${@D} ; ln -fs $(LIB_SONAME) $(LIB_LDNAME)

build/%.o: %.c
	@mkdir -p ${@D}
	${CC} ${CFLAGS} $< -c -o $@ -MD -MF $@.d

qcsapi_cpptest: ${COMMON_PROG_OBJS:%=build/%} ${LIB_OBJS}
	g++ $(CFLAGS) $(COMMON_PROG_OBJS:%=build/%) $(LIB_OBJS) -lpthread -o $@ $@.cpp
	rm $@

clean:
	rm -rf build $(LIB_LDNAME)* $(TARGETS) $(LIB_OBJS)

install:
	-mkdir -p ${DEST}/usr/bin
	-mkdir -p ${DEST}/usr/lib
	cp qcsapi_pcie qcsapi_sockrpc qcsapi_sockraw ${DEST}/usr/bin
	cp ${LIB_REALNAME} ${DEST}/usr/lib && \
	    pushd ${DEST}/usr/lib && \
	    ln -sf ${LIB_REALNAME} ${LIB_SONAME} && \
	    ln -sf ${LIB_SONAME} ${LIB_LDNAME} && \
	    popd
