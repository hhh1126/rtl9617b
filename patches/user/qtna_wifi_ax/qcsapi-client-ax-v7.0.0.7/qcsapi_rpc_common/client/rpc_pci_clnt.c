/*
 * Copyright (C) 1987, Sun Microsystems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of Sun Microsystems, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <alloca.h>
#include <errno.h>
#include <string.h>
#include <rpc/rpc.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netdb.h>
#include <linux/netlink.h>
#include <rpc/clnt.h>
#include <sys/poll.h>
#include <assert.h>
#include <qcsapi_rpc_common/common/rpc_pci.h>
#include <fcntl.h>
#ifdef PCIE_RPC_LOCK_ENABLE
#include "qcsapi_rpc/client/qcsapi_rpc_client.h"
#include "qcsapi_rpc/generated/qcsapi_rpc.h"
#include "qcsapi_sem.h"
#endif

#ifndef PCIE_RPC_TYPE
	#error "Not configure PCIE_RPC_TYPE"
#else
	#if (PCIE_RPC_TYPE != RPC_TYPE_CALL_QCSAPI_PCIE) && (PCIE_RPC_TYPE != RPC_TYPE_QCSAPI_PCIE)
	#error "Configuration invalid value for PCIE_RPC_TYPE"
	#endif
#endif

void * qcsapi_pcie_pre(const char *tmp);
void qcsapi_pcie_post(const char *tmp, int was_error);
extern void hidden_client_qcsapi_set_rpcclient(CLIENT * clnt);

#ifdef PCIE_RPC_LOCK_ENABLE
extern int qtn_psafe_mutex_flock();
extern int qtn_psafe_mutex_unflock(int);
#endif

/*
 * Private data kept per client handle
 */
struct cu_data {
	int cu_sock;
	struct sockaddr_nl cu_saddr;
	struct sockaddr_nl cu_daddr;
	//struct sockaddr_in cu_raddr;
	//int cu_rlen;
	int cu_slen;
	int cu_dlen;
	struct timeval cu_wait;
	struct timeval cu_total;
	struct rpc_err cu_error;
	XDR cu_outxdrs;
	u_int cu_xdrpos;
	u_int cu_sendsz;
	u_int cu_recvsz;
	char *cu_outbuf;
	char *cu_inbuf;
	struct nlmsghdr *cu_reqnlh;
	struct nlmsghdr *cu_respnlh;
};

#define PCI_QDPC_HOST_FILE	"/proc/bus/pci/devices"
#define CLNT_PCI_DATA_BUF_LEN	512
char qdpc_pcie_driver_name[] =	"qdpc_host";

static int clnt_pcie_check_existence()
{
	char temp[CLNT_PCI_DATA_BUF_LEN] = {0};

	FILE *fp = fopen(PCI_QDPC_HOST_FILE, "r");
	if (fp == NULL)
		return 0;

	while (fgets(temp, sizeof(temp), fp) != NULL) {
		if (strstr(temp, qdpc_pcie_driver_name) != NULL) {
			fclose(fp);
			return 1;
		}
	}
	fclose(fp);

	return 0;
}

#ifdef PCIE_RPC_LOCK_ENABLE
CLIENT *clnt_pci_create(const char *hostname,
		u_long prog, u_long vers, const char *proto)
{
	if (!clnt_pcie_check_existence())
		return NULL;

	client_qcsapi_set_callbacks(qcsapi_pcie_pre, qcsapi_pcie_post, NULL);
	client_qcsapi_set_lock(qtn_psafe_mutex_flock, qtn_psafe_mutex_unflock);
	return (CLIENT *)PCIE_VIRTUAL_CLNT_ADDR;
}
#endif

static CLIENT *_clnt_pci_create(int sock_fd,
				struct sockaddr_nl *src,
				struct sockaddr_nl *dst,
				u_long prog, u_long vers);
/*
 * Generic client creation: takes (hostname, program-number, protocol) and
 * returns client handle. Default options are set, which the user can
 * change using the rpc equivalent of ioctl()'s.
 */
#ifdef PCIE_RPC_LOCK_ENABLE
CLIENT *hidden_clnt_pci_create(const char *hostname,
			u_long prog, u_long vers, const char *proto)
#else
CLIENT *clnt_pci_create(const char *hostname,
			u_long prog, u_long vers, const char *proto)
#endif
{
	CLIENT *client;
	struct sockaddr_nl src_addr, dest_addr;
	int sock_fd = -1;
	int ret = 0;
#ifndef PCIE_RPC_LOCK_ENABLE
	if (!clnt_pcie_check_existence())
		return NULL;
#endif
#ifdef SOCK_CLOEXEC
	sock_fd = socket(PF_NETLINK, SOCK_RAW|SOCK_CLOEXEC, NETLINK_RPC_PCI_CLNT);
	if (sock_fd < 0)
		goto err;
#else
	sock_fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_RPC_PCI_CLNT);
	if (sock_fd < 0)
		goto err;
	ret = fcntl(sock_fd, F_GETFD, 0);
	if (ret < 0) {
		perror("clnt_pci_create fcntl get F_GETFD error");
	} else {
		ret |= FD_CLOEXEC;
		ret = fcntl(sock_fd, F_SETFD, ret);
		if (ret < 0)
			perror("clnt_pci_create fcntl set FD_CLOEXEC error");
	}
#endif

	memset(&src_addr, 0, sizeof(src_addr));
	src_addr.nl_family = AF_NETLINK;
	src_addr.nl_pid = 0; /*Let Kernel perform auto-bind*/

	ret = bind(sock_fd, (struct sockaddr *)&src_addr, sizeof(src_addr));
	if (ret) {
		perror("clnt_pci_create bind fail");
		goto err;
	}

	memset(&dest_addr, 0, sizeof(dest_addr));
	memset(&dest_addr, 0, sizeof(dest_addr));
	dest_addr.nl_family = AF_NETLINK;
	dest_addr.nl_pid = 0;	/* For Linux Kernel */
	dest_addr.nl_groups = 0;	/* unicast */

	client = _clnt_pci_create(sock_fd, &src_addr, &dest_addr, prog, vers);

	if (client == NULL)
		close(sock_fd);

	return client;

err:
#if 0
	if (errno) {
		struct rpc_createerr *ce = &get_rpc_createerr();
		ce->cf_stat = RPC_SYSTEMERROR;
		ce->cf_error.re_errno = error;
		return NULL;
	}
#endif
	if (sock_fd >= 0)
		close(sock_fd);

	return NULL;
}

extern u_long _create_xid(void);

#if defined(USE_TIRPC)
static enum clnt_stat clnt_pci_call(CLIENT *, rpcproc_t, xdrproc_t, void *, xdrproc_t,
					void *, struct timeval);
static void clnt_pci_abort(CLIENT *);
static bool_t clnt_pci_freeres(CLIENT *, xdrproc_t, void *);
static bool_t clnt_pci_control(CLIENT *, unsigned int, void *);
#else
static enum clnt_stat clnt_pci_call(CLIENT *, u_long, xdrproc_t, caddr_t, xdrproc_t,
					caddr_t, struct timeval);
static void clnt_pci_abort(void);
static bool_t clnt_pci_freeres(CLIENT *, xdrproc_t, caddr_t);
static bool_t clnt_pci_control(CLIENT *, int, char *);
#endif
static void clnt_pci_geterr(CLIENT *, struct rpc_err *);
static void clnt_pci_destroy(CLIENT *);

static const struct clnt_ops pci_ops = {
	clnt_pci_call,
	clnt_pci_abort,
	clnt_pci_geterr,
	clnt_pci_freeres,
	clnt_pci_destroy,
	clnt_pci_control
};

/*
 * Create a UDP based client handle.
 * If *sockp<0, *sockp is set to a newly created UPD socket.
 * If raddr->sin_port is 0 a binder on the remote machine
 * is consulted for the correct port number.
 * NB: It is the clients responsibility to close *sockp.
 * NB: The rpch->cl_auth is initialized to null authentication.
 *     Caller may wish to set this something more useful.
 *
 * wait is the amount of time used between retransmitting a call if
 * no response has been heard; retransmission occurs until the actual
 * rpc call times out.
 *
 * sendsz and recvsz are the maximum allowable packet sizes that can be
 * sent and received.
 */

static CLIENT *_clnt_pci_create(int sock_fd,
				struct sockaddr_nl *src,
				struct sockaddr_nl *dst,
				u_long prog, u_long vers)
{

	struct timeval wait;
	CLIENT *cl;
	struct cu_data *cu = NULL;
	struct rpc_msg call_msg;
	struct nlmsghdr *preqnlh, *prespnlh;
	struct iovec iov;
	struct msghdr msg;
	int slen = sizeof(struct sockaddr_nl);
	//u_int sendsz, recvsz;

	wait.tv_sec = 5;
	wait.tv_usec = 0;

	cl = (CLIENT *) malloc(sizeof(CLIENT));
	//sendsz = ((PCIMSGSIZE + 3) / 4) * 4;
	//recvsz = ((PCIMSGSIZE + 3) / 4) * 4;
	cu = (struct cu_data *)calloc(1, sizeof(*cu));

	/* Allocate memory for nlm headers */
	preqnlh = (struct nlmsghdr *)calloc(1, NLMSG_SPACE(PCIMSGSIZE));
	prespnlh = (struct nlmsghdr *)calloc(1, NLMSG_SPACE(PCIMSGSIZE));

	if (cl == NULL || cu == NULL || preqnlh == NULL || prespnlh == NULL) {
		fprintf(stderr, "pci_clnt_create out of memory\n");
		goto fooy;
	}

	if (getsockname(sock_fd, (struct sockaddr *)src, (socklen_t *)&slen)) {
		fprintf(stderr, "%s:%d can't get autobind netlink ID\n", __FUNCTION__, __LINE__);
		goto fooy;
	}

	cl->cl_ops = (struct clnt_ops *)&pci_ops;
	cl->cl_private = (caddr_t) cu;
	cu->cu_saddr = *src;
	cu->cu_daddr = *dst;
	cu->cu_slen = sizeof(cu->cu_saddr);
	cu->cu_dlen = sizeof(cu->cu_daddr);
	cu->cu_wait = wait;
	cu->cu_total.tv_sec = -1;
	cu->cu_total.tv_usec = -1;
	cu->cu_sendsz = PCIMSGSIZE;
	cu->cu_recvsz = PCIMSGSIZE;

	// setup req/resp netlink headers
	cu->cu_reqnlh = preqnlh;
	cu->cu_respnlh = prespnlh;

	memset(preqnlh, 0, NLMSG_SPACE(PCIMSGSIZE));
	preqnlh->nlmsg_len = NLMSG_SPACE(PCIMSGSIZE);
	preqnlh->nlmsg_pid = src->nl_pid;
	preqnlh->nlmsg_flags = NLM_F_REQUEST;
	cu->cu_outbuf = NLMSG_DATA(preqnlh);

	memset(prespnlh, 0, NLMSG_SPACE(PCIMSGSIZE));
	prespnlh->nlmsg_len = NLMSG_SPACE(PCIMSGSIZE);
	prespnlh->nlmsg_pid = src->nl_pid;
	prespnlh->nlmsg_flags = NLM_F_REQUEST;
	cu->cu_inbuf = NLMSG_DATA(prespnlh);

	call_msg.rm_xid = getpid();
	call_msg.rm_direction = CALL;
	call_msg.rm_call.cb_rpcvers = RPC_MSG_VERSION;
	call_msg.rm_call.cb_prog = prog;
	call_msg.rm_call.cb_vers = vers;

	xdrmem_create(&(cu->cu_outxdrs), cu->cu_outbuf, PCIMSGSIZE, XDR_ENCODE);
	if (!xdr_callhdr(&(cu->cu_outxdrs), &call_msg)) {
		goto fooy;
	}
	cu->cu_xdrpos = XDR_GETPOS(&(cu->cu_outxdrs));
	cu->cu_sock = sock_fd;
	cl->cl_auth = authnone_create();

	// Register the client. May not be necessary. FIXME
	preqnlh->nlmsg_len = 0;
	preqnlh->nlmsg_type = NETLINK_TYPE_CLNT_REGISTER;

	iov.iov_base = (void *)cu->cu_reqnlh;
	iov.iov_len = NLMSG_SPACE(0);

	memset((caddr_t) & msg, 0, sizeof(msg));
	msg.msg_name = (void *)&cu->cu_daddr;
	msg.msg_namelen = cu->cu_dlen;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	if (sendmsg(cu->cu_sock, &msg, 0) < 0)
		goto fooy;

	return cl;

fooy:
	if (cu)
		free((caddr_t) cu);
	if (cl)
		free((caddr_t) cl);
	if (preqnlh)
		free((caddr_t) preqnlh);
	if (prespnlh)
		free((caddr_t) prespnlh);

	return (CLIENT *) NULL;
}

enum clnt_stat clnt_pci_call(cl, proc, xargs, argsp, xresults, resultsp, utimeout)
#if defined(USE_TIRPC)
CLIENT *cl;			/* client handle */
rpcproc_t proc;			/* procedure number */
xdrproc_t xargs;		/* xdr routine for args */
void *argsp;			/* pointer to args */
xdrproc_t xresults;		/* xdr routine for results */
void *resultsp;			/* pointer to results */
struct timeval utimeout;	/* seconds to wait before giving up */
#else
CLIENT *cl;			/* client handle */
u_long proc;			/* procedure number */
xdrproc_t xargs;		/* xdr routine for args */
caddr_t argsp;			/* pointer to args */
xdrproc_t xresults;		/* xdr routine for results */
caddr_t resultsp;		/* pointer to results */
struct timeval utimeout;	/* seconds to wait before giving up */
#endif
{
#ifdef PCIE_RPC_LOCK_ENABLE
	if(__clnt_pci){
		cl = __clnt_pci;
	}
	else{
			return (RPC_CANTSEND);
	}
#endif
	struct cu_data *cu = (struct cu_data *)cl->cl_private;
	XDR *xdrs;
	int outlen = 0;
	int inlen;
	//socklen_t fromlen;
	struct pollfd fd;
	int milliseconds = (cu->cu_wait.tv_sec * 1000) +
	    (cu->cu_wait.tv_usec / 1000);
	//struct sockaddr_in from;
	struct rpc_msg reply_msg;
	XDR reply_xdrs;
	struct timeval time_waited;
	bool_t ok;
	int nrefreshes = 2;	/* number of times to refresh cred */
	struct timeval timeout;
	//int anyup;                    /* any network interface up */
	struct iovec iov;
	struct msghdr msg;
	//int ret;

	//printf("In clnt_pci_call\n");

	if (cu->cu_total.tv_usec == -1) {
		timeout = utimeout;	/* use supplied timeout */
	} else {
		timeout = cu->cu_total;	/* use default timeout */
	}

	time_waited.tv_sec = 0;
	time_waited.tv_usec = 0;

call_again:
	xdrs = &(cu->cu_outxdrs);
	if (xargs == NULL)
		goto get_reply;
	xdrs->x_op = XDR_ENCODE;
	XDR_SETPOS(xdrs, cu->cu_xdrpos);

	/*
	 * the transaction is the first thing in the out buffer
	 */
	(*(uint32_t *) (cu->cu_outbuf))++;
	if ((!XDR_PUTLONG(xdrs, (long *)&proc)) ||
	    (!AUTH_MARSHALL(cl->cl_auth, xdrs)) || (!(*xargs) (xdrs, argsp)))
		return (cu->cu_error.re_status = RPC_CANTENCODEARGS);
	outlen = (int)XDR_GETPOS(xdrs);

	// Set up the netlink msg headers
	cu->cu_reqnlh->nlmsg_len = outlen;
	iov.iov_base = (void *)cu->cu_reqnlh;
	iov.iov_len = NLMSG_SPACE(outlen);

	memset((caddr_t) & msg, 0, sizeof(msg));
	msg.msg_name = (void *)&cu->cu_daddr;
	msg.msg_namelen = cu->cu_dlen;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	cu->cu_reqnlh->nlmsg_type = NETLINK_TYPE_CLNT_REQUEST;

	assert(outlen <= PCIMSGSIZE);

	//send_again:
	//ret = sendmsg(cu->cu_sock, &msg, 0);
	if (sendmsg(cu->cu_sock, &msg, 0) < 0) {
		perror("sendmsg");
		cu->cu_error.re_errno = errno;
		return (cu->cu_error.re_status = RPC_CANTSEND);
	}
	//fprintf(stderr, "sendmsg data len %d, sent %d\n", outlen, ret );

	/*
	 * report error if it could not send. 
	 {
	 cu->cu_error.re_errno = errno;
	 return (cu->cu_error.re_status = RPC_CANTSEND);
	 }
	 */

	/*
	 * Hack to provide rpc-based message passing
	 */
	if (timeout.tv_sec == 0 && timeout.tv_usec == 0) {
		return (cu->cu_error.re_status = RPC_TIMEDOUT);
	}
	// Set up the netlink msg headers
	iov.iov_base = (void *)cu->cu_respnlh;
	iov.iov_len = cu->cu_respnlh->nlmsg_len;
	msg.msg_name = (void *)&cu->cu_daddr;
	msg.msg_namelen = cu->cu_dlen;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

get_reply:
	/*
	 * sub-optimal code appears here because we have
	 * some clock time to spare while the packets are in flight.
	 * (We assume that this is actually only executed once.)
	 */
	reply_msg.acpted_rply.ar_verf = _null_auth;
	reply_msg.acpted_rply.ar_results.where = resultsp;
	reply_msg.acpted_rply.ar_results.proc = xresults;
	fd.fd = cu->cu_sock;
	fd.events = POLLIN;
	for (;;) {
		switch (poll(&fd, 1, milliseconds)) {
		case 0:

			time_waited.tv_sec += cu->cu_wait.tv_sec;
			time_waited.tv_usec += cu->cu_wait.tv_usec;
			while (time_waited.tv_usec >= 1000000) {
				time_waited.tv_sec++;
				time_waited.tv_usec -= 1000000;
			}
			if ((time_waited.tv_sec < timeout.tv_sec) ||
			    ((time_waited.tv_sec == timeout.tv_sec) &&
			     (time_waited.tv_usec < timeout.tv_usec))) {
				continue;
			}
			return (cu->cu_error.re_status = RPC_TIMEDOUT);

			/*
			 * buggy in other cases because time_waited is not being
			 * updated.
			 */

		case -1:
			if (errno == EINTR)
				continue;
			cu->cu_error.re_errno = errno;
			return (cu->cu_error.re_status = RPC_CANTRECV);
		}

		do {
			iov.iov_len = NLMSG_SPACE(PCIMSGSIZE);
			inlen = recvmsg(cu->cu_sock, &msg, 0);
		} while (inlen < 0 && errno == EINTR);

		if (inlen < 0) {
			if (errno == EWOULDBLOCK)
				continue;
			cu->cu_error.re_errno = errno;
			return (cu->cu_error.re_status = RPC_CANTRECV);
		}

		if (inlen < NLMSG_HDRLEN)
			continue;

		/* see if reply transaction id matches sent id.
		   Don't do this if we only wait for a replay */
		if (xargs != NULL && (*((u_int32_t *) (cu->cu_inbuf))
				      != *((u_int32_t *) (cu->cu_outbuf))))
			continue;
		/* we now assume we have the proper reply */
		break;
	}

	/*
	 * now decode and validate the response
	 */
	xdrmem_create(&reply_xdrs, cu->cu_inbuf, (u_int) inlen, XDR_DECODE);
	ok = xdr_replymsg(&reply_xdrs, &reply_msg);
	/* XDR_DESTROY(&reply_xdrs);  save a few cycles on noop destroy */
	if (ok) {
		_seterr_reply(&reply_msg, &(cu->cu_error));
		if (cu->cu_error.re_status == RPC_SUCCESS) {
			if (!AUTH_VALIDATE(cl->cl_auth,
					   &reply_msg.acpted_rply.ar_verf)) {
				cu->cu_error.re_status = RPC_AUTHERROR;
				cu->cu_error.re_why = AUTH_INVALIDRESP;
			}
			if (reply_msg.acpted_rply.ar_verf.oa_base != NULL) {
				xdrs->x_op = XDR_FREE;
				(void)xdr_opaque_auth(xdrs,
						      &(reply_msg.acpted_rply.
							ar_verf));
			}
		} /* end successful completion */
		else {
			/* maybe our credentials need to be refreshed ... */
#if defined(USE_TIRPC)
			if (nrefreshes > 0 && AUTH_REFRESH(cl->cl_auth, &reply_msg)) {
#else
			if (nrefreshes > 0 && AUTH_REFRESH(cl->cl_auth)) {
#endif
				nrefreshes--;
				goto call_again;
			}
		}		/* end of unsuccessful completion */
	} /* end of valid reply message */
	else {
		cu->cu_error.re_status = RPC_CANTDECODERES;
	}
	return cu->cu_error.re_status;

	return 0;
}

void clnt_pci_geterr(CLIENT * cl, struct rpc_err *errp)
{

}

#if defined(USE_TIRPC)
bool_t clnt_pci_freeres(CLIENT *cl, xdrproc_t xdr_res, void *res_ptr)
#else
bool_t clnt_pci_freeres(CLIENT *cl, xdrproc_t xdr_res, caddr_t res_ptr)
#endif
{
	return 0;
}

#if defined(USE_TIRPC)
void clnt_pci_abort(CLIENT *cl)
#else
void clnt_pci_abort(void)
#endif
{

}

#if defined(USE_TIRPC)
bool_t clnt_pci_control(CLIENT *cl, unsigned int request, void *info)
#else
bool_t clnt_pci_control(CLIENT *cl, int request, char *info)
#endif
{
	return 0;
}

#ifdef PCIE_RPC_LOCK_ENABLE
void hidden_clnt_pci_destroy(CLIENT * cl)
#else
void clnt_pci_destroy(CLIENT * cl)
#endif
{
	struct cu_data *cu = (struct cu_data *)cl->cl_private;

	if (cu->cu_sock >= 0) {
		close(cu->cu_sock);
	}
	XDR_DESTROY(&(cu->cu_outxdrs));
	free((caddr_t) cu->cu_reqnlh);
	free((caddr_t) cu->cu_respnlh);
	free((caddr_t) cu);
	free((caddr_t) cl);
}

#ifdef PCIE_RPC_LOCK_ENABLE
void clnt_pci_destroy(CLIENT * cl)
{
	client_qcsapi_set_callbacks(NULL, NULL, NULL);
}

void * qcsapi_pcie_pre(const char *tmp)
{
	CLIENT *tmp_clnt_pci;
	tmp_clnt_pci = hidden_clnt_pci_create("localhost", QCSAPI_PROG, QCSAPI_VERS, NULL);

	if (tmp_clnt_pci)
		hidden_client_qcsapi_set_rpcclient(tmp_clnt_pci);
	else
		printf("%s %d clnt_pci_create error\n", __FUNCTION__, __LINE__);

	return tmp_clnt_pci;
}

void qcsapi_pcie_post(const char *tmp, int was_error)
{
	if (__clnt_pci)
		hidden_clnt_pci_destroy(__clnt_pci);
	else
		printf("%s %d __clnt_pci is NULL\n", __FUNCTION__, __LINE__);
}
#endif

