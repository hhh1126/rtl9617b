/*SH0
*******************************************************************************
**                                                                           **
**         Copyright (c) 2009 - 2011 Quantenna Communications Inc            **
**                                                                           **
**  File        : call_qcsapi_local.c                                        **
**  Description : tiny wrapper to invoke call_qcsapi locally, from main()    **
**                                                                           **
*******************************************************************************
**                                                                           **
**  Redistribution and use in source and binary forms, with or without       **
**  modification, are permitted provided that the following conditions       **
**  are met:                                                                 **
**  1. Redistributions of source code must retain the above copyright        **
**     notice, this list of conditions and the following disclaimer.         **
**  2. Redistributions in binary form must reproduce the above copyright     **
**     notice, this list of conditions and the following disclaimer in the   **
**     documentation and/or other materials provided with the distribution.  **
**  3. The name of the author may not be used to endorse or promote products **
**     derived from this software without specific prior written permission. **
**                                                                           **
**  Alternatively, this software may be distributed under the terms of the   **
**  GNU General Public License ("GPL") version 2, or (at your option) any    **
**  later version as published by the Free Software Foundation.              **
**                                                                           **
**  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR       **
**  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES**
**  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  **
**  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,         **
**  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT **
**  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,**
**  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    **
**  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      **
**  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF **
**  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.        **
**                                                                           **
*******************************************************************************
EH0*/

#include <stdio.h>
#include <call_qcsapi.h>
#include "qcsapi_bldinfo.h"
#include <qcsapi_output.h>
#include <qcsapi_rpc_common/client/find_host_addr.h>
#include <qcsapi_rpc/client/qcsapi_rpc_client.h>
#include <qcsapi_rpc/generated/qcsapi_rpc.h>

static int client_qcsapi_get_udp_retry_timeout(int *argc, char ***argv)
{
	int timeout = -1;

	if (argc && argv && *argc >= 2 && strcmp((*argv)[1], "--udp-retry-timeout") == 0) {
		timeout = atoi((const char *)(*argv)[2]);

		/* move program argv[0] */
		(*argv)[2] = (*argv)[0];

		/* skip over --host <arg> args */
		*argc = *argc - 2;
		*argv = &(*argv)[2];
	}

	return timeout;
}

int main(int argc, char **argv)
{
	int ret, param_cut = 0;
	const char *host;
	int udp_retry_timeout;
	CLIENT *clnt;
	struct qcsapi_output output;
	char *gctrl_iface;
	const char *usage_ap = "QRPC: reload_in_mode <WiFi interface> ap"
					" [ <global ctrl iface> ] [ <config file> ]";
	const char *usage_sta = "QRPC: reload_in_mode <WiFi interface> sta";

	if (argc >= 2 && strcmp(argv[1], "--version") == 0) {
		const char *label = qcsapi_bldinfo_label();

		printf("%s\n", &label[1]);	/* strip leading 'v' */
		exit(1);
	}

	if (argc < 2) {
		printf("Usage:\n");
		printf("    %s [ --host <device IP address> ] <QCSAPI command> [<arg> ...]\n",
				argv[0]);
		printf("    %s --version\n", argv[0]);
		exit(1);
	}

	/* When switch to ap, there are two more parameters(global_ctrl_iface and hostapd_conf_file)
	 * which only used in this function, so never pass these two parameters. */
	if (!strcmp(argv[1], "reload_in_mode")) {
		if (argc < 4) {
			printf("Usage:\n");
			printf("    %s\n", usage_ap);
			printf("    %s\n", usage_sta);
			exit(1);
		}

		if (!strcmp(argv[3], "ap")) {
			if (argc != 4 && argc != 6) {
				printf("Usage:\n");
				printf("    %s\n", usage_ap);
				exit(1);
			}

			if (argc == 6) {
				gctrl_iface = strrchr(argv[4], '/');
				if (gctrl_iface == NULL) {
					printf("global ctrl iface is incorrect!\n");
					exit(1);
				}
				param_cut = 2;
			}
		}
	}

	output = qcsapi_output_stdio_adapter();

	host = client_qcsapi_find_host_addr(&argc, &argv);
	if (!host) {
		client_qcsapi_find_host_errmsg(argv[0]);
		exit(1);
	}

	udp_retry_timeout = client_qcsapi_get_udp_retry_timeout(&argc, &argv);

	clnt = clnt_create(host, QCSAPI_PROG, QCSAPI_VERS, "udp");
	if (clnt == NULL) {
		clnt = clnt_create(host, QCSAPI_PROG, QCSAPI_VERS, "tcp");
	} else {
		if (udp_retry_timeout>0) {
			struct timeval value;
			value.tv_sec = (time_t)udp_retry_timeout;
			value.tv_usec = (suseconds_t)0;
			clnt_control(clnt, CLSET_RETRY_TIMEOUT, (char *)&value);
		}
	}

	if (clnt == NULL) {
		clnt_pcreateerror(host);
		exit(1);
	}

	client_qcsapi_set_rpcclient(clnt);
	/* coverity[tainted_string] - No need to validate argv which may be random string value */
	ret = qcsapi_main(&output, argc - param_cut, argv);

	/* When switch to ap, start hostapd daemon, or add a hostapd iface when hostapd daemon is already running. */
	if ((ret == 0) && (!strcmp(argv[1], "reload_in_mode"))) {
		if (!strcmp(argv[3], "ap")) {
			/* Run hostapd command on host for Host-based hostapd mode */
			if (argc == 6)
				/* coverity[tainted_string] - No need to validate argv which may be random string value */
				rpc_run_hostapd(&argv[2]);
		}
	}

	clnt_destroy(clnt);

	return ret;
}
