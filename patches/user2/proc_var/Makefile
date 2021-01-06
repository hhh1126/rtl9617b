ifdef CONFIG_YUEME
YUEME_CU_FEATURE=y
endif
ifdef CONFIG_CU_BASEON_YUEME
YUEME_CU_FEATURE=y
endif

RC_LEVEL_FIRST=/etc/init.d/rc2
RC_LEVEL_START=/etc/init.d/rc2
RC_LEVEL_INS_MOD=/etc/init.d/rc3
ifdef CONFIG_OVERLAY_FS
ifdef YUEME_CU_FEATURE
RC_LEVEL_OVERLAY=/etc/init.d/rc4
endif
endif
ifdef CONFIG_CMCC
RC_LEVEL_REPLUG=/etc/init.d/rc14
endif
ifdef CONFIG_CU_BASEON_CMCC
RC_LEVEL_REPLUG=/etc/init.d/rc14
endif
RC_LEVEL_UDEV=/etc/init.d/rc20
RC_LEVEL_PROC=/etc/init.d/rc10
ifeq ($(CONFIG_GPON_FEATURE),y)
RC_LEVEL_PON_DRV=/etc/init.d/rc3
RC_LEVEL_PON=/etc/init.d/rc3
else
RC_LEVEL_PON_DRV=/etc/init.d/rc21
ifeq ($(CONFIG_RTK_OAM_V1),y)
RC_LEVEL_PON=/etc/init.d/rc22
else
RC_LEVEL_PON=/etc/init.d/rc32
endif
endif
#RC_LEVEL_WDT MUST be greater than RC_LEVEL_PON for OMCI firmware upgrade fail-safe requirement
RC_LEVEL_WDT=/etc/init.d/rc34
RC_LEVEL_IPV6=/etc/init.d/rc14
RC_LEVEL_SLAVE=/etc/init.d/rc3
RC_LEVEL_FINISH=/etc/init.d/rc35
RC_LEVEL_DEBUGFS=/etc/init.d/rc10

-include $(LINUX_CONFIG)

ifdef CONFIG_MTD_LUNA_NOR_SPI
_CONFIG_LUNA_HAS_NOR=y
endif
ifdef CONFIG_MTD_NOR_SPIF_G3
_CONFIG_LUNA_HAS_NOR=y
endif
ifdef CONFIG_MTD_CORTINA_CA_SERIAL
_CONFIG_LUNA_HAS_NOR=y
endif

ifdef CONFIG_YUEME_TMPFS_SIZE_FEATURE
TMPFS=${CONFIG_YUEME_TMPFS_SIZE_FEATURE}
endif

USE_BOARD_LED_INIT=1

FRAMWORK_PATH = $(subst ",,$(CONFIG_DBUS_FRAMEWORK_PATH))

all:

romfs:
	[ -d $(ROMFSDIR)/overlay ] || mkdir -p $(ROMFSDIR)/overlay
	[ -d  $(ROMFSDIR)/run ] || mkdir -p  $(ROMFSDIR)/run
	[ -d  $(ROMFSDIR)/etc/default ] || mkdir -p  $(ROMFSDIR)/etc/default
	[ -d $(ROOTDIR)/images/ ] || mkdir $(ROOTDIR)/images/
ifdef CONFIG_USER_CONF_ON_XMLFILE
	[ ! -f $(ROOTDIR)/user2/proc_var/gen_static_conf.sh ] || rm gen_static_conf.sh
	ln -s gen_static_conf_xmlfile.sh gen_static_conf.sh
else
	[ ! -f $(ROOTDIR)/user2/proc_var/gen_static_conf.sh ] || rm gen_static_conf.sh
	ln -s gen_static_conf_xmlconfig.sh gen_static_conf.sh
endif
	cp gen_static_conf.sh $(ROOTDIR)/images/

ifdef CONFIG_DEV_xDSL
	$(ROMFSINST) -s /sbin/init /init
endif

.PHONY: rcX
rcX:	rcX_kernel
ifeq ($(CONFIG_PRODUCT), luna_xdsl)
	$(ROMFSINST) -a "/bin/mount -t proc proc /proc" $(RC_LEVEL_FIRST)
endif
	$(ROMFSINST) devpts /etc/default/devpts
	$(ROMFSINST) sysfs.sh /etc/scripts/sysfs.sh
	$(ROMFSINST) devpts.sh /etc/scripts/devpts.sh
	$(ROMFSINST) rcS /etc/default/rcS
	$(ROMFSINST) checkroot.sh /etc/scripts/checkroot.sh
ifneq ($(CONFIG_PRODUCT), luna_xdsl)
	$(ROMFSINST) -a "/bin/sh /etc/scripts/sysfs.sh" $(RC_LEVEL_FIRST)
	$(ROMFSINST) -a "/bin/sh /etc/scripts/devpts.sh" $(RC_LEVEL_FIRST)
	$(ROMFSINST) -a "/bin/sh /etc/scripts/checkroot.sh" $(RC_LEVEL_FIRST)
endif
#For all g3hgu/venus board.
ifdef CONFIG_ARCH_CORTINA
ifdef CONFIG_ARCH_CORTINA_G3HGU
	$(ROMFSINST) ca8277_tem_dump.sh /etc/scripts/hgu-show-temp.sh
endif
ifdef CONFIG_ARCH_CORTINA_VENUS
	$(ROMFSINST) ca8277b_tem_dump.sh /etc/scripts/hgu-show-temp.sh
endif

#Change default VM paratemer
	$(ROMFSINST) vm_tuning_g3hgu.sh /etc/scripts/vm_tuning_g3hgu.sh
	$(ROMFSINST) -a "/bin/sh /etc/scripts/vm_tuning_g3hgu.sh" $(RC_LEVEL_FINISH)
endif

ifdef CONFIG_RTK_KERNEL_OMD
	$(ROMFSINST) -a "echo 1 > /proc/sys/kernel/panic_on_oops" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 5 > /proc/sys/kernel/panic" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 1 > /proc/sys/vm/panic_on_oom" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 1 > /proc/sys/vm/oom_kill_allocating_task" $(RC_LEVEL_PROC)
endif
ifdef YUEME_CU_FEATURE
ifdef CONFIG_TMPFS
	$(ROMFSINST) mnt_tmpfs.sh /etc/scripts/mnt_tmpfs.sh
	$(ROMFSINST) -a "/bin/sh /etc/scripts/mnt_tmpfs.sh var ${TMPFS}" $(RC_LEVEL_FIRST)
	$(ROMFSINST) -a "/bin/sh /etc/scripts/mnt_tmpfs.sh mnt ${TMPFS}" $(RC_LEVEL_FIRST)
else
	$(ROMFSINST) -a "/bin/mount -t ramfs ramfs /var" $(RC_LEVEL_FIRST)
endif
else
	$(ROMFSINST) -a "/bin/mount -t ramfs ramfs /var" $(RC_LEVEL_FIRST)
endif
#	$(ROMFSINST) -a "/bin/mount -t sysfs sysfs /sys" $(RC_LEVEL_FIRST)
ifdef CONFIG_CORTINA_EUDEV
# UDEV Daemon
	$(ROMFSINST) -a "/bin/sh /etc/init.d/udev start" $(RC_LEVEL_UDEV)
	$(ROMFSINST) -a "/bin/sh /etc/init.d/udev-cache" $(RC_LEVEL_FINISH)
endif
# Insmod extra modules
ifneq ($(CONFIG_PRODUCT), luna_xdsl)
	$(ROMFSINST) -a "/bin/sh /etc/scripts/modutils.sh" $(RC_LEVEL_PON_DRV)
endif

# For G3 MTD
ifdef CONFIG_UBIFS_FS
ifdef CONFIG_USER_G3_MTD_UTILS
	$(ROMFSINST) mount_G3_UBIFS.sh /etc/scripts/mount_G3_UBIFS.sh
#Example:  mount_G3_UBIFS.sh <MTD_NAME> <Your UBI_VolNAME> <MountPoint>
#       : mount mtd "apps" with UBI_Volename "apps" to /opt/upt/apps/
ifdef CONFIG_USER_UBIFS_APPS
ifdef CONFIG_RTK_USE_ONE_UBI_DEVICE
	$(ROMFSINST) -a "mount -t ubifs ubi0:ubi_apps /opt/upt/apps/ || mount -t ubifs ubi0:ubi_app /opt/upt/apps/" $(RC_LEVEL_START)
else
	$(ROMFSINST) -a "/bin/sh /etc/scripts/mount_G3_UBIFS.sh apps apps /opt/upt/apps/" $(RC_LEVEL_START)
endif
endif 
endif #CONFIG_USER_G3_MTD_UTILS
endif #CONFIG_UBIFS_FS

#ifdef CONFIG_OVERLAY_FS
ifdef CONFIG_UBIFS_FS
ifdef YUEME_CU_FEATURE
ifdef CONFIG_RTK_USE_ONE_UBI_DEVICE
	$(ROMFSINST) -a "mount -t ubifs ubi0:ubi_apps $(FRAMWORK_PATH)/apps/ || mount -t ubifs ubi0:ubi_app $(FRAMWORK_PATH)/apps/" $(RC_LEVEL_START)
else
#	$(ROMFSINST) mount_ubifs.sh /etc/scripts/mount_ubifs.sh
#	$(ROMFSINST) -a "/bin/sh /etc/scripts/mount_ubifs.sh apps /dev/ubi0 /dev/ubi0_0 $(FRAMWORK_PATH)/apps zlib" $(RC_LEVEL_OVERLAY)
	$(ROMFSINST) -a "/bin/sh /etc/scripts/mount_G3_UBIFS.sh apps apps $(FRAMWORK_PATH)/apps/" $(RC_LEVEL_START)
endif
	$(ROMFSINST) -a "mkdir -p $(FRAMWORK_PATH)/apps/etc/dbus-1/system.d" $(RC_LEVEL_OVERLAY)
ifdef CONFIG_CU
	$(ROMFSINST) -a "mkdir -p $(FRAMWORK_PATH)/apps/etc/dbus-1/session.d" $(RC_LEVEL_OVERLAY)
	$(ROMFSINST) -a "mkdir -p $(FRAMWORK_PATH)/apps/etc/dbus-1/services" $(RC_LEVEL_OVERLAY)
	$(ROMFSINST) -a "mkdir -p $(FRAMWORK_PATH)/apps/etc/dbus-1/system-services" $(RC_LEVEL_OVERLAY)
endif
endif
endif

ifdef CONFIG_CMCC
	$(ROMFSINST) -a "netlink &" $(RC_LEVEL_REPLUG)
	$(ROMFSINST) cmcc_loopback.sh /etc/scripts/cmcc_loopback.sh
	$(ROMFSINST) update_monitord /etc/scripts/update_monitord
	$(ROMFSINST) wlan_off.sh /etc/scripts/wlan_off.sh
#	$(ROMFSINST) -a "/etc/scripts/cmcc_loopback.sh &" $(RC_LEVEL_FINISH)
ifdef CONFIG_CMCC_DMIPS
	$(ROMFSINST) -a "echo 3/2 > /proc/dmipsapp" $(RC_LEVEL_FINISH)
endif
endif

ifdef CONFIG_CU_BASEON_CMCC
	$(ROMFSINST) -a "netlink &" $(RC_LEVEL_REPLUG)
	$(ROMFSINST) cmcc_loopback.sh /etc/scripts/cmcc_loopback.sh
	$(ROMFSINST) update_monitord /etc/scripts/update_monitord
	$(ROMFSINST) wlan_off.sh /etc/scripts/wlan_off.sh
#	$(ROMFSINST) -a "/etc/scripts/cmcc_loopback.sh &" $(RC_LEVEL_FINISH)
ifdef CONFIG_CMCC_DMIPS
	$(ROMFSINST) -a "echo 3/2 > /proc/dmipsapp" $(RC_LEVEL_FINISH)
endif
endif

	$(ROMFSINST) -a "/bin/echo 4096 > /proc/sys/vm/min_free_kbytes" $(RC_LEVEL_FINISH)
ifdef CONFIG_HIGHMEM
	$(ROMFSINST) vm_tuning.sh /etc/scripts/vm_tuning.sh
	$(ROMFSINST) -a "/etc/scripts/vm_tuning.sh" $(RC_LEVEL_FINISH)
endif

ifdef CONFIG_USER_JAMVM
	$(ROMFSINST) -a "jamvm -Xms30M -Xmx60M -Dgosh.args=--noi -jar /usr/local/class/felix/bin/felix.jar /tmp/felix-cache &" $(RC_LEVEL_FINISH)
endif
ifdef CONFIG_USER_OPENJDK8
ifdef CONFIG_APACHE_FELIX_FRAMEWORK
	$(ROMFSINST) -a "export USER=CMCCAdmin" $(RC_LEVEL_FINISH)
	$(ROMFSINST) -a "[ -d /var/osgi_app/bundle ] || mkdir -p /var/osgi_app/bundle" $(RC_LEVEL_FINISH)
	$(ROMFSINST) -a "[ -f /var/osgi_app/config.properties ] || cp /usr/local/class/felix/conf/config.properties /var/osgi_app/config.properties" $(RC_LEVEL_FINISH)
	$(ROMFSINST) -a "[ -f /var/osgi_app/security.policy ] || /etc/scripts/osgi_default_permission /var/osgi_app/security.policy" $(RC_LEVEL_FINISH)
ifndef CONFIG_RTK_IPV6_READYLOGO_ROUTER
ifndef CONFIG_RTK_IPV6_READYLOGO_HOST
	$(ROMFSINST) -a "echo 'java' >> /var/monitor_list" $(RC_LEVEL_FINISH)
endif
endif
	$(ROMFSINST) -a "/etc/scripts/update_monitord" $(RC_LEVEL_FINISH)
	$(ROMFSINST) -a "/sbin/mkfifo /tmp/osgi_fifo" $(RC_LEVEL_REPLUG)
	$(ROMFSINST) -a "/sbin/mkfifo /tmp/permission_fifo" $(RC_LEVEL_REPLUG)
endif
endif

ifndef CONFIG_USER_NFBI_SLAVE
ifndef CONFIG_ARCH_LUNA_SLAVE
	$(ROMFSINST) -a "/bin/echo /sbin/mdev > /proc/sys/kernel/hotplug" $(RC_LEVEL_FIRST)
endif
#	$(ROMFSINST) -a "/bin/mount -t tmpfs mdev /dev" $(RC_LEVEL_FIRST)
ifndef CONFIG_ARCH_LUNA_SLAVE
#	$(ROMFSINST) -a "/bin/mknod /dev/adsl0 c 100 0" $(RC_LEVEL_FIRST)
endif
#	$(ROMFSINST) mdev.conf /etc/mdev.conf
ifdef CONFIG_USER_NFBI_MASTER
	$(ROMFSINST) -a "/bin/mknod /dev/rtl_nfbi c 14 0" $(RC_LEVEL_FIRST)
endif #CONFIG_USER_NFBI_MASTER
ifdef CONFIG_USER_NFBI_SLAVE
	$(ROMFSINST) -a "/bin/mknod /dev/rtl_mdio c 14 0" $(RC_LEVEL_FIRST)
endif #CONFIG_USER_NFBI_SLAVE
ifneq ($(CONFIG_PRODUCT), luna_xdsl)
	$(ROMFSINST) -a "mdev -s" $(RC_LEVEL_FIRST)
endif
endif
ifneq ($(CONFIG_PRODUCT), luna_xdsl)
	$(ROMFSINST) -a "mkdir /var/tmp" $(RC_LEVEL_START)
endif
ifdef YUEME_CU_FEATURE
ifdef CONFIG_TMPFS
	$(ROMFSINST) -a "/bin/sh /etc/scripts/mnt_tmpfs.sh tmp ${TMPFS}" $(RC_LEVEL_START)
	$(ROMFSINST) -a "/bin/sh /etc/scripts/mnt_tmpfs.sh mnt ${TMPFS}" $(RC_LEVEL_START)
endif
endif
	$(ROMFSINST) -a "mkdir /var/log" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/run" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/lock" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/ppp" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/config" $(RC_LEVEL_START)
ifdef CONFIG_USER_ADAPTER_API_ISP
	$(ROMFSINST) -a "mkdir /var/app" $(RC_LEVEL_START)
endif
ifdef YUEME_CU_FEATURE
	$(ROMFSINST) -a "mkdir /var/tmp2" $(RC_LEVEL_START)
	$(ROMFSINST) -a "/bin/mount -t ramfs ramfs /var/tmp2" $(RC_LEVEL_START)
#### Prevent notify to dbus in booting, and remove it when startup process	
	$(ROMFSINST) -a "touch /tmp/.runRestore" $(RC_LEVEL_START)
	$(ROMFSINST) -a "ln -s /tmp/TZ /var" $(RC_LEVEL_START)
	$(ROMFSINST) -a "ln -s /var/config/firmware_upgrade_status  /var/firmware_upgrade_status" $(RC_LEVEL_START)

endif
ifdef CONFIG_CMCC
	$(ROMFSINST) -a "mkdir /var/interface" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/interface/uptime" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/bundle/" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/bundle/event" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/osgi_app" $(RC_LEVEL_START)
endif
ifdef CONFIG_CU_BASEON_CMCC
	$(ROMFSINST) -a "mkdir /var/interface" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/interface/uptime" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/bundle/" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/bundle/event" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/osgi_app" $(RC_LEVEL_START)
endif
ifeq ($(CONFIG_PRODUCT), luna_xdsl)
	$(ROMFSINST) -e CONFIG_DEV_xDSL -a "alias adslctrl=\"adslctrl --\"" /etc/profile
endif
ifdef CONFIG_MTD_NAND
ifdef CONFIG_YAFFS_FS
#	$(ROMFSINST) -a "mount -t yaffs2 -o tags-ecc-off /dev/mtdblock4 /var/config/" $(RC_LEVEL_START)
ifdef CONFIG_RTK_DEV_AP
	$(ROMFSINST) mnt_cfgfs_ap.sh /etc/scripts/mnt_cfgfs.sh
else
	$(ROMFSINST) mnt_cfgfs.sh /etc/scripts/mnt_cfgfs.sh
endif
ifndef  CONFIG_RTK_USE_ONE_UBI_DEVICE
	$(ROMFSINST) -a "/etc/scripts/mnt_cfgfs.sh \"-t yaffs2 -o tags-ecc-off\" \"config\" \"/var/config\" || /etc/scripts/mnt_cfgfs.sh \"-t yaffs2 -o tags-ecc-off\" \"rootfs3\" \"/var/config\"" $(RC_LEVEL_START)
endif
else
ifdef CONFIG_JFFS2_FS
	$(ROMFSINST) mnt_cfgfs.sh /etc/scripts/mnt_cfgfs.sh
	$(ROMFSINST) -a "/etc/scripts/mnt_cfgfs.sh \"-t jffs2\" \"config\" \"/var/config\"" $(RC_LEVEL_START)
else
	echo "ERROR: No valid file system"
	exit 0
endif
endif
endif

ifdef CONFIG_APACHE_FELIX_FRAMEWORK
ifdef CONFIG_UBIFS_FS
ifdef CONFIG_RTK_USE_ONE_UBI_DEVICE
	$(ROMFSINST) -a "mount -t ubifs -o sync ubi0:ubi_app /var/osgi_app" $(RC_LEVEL_START)
endif
endif
ifdef CONFIG_CMCC
	$(ROMFSINST) mnt_fs.sh /etc/scripts/mnt_fs.sh
	$(ROMFSINST) -a "/etc/scripts/mnt_fs.sh \"-t squashfs -o loop\" \"osgi\" \"/usr/local\"" $(RC_LEVEL_START)
endif
ifdef CONFIG_CU_BASEON_CMCC
	$(ROMFSINST) mnt_fs.sh /etc/scripts/mnt_fs.sh
	$(ROMFSINST) -a "/etc/scripts/mnt_fs.sh \"-t squashfs -o loop\" \"osgi\" \"/usr/local\"" $(RC_LEVEL_START)
endif
endif

ifdef CONFIG_STATIC_CONFIG
	$(ROMFSINST) mnt_static_cfg.sh /etc/scripts/mnt_static_cfg.sh
	$(ROMFSINST) -a "/etc/scripts/mnt_static_cfg.sh \"-t squashfs\" \"static_conf\" \"/var/config_static\"" $(RC_LEVEL_START)
endif

ifdef CONFIG_RTK_USE_ONE_UBI_DEVICE
	$(ROMFSINST) -a "mount -t ubifs ubi0:ubi_Config /var/config" $(RC_LEVEL_START)
endif
ifdef CONFIG_USER_ADAPTER_API_ISP
	$(ROMFSINST) -a "mount -t ubifs ubi0:ubi_app /var/app" $(RC_LEVEL_START)
endif
ifdef CONFIG_USER_CTCAPD
	$(ROMFSINST) -a "mkdir /var/app/ctcap" $(RC_LEVEL_START)
endif
ifdef _CONFIG_LUNA_HAS_NOR
ifdef CONFIG_JFFS2_FS
	$(ROMFSINST) mnt_cfgfs.sh /etc/scripts/mnt_cfgfs.sh
	$(ROMFSINST) -a "/etc/scripts/mnt_cfgfs.sh \"-t jffs2\" \"config\" \"/var/config\"" $(RC_LEVEL_START)
else
	echo "ERROR: No valid file system-require JFFS2 enabled in linux kernel"
	exit 1
endif
endif
ifdef CONFIG_ARCH_CORTINA
	$(ROMFSINST) create_scfg_txt.sh /etc/scripts/create_scfg_txt.sh
	$(ROMFSINST) -a "/bin/sh /etc/scripts/create_scfg_txt.sh" $(RC_LEVEL_START)
endif

ifeq ($(CONFIG_PRODUCT), luna_xdsl)
	$(ROMFSINST) -a "/bin/mount -t sysfs sysfs /sys" $(RC_LEVEL_FIRST)
	$(ROMFSINST) -a "/bin/mount -t tmpfs mdev /dev" $(RC_LEVEL_FIRST)
	$(ROMFSINST) -a "/bin/mknod /dev/adsl0 c 100 0" $(RC_LEVEL_FIRST)
	$(ROMFSINST) -a "/bin/mdev -s" $(RC_LEVEL_FIRST)
	$(ROMFSINST) -a "mkdir /var/tmp" $(RC_LEVEL_START)	
endif

	$(ROMFSINST) -a "mkdir /var/udhcpd" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/udhcpc" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/mnt" $(RC_LEVEL_START)
	$(ROMFSINST) -a "chmod 777 /var/mnt" $(RC_LEVEL_START)
ifdef CONFIG_CMCC
	$(ROMFSINST) -a "[ -d /var/osgi_app/security ] || mkdir -p /var/osgi_app/security" $(RC_LEVEL_START)
endif
ifdef CONFIG_CU_BASEON_CMCC
	$(ROMFSINST) -a "[ -d /var/osgi_app/security ] || mkdir -p /var/osgi_app/security" $(RC_LEVEL_START)
endif
	$(ROMFSINST) -a "mkdir /var/ct" $(RC_LEVEL_START)
	$(ROMFSINST) -a "mkdir /var/ct/tmp" $(RC_LEVEL_START)

ifdef CONFIG_RTK_SMART_ROAMING
	$(ROMFSINST) -a "mkdir /var/capwap" $(RC_LEVEL_START)
endif
	$(ROMFSINST) -a "ifconfig lo 127.0.0.1" $(RC_LEVEL_START)
	$(ROMFSINST) -a "route add -net 127.0.0.0 netmask 255.255.255.0 lo" $(RC_LEVEL_START)
	$(ROMFSINST) -a "PATH=.:$\PATH" $(RC_LEVEL_START)
	$(ROMFSINST) -a "cd /etc/scripts" $(RC_LEVEL_START)

ifeq ($(CONFIG_GPON_FEATURE),y)
ifeq ($(CONFIG_RTK_L34_ENABLE),y)
	$(ROMFSINST) -a "/bin/initRGapi" $(RC_LEVEL_PON_DRV)
endif
	$(ROMFSINST) -a "/etc/insdrv.sh" $(RC_LEVEL_PON_DRV)
ifeq ($(CONFIG_EPON_FEATURE),y)
	$(ROMFSINST) -a "/bin/checkoam" $(RC_LEVEL_PON_DRV)
endif
	$(ROMFSINST) -a "if [ -f /bin/nice ]; then nice -n -18 /etc/runsdk.sh ; else /etc/runsdk.sh; fi" $(RC_LEVEL_PON)
else
ifeq ($(CONFIG_EPON_FEATURE),y)
        $(ROMFSINST) -a "/bin/checkoam" $(RC_LEVEL_PON_DRV)
	$(ROMFSINST) -a "if [ -f /bin/nice ]; then nice -n -18 /etc/runsdk.sh ; else /etc/runsdk.sh; fi" $(RC_LEVEL_PON)
else
ifeq ($(CONFIG_FIBER_FEATURE),y)
	$(ROMFSINST) -a "if [ -f /bin/nice ]; then nice -n -18 /etc/runsdk.sh ; else /etc/runsdk.sh; fi" $(RC_LEVEL_PON)
else
ifeq ($(CONFIG_CA_RTK_GPON_FEATURE),y)
	$(ROMFSINST) -a "/etc/insdrv.sh" $(RC_LEVEL_PON_DRV)
	$(ROMFSINST) -a "if [ -f /bin/nice ]; then nice -n -18 /etc/runsdk.sh ; else /etc/runsdk.sh; fi" $(RC_LEVEL_PON)
else
ifeq ($(CONFIG_CA_RTK_EPON_FEATURE),y)
	$(ROMFSINST) -a "if [ -f /bin/nice ]; then nice -n -18 /etc/runsdk.sh ; else /etc/runsdk.sh; fi" $(RC_LEVEL_PON)
endif
endif
endif
endif
endif

ifeq ($(CONFIG_RTK_IGMP_MLD_SNOOPING_MODULE),m)
	$(ROMFSINST) ins_rtk_snooping_mod.sh /etc/scripts/ins_rtk_snooping_mod.sh  
	$(ROMFSINST) -a "/bin/sh /etc/scripts/ins_rtk_snooping_mod.sh" $(RC_LEVEL_INS_MOD)
endif

ifeq ($(CONFIG_LAN_SDS_FEATURE),y)
	 $(ROMFSINST) -a "/etc/runlansds.sh" $(RC_LEVEL_PON)
endif
ifeq ($(CONFIG_LAN_SDS1_FEATURE),y)
	$(ROMFSINST) -a "/etc/runlansds1.sh" $(RC_LEVEL_PON)
endif

ifeq ($(CONFIG_USER_G3_WATCHDOG),y)
	#-p period: 6 sec
	$(ROMFSINST) -a "nice -n -20 /bin/rtk_watchdog -p 6 &" $(RC_LEVEL_WDT)
endif

ifeq ($(CONFIG_LUNA_WDT_KTHREAD),y)
ifeq ($(CONFIG_WDT_ENABLE),1)
	#kick when wdt module inserted
else ifeq ($(CONFIG_WDT_ENABLE),2)
	#kick when RC34 runs
	$(ROMFSINST) -a "echo 1 > /proc/luna_watchdog/watchdog_flag" $(RC_LEVEL_WDT)
endif
endif

rcX_kernel:
ifdef CONFIG_LUNA_DUAL_LINUX
ifndef CONFIG_ARCH_LUNA_SLAVE
	$(ROMFSINST) -a "echo 'Bootup Slave CPU'" $(RC_LEVEL_SLAVE)
	$(ROMFSINST) -a "/etc/rc_boot_dual_linux" $(RC_LEVEL_SLAVE)
	$(ROMFSINST) -a "mkdir -p /tmp/slave" $(RC_LEVEL_SLAVE)
	$(ROMFSINST) -s /var/ethers /etc/ethers
endif
endif
ifeq ($(CONFIG_RTL867X_NFBI_MASTER), y)
ifneq ($(CONFIG_REMOTE_ADSL_HOST), y)
	$(ROMFSINST) -a "nfbi fw_w_boot 0 /etc/fw/boot.bin /etc/fw/linux.bin" $(RC_LEVEL_SLAVE)
endif
endif
	$(ROMFSINST) -a "echo 2000 > /proc/sys/net/core/netdev_max_backlog" $(RC_LEVEL_PROC)
ifndef CONFIG_ARCH_LUNA_SLAVE
	$(ROMFSINST) -a "echo 600 > /proc/sys/net/ipv4/netfilter/ip_conntrack_tcp_timeout_established" $(RC_LEVEL_PROC)
endif

ifneq ($(CONFIG_MODULE_BUILD_IN),y)
	$(ROMFSINST) -a "if [ -f /bin/nice ]; then nice -n -18 /etc/runsdk.sh ; else /etc/runsdk.sh; fi" /etc/rc
endif

ifdef CONFIG_MEM_LIMITATION
	$(ROMFSINST) -a "echo 1500 > /proc/sys/net/ipv4/route/max_size" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 1500 > /proc/sys/net/ipv4/inet_peer_threshold" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 20 > /proc/sys/net/ipv4/inet_peer_minttl" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 60 > /proc/sys/net/ipv4/inet_peer_maxttl" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 50 > /proc/sys/net/ipv4/netfilter/ip_conntrack_generic_timeout" $(RC_LEVEL_PROC)
endif
	$(ROMFSINST) -a "echo 10 > /proc/sys/net/ipv4/tcp_keepalive_intvl" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 3 > /proc/sys/net/ipv4/tcp_keepalive_probes" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 10 > /proc/sys/net/ipv4/tcp_keepalive_time" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 60 > /proc/sys/net/netfilter/nf_conntrack_tcp_timeout_time_wait" $(RC_LEVEL_PROC)
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv4/ip_forward" $(RC_LEVEL_PROC)
ifdef CONFIG_RTK_DEV_AP
	$(ROMFSINST) -a "echo 0 > /proc/sys/net/ipv4/conf/all/arp_ignore" $(RC_LEVEL_PROC)
else
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv4/conf/all/arp_ignore" $(RC_LEVEL_PROC)
endif
ifdef YUEME_CU_FEATURE #Specfic IGMPv2 only, because the test environment exist IGMPv3 but media server only accept IGMPv2
	$(ROMFSINST) -a "echo 2 > /proc/sys/net/ipv4/conf/default/force_igmp_version" $(RC_LEVEL_PROC)
endif
ifdef CONFIG_CMCC
	$(ROMFSINST) -a "echo 2 > /proc/sys/net/ipv4/conf/default/force_igmp_version" $(RC_LEVEL_PROC)
endif
ifdef CONFIG_CU_BASEON_CMCC
	$(ROMFSINST) -a "echo 2 > /proc/sys/net/ipv4/conf/default/force_igmp_version" $(RC_LEVEL_PROC)
endif
ifdef CONFIG_CU
	$(ROMFSINST) -a "echo 500 > /proc/sys/net/ipv4/icmp_ratelimit" $(RC_LEVEL_PROC)
endif
ifdef CONFIG_ARCH_LUNA_SLAVE # specific networking interface configuration with Slave CPU
	$(ROMFSINST) -a "echo 'Config Network Interface...'" $(RC_LEVEL_SLAVE)
	$(ROMFSINST) -a "/bin/brctl addbr br0" $(RC_LEVEL_SLAVE)
	$(ROMFSINST) -a "/bin/ifconfig vwlan hw ether $(CONFIG_DEFAULT_SLAVE_IPC_MAC_ADDRESS)" $(RC_LEVEL_SLAVE)
	$(ROMFSINST) -a "/bin/ifconfig vwlan 10.253.253.2 netmask 255.255.255.252 up" $(RC_LEVEL_SLAVE)
	$(ROMFSINST) -a "arp -s 10.253.253.1 $(CONFIG_DEFAULT_MASTER_IPC_MAC_ADDRESS)" $(RC_LEVEL_SLAVE)
endif

# Added by Mason Yu.
ifdef CONFIG_IPV6
	$(ROMFSINST) -a "echo 2 > /proc/sys/net/ipv6/conf/default/accept_dad" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv6/conf/all/forwarding" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv6/conf/all/autoconf" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6" $(RC_LEVEL_IPV6)
	# We need to open lo IPv6 for ping our interface's IP
	$(ROMFSINST) -a "echo 0 > /proc/sys/net/ipv6/conf/lo/disable_ipv6" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv6/conf/default/disable_ipv6" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 0 > /proc/sys/net/ipv6/conf/default/forwarding" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv6/conf/default/autoconf" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 0 > /proc/sys/net/ipv6/conf/default/accept_ra_defrtr" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 60 > /proc/sys/net/ipv6/conf/default/router_solicitations" $(RC_LEVEL_IPV6)
	$(ROMFSINST) -a "echo 10 > /proc/sys/net/ipv6/conf/default/router_solicitation_interval" $(RC_LEVEL_IPV6)
# ip_nonlocal_bind allows processes to bind() to non-local IP addresses. We need to use this for solving
# DHCP-4.4.1 can't get(bind) Link-local Address before kernel finishing DAD. It will cause DHCPv6 process exit. 
# Maybe we have another solution that modify systemd RTM_NEWADDR to get notify from LLA which finished DAD, then start DHCPv6 process in the future.
	$(ROMFSINST) -a "[ -f /proc/sys/net/ipv6/ip_nonlocal_bind ] && echo 1 > /proc/sys/net/ipv6/ip_nonlocal_bind" $(RC_LEVEL_IPV6)
ifdef CONFIG_CMCC
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv6/conf/default/force_mld_version" $(RC_LEVEL_IPV6)
endif
ifdef CONFIG_CU_BASEON_CMCC
	$(ROMFSINST) -a "echo 1 > /proc/sys/net/ipv6/conf/default/force_mld_version" $(RC_LEVEL_IPV6)
endif
endif

ifdef CONFIG_LUNA_FIRMWARE_UPGRADE_SUPPORT
ifdef YUEME_CU_FEATURE
	$(ROMFSINST) fwu_starter.sh_yueme /etc/scripts/fwu_starter.sh
else
ifdef CONFIG_00R0
	$(ROMFSINST) fwu_starter_crc.sh /etc/scripts/fwu_starter.sh
else
	$(ROMFSINST) fwu_starter.sh /etc/scripts/fwu_starter.sh
endif
endif
	$(ROMFSINST) fw_loaded.sh /etc/scripts/fw_loaded.sh
	$(ROMFSINST) -a "fw_loaded.sh" $(RC_LEVEL_FINISH)
ifdef CONFIG_LUNA_FWU_SYNC
	$(ROMFSINST) fw_cp.sh /etc/scripts/fw_cp.sh
	$(ROMFSINST) fw_sync.sh /etc/scripts/fw_sync.sh
	$(ROMFSINST) -a "fw_sync.sh" $(RC_LEVEL_FINISH)
endif
endif

ifdef CONFIG_RGMII_RESET_PROCESS
	$(ROMFSINST) -a "echo 'Enable RGMII Port...'" $(RC_LEVEL_FINISH)
	$(ROMFSINST) -a "echo 1 > /proc/rgmii_rst" $(RC_LEVEL_FINISH)
	$(ROMFSINST) -a "cp /etc/topaz-linux.lzma.img /tmp/topaz-linux.lzma.img" $(RC_LEVEL_FINISH)
endif

ifdef CONFIG_SWITCH_INIT_LINKDOWN
	$(ROMFSINST) -a "echo 'Turn on phy power...'" $(RC_LEVEL_FINISH)
	#$(ROMFSINST) -a "bin/diag port set phy-force-power-down port all state disable" $(RC_LEVEL_FINISH)
endif
	$(ROMFSINST) reset_default.sh /etc/scripts/reset_default.sh
	$(ROMFSINST) disable_printk.sh /etc/scripts/disable_printk.sh
	$(ROMFSINST) -a "/etc/scripts/disable_printk.sh 1" $(RC_LEVEL_PON_DRV)
	$(ROMFSINST) -a "/etc/scripts/disable_printk.sh 0" $(RC_LEVEL_FINISH)
ifeq ($(CONFIG_GPON_FEATURE),y)
ifeq ($(CONFIG_XML_ENABLE_ENCRYPT),y)
	$(ROMFSINST) update_ver.sh /etc/scripts/update_ver.sh
	$(ROMFSINST) -a "/etc/scripts/update_ver.sh" $(RC_LEVEL_PON_DRV)
else
	$(ROMFSINST) update_ver_normal.sh /etc/scripts/update_ver_normal.sh
	$(ROMFSINST) -a "/etc/scripts/update_ver_normal.sh" $(RC_LEVEL_PON_DRV)
endif
	$(ROMFSINST) -a "/bin/checkomci" $(RC_LEVEL_PON_DRV)
endif

ifeq ($(USE_BOARD_LED_INIT),1)
#	$(ROMFSINST) board_init.sh /etc/scripts/board_init.sh
#	$(ROMFSINST) -a "/bin/sh /etc/scripts/board_init.sh" $(RC_LEVEL_FINISH)
endif
ifeq ($(CONFIG_RPS),y)
ifneq (, $(wildcard $(ROOTDIR)/user2/proc_var/rps.sh)); then
	$(ROMFSINST) rps.sh /etc/scripts/rps.sh
endif
	$(ROMFSINST) -a "/bin/sh /etc/scripts/rps.sh on" $(RC_LEVEL_FINISH)
endif

ifeq ($(CONFIG_USER_RTK_SYSLOG),y)
	$(ROMFSINST) slogd_sync.sh /etc/scripts/slogd_sync.sh
endif

ifeq ($(CONFIG_RTK_MULTI_AP_WFA),y)
	$(ROMFSINST) -a "telnetd &"	$(RC_LEVEL_FINISH)
endif

#run_test.sh need at the last position
	$(ROMFSINST) -a "/var/config/run_test.sh >/dev/null 2>&1" $(RC_LEVEL_FINISH)

clean:

