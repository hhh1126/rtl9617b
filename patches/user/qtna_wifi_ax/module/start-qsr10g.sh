#/bin/sh

echo "Loading QSR10G driver"
insmod /lib/modules/qsr10g-pcie.ko
sleep 15

echo "bridge host0 interface to LAN"
brctl addif br0 host0
sleep 2
ifconfig host0 up
sleep 2

echo "Config FC (Fleet Contrak)"
echo wlan 0 mode 1 cpu 3 > /proc/fc/ctrl/smp_dispatch_wifi_tx
echo wlan 0 mode 1 cpu 2 > /proc/fc/ctrl/smp_dispatch_wifi_rx
sleep 2

echo "Enable nic rx skb refill"
echo 1 > /proc/driver/cortina/ni/qm_eq_refill_inttrupt
sleep 2

echo "Config smp_affinity"
echo 2 > /proc/irq/23/smp_affinity
echo 2 > /proc/irq/24/smp_affinity
echo 1 > /proc/irq/13/smp_affinity
sleep 2

echo "Enable statistics"
echo 1 > /proc/fc/sw_dump/fwd_statistic
echo 1 > /proc/fc/sw_dump/smp_statistic
sleep 2

echo "Verify wifi FC status"
cat /proc/fc/sw_dump/wlan_devmap

echo ""
echo "To dump the FC statistics, run the following command:"
echo "cat /proc/interrupts"
echo "cat /proc/fc/hw_dump/dropcount"
echo "cat /proc/fc/sw_dump/fwd_statistic"
echo "cat /proc/fc/sw_dump/smp_statistic"
echo ""

echo "Done"
