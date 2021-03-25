#!/bin/sh

#USB2.0/3.0 LED Trigger
#If PORT1 exist , you need to add other trigger.
echo "1-1,2-1" > /sys/class/leds/LED_USB0/device_name
echo "1-2" > /sys/class/leds/LED_USB1/device_name

#NETDEV LED Trigger
#netdev
echo "eth0.2"  > /sys/class/leds/LED_EPHY0/device_name
echo "eth0.3"  > /sys/class/leds/LED_EPHY1/device_name
echo "eth0.4"  > /sys/class/leds/LED_EPHY2/device_name
echo "eth0.5"  > /sys/class/leds/LED_EPHY3/device_name
#### Power down unused IPs, because power on all by default.
echo 1 > /proc/realtek/8277x_POWER_DOWN_UNUSED_UNIT
echo 'cfg_xfi0_10g' > /proc/realtek/g3_phy_ctrl_bit_clr
echo 'cfg_xfi1_10g' > /proc/realtek/g3_phy_ctrl_bit_clr

