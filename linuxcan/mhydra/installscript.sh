#!/bin/sh

DEPMOD=`which depmod`
UDEVCTRL=`which udevcontrol`
UDEVADM=`which udevadm`

install -D -m 700 mhydra.ko /lib/modules/`uname -r`/kernel/drivers/usb/misc/mhydra.ko
install -m 700 mhydra.sh /usr/sbin/
if [ -d /etc/hotplug ] ; then
  install -m 777 mhydra /etc/hotplug/usb/ ;
  install -m 644 mhydra.usermap /etc/hotplug/mhydra.usermap
fi
install -m 644 ../10-kvaser.rules /etc/udev/rules.d

if [ `udevd --version` -lt 128 ] ; then
  $UDEVCTRL reload_rules ;
else
  $UDEVADM control --reload-rules ;
fi

$DEPMOD -a
if [ "$?" -ne 0 ] ; then
  echo Failed to execute $DEPMOD -a
fi
