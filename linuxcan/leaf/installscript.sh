#!/bin/sh

DEPMOD=`which depmod`
UDEVCTRL=`which udevcontrol`
UDEVADM=`which udevadm`

install -D -m 700 leaf.ko /lib/modules/`uname -r`/kernel/drivers/usb/misc/leaf.ko
install -m 700 leaf.sh /usr/sbin/
if [ -d /etc/hotplug ] ; then
  install -m 777 leaf /etc/hotplug/usb/ ;
  install -m 644 leaf.usermap /etc/hotplug/leaf.usermap
fi
install -m 644 ../10-kvaser.rules /etc/udev/rules.d

if [ `udevd --version` -lt 128 ] ; then
  $UDEVCTRL reload_rules ;
else
  $UDEVADM control --reload-rules ;
fi

echo Checking for loaded SocketCAN driver for Kvaser USB devices.
if lsmod | grep kvaser_usb ; then
  echo Unloading SocketCAN driver...
  modprobe -r kvaser_usb
else
  echo SocketCAN driver not found.
fi

echo Blacklisting SocketCAN Kvaser USB driver to prevent it from auto-loading.

if [ -f /etc/modprobe.conf ] ; then
  # CentOS/Redhat/RHEL/Fedora Linux...
  CONF=/etc/modprobe.conf
  BLACKLIST="alias     kvaser_usb   /dev/null"
else
  # Debian/Ubuntu Linux
  CONF=/etc/modprobe.d/kvaser.conf
  BLACKLIST="blacklist kvaser_usb"
  if [ ! -f $CONF ] ; then
    touch $CONF
  fi
fi

# Since it conflicts with leaf, we disable kvaser_usb (SocketCAN).
grep -v "^${BLACKLIST}"  < $CONF                         > newconf
echo "${BLACKLIST}"                                     >> newconf

cat newconf > $CONF
rm newconf

$DEPMOD -a
if [ "$?" -ne 0 ] ; then
  echo Failed to execute $DEPMOD -a
fi
