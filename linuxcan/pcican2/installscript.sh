#!/bin/sh

DEPMOD=`which depmod`

install -m 600 kvpcicanII.ko /lib/modules/`uname -r`/kernel/drivers/char/
install -m 700 pcicanII.sh /usr/sbin/

echo Checking for loaded SocketCAN driver for Kvaser PCI devices.
if lsmod | grep kvaser_pci ; then
  echo Unloading SocketCAN driver...
  modprobe -r kvaser_pci
else
  echo SocketCAN driver not found.
fi

echo Blacklisting SocketCAN Kvaser PCI driver to prevent it from auto-loading.

if [ -f /etc/modprobe.conf ] ; then
  # CentOS/Redhat/RHEL/Fedora Linux...
  CONF=/etc/modprobe.conf
  BLACKLIST="alias     kvaser_pci   /dev/null"
else
  # Debian/Ubuntu Linux
  CONF=/etc/modprobe.d/kvaser.conf
  BLACKLIST="blacklist kvaser_pci"
  if [ ! -f $CONF ] ; then
    touch $CONF
  fi
fi

# First, remove any old PCIcan2 settings.
#
grep -v "pcicanII"      < $CONF                            > newconfx
grep -v "^${BLACKLIST}"  < newconfx                         > newconf
rm newconfx

# Add PCIcan.
echo "alias     pcicanII     kvpcicanII"                  >> newconf
echo "install   kvpcicanII   /usr/sbin/pcicanII.sh start" >> newconf
echo "remove    kvpcicanII   /usr/sbin/pcicanII.sh stop"  >> newconf
# Since may conflict with pcicanII, we disable kvaser_pci (SocketCAN).
echo "${BLACKLIST}"                                       >> newconf

cat newconf > $CONF
rm newconf

$DEPMOD -a
if [ "$?" -ne 0 ] ; then
    echo Failed to execute $DEPMOD -a
fi
