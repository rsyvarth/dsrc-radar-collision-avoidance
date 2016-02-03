#!/bin/sh

DEPMOD=`which depmod`

install -m 600 kvpciefd.ko /lib/modules/`uname -r`/kernel/drivers/char/
install -m 700 pciefd.sh /usr/sbin/

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

# First, remove any old PCIeFD settings.
#
grep -v "pciefd"         < $CONF                           > newconfx
grep -v "^${BLACKLIST}"  < newconfx                        > newconf
rm newconfx

# Add PCIeFD.
echo "alias     pciefd       kvpciefd"                    >> newconf
echo "install   kvpciefd     /usr/sbin/pciefd.sh start"   >> newconf
echo "remove    kvpciefd     /usr/sbin/pciefd.sh stop"    >> newconf
# Since may conflict with pciefd, we disable kvaser_pci (SocketCAN).
echo "${BLACKLIST}"                                       >> newconf

cat newconf > $CONF
rm newconf

$DEPMOD -a
if [ "$?" -ne 0 ] ; then
  echo Failed to execute $DEPMOD -a
fi
