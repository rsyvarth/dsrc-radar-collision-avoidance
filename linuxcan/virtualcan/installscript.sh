#!/bin/sh

DEPMOD=`which depmod`

install -m 600 kvvirtualcan.ko /lib/modules/`uname -r`/kernel/drivers/char/
install -m 700 virtualcan.sh /usr/sbin/

if [ -f /etc/modprobe.conf ] ; then
  # CentOS/Redhat/RHEL/Fedora Linux...
  CONF=/etc/modprobe.conf
else
  # Debian/Ubuntu Linux
  CONF=/etc/modprobe.d/kvaser.conf
  if [ ! -f $CONF ] ; then
    touch $CONF
  fi
fi

grep -v virtualcan $CONF                                     > newconf
echo "alias     virtualcan   kvvirtualcan"                  >> newconf
echo "install   kvvirtualcan /usr/sbin/virtualcan.sh start" >> newconf
echo "remove    kvvirtualcan /usr/sbin/virtualcan.sh stop"  >> newconf

cat newconf > $CONF
rm newconf

$DEPMOD -a
if [ "$?" -ne 0 ] ; then
    echo Failed to execute $DEPMOD -a
fi
