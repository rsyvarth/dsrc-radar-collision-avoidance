#!/bin/sh

DEPMOD=`which depmod`
UDEVCTRL=`which udevcontrol`
UDEVADM=`which udevadm`

install -D -m 700 kvcommon.ko /lib/modules/`uname -r`/kernel/drivers/usb/misc

$DEPMOD -a
if [ "$?" -ne 0 ] ; then
  echo Failed to execute $DEPMOD -a
fi
