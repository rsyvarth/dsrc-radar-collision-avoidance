import sys

import kvrlib

canChannel = 0
deviceName = "MrKvaser"

kl = kvrlib.kvrlib()
print "Configurating device on channel %d." % canChannel
try:
    config = kl.configOpen(channel=canChannel, mode=kvrlib.kvrConfig.MODE_RW)
    config.getXml()
    print "Current device configuration:\n%s" % config.xml.toxml()
    print "Device name was '%s'" % config.xml.getElementsByTagName('NETWORK')[0].attributes['device_name'].value
    print "Setting new device name '%s'" % deviceName
    config.xml.getElementsByTagName('NETWORK')[0].attributes['device_name'].value = deviceName
    config.setXml()
    config.close()
except (kvrlib.kvrError) as ex:
    print ex

kl.unload()
