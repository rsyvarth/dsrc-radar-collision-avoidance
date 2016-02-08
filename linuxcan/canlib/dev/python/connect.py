import sys
import kvrlib

"""
Connects to the DSRC module with the given serialNo using the
KvaserLib SDK.

"""

kl = kvrlib.kvrlib()
print "kvrlib version: %s" % kl.getVersion()

# CHANGE THIS
serialNo = 16;
print "Connecting to device with serial number %s" % serialNo
addressList = kvrlib.kvrDiscovery.getDefaultAddresses(kvrlib.kvrAddressTypeFlag_BROADCAST)
print "Found %d addresses." % addressList.count
#print addressList
discovery = kl.discoveryOpen()
discovery.setAddresses(addressList)

delay_ms = 100
timeout_ms = 1000
discovery.setScanTime(delay_ms, timeout_ms)
print "Scanning devices...\n"
deviceInfos = discovery.getResults()
#print "Scanning result:\n%s" % deviceInfos
for deviceInfo in deviceInfos:
    if deviceInfo.ser_no == serialNo:
        deviceInfo.connect()
        print 'Connecting to the following device:'
        print '---------------------------------------------'
        print deviceInfo
        discovery.storeDevices(deviceInfos)
        break;
discovery.close()

