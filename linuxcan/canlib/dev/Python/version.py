import sys
import canlib
"""
connects to Kvaser LeafLight v2 using the canlib SDK and prints the canlib
version and channel numbers.

"""


cl = canlib.canlib()
print "canlib version: %s" % cl.getVersion()

channel = 0
handle1 = cl.openChannel(channel, canlib.canOPEN_ACCEPT_VIRTUAL)
print "Using channel: %s, EAN: %s" % (handle1.getChannelData_Name(),
                                      handle1.getChannelData_EAN())

handle1.setBusOutputControl(canlib.canDRIVER_NORMAL)
handle1.setBusParams(canlib.canBITRATE_1M)
handle1.busOn()

